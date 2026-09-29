#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$ROOT_DIR/install/setup.bash"
  set -u
else
  echo "Missing $ROOT_DIR/install/setup.bash. Build and source first." >&2
  exit 1
fi

DURATION_SEC="${1:-5}"
CMD_VEL_LINEAR="${2:-0.2}"
CMD_VEL_RATE="${3:-10}"
RUNS="${4:-10}"
WORLD="${WORLD:-random_world.sdf}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre2}"
RENDER_ENGINE_GUI="${RENDER_ENGINE_GUI:-ogre2}"
HEADLESS="${HEADLESS:-1}"
NAV2_CHECK="${NAV2_CHECK:-1}"
GOAL_X="${GOAL_X:-0.8}"
GOAL_Y="${GOAL_Y:-0.0}"
GOAL_TIMEOUT_SEC="${GOAL_TIMEOUT_SEC:-40}"

SIM_LAUNCH_CMD=(ros2 launch maya_bringup maya.launch.xml "rviz:=false")
if [[ "$HEADLESS" == "1" ]]; then
  SIM_LAUNCH_CMD+=("gz_args:=${WORLD} -s --render-engine ${RENDER_ENGINE} --render-engine-gui ${RENDER_ENGINE_GUI} -r")
else
  SIM_LAUNCH_CMD+=("world:=${WORLD}")
fi

tmpdir="$(mktemp -d)"
export TMPDIR_PATH="$tmpdir"
ros_log_dir="$tmpdir/roslog"
mkdir -p "$ros_log_dir"
export ROS_LOG_DIR="$ros_log_dir"
cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT -- "-$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
  rm -rf "$tmpdir"
}
trap cleanup EXIT

setsid "${SIM_LAUNCH_CMD[@]}" >/dev/null 2>&1 &
LAUNCH_PID=$!

# Wait for /clock and /odom to exist
for _ in {1..60}; do
  topics="$(ros2 topic list 2>/dev/null || true)"
  if printf '%s\n' "$topics" | grep -q '^/clock$' && printf '%s\n' "$topics" | grep -q '^/odom$'; then
    break
  fi
  sleep 1
  if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
    echo "Launch exited early; check logs." >&2
    exit 1
  fi
done

topics="$(ros2 topic list 2>/dev/null || true)"
if ! printf '%s\n' "$topics" | grep -q '^/clock$'; then
  echo "Timed out waiting for /clock." >&2
  exit 1
fi

WORLD_TOPIC="$(gz topic -l | awk '/\/world\/.*\/dynamic_pose\/info/ {print $1; exit}')"
if [[ -z "$WORLD_TOPIC" ]]; then
  echo "Could not find /world/*/dynamic_pose/info topic." >&2
  exit 1
fi

echo "Using Gazebo pose topic: $WORLD_TOPIC"

get_maya_pose() {
  gz topic -e -n 1 -t "$WORLD_TOPIC" | awk '
    $0 ~ /name: "maya"/ {in_block=1}
    in_block && $1 ~ /^position/ {pos=1}
    in_block && pos && $1=="x:" {x=$2}
    in_block && pos && $1=="y:" {y=$2}
    in_block && pos && $1=="z:" {z=$2; pos=0}
    in_block && $1 ~ /^orientation/ {ori=1}
    in_block && ori && $1=="x:" {qx=$2}
    in_block && ori && $1=="y:" {qy=$2}
    in_block && ori && $1=="z:" {qz=$2}
    in_block && ori && $1=="w:" {qw=$2; ori=0; in_block=0}
    END {printf("%s %s %s %s %s %s %s\n", x, y, z, qx, qy, qz, qw)}'
}

get_maya_pose_retry() {
  local tries=5
  local out=""
  for _ in $(seq 1 "$tries"); do
    out="$(get_maya_pose)"
    if [[ -n "$out" && "$out" != "      " ]]; then
      printf '%s\n' "$out"
      return 0
    fi
    sleep 0.2
  done
  return 1
}

capture_all() {
  local label="$1"
  ros2 topic echo --once /clock >"$tmpdir/${label}_clock.txt"
  if ! get_maya_pose_retry >"$tmpdir/${label}_gz_pose.txt"; then
    echo "Failed to capture Gazebo pose for $label." >&2
    return 1
  fi
  ros2 topic echo --once /odom >"$tmpdir/${label}_odom.txt"
}

# Multi-run loop (each run overwrites capture files)
for run in $(seq 1 "$RUNS"); do
  echo "Run $run/${RUNS}..." >&2
  capture_all start || exit 1
  sleep 1
  if command -v timeout >/dev/null 2>&1; then
    timeout "${DURATION_SEC}s" ros2 topic pub -r "$CMD_VEL_RATE" /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $CMD_VEL_LINEAR}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  else
    ros2 topic pub -r "$CMD_VEL_RATE" /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $CMD_VEL_LINEAR}, angular: {z: 0.0}}" >/dev/null 2>&1 &
    pub_pid=$!
    sleep "$DURATION_SEC"
    kill -INT "$pub_pid" >/dev/null 2>&1 || true
  fi
  if command -v timeout >/dev/null 2>&1; then
    timeout 3s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  else
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  fi
  sleep 2
  capture_all end || exit 1
  python3 - <<'PY' "$run" "$tmpdir"
import math, re
from pathlib import Path
import json
import os
import sys

run = sys.argv[1]
cand = Path(sys.argv[2])
if not cand or not (cand / 'start_clock.txt').exists():
    print("Failed to locate capture files.")
    raise SystemExit(1)

def read_clock(path: Path):
    text = path.read_text()
    sec = int(re.search(r'sec:\s*(\d+)', text).group(1))
    nsec = int(re.search(r'nanosec:\s*(\d+)', text).group(1))
    return sec + nsec * 1e-9

def read_pose_xyz(path: Path):
    parts = path.read_text().strip().split()
    return tuple(float(p) for p in parts[:3])

def read_odom_xyz(path: Path):
    text = path.read_text()
    x = float(re.search(r'position:\s*\n\s*x:\s*([\-0-9.eE]+)', text).group(1))
    return x

start_clock = read_clock(cand / 'start_clock.txt')
end_clock = read_clock(cand / 'end_clock.txt')
start_gz = read_pose_xyz(cand / 'start_gz_pose.txt')
end_gz = read_pose_xyz(cand / 'end_gz_pose.txt')
start_odom = read_odom_xyz(cand / 'start_odom.txt')
end_odom = read_odom_xyz(cand / 'end_odom.txt')
gz_dx = end_gz[0] - start_gz[0]
odom_dx = end_odom - start_odom
scale = (odom_dx / gz_dx) if abs(gz_dx) > 1e-9 else float('nan')
out = {"dt": end_clock-start_clock, "gz_dx": gz_dx, "odom_dx": odom_dx, "scale": scale}
Path(cand / f"run_{run}.json").write_text(json.dumps(out))
print(json.dumps(out))
PY
done

python3 - <<'PY' "$tmpdir"
import json, statistics as stats, sys
from pathlib import Path

cand = Path(sys.argv[1])
rows = []
for p in sorted(cand.glob("run_*.json")):
    rows.append(json.loads(p.read_text()))

if rows:
    scales = [r["scale"] for r in rows]
    dts = [r["dt"] for r in rows]
    print("\n=== 10-run Summary ===")
    print(f"scale mean: {stats.mean(scales):.3f}")
    print(f"scale median: {stats.median(scales):.3f}")
    print(f"scale stdev: {stats.pstdev(scales):.3f}")
    print(f"dt mean: {stats.mean(dts):.3f}s")
PY

diag_dir="$tmpdir/diagnostics"
mkdir -p "$diag_dir"
echo ""
echo "=== TF / SLAM Diagnostics ==="

timeout 6s ros2 run tf2_ros tf2_echo odom base_footprint >"$diag_dir/tf_odom_base.txt" 2>&1 || true
timeout 6s ros2 run tf2_ros tf2_echo map odom >"$diag_dir/tf_map_odom.txt" 2>&1 || true
timeout 6s ros2 run tf2_ros tf2_echo base_footprint lidar_link >"$diag_dir/tf_base_lidar.txt" 2>&1 || true

(cd "$diag_dir" && timeout 8s ros2 run tf2_tools view_frames >/dev/null 2>&1 || true)

timeout 6s ros2 topic echo --once /map >"$diag_dir/map.txt" 2>&1 || true
timeout 6s ros2 topic hz /tf >"$diag_dir/hz_tf.txt" 2>&1 || true
timeout 6s ros2 topic hz /tf_static >"$diag_dir/hz_tf_static.txt" 2>&1 || true
timeout 6s ros2 topic hz /scan_merged >"$diag_dir/hz_scan_merged.txt" 2>&1 || true
timeout 6s ros2 topic hz /map >"$diag_dir/hz_map.txt" 2>&1 || true

echo "Diagnostics saved in: $diag_dir"
echo ""
echo "-- tf odom -> base_footprint (last lines) --"
tail -n 3 "$diag_dir/tf_odom_base.txt" 2>/dev/null || true
echo ""
echo "-- tf map -> odom (last lines) --"
tail -n 3 "$diag_dir/tf_map_odom.txt" 2>/dev/null || true
echo ""
echo "-- tf base_footprint -> lidar_link (last lines) --"
tail -n 3 "$diag_dir/tf_base_lidar.txt" 2>/dev/null || true
echo ""
echo "-- /map header (frame_id/resolution/width/height) --"
grep -E 'frame_id|resolution|width|height' "$diag_dir/map.txt" 2>/dev/null | head -n 8 || true
echo ""
echo "-- hz summaries (last line) --"
for f in hz_tf hz_tf_static hz_scan_merged hz_map; do
  printf "%s: " "$f"
  tail -n 1 "$diag_dir/${f}.txt" 2>/dev/null || echo "no data"
done

if [[ "$NAV2_CHECK" == "1" ]]; then
  echo ""
  echo "=== Nav2 Action Diagnostics ==="

  map_meta_file="$diag_dir/map_metadata_before_nav2.txt"
  timeout 6s ros2 topic echo --once /map_metadata >"$map_meta_file" 2>&1 || true
  map_w="$(awk '/^width:/ {print $2}' "$map_meta_file" 2>/dev/null || true)"
  map_h="$(awk '/^height:/ {print $2}' "$map_meta_file" 2>/dev/null || true)"
  map_w="${map_w:-0}"
  map_h="${map_h:-0}"
  echo "Map metadata before Nav2 test: width=${map_w} height=${map_h}"

  if [[ "${map_w}" == "0" || "${map_h}" == "0" ]]; then
    echo "Map is empty; performing short warmup motion for SLAM..."
    timeout 8s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
    timeout 2s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
    sleep 1
    timeout 6s ros2 topic echo --once /map_metadata >"$diag_dir/map_metadata_after_warmup.txt" 2>&1 || true
  fi

  compute_goal="{goal: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, planner_id: GridBased, use_start: false}"
  nav_goal="{pose: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

  echo "Running /compute_path_to_pose ..."
  timeout "${GOAL_TIMEOUT_SEC}s" ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "$compute_goal" --feedback >"$diag_dir/compute_path_to_pose.txt" 2>&1 || true
  echo "Running /navigate_to_pose ..."
  timeout "${GOAL_TIMEOUT_SEC}s" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$nav_goal" --feedback >"$diag_dir/navigate_to_pose.txt" 2>&1 || true

  echo "-- ComputePath result --"
  grep -E "Goal finished with status|error_code:|error_msg:" "$diag_dir/compute_path_to_pose.txt" 2>/dev/null || true
  echo "-- NavigateToPose result --"
  grep -E "Goal finished with status|error_code:|error_msg:" "$diag_dir/navigate_to_pose.txt" 2>/dev/null || true
fi
