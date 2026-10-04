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

# Use Gazebo's flat empty world by default so turn diagnostics isolate drive
# and odometry behavior from terrain/heightmap effects.
WORLD="${WORLD:-empty.sdf}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre2}"
RENDER_ENGINE_GUI="${RENDER_ENGINE_GUI:-ogre2}"
STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-120}"
RECORD_DURATION_SEC="${RECORD_DURATION_SEC:-35}"
SAMPLE_HZ="${SAMPLE_HZ:-5.0}"
TURN_MODE="${TURN_MODE:-scripted}"
TURN_SETTLE_SEC="${TURN_SETTLE_SEC:-2}"
TURN_CMD_VEL_Z="${TURN_CMD_VEL_Z:-0.6}"
TURN_DEG="${TURN_DEG:-180}"
TURN_TIMEOUT_SEC="${TURN_TIMEOUT_SEC:-40}"
RVIZ="${RVIZ:-false}"
CLEANUP_DEBUG="${CLEANUP_DEBUG:-false}"
STRICT_CLEANUP="${STRICT_CLEANUP:-false}"

CLEANUP_GRACEFUL_PATTERN="ros2 launch maya_bringup|gz sim|ros_gz_sim|rviz2|slam_toolbox|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|lifecycle_manager|map_server|amcl|waypoint_follower|collision_monitor|ekf_node|robot_state_publisher|component_container"
CLEANUP_FORCE_PATTERN="$CLEANUP_GRACEFUL_PATTERN"

tmpdir="$(mktemp -d)"
diag_root="$tmpdir/turn_drift_diagnostic"
mkdir -p "$diag_root"
cleanup_done="false"
collector_pid=""
TURN_PUB_PID=""

terminate_background_jobs() {
  if [[ -n "${TURN_PUB_PID:-}" ]]; then
    kill -INT "$TURN_PUB_PID" >/dev/null 2>&1 || true
    wait "$TURN_PUB_PID" >/dev/null 2>&1 || true
    unset TURN_PUB_PID
  fi
  if [[ -n "${collector_pid:-}" ]]; then
    kill -INT "$collector_pid" >/dev/null 2>&1 || true
    wait "$collector_pid" >/dev/null 2>&1 || true
    unset collector_pid
  fi
}

cleanup_once() {
  local stage="${1:-exit}"
  [[ "$cleanup_done" == "true" ]] && return 0
  cleanup_done="true"
  terminate_background_jobs
  stop_sim
  run_cleanup_cycle "$stage" >/dev/null 2>&1 || true
  echo "Diagnostics root: $diag_root"
}

on_signal() {
  cleanup_once "signal"
  exit 130
}

trap 'cleanup_once "exit"' EXIT
trap on_signal INT TERM

cleanup_debug_log() {
  if [[ "$CLEANUP_DEBUG" == "true" ]]; then
    echo "$@"
  fi
}

kill_matching_processes() {
  local signal="$1"
  local pattern="$2"
  local stage="$3"
  local self_pid="$$"
  local parent_pid="$PPID"
  local self_pgid
  self_pgid="$(ps -o pgid= "$self_pid" 2>/dev/null | tr -d '[:space:]' || true)"
  while read -r pid cmdline; do
    [[ -z "${pid:-}" ]] && continue
    [[ "$pid" == "$self_pid" || "$pid" == "$parent_pid" ]] && continue
    [[ "${cmdline:-}" == *"turn_drift_diagnostic.sh"* ]] && continue
    [[ "${cmdline:-}" == *"nav2_reliability_trials.sh"* ]] && continue
    [[ "${cmdline:-}" == *"ros2cli.daemon.daemonize"* ]] && continue
    if [[ -n "${self_pgid:-}" ]]; then
      pid_pgid="$(ps -o pgid= "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
      [[ -n "${pid_pgid:-}" && "$pid_pgid" == "$self_pgid" ]] && continue
    fi
    cleanup_debug_log "Cleanup [$stage]: kill -${signal} pid=$pid cmd=${cmdline:-<unknown>}"
    kill "-$signal" "$pid" >/dev/null 2>&1 || true
  done < <(pgrep -af "$pattern" || true)
}

list_stale_processes() {
  local pattern="$1"
  local self_pid="$$"
  local parent_pid="$PPID"
  local self_pgid
  self_pgid="$(ps -o pgid= "$self_pid" 2>/dev/null | tr -d '[:space:]' || true)"
  local stale=""
  while read -r pid cmdline; do
    [[ -z "${pid:-}" ]] && continue
    [[ "$pid" == "$self_pid" || "$pid" == "$parent_pid" ]] && continue
    [[ "${cmdline:-}" == *"turn_drift_diagnostic.sh"* ]] && continue
    [[ "${cmdline:-}" == *"nav2_reliability_trials.sh"* ]] && continue
    [[ "${cmdline:-}" == *"ros2cli.daemon.daemonize"* ]] && continue
    if [[ -n "${self_pgid:-}" ]]; then
      pid_pgid="$(ps -o pgid= "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
      [[ -n "${pid_pgid:-}" && "$pid_pgid" == "$self_pgid" ]] && continue
    fi
    stale+="${pid} ${cmdline:-<unknown>}"$'\n'
  done < <(pgrep -af "$pattern" || true)
  printf '%s' "$stale"
}

verify_clean_state() {
  local stage="$1"
  local stale
  stale="$(list_stale_processes "$CLEANUP_FORCE_PATTERN")"
  if [[ -n "$stale" ]]; then
    echo "Cleanup [$stage]: stale processes remain after cleanup:" >&2
    printf '%s' "$stale" >&2
    if [[ "$STRICT_CLEANUP" == "true" ]]; then
      echo "Cleanup [$stage]: STRICT_CLEANUP=true, treating stale state as failure." >&2
      return 1
    fi
  fi
  return 0
}

run_cleanup_cycle() {
  local stage="$1"
  echo "Cleanup [$stage]: stopping stale ROS/Gazebo processes and resetting ROS 2 daemon."
  kill_matching_processes "INT" "$CLEANUP_GRACEFUL_PATTERN" "$stage"
  sleep 3
  kill_matching_processes "TERM" "$CLEANUP_FORCE_PATTERN" "$stage"
  sleep 2
  kill_matching_processes "KILL" "$CLEANUP_FORCE_PATTERN" "$stage"
  ros2 daemon stop >/dev/null 2>&1 || true
  pkill -f "_ros2_daemon" >/dev/null 2>&1 || true
  sleep 1
  ros2 daemon start >/dev/null 2>&1 || true
  verify_clean_state "$stage"
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  local end=$((SECONDS + timeout_s))
  while (( SECONDS < end )); do
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

lookup_gz_world_topic() {
  gz topic -l | awk '/\/world\/.*\/dynamic_pose\/info/ {print $1; exit}'
}

get_maya_pose() {
  local world_topic="$1"
  local snapshot=""
  snapshot="$(timeout 2s gz topic -e -n 1 -t "$world_topic" 2>/dev/null || true)"
  [[ -z "$snapshot" ]] && return 1

  awk '
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
    END {printf("%s %s %s %s %s %s %s\n", x, y, z, qx, qy, qz, qw)}' <<< "$snapshot"
}

get_maya_pose_retry() {
  local world_topic="$1"
  local tries=8
  local out=""
  for _ in $(seq 1 "$tries"); do
    out="$(get_maya_pose "$world_topic" || true)"
    if [[ -n "$out" && "$out" != "      " ]]; then
      printf '%s\n' "$out"
      return 0
    fi
    sleep 0.25
  done
  return 1
}

normalize_angle_diff() {
  local start_yaw="$1"
  local current_yaw="$2"
  python3 - <<'PY' "$start_yaw" "$current_yaw"
import math
import sys
s = float(sys.argv[1])
c = float(sys.argv[2])
d = c - s
while d > math.pi:
    d -= 2.0 * math.pi
while d < -math.pi:
    d += 2.0 * math.pi
print(d)
PY
}

start_sim() {
  local run_dir="$1"
  local ros_log_dir="$run_dir/roslog"
  mkdir -p "$ros_log_dir"
  export ROS_LOG_DIR="$ros_log_dir"
  local -a cmd=(ros2 launch maya_bringup maya.launch.xml "rviz:=${RVIZ}" "gz_args:=${WORLD} -s --render-engine ${RENDER_ENGINE} --render-engine-gui ${RENDER_ENGINE_GUI} -r")
  setsid "${cmd[@]}" >"$run_dir/launch.log" 2>&1 &
  LAUNCH_PID=$!
}

stop_sim() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT -- "-$LAUNCH_PID" >/dev/null 2>&1 || true
    for _ in $(seq 1 20); do
      if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
        break
      fi
      sleep 0.25
    done
    if kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
      kill -TERM -- "-$LAUNCH_PID" >/dev/null 2>&1 || true
      for _ in $(seq 1 20); do
        if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
          break
        fi
        sleep 0.25
      done
    fi
    if kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
      kill -KILL -- "-$LAUNCH_PID" >/dev/null 2>&1 || true
    fi
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
    unset LAUNCH_PID
  fi
}

run_dir="$diag_root/run_1"
mkdir -p "$run_dir"

run_cleanup_cycle "pre"
start_sim "$run_dir"

wait_for_topic /clock "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /clock" >&2; stop_sim; exit 1; }
wait_for_topic /scan "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /scan" >&2; stop_sim; exit 1; }
wait_for_topic /odom "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /odom" >&2; stop_sim; exit 1; }
wait_for_topic /odom_with_covariance "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /odom_with_covariance" >&2; stop_sim; exit 1; }
wait_for_topic /odometry/filtered "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /odometry/filtered" >&2; stop_sim; exit 1; }
wait_for_topic /imu "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /imu" >&2; stop_sim; exit 1; }
wait_for_topic /imu_with_covariance "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /imu_with_covariance" >&2; stop_sim; exit 1; }
wait_for_topic /map "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /map" >&2; stop_sim; exit 1; }

world_topic="$(lookup_gz_world_topic || true)"
if [[ -z "$world_topic" ]]; then
  echo "Could not find Gazebo dynamic pose topic" >&2
  stop_sim
  exit 1
fi
printf '%s\n' "$world_topic" >"$run_dir/gz_world_topic.txt"

echo "Turn drift diagnostics: sim is ready."
echo "Mode: $TURN_MODE"
echo "World: $WORLD"

python3 - <<'PY' "$run_dir" "$world_topic" "$RECORD_DURATION_SEC" "$SAMPLE_HZ" >"$run_dir/collector_stdout.log" 2>"$run_dir/collector_stderr.log" &
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

run_dir = Path(sys.argv[1])
world_topic = sys.argv[2]
duration_s = float(sys.argv[3])
sample_hz = float(sys.argv[4])
samples_path = run_dir / "samples.jsonl"
meta_path = run_dir / "collector_meta.json"

state = {
    "odom": None,
    "odom_cov": None,
    "filtered": None,
    "imu": None,
    "imu_cov": None,
}

def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def cov_diag(cov, idx):
    if cov is None or len(cov) <= idx:
      return None
    return float(cov[idx])

def odom_to_dict(msg):
    pose = msg.pose.pose
    twist = msg.twist.twist
    return {
        "stamp_sec": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "yaw": quat_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        ),
        "vx": float(twist.linear.x),
        "wz": float(twist.angular.z),
        "cov_x": cov_diag(msg.pose.covariance, 0),
        "cov_y": cov_diag(msg.pose.covariance, 7),
        "cov_yaw": cov_diag(msg.pose.covariance, 35),
        "twist_cov_vx": cov_diag(msg.twist.covariance, 0),
        "twist_cov_wz": cov_diag(msg.twist.covariance, 35),
    }

def imu_to_dict(msg):
    q = msg.orientation
    av = msg.angular_velocity
    return {
        "stamp_sec": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
        "frame_id": msg.header.frame_id,
        "yaw": quat_to_yaw(q.x, q.y, q.z, q.w),
        "wz": float(av.z),
        "cov_yaw": cov_diag(msg.orientation_covariance, 8),
        "cov_wz": cov_diag(msg.angular_velocity_covariance, 8),
    }

def parse_gz_pose(text):
    block = None
    lines = text.splitlines()
    current = []
    in_block = False
    for line in lines:
        if 'name: "maya"' in line:
            in_block = True
            current = [line]
            continue
        if in_block:
            current.append(line)
            if re.match(r'^\s*-\s*header:', line):
                block = "\n".join(current[:-1])
                break
    if block is None and in_block:
        block = "\n".join(current)
    if not block:
        return None
    nums = {}
    pos = re.search(r'position\s*\{[^}]*x:\s*([\-0-9.eE]+)[^}]*y:\s*([\-0-9.eE]+)[^}]*z:\s*([\-0-9.eE]+)', block, re.S)
    ori = re.search(r'orientation\s*\{[^}]*x:\s*([\-0-9.eE]+)[^}]*y:\s*([\-0-9.eE]+)[^}]*z:\s*([\-0-9.eE]+)[^}]*w:\s*([\-0-9.eE]+)', block, re.S)
    if not pos or not ori:
        return None
    x, y, z = map(float, pos.groups())
    qx, qy, qz, qw = map(float, ori.groups())
    return {
        "x": x, "y": y, "z": z,
        "yaw": quat_to_yaw(qx, qy, qz, qw),
    }

def sample_gz_pose():
    try:
        proc = subprocess.run(
            ["timeout", "2s", "gz", "topic", "-e", "-n", "1", "-t", world_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=3.0,
            check=False,
        )
    except Exception:
        return None
    if proc.returncode not in (0, 124):
        return None
    return parse_gz_pose(proc.stdout)

rclpy.init()
node = rclpy.create_node("turn_drift_collector")
buf = Buffer()
listener = TransformListener(buf, node, spin_thread=False)

node.create_subscription(Odometry, "/odom", lambda msg: state.__setitem__("odom", odom_to_dict(msg)), 10)
node.create_subscription(Odometry, "/odom_with_covariance", lambda msg: state.__setitem__("odom_cov", odom_to_dict(msg)), 10)
node.create_subscription(Odometry, "/odometry/filtered", lambda msg: state.__setitem__("filtered", odom_to_dict(msg)), 10)
node.create_subscription(Imu, "/imu", lambda msg: state.__setitem__("imu", imu_to_dict(msg)), 10)
node.create_subscription(Imu, "/imu_with_covariance", lambda msg: state.__setitem__("imu_cov", imu_to_dict(msg)), 10)

start = time.time()
next_sample = start
count = 0
with samples_path.open("w", encoding="utf-8") as f:
    while rclpy.ok():
        now = time.time()
        if now - start >= duration_s:
            break
        rclpy.spin_once(node, timeout_sec=0.05)
        if now < next_sample:
            continue

        rec = {
            "wall_time_sec": now - start,
            "odom": state["odom"],
            "odom_with_covariance": state["odom_cov"],
            "odometry_filtered": state["filtered"],
            "imu": state["imu"],
            "imu_with_covariance": state["imu_cov"],
            "gazebo": sample_gz_pose(),
            "tf": {},
        }

        for target, source, key in [
            ("odom", "base_footprint", "odom_base"),
            ("map", "base_footprint", "map_base"),
        ]:
            try:
                t = buf.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.2))
                rec["tf"][key] = {
                    "x": float(t.transform.translation.x),
                    "y": float(t.transform.translation.y),
                    "yaw": quat_to_yaw(
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w,
                    ),
                }
            except TransformException:
                rec["tf"][key] = None

        f.write(json.dumps(rec) + "\n")
        f.flush()
        count += 1
        next_sample = now + (1.0 / sample_hz)

meta_path.write_text(json.dumps({
    "duration_sec": duration_s,
    "sample_hz": sample_hz,
    "samples_written": count,
}, indent=2))

node.destroy_node()
rclpy.shutdown()
PY
collector_pid=$!

if [[ "$TURN_MODE" == "scripted" ]]; then
  echo "Running scripted pure rotation: target=${TURN_DEG} deg cmd_wz=${TURN_CMD_VEL_Z} rad/s"
  start_pose="$(get_maya_pose_retry "$world_topic" || true)"
  if [[ -z "$start_pose" ]]; then
    echo "Could not sample initial Gazebo yaw for scripted turn." >&2
  else
    start_yaw="$(python3 - <<'PY' "$start_pose"
import math
import sys
parts = sys.argv[1].split()
_, _, _, qx, qy, qz, qw = map(float, parts[:7])
siny_cosp = 2.0 * (qw * qz + qx * qy)
cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
print(math.atan2(siny_cosp, cosy_cosp))
PY
)"
    target_turn_rad="$(python3 - <<'PY' "$TURN_DEG"
import math
import sys
print(abs(float(sys.argv[1])) * math.pi / 180.0)
PY
)"
    turn_sign="$(python3 - <<'PY' "$TURN_DEG"
import sys
print(1.0 if float(sys.argv[1]) >= 0.0 else -1.0)
PY
)"
    turn_cmd_z="$(python3 - <<'PY' "$TURN_CMD_VEL_Z" "$turn_sign"
import sys
print(abs(float(sys.argv[1])) * float(sys.argv[2]))
PY
)"
    ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: ${turn_cmd_z}}}" >/dev/null 2>&1 &
    TURN_PUB_PID=$!
    turn_start_sec=$SECONDS
    while :; do
      current_pose="$(get_maya_pose_retry "$world_topic" || true)"
      if [[ -n "$current_pose" ]]; then
        current_yaw="$(python3 - <<'PY' "$current_pose"
import math
import sys
parts = sys.argv[1].split()
_, _, _, qx, qy, qz, qw = map(float, parts[:7])
siny_cosp = 2.0 * (qw * qz + qx * qy)
cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
print(math.atan2(siny_cosp, cosy_cosp))
PY
)"
        turn_delta="$(normalize_angle_diff "$start_yaw" "$current_yaw")"
        turn_abs="$(python3 - <<'PY' "$turn_delta"
import sys
print(abs(float(sys.argv[1])))
PY
)"
        reached="$(python3 - <<'PY' "$turn_abs" "$target_turn_rad"
import sys
print("1" if float(sys.argv[1]) >= float(sys.argv[2]) else "0")
PY
)"
        if [[ "$reached" == "1" ]]; then
          break
        fi
      fi
      if (( SECONDS - turn_start_sec >= TURN_TIMEOUT_SEC )); then
        echo "Scripted turn timed out after ${TURN_TIMEOUT_SEC}s." >&2
        break
      fi
      sleep 0.2
    done
    kill -INT "$TURN_PUB_PID" >/dev/null 2>&1 || true
    wait "$TURN_PUB_PID" >/dev/null 2>&1 || true
    unset TURN_PUB_PID
    timeout 3s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  fi
else
  echo "Manual mode: drive the rover now. Recording window is ${RECORD_DURATION_SEC}s."
fi

wait "$collector_pid"
unset collector_pid
sleep "$TURN_SETTLE_SEC"

python3 - <<'PY' "$run_dir" "$TURN_MODE" "$TURN_DEG"
import json
import math
import statistics as stats
import sys
from pathlib import Path

run_dir = Path(sys.argv[1])
turn_mode = sys.argv[2]
turn_deg = float(sys.argv[3])
samples_path = run_dir / "samples.jsonl"
summary_path = run_dir / "summary.json"

def read_samples(path):
    rows = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line:
            continue
        rows.append(json.loads(line))
    return rows

def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def relative_series(samples, getter):
    vals = []
    start = None
    for s in samples:
        obj = getter(s)
        if obj is None:
            vals.append(None)
            continue
        yaw = obj.get("yaw")
        if yaw is None:
            vals.append(None)
            continue
        if start is None:
            start = yaw
        vals.append(norm_angle(yaw - start))
    return vals

def final_valid(vals):
    for v in reversed(vals):
        if v is not None:
            return v
    return None

def xy_drift_stats(samples, getter):
    pts = []
    for s in samples:
        obj = getter(s)
        if obj is None:
            continue
        x = obj.get("x")
        y = obj.get("y")
        if x is None or y is None:
            continue
        pts.append((float(x), float(y)))
    if len(pts) < 2:
        return None
    x0, y0 = pts[0]
    rel = [(x - x0, y - y0) for x, y in pts]
    radii = [math.hypot(x, y) for x, y in rel]
    xf, yf = rel[-1]
    return {
        "count": len(pts),
        "final_dx_m": xf,
        "final_dy_m": yf,
        "final_drift_m": math.hypot(xf, yf),
        "max_drift_from_start_m": max(radii),
        "mean_drift_from_start_m": stats.mean(radii),
    }

def yaw_error_stats(ref, other):
    errs = []
    for a, b in zip(ref, other):
        if a is None or b is None:
            continue
        errs.append(norm_angle(b - a))
    if not errs:
        return None
    abs_errs = [abs(x) for x in errs]
    return {
        "count": len(errs),
        "mean_abs_rad": stats.mean(abs_errs),
        "max_abs_rad": max(abs_errs),
        "mean_abs_deg": math.degrees(stats.mean(abs_errs)),
        "max_abs_deg": math.degrees(max(abs_errs)),
    }

def mean_of(path, key):
    vals = []
    for s in samples:
        obj = path(s)
        if obj is None:
            continue
        v = obj.get(key)
        if v is not None:
            vals.append(float(v))
    return stats.mean(vals) if vals else None

samples = read_samples(samples_path)

series = {
    "gazebo": relative_series(samples, lambda s: s.get("gazebo")),
    "odom_msg": relative_series(samples, lambda s: s.get("odom")),
    "odom_with_covariance": relative_series(samples, lambda s: s.get("odom_with_covariance")),
    "odometry_filtered": relative_series(samples, lambda s: s.get("odometry_filtered")),
    "imu": relative_series(samples, lambda s: s.get("imu")),
    "imu_with_covariance": relative_series(samples, lambda s: s.get("imu_with_covariance")),
    "tf_odom_base": relative_series(samples, lambda s: (s.get("tf") or {}).get("odom_base")),
    "tf_map_base": relative_series(samples, lambda s: (s.get("tf") or {}).get("map_base")),
}

summary = {
    "turn_mode": turn_mode,
    "commanded_turn_deg": turn_deg,
    "samples_count": len(samples),
    "duration_sec": samples[-1]["wall_time_sec"] if samples else None,
    "covariance_means": {
        "odom_cov_yaw_raw": mean_of(lambda s: s.get("odom"), "cov_yaw"),
        "odom_cov_yaw_relay": mean_of(lambda s: s.get("odom_with_covariance"), "cov_yaw"),
        "odom_filtered_cov_yaw": mean_of(lambda s: s.get("odometry_filtered"), "cov_yaw"),
        "imu_cov_yaw_raw": mean_of(lambda s: s.get("imu"), "cov_yaw"),
        "imu_cov_yaw_relay": mean_of(lambda s: s.get("imu_with_covariance"), "cov_yaw"),
        "imu_cov_wz_raw": mean_of(lambda s: s.get("imu"), "cov_wz"),
        "imu_cov_wz_relay": mean_of(lambda s: s.get("imu_with_covariance"), "cov_wz"),
    },
    "final_relative_yaw_deg": {},
    "xy_drift": {},
    "yaw_error_vs_gazebo": {},
}

xy_sources = {
    "gazebo": lambda s: s.get("gazebo"),
    "odom_msg": lambda s: s.get("odom"),
    "odom_with_covariance": lambda s: s.get("odom_with_covariance"),
    "odometry_filtered": lambda s: s.get("odometry_filtered"),
    "tf_odom_base": lambda s: (s.get("tf") or {}).get("odom_base"),
    "tf_map_base": lambda s: (s.get("tf") or {}).get("map_base"),
}

for key, vals in series.items():
    fv = final_valid(vals)
    summary["final_relative_yaw_deg"][key] = math.degrees(fv) if fv is not None else None

for key, getter in xy_sources.items():
    summary["xy_drift"][key] = xy_drift_stats(samples, getter)

for key in ("odom_msg", "odom_with_covariance", "odometry_filtered", "imu", "imu_with_covariance", "tf_odom_base", "tf_map_base"):
    summary["yaw_error_vs_gazebo"][key] = yaw_error_stats(series["gazebo"], series[key])

summary_path.write_text(json.dumps(summary, indent=2))

print("\n=== Turn Drift Diagnostic Summary ===")
print(f"samples={summary['samples_count']} duration_sec={summary['duration_sec']} turn_mode={turn_mode} commanded_turn_deg={turn_deg}")
print("Final relative yaw (deg):")
for key, val in summary["final_relative_yaw_deg"].items():
    print(f"- {key}: {val}")
print("XY drift during pure turn:")
for key, val in summary["xy_drift"].items():
    print(f"- {key}: {val}")
print("Yaw error vs Gazebo:")
for key, val in summary["yaw_error_vs_gazebo"].items():
    print(f"- {key}: {val}")
print("Mean covariance:")
for key, val in summary["covariance_means"].items():
    print(f"- {key}: {val}")
print(f"\nDetailed report: {summary_path}")
PY

stop_sim
run_cleanup_cycle "post"
cleanup_done="true"
