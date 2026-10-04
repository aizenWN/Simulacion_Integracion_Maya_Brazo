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

TRIALS="${TRIALS:-5}"
WORLD="${WORLD:-random_world.sdf}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre2}"
RENDER_ENGINE_GUI="${RENDER_ENGINE_GUI:-ogre2}"
WARMUP_FORWARD_DISTANCE_M="${WARMUP_FORWARD_DISTANCE_M:-10.0}"
WARMUP_CMD_VEL_X="${WARMUP_CMD_VEL_X:-0.35}"
WARMUP_MIN_PROGRESS_M="${WARMUP_MIN_PROGRESS_M:-0.5}"
WARMUP_NO_PROGRESS_TIMEOUT_SEC="${WARMUP_NO_PROGRESS_TIMEOUT_SEC:-12}"
GOAL_TIMEOUT_SEC="${GOAL_TIMEOUT_SEC:-120}"
STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-120}"
WARMUP_TIMEOUT_SEC="${WARMUP_TIMEOUT_SEC:-60}"
WARMUP_ACCEPT_MIN_DISTANCE_M="${WARMUP_ACCEPT_MIN_DISTANCE_M:-2.0}"
POST_WARMUP_SETTLE_SEC="${POST_WARMUP_SETTLE_SEC:-1}"
POST_WARMUP_TURN_DEG="${POST_WARMUP_TURN_DEG:-0}"
POST_WARMUP_TURN_MODE="${POST_WARMUP_TURN_MODE:-spin}"
POST_WARMUP_TURN_CMD_VEL_Z="${POST_WARMUP_TURN_CMD_VEL_Z:-0.6}"
POST_WARMUP_TURN_TIMEOUT_SEC="${POST_WARMUP_TURN_TIMEOUT_SEC:-25}"
POST_WARMUP_TURN_STAGE_TIMEOUT_SEC="${POST_WARMUP_TURN_STAGE_TIMEOUT_SEC:-60}"
POST_WARMUP_TURN_SETTLE_SEC="${POST_WARMUP_TURN_SETTLE_SEC:-1}"
POST_WARMUP_TURN_TOL_DEG="${POST_WARMUP_TURN_TOL_DEG:-10}"
DUAL_PHASE_TURN_STRESS_TEST="${DUAL_PHASE_TURN_STRESS_TEST:-false}"
PHASE_A_POST_WARMUP_TURN_DEG="${PHASE_A_POST_WARMUP_TURN_DEG:-0}"
PHASE_B_POST_WARMUP_TURN_DEG="${PHASE_B_POST_WARMUP_TURN_DEG:-90}"
PHASE_B_TURN_SWEEP_ENABLE="${PHASE_B_TURN_SWEEP_ENABLE:-false}"
PHASE_B_TURN_SWEEP_START_NUM="${PHASE_B_TURN_SWEEP_START_NUM:-1}"
PHASE_B_TURN_SWEEP_END_NUM="${PHASE_B_TURN_SWEEP_END_NUM:-32}"
PHASE_B_TURN_SWEEP_DEN="${PHASE_B_TURN_SWEEP_DEN:-32}"
PHASE_B_TURN_SWEEP_REPEATS="${PHASE_B_TURN_SWEEP_REPEATS:-3}"
PHASE_B_TURN_SWEEP_SIGN="${PHASE_B_TURN_SWEEP_SIGN:-1}"
START_SANITY_GUARD="${START_SANITY_GUARD:-true}"
REQUIRE_MAP_START_TF="${REQUIRE_MAP_START_TF:-true}"
START_SANITY_MAX_ABS_Z_M="${START_SANITY_MAX_ABS_Z_M:-0.15}"
START_SANITY_MAX_ABS_ROLL_DEG="${START_SANITY_MAX_ABS_ROLL_DEG:-20.0}"
START_SANITY_MAX_ABS_PITCH_DEG="${START_SANITY_MAX_ABS_PITCH_DEG:-20.0}"
PRECHECK_COMPUTE_PATH="${PRECHECK_COMPUTE_PATH:-true}"
PRECHECK_TIMEOUT_SEC="${PRECHECK_TIMEOUT_SEC:-20}"
COLLISION_STATE_CAPTURE_SEC="${COLLISION_STATE_CAPTURE_SEC:-90}"
RVIZ="${RVIZ:-false}"
CLEANUP_DEBUG="${CLEANUP_DEBUG:-false}"
STRICT_CLEANUP="${STRICT_CLEANUP:-false}"

CLEANUP_GRACEFUL_PATTERN="ros2 launch maya_bringup|gz sim|ros_gz_sim|rviz2|slam_toolbox|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|lifecycle_manager|map_server|amcl|waypoint_follower|collision_monitor|ekf_node|robot_state_publisher|component_container"
CLEANUP_FORCE_PATTERN="$CLEANUP_GRACEFUL_PATTERN"

tmpdir="$(mktemp -d)"
diag_root="$tmpdir/nav2_reliability"
mkdir -p "$diag_root"

if [[ "$PHASE_B_TURN_SWEEP_ENABLE" == "true" ]]; then
  levels=$((PHASE_B_TURN_SWEEP_END_NUM - PHASE_B_TURN_SWEEP_START_NUM + 1))
  required_trials_for_sweep=$((levels * PHASE_B_TURN_SWEEP_REPEATS))
  if (( levels <= 0 || PHASE_B_TURN_SWEEP_REPEATS <= 0 || PHASE_B_TURN_SWEEP_DEN <= 0 )); then
    echo "Invalid PHASE_B_TURN_SWEEP_* configuration." >&2
    exit 1
  fi
  if [[ "$DUAL_PHASE_TURN_STRESS_TEST" != "true" ]]; then
    echo "PHASE_B_TURN_SWEEP_ENABLE=true requires DUAL_PHASE_TURN_STRESS_TEST=true." >&2
    exit 1
  fi
  if (( TRIALS > required_trials_for_sweep )); then
    echo "TRIALS=$TRIALS exceeds configured phase-B sweep capacity ($required_trials_for_sweep)." >&2
    exit 1
  fi
fi

cleanup_global() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT -- "-$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
  echo "Diagnostics root: $diag_root"
  echo "Summary report: $diag_root/summary.json"
}
trap cleanup_global EXIT

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

wait_for_active_nav2() {
  local timeout_s="$1"
  local end=$((SECONDS + timeout_s))
  while (( SECONDS < end )); do
    local out
    out="$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
    if printf '%s' "$out" | grep -qi "active"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

lookup_tf_xyyaw() {
  local target_frame="$1"
  local source_frame="$2"
  local timeout_s="$3"
  python3 - <<'PY' "$target_frame" "$source_frame" "$timeout_s"
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

target = sys.argv[1]
source = sys.argv[2]
timeout_s = float(sys.argv[3])

rclpy.init()
node = rclpy.create_node('tf_lookup_once')
buf = Buffer()
listener = TransformListener(buf, node, spin_thread=False)
deadline = time.time() + timeout_s
ok = False

try:
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = buf.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.2))
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            print(f"{tx} {ty} {yaw} {qx} {qy} {qz} {qw}")
            ok = True
            break
        except TransformException:
            continue
finally:
    node.destroy_node()
    rclpy.shutdown()

if not ok:
    raise SystemExit(1)
PY
}

read_pose_component() {
  local topic="$1"
  local outfile="$2"
  timeout 6s ros2 topic echo --once "$topic" >"$outfile" 2>&1 || true
}

read_map_metadata() {
  local outfile="$1"
  timeout 6s ros2 topic echo --once /map_metadata >"$outfile" 2>&1 || true
}

lookup_gz_world_topic() {
  gz topic -l | awk '/\/world\/.*\/dynamic_pose\/info/ {print $1; exit}'
}

get_maya_pose() {
  local world_topic="$1"
  local snapshot=""

  # Guard against rare Gazebo CLI stalls: never block indefinitely while
  # sampling one dynamic pose message.
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
  local tries=6
  local out=""
  for _ in $(seq 1 "$tries"); do
    out="$(get_maya_pose "$world_topic" || true)"
    if [[ -n "$out" && "$out" != "      " ]]; then
      printf '%s\n' "$out"
      return 0
    fi
    sleep 0.3
  done
  return 1
}

compute_xy_distance() {
  local x0="$1"
  local y0="$2"
  local x1="$3"
  local y1="$4"
  python3 - <<'PY' "$x0" "$y0" "$x1" "$y1"
import math
import sys
x0 = float(sys.argv[1]); y0 = float(sys.argv[2]); x1 = float(sys.argv[3]); y1 = float(sys.argv[4])
print(math.hypot(x1 - x0, y1 - y0))
PY
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

parse_nav_action_result_field() {
  local path="$1"
  local pattern="$2"
  grep -m1 -E "$pattern" "$path" | awk -F': ' '{print $2}' || true
}

compute_phase_b_turn_deg() {
  local trial="$1"
  if [[ "$PHASE_B_TURN_SWEEP_ENABLE" != "true" ]]; then
    printf '%s\n' "$PHASE_B_POST_WARMUP_TURN_DEG"
    return 0
  fi

  local levels=$((PHASE_B_TURN_SWEEP_END_NUM - PHASE_B_TURN_SWEEP_START_NUM + 1))
  if (( levels <= 0 || PHASE_B_TURN_SWEEP_REPEATS <= 0 || PHASE_B_TURN_SWEEP_DEN <= 0 )); then
    echo "Invalid PHASE_B_TURN_SWEEP_* configuration." >&2
    return 1
  fi

  local required_trials=$((levels * PHASE_B_TURN_SWEEP_REPEATS))
  if (( trial > required_trials )); then
    echo "Trial $trial exceeds configured phase-B sweep capacity ($required_trials)." >&2
    return 1
  fi

  local idx=$((trial - 1))
  local level_idx=$((idx / PHASE_B_TURN_SWEEP_REPEATS))
  local turn_num=$((PHASE_B_TURN_SWEEP_START_NUM + level_idx))
  python3 - <<'PY' "$turn_num" "$PHASE_B_TURN_SWEEP_DEN" "$PHASE_B_TURN_SWEEP_SIGN"
import sys
num = float(sys.argv[1])
den = float(sys.argv[2])
sign = float(sys.argv[3])
print(sign * num * 180.0 / den)
PY
}

sanity_check_gz_pose_start() {
  local pose_file="$1"
  python3 - <<'PY' "$pose_file" "$START_SANITY_MAX_ABS_Z_M" "$START_SANITY_MAX_ABS_ROLL_DEG" "$START_SANITY_MAX_ABS_PITCH_DEG"
import math
import sys
from pathlib import Path

pose_path = Path(sys.argv[1])
max_abs_z = float(sys.argv[2])
max_abs_roll_deg = float(sys.argv[3])
max_abs_pitch_deg = float(sys.argv[4])

txt = pose_path.read_text().strip().split()
if len(txt) < 7:
    print("invalid pose sample", file=sys.stderr)
    raise SystemExit(1)

x, y, z, qx, qy, qz, qw = map(float, txt[:7])

# quaternion -> roll, pitch, yaw
sinr_cosp = 2.0 * (qw * qx + qy * qz)
cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
roll = math.atan2(sinr_cosp, cosr_cosp)

sinp = 2.0 * (qw * qy - qz * qx)
if abs(sinp) >= 1:
    pitch = math.copysign(math.pi / 2.0, sinp)
else:
    pitch = math.asin(sinp)

roll_deg = math.degrees(roll)
pitch_deg = math.degrees(pitch)

if abs(z) > max_abs_z:
    print(f"start pose z out of bounds: z={z:.3f}, limit={max_abs_z}", file=sys.stderr)
    raise SystemExit(1)
if abs(roll_deg) > max_abs_roll_deg:
    print(f"start pose roll out of bounds: roll={roll_deg:.2f} deg, limit={max_abs_roll_deg}", file=sys.stderr)
    raise SystemExit(1)
if abs(pitch_deg) > max_abs_pitch_deg:
    print(f"start pose pitch out of bounds: pitch={pitch_deg:.2f} deg, limit={max_abs_pitch_deg}", file=sys.stderr)
    raise SystemExit(1)
PY
}

start_sim() {
  local trial_dir="$1"
  local ros_log_dir="$trial_dir/roslog"
  mkdir -p "$ros_log_dir"
  export ROS_LOG_DIR="$ros_log_dir"

  local -a cmd=(ros2 launch maya_bringup maya.launch.xml "rviz:=${RVIZ}" "gz_args:=${WORLD} -s --render-engine ${RENDER_ENGINE} --render-engine-gui ${RENDER_ENGINE_GUI} -r")
  setsid "${cmd[@]}" >"$trial_dir/launch.log" 2>&1 &
  LAUNCH_PID=$!
}

stop_sim() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT -- "-$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
    unset LAUNCH_PID
  fi
}

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
    [[ "${cmdline:-}" == *"tools/nav2_reliability_trials.sh"* ]] && continue
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
    [[ "${cmdline:-}" == *"tools/nav2_reliability_trials.sh"* ]] && continue
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
    return 0
  fi
  cleanup_debug_log "Cleanup [$stage]: no stale processes detected."
  return 0
}

run_cleanup_cycle() {
  local stage="$1"
  echo "Cleanup [$stage]: stopping stale ROS/Gazebo processes and resetting ROS 2 daemon."

  # 1) Stop known Maya bringup processes gracefully.
  kill_matching_processes "INT" "$CLEANUP_GRACEFUL_PATTERN" "$stage"
  sleep 3

  # 2) Escalate if anything is still alive.
  kill_matching_processes "TERM" "$CLEANUP_FORCE_PATTERN" "$stage"
  sleep 2
  kill_matching_processes "KILL" "$CLEANUP_FORCE_PATTERN" "$stage"

  # 3) Reset ROS 2 daemon to avoid stale graph state.
  ros2 daemon stop >/dev/null 2>&1 || true
  pkill -f "_ros2_daemon" >/dev/null 2>&1 || true
  sleep 1
  ros2 daemon start >/dev/null 2>&1 || true

  verify_clean_state "$stage"
}

capture_tf_pose() {
  local target="$1"
  local source="$2"
  local outfile="$3"
  if out="$(lookup_tf_xyyaw "$target" "$source" 20 2>/dev/null || true)"; then
    if [[ -n "${out:-}" ]]; then
      printf '%s\n' "$out" >"$outfile"
      return 0
    fi
  fi
  return 1
}

run_nav_phase() {
  local trial="$1"
  local world_topic="$2"
  local phase_dir="$3"
  local phase_label="$4"
  local phase_turn_deg="$5"

  mkdir -p "$phase_dir"

  # Capture initial pose before warmup and then drive forward ~10m (manual workflow mimic).
  read_map_metadata "$phase_dir/map_metadata_before.txt"
  read_pose_component /clock "$phase_dir/clock_pre_warmup.txt"
  read_pose_component /odom "$phase_dir/odom_pre_warmup.txt"
  read_pose_component /odometry/filtered "$phase_dir/odom_filtered_pre_warmup.txt"
  capture_tf_pose odom base_footprint "$phase_dir/tf_odom_base_pre_warmup.txt" || true
  get_maya_pose_retry "$world_topic" >"$phase_dir/gz_pose_pre_warmup.txt" || true

  echo "Trial $trial [$phase_label]: warming SLAM map + driving forward ${WARMUP_FORWARD_DISTANCE_M}m..."
  if [[ ! -s "$phase_dir/gz_pose_pre_warmup.txt" ]]; then
    echo "Trial $trial [$phase_label]: missing initial Gazebo pose" >&2
    return 1
  fi
  read -r gzsx gzsy _ <"$phase_dir/gz_pose_pre_warmup.txt"

  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${WARMUP_CMD_VEL_X}}, angular: {z: 0.0}}" >/dev/null 2>&1 &
  WARMUP_PUB_PID=$!
  warmup_start_sec=$SECONDS
  warmup_last_progress_sec=$SECONDS
  warmup_last_progress_print_sec=$SECONDS
  warmup_best_dist="0.0"
  warmup_target_reached=0
  spawn_ref_captured=0
  : >"$phase_dir/warmup_progress.log"
  while :; do
    # Capture first available map pose as the return target (near spawn after SLAM initializes).
    if [[ "$spawn_ref_captured" -eq 0 ]]; then
      if capture_tf_pose map base_footprint "$phase_dir/tf_map_base_spawn_ref.txt"; then
        spawn_ref_captured=1
        echo "Trial $trial [$phase_label]: captured map-frame return reference pose." | tee -a "$phase_dir/warmup_progress.log" >/dev/null
      fi
    fi

    if get_maya_pose_retry "$world_topic" >"$phase_dir/gz_pose_warmup_current.txt"; then
      read -r cgx cgy _ <"$phase_dir/gz_pose_warmup_current.txt"
      warmup_dist="$(compute_xy_distance "$gzsx" "$gzsy" "$cgx" "$cgy")"
      printf 'seconds=%s gz_distance_m=%s\n' "$((SECONDS - warmup_start_sec))" "$warmup_dist" >>"$phase_dir/warmup_progress.log"
      warmup_improved="$(python3 - <<'PY' "$warmup_dist" "$warmup_best_dist"
import sys
print("1" if float(sys.argv[1]) > float(sys.argv[2]) + 1e-3 else "0")
PY
)"
      if [[ "$warmup_improved" == "1" ]]; then
        warmup_best_dist="$warmup_dist"
        warmup_last_progress_sec=$SECONDS
      fi
      if (( SECONDS - warmup_last_progress_print_sec >= 2 )); then
        echo "Trial $trial [$phase_label]: warmup progress ${warmup_dist}m / ${WARMUP_FORWARD_DISTANCE_M}m (best=${warmup_best_dist}m, t=$((SECONDS - warmup_start_sec))s)"
        warmup_last_progress_print_sec=$SECONDS
      fi
      warmup_reached="$(python3 - <<'PY' "$warmup_dist" "$WARMUP_FORWARD_DISTANCE_M"
import sys
print("1" if float(sys.argv[1]) >= float(sys.argv[2]) else "0")
PY
)"
      if [[ "$warmup_reached" == "1" ]]; then
        warmup_target_reached=1
        break
      fi
    fi

    if (( SECONDS - warmup_start_sec >= WARMUP_NO_PROGRESS_TIMEOUT_SEC )); then
      enough_progress="$(python3 - <<'PY' "$warmup_best_dist" "$WARMUP_MIN_PROGRESS_M"
import sys
print("1" if float(sys.argv[1]) >= float(sys.argv[2]) else "0")
PY
)"
      if [[ "$enough_progress" != "1" ]]; then
        echo "Trial $trial [$phase_label]: warmup fail-fast (only ${warmup_best_dist}m progress after ${WARMUP_NO_PROGRESS_TIMEOUT_SEC}s)." | tee -a "$phase_dir/warmup_progress.log" >/dev/null
        break
      fi
    fi

    if (( SECONDS - warmup_start_sec >= WARMUP_TIMEOUT_SEC )); then
      echo "Trial $trial [$phase_label]: warmup timed out before reaching target distance." | tee -a "$phase_dir/warmup_progress.log" >/dev/null
      break
    fi
    sleep 0.5
  done

  kill -INT "$WARMUP_PUB_PID" >/dev/null 2>&1 || true
  wait "$WARMUP_PUB_PID" >/dev/null 2>&1 || true
  timeout 3s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  sleep "$POST_WARMUP_SETTLE_SEC"
  read_map_metadata "$phase_dir/map_metadata_after_warmup.txt"

  warmup_accept_ok="$(python3 - <<'PY' "$warmup_best_dist" "$WARMUP_ACCEPT_MIN_DISTANCE_M"
import sys
print("1" if float(sys.argv[1]) >= float(sys.argv[2]) else "0")
PY
)"
  if [[ "$warmup_accept_ok" != "1" ]]; then
    echo "Trial $trial [$phase_label]: warmup rejected (best=${warmup_best_dist}m < min=${WARMUP_ACCEPT_MIN_DISTANCE_M}m)." >&2
    return 1
  fi

  echo "Trial $trial [$phase_label]: warmup done (best Gazebo progress=${warmup_best_dist}m)."

  # Optional heading stress test before capturing return-leg start pose.
  if [[ "${phase_turn_deg}" != "0" && "${phase_turn_deg}" != "0.0" ]]; then
    turn_stage_start_sec=$SECONDS
    target_turn_rad="$(python3 - <<'PY' "$phase_turn_deg"
import math
import sys
print(abs(float(sys.argv[1])) * math.pi / 180.0)
PY
)"
    target_turn_signed_rad="$(python3 - <<'PY' "$phase_turn_deg"
import math
import sys
print(float(sys.argv[1]) * math.pi / 180.0)
PY
)"
    target_turn_tol_rad="$(python3 - <<'PY' "$POST_WARMUP_TURN_TOL_DEG"
import math
import sys
print(abs(float(sys.argv[1])) * math.pi / 180.0)
PY
)"
    turn_sign="$(python3 - <<'PY' "$phase_turn_deg"
import sys
print(1.0 if float(sys.argv[1]) >= 0.0 else -1.0)
PY
)"
    turn_cmd_z="$(python3 - <<'PY' "$POST_WARMUP_TURN_CMD_VEL_Z" "$turn_sign"
import sys
print(abs(float(sys.argv[1])) * float(sys.argv[2]))
PY
)"

    read_pose_component /clock "$phase_dir/clock_pre_turn.txt"
    read_pose_component /odom "$phase_dir/odom_pre_turn.txt"
    read_pose_component /odometry/filtered "$phase_dir/odom_filtered_pre_turn.txt"
    read_pose_component /imu_with_covariance "$phase_dir/imu_with_covariance_pre_turn.txt"
    capture_tf_pose map base_footprint "$phase_dir/tf_map_base_pre_turn.txt" || true
    capture_tf_pose odom base_footprint "$phase_dir/tf_odom_base_pre_turn.txt" || true

    if get_maya_pose_retry "$world_topic" >"$phase_dir/gz_pose_pre_turn.txt"; then
      start_yaw="$(python3 - <<'PY' "$phase_dir/gz_pose_pre_turn.txt"
import math
import sys
from pathlib import Path
txt = Path(sys.argv[1]).read_text().strip().split()
if len(txt) < 7:
    raise SystemExit(1)
_, _, _, qx, qy, qz, qw = map(float, txt[:7])
siny_cosp = 2.0 * (qw * qz + qx * qy)
cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
print(math.atan2(siny_cosp, cosy_cosp))
PY
)" || start_yaw=""
    else
      start_yaw=""
    fi

    if [[ -n "${start_yaw:-}" ]]; then
      echo "Trial $trial [$phase_label]: post-warmup rotate ${phase_turn_deg} deg via ${POST_WARMUP_TURN_MODE} before return start sample."
      turn_start_sec=$SECONDS
      turn_done=0

      if [[ "$POST_WARMUP_TURN_MODE" == "spin" ]]; then
        spin_goal_yaml="{target_yaw: ${target_turn_signed_rad}, time_allowance: {sec: ${POST_WARMUP_TURN_STAGE_TIMEOUT_SEC}, nanosec: 0}}"
        printf '%s\n' "$spin_goal_yaml" >"$phase_dir/spin_goal.yaml"
        timeout "${POST_WARMUP_TURN_STAGE_TIMEOUT_SEC}s" ros2 action send_goal /spin nav2_msgs/action/Spin "$spin_goal_yaml" --feedback >"$phase_dir/spin_action.txt" 2>&1
        spin_cmd_rc=$?
        spin_status="$(parse_nav_action_result_field "$phase_dir/spin_action.txt" "Goal finished with status:")"
        spin_error_code="$(parse_nav_action_result_field "$phase_dir/spin_action.txt" "^[[:space:]]*error_code:")"
        if [[ "$spin_cmd_rc" -eq 124 ]]; then
          echo "Trial $trial [$phase_label]: spin action shell timeout after ${POST_WARMUP_TURN_STAGE_TIMEOUT_SEC}s (see $phase_dir/spin_action.txt)." >&2
          printf 'timeout\n' >"$phase_dir/spin_action_timeout.flag"
        elif [[ "${spin_status:-}" == "SUCCEEDED" && "${spin_error_code:-}" == "0" ]]; then
          turn_done=1
        else
          echo "Trial $trial [$phase_label]: spin action non-success (rc=${spin_cmd_rc}, status=${spin_status:-unknown}, error_code=${spin_error_code:-unknown}). See $phase_dir/spin_action.txt" >&2
        fi
      else
        ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: ${turn_cmd_z}}}" >/dev/null 2>&1 &
        TURN_PUB_PID=$!
        while :; do
          if (( SECONDS - turn_stage_start_sec >= POST_WARMUP_TURN_STAGE_TIMEOUT_SEC )); then
            echo "Trial $trial [$phase_label]: post-warmup turn stage watchdog timeout (${POST_WARMUP_TURN_STAGE_TIMEOUT_SEC}s)." >&2
            break
          fi
          if get_maya_pose_retry "$world_topic" >"$phase_dir/gz_pose_turn_current.txt"; then
            current_yaw="$(python3 - <<'PY' "$phase_dir/gz_pose_turn_current.txt"
import math
import sys
from pathlib import Path
txt = Path(sys.argv[1]).read_text().strip().split()
if len(txt) < 7:
    raise SystemExit(1)
_, _, _, qx, qy, qz, qw = map(float, txt[:7])
siny_cosp = 2.0 * (qw * qz + qx * qy)
cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
print(math.atan2(siny_cosp, cosy_cosp))
PY
)" || current_yaw=""
            if [[ -n "${current_yaw:-}" ]]; then
              turn_delta="$(normalize_angle_diff "$start_yaw" "$current_yaw")"
              turn_abs="$(python3 - <<'PY' "$turn_delta"
import sys
print(abs(float(sys.argv[1])))
PY
)"
              turn_reached="$(python3 - <<'PY' "$turn_abs" "$target_turn_rad" "$target_turn_tol_rad"
import sys
turn_abs = float(sys.argv[1]); target = float(sys.argv[2]); tol = float(sys.argv[3])
threshold = max(0.0, target - tol)
print("1" if turn_abs >= threshold else "0")
PY
)"
              if [[ "$turn_reached" == "1" ]]; then
                turn_done=1
                break
              fi
            fi
          fi

          if (( SECONDS - turn_start_sec >= POST_WARMUP_TURN_TIMEOUT_SEC )); then
            echo "Trial $trial [$phase_label]: post-warmup turn timed out before reaching target yaw." >&2
            break
          fi
          sleep 0.2
        done

        kill -INT "$TURN_PUB_PID" >/dev/null 2>&1 || true
        timeout 2s bash -lc 'wait "$1" >/dev/null 2>&1 || true' _ "$TURN_PUB_PID" || true
        timeout 3s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
      fi

      sleep "$POST_WARMUP_TURN_SETTLE_SEC"
      read_pose_component /clock "$phase_dir/clock_post_turn.txt"
      read_pose_component /odom "$phase_dir/odom_post_turn.txt"
      read_pose_component /odometry/filtered "$phase_dir/odom_filtered_post_turn.txt"
      read_pose_component /imu_with_covariance "$phase_dir/imu_with_covariance_post_turn.txt"
      capture_tf_pose map base_footprint "$phase_dir/tf_map_base_post_turn.txt" || true
      capture_tf_pose odom base_footprint "$phase_dir/tf_odom_base_post_turn.txt" || true
      get_maya_pose_retry "$world_topic" >"$phase_dir/gz_pose_post_turn.txt" || true
      if [[ "$turn_done" -eq 1 ]]; then
        echo "Trial $trial [$phase_label]: post-warmup turn complete."
      fi
    else
      echo "Trial $trial [$phase_label]: skipped post-warmup turn (could not read initial Gazebo yaw)." >&2
    fi
  fi

  # Start pose for the navigation phase is after warmup/optional turn.
  read_pose_component /clock "$phase_dir/clock_start.txt"
  read_pose_component /odom "$phase_dir/odom_start.txt"
  read_pose_component /odometry/filtered "$phase_dir/odom_filtered_start.txt"
  capture_tf_pose map base_footprint "$phase_dir/tf_map_base_start.txt" || true
  capture_tf_pose odom base_footprint "$phase_dir/tf_odom_base_start.txt" || true
  get_maya_pose_retry "$world_topic" >"$phase_dir/gz_pose_start.txt" || true

  if [[ "$REQUIRE_MAP_START_TF" == "true" && ! -s "$phase_dir/tf_map_base_start.txt" ]]; then
    echo "Trial $trial [$phase_label]: missing map->base_footprint start TF sample." >&2
    return 1
  fi
  if [[ "$START_SANITY_GUARD" == "true" ]]; then
    if [[ ! -s "$phase_dir/gz_pose_start.txt" ]]; then
      echo "Trial $trial [$phase_label]: missing Gazebo start pose for sanity check." >&2
      return 1
    fi
    if ! sanity_check_gz_pose_start "$phase_dir/gz_pose_start.txt"; then
      echo "Trial $trial [$phase_label]: rejected by start sanity guard." >&2
      return 1
    fi
  fi

  if [[ ! -s "$phase_dir/tf_map_base_spawn_ref.txt" ]]; then
    echo "Trial $trial [$phase_label]: could not capture a map-frame return reference during warmup" >&2
    return 1
  fi
  read -r gx gy gyaw gqx gqy gqz gqw <"$phase_dir/tf_map_base_spawn_ref.txt"
  printf '%s\n' "{\"pose\": {\"header\": {\"frame_id\": \"map\"}, \"pose\": {\"position\": {\"x\": ${gx}, \"y\": ${gy}, \"z\": 0.0}, \"orientation\": {\"x\": ${gqx:-0.0}, \"y\": ${gqy:-0.0}, \"z\": ${gqz}, \"w\": ${gqw}}}}}" >"$phase_dir/goal.json"

  collision_type="$(ros2 topic type /collision_monitor_state 2>/dev/null || true)"
  printf '%s\n' "${collision_type:-}" >"$phase_dir/collision_monitor_state_type.txt"
  if [[ -n "${collision_type:-}" ]]; then
    timeout "${COLLISION_STATE_CAPTURE_SEC}s" ros2 topic echo /collision_monitor_state >"$phase_dir/collision_monitor_state.txt" 2>&1 &
    COLLISION_ECHO_PID=$!
  else
    COLLISION_ECHO_PID=""
  fi

  goal_yaml="{pose: {header: {frame_id: map}, pose: {position: {x: ${gx}, y: ${gy}, z: 0.0}, orientation: {x: ${gqx:-0.0}, y: ${gqy:-0.0}, z: ${gqz}, w: ${gqw}}}}}"
  compute_goal_yaml="{goal: {header: {frame_id: map}, pose: {position: {x: ${gx}, y: ${gy}, z: 0.0}, orientation: {x: ${gqx:-0.0}, y: ${gqy:-0.0}, z: ${gqz}, w: ${gqw}}}}, planner_id: '', use_start: false}"
  if [[ "$PRECHECK_COMPUTE_PATH" == "true" ]]; then
    if timeout "${PRECHECK_TIMEOUT_SEC}s" ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "$compute_goal_yaml" >"$phase_dir/compute_path_precheck.txt" 2>&1; then
      true
    else
      echo "Trial $trial [$phase_label]: compute_path precheck command timeout/failure." >&2
      return 1
    fi
    precheck_status="$(grep -m1 "Goal finished with status:" "$phase_dir/compute_path_precheck.txt" | awk -F': ' '{print $2}' || true)"
    precheck_code="$(grep -m1 -E '^[[:space:]]*error_code:' "$phase_dir/compute_path_precheck.txt" | awk '{print $2}' || true)"
    if [[ "${precheck_status:-}" != "SUCCEEDED" || "${precheck_code:-}" != "0" ]]; then
      echo "Trial $trial [$phase_label]: compute_path precheck failed (status=${precheck_status:-unknown}, error_code=${precheck_code:-unknown})." >&2
      return 1
    fi
  fi

  echo "Trial $trial [$phase_label]: sending NavigateToPose goal (return to recorded start pose)."
  if timeout "${GOAL_TIMEOUT_SEC}s" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$goal_yaml" >"$phase_dir/nav_goal.txt" 2>&1; then
    true
  else
    echo "Trial $trial [$phase_label]: nav goal command timeout/failure (see nav_goal.txt)" >&2
    printf 'timeout_or_failure\n' >"$phase_dir/nav_goal_timeout.flag"
  fi

  if [[ -n "${COLLISION_ECHO_PID:-}" ]]; then
    wait "$COLLISION_ECHO_PID" >/dev/null 2>&1 || true
  fi

  read_pose_component /clock "$phase_dir/clock_end.txt"
  read_pose_component /odom "$phase_dir/odom_end.txt"
  read_pose_component /odometry/filtered "$phase_dir/odom_filtered_end.txt"
  capture_tf_pose map base_footprint "$phase_dir/tf_map_base_end.txt" || true
  capture_tf_pose odom base_footprint "$phase_dir/tf_odom_base_end.txt" || true
  get_maya_pose_retry "$world_topic" >"$phase_dir/gz_pose_end.txt" || true
  read_map_metadata "$phase_dir/map_metadata_end.txt"

  timeout 6s ros2 topic info /cmd_vel --verbose >"$phase_dir/cmd_vel_info.txt" 2>&1 || true
  timeout 6s ros2 topic info /collision_monitor_state --verbose >"$phase_dir/collision_monitor_state_info.txt" 2>&1 || true
  printf '%s\n' "$phase_turn_deg" >"$phase_dir/post_warmup_turn_deg_used.txt"
  printf '%s\n' "$POST_WARMUP_TURN_MODE" >"$phase_dir/post_warmup_turn_mode_used.txt"

  echo "Trial $trial [$phase_label]: phase complete. Diagnostics: $phase_dir"
  return 0
}

for trial in $(seq 1 "$TRIALS"); do
  trial_dir="$diag_root/trial_${trial}"
  mkdir -p "$trial_dir"
  echo "=== Trial $trial/$TRIALS ==="

  run_cleanup_cycle "trial_${trial}_pre"
  if ! verify_clean_state "trial_${trial}_pre_verify"; then
    echo "Trial $trial: stale process state before startup; skipping trial." >&2
    continue
  fi
  start_sim "$trial_dir"

  wait_for_topic /clock "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: timed out waiting /clock" >&2; stop_sim; run_cleanup_cycle "trial_${trial}_startup_fail"; continue; }
  wait_for_topic /scan "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: timed out waiting /scan" >&2; stop_sim; run_cleanup_cycle "trial_${trial}_startup_fail"; continue; }
  wait_for_topic /odom "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: timed out waiting /odom" >&2; stop_sim; run_cleanup_cycle "trial_${trial}_startup_fail"; continue; }
  wait_for_topic /odometry/filtered "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: timed out waiting /odometry/filtered" >&2; stop_sim; run_cleanup_cycle "trial_${trial}_startup_fail"; continue; }
  wait_for_topic /map "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: timed out waiting /map" >&2; stop_sim; run_cleanup_cycle "trial_${trial}_startup_fail"; continue; }
  wait_for_active_nav2 "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: Nav2 bt_navigator not active" >&2; stop_sim; run_cleanup_cycle "trial_${trial}_startup_fail"; continue; }

  world_topic="$(lookup_gz_world_topic || true)"
  if [[ -z "$world_topic" ]]; then
    echo "Trial $trial: could not find Gazebo dynamic pose topic" >&2
    stop_sim
    run_cleanup_cycle "trial_${trial}_startup_fail"
    continue
  fi
  printf '%s\n' "$world_topic" >"$trial_dir/gz_world_topic.txt"

  phase_failed=0
  if [[ "$DUAL_PHASE_TURN_STRESS_TEST" == "true" ]]; then
    phase_b_turn_this_trial="$(compute_phase_b_turn_deg "$trial")" || {
      echo "Trial $trial: failed to resolve phase-B turn schedule." >&2
      phase_failed=1
    }
    if [[ "$phase_failed" -eq 0 ]]; then
      printf '%s\n' "$phase_b_turn_this_trial" >"$trial_dir/phase_b_turn_deg_scheduled.txt"
    fi
    run_nav_phase "$trial" "$world_topic" "$trial_dir/phase_a" "phase_a_easy" "$PHASE_A_POST_WARMUP_TURN_DEG" || phase_failed=1
    if [[ "$phase_failed" -eq 0 ]]; then
      run_nav_phase "$trial" "$world_topic" "$trial_dir/phase_b" "phase_b_turn_stress" "$phase_b_turn_this_trial" || phase_failed=1
    fi
  else
    run_nav_phase "$trial" "$world_topic" "$trial_dir" "single" "$POST_WARMUP_TURN_DEG" || phase_failed=1
  fi

  if [[ "$phase_failed" -eq 1 ]]; then
    echo "Trial $trial: one or more phases failed (see trial diagnostics)." >&2
  fi
  echo "Trial $trial: complete, shutting down simulation."
  stop_sim
  run_cleanup_cycle "trial_${trial}_post"
  if ! verify_clean_state "trial_${trial}_post_verify"; then
    echo "Trial $trial: stale process state remains after post-trial cleanup." >&2
  fi
done

python3 - <<'PY' "$diag_root" "$TRIALS" "$WARMUP_FORWARD_DISTANCE_M"
import json
import math
import re
import statistics as stats
import sys
from pathlib import Path

diag_root = Path(sys.argv[1])
trials_expected = int(sys.argv[2])
warmup_forward_distance_m = float(sys.argv[3])


def read_text(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8", errors="ignore")


def parse_clock(path: Path):
    txt = read_text(path)
    ms = re.search(r"sec:\s*(\d+)", txt)
    mn = re.search(r"nanosec:\s*(\d+)", txt)
    if not ms or not mn:
        return None
    return int(ms.group(1)) + int(mn.group(1)) * 1e-9


def parse_header_stamp(path: Path):
    txt = read_text(path)
    m = re.search(r"header:\s*\n\s*stamp:\s*\n\s*sec:\s*(\d+)\s*\n\s*nanosec:\s*(\d+)", txt)
    if not m:
        return None
    return int(m.group(1)) + int(m.group(2)) * 1e-9


def parse_tf_pose(path: Path):
    txt = read_text(path).strip()
    if not txt:
        return None
    parts = txt.split()
    if len(parts) < 3:
        return None
    vals = [float(p) for p in parts]
    out = {"x": vals[0], "y": vals[1], "yaw": vals[2]}
    if len(vals) >= 7:
        out["qx"], out["qy"], out["qz"], out["qw"] = vals[3:7]
    return out


def parse_gz_pose(path: Path):
    txt = read_text(path).strip()
    if not txt:
        return None
    parts = txt.split()
    if len(parts) < 7:
        return None
    x, y, z, qx, qy, qz, qw = [float(p) for p in parts[:7]]
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return {"x": x, "y": y, "z": z, "yaw": yaw, "qx": qx, "qy": qy, "qz": qz, "qw": qw}


def parse_odom_xy(path: Path):
    txt = read_text(path)
    mx = re.search(r"position:\s*\n\s*x:\s*([\-0-9.eE]+)", txt)
    my = re.search(r"position:\s*\n\s*x:\s*[\-0-9.eE]+\s*\n\s*y:\s*([\-0-9.eE]+)", txt)
    if not mx or not my:
        return None
    return {"x": float(mx.group(1)), "y": float(my.group(1))}


def parse_goal_json(path: Path):
    txt = read_text(path)
    if not txt:
        return None
    try:
        data = json.loads(txt)
        pose = data["pose"]["pose"]
        return {
            "x": float(pose["position"]["x"]),
            "y": float(pose["position"]["y"]),
            "qz": float(pose["orientation"]["z"]),
            "qw": float(pose["orientation"]["w"]),
        }
    except Exception:
        return None


def parse_nav_goal_result(path: Path):
    txt = read_text(path)
    status_m = re.search(r"Goal finished with status:\s*([A-Z_]+)", txt)
    error_m = re.search(r"error_code:\s*([0-9]+)", txt)
    return {
        "status": status_m.group(1) if status_m else "UNKNOWN",
        "error_code": int(error_m.group(1)) if error_m else None,
        "raw_available": bool(txt.strip()),
    }


def parse_spin_action_result(path: Path, timeout_flag: Path):
    txt = read_text(path)
    status_m = re.search(r"Goal finished with status:\s*([A-Z_]+)", txt)
    error_m = re.search(r"error_code:\s*([0-9]+)", txt)
    feedback = [float(v) for v in re.findall(r"angular_distance_traveled:\s*([\-0-9.eE]+)", txt)]
    result = {
        "status": status_m.group(1) if status_m else "UNKNOWN",
        "error_code": int(error_m.group(1)) if error_m else None,
        "raw_available": bool(txt.strip()),
        "feedback_count": len(feedback),
        "feedback_last_angular_distance_traveled_rad": feedback[-1] if feedback else None,
    }
    if result["status"] == "UNKNOWN" and timeout_flag.exists():
        result["status"] = "TIMEOUT"
    return result


def parse_map_metadata(path: Path):
    txt = read_text(path)
    if not txt:
        return None
    w = re.search(r"^width:\s*(\d+)", txt, flags=re.M)
    h = re.search(r"^height:\s*(\d+)", txt, flags=re.M)
    res = re.search(r"^resolution:\s*([0-9.eE+-]+)", txt, flags=re.M)
    return {
        "width": int(w.group(1)) if w else None,
        "height": int(h.group(1)) if h else None,
        "resolution": float(res.group(1)) if res else None,
    }


def parse_collision_states(path: Path):
    txt = read_text(path)
    if not txt.strip():
        return {"message_count": 0, "unique_message_count": 0, "changed": None}
    blocks = [b.strip() for b in txt.split("---") if b.strip()]
    if not blocks:
        return {"message_count": 0, "unique_message_count": 0, "changed": None}
    uniq = len(set(blocks))
    return {"message_count": len(blocks), "unique_message_count": uniq, "changed": uniq > 1}


def dxy(a, b):
    return {"dx": b["x"] - a["x"], "dy": b["y"] - a["y"]}


def mag(v):
    return math.hypot(v["dx"], v["dy"])


def dist_xy(a, b):
    return math.hypot(b["x"] - a["x"], b["y"] - a["y"])


report = {
    "trials_expected": trials_expected,
    "warmup_forward_distance_commanded_m": warmup_forward_distance_m,
    "trials": [],
    "aggregate": {},
}


def parse_phase_dir(phase_dir: Path, phase_name: str):
    start_map = parse_tf_pose(phase_dir / "tf_map_base_start.txt")
    end_map = parse_tf_pose(phase_dir / "tf_map_base_end.txt")
    start_odom_tf = parse_tf_pose(phase_dir / "tf_odom_base_start.txt")
    end_odom_tf = parse_tf_pose(phase_dir / "tf_odom_base_end.txt")
    start_gz = parse_gz_pose(phase_dir / "gz_pose_start.txt")
    end_gz = parse_gz_pose(phase_dir / "gz_pose_end.txt")
    odom_start = parse_odom_xy(phase_dir / "odom_start.txt")
    odom_end = parse_odom_xy(phase_dir / "odom_end.txt")
    filt_start = parse_odom_xy(phase_dir / "odom_filtered_start.txt")
    filt_end = parse_odom_xy(phase_dir / "odom_filtered_end.txt")
    goal = parse_goal_json(phase_dir / "goal.json")
    nav = parse_nav_goal_result(phase_dir / "nav_goal.txt")
    spin = parse_spin_action_result(phase_dir / "spin_action.txt", phase_dir / "spin_action_timeout.flag")
    if nav["status"] == "UNKNOWN" and (phase_dir / "nav_goal_timeout.flag").exists():
        nav["status"] = "TIMEOUT"
    collision = parse_collision_states(phase_dir / "collision_monitor_state.txt")
    map_meta_before = parse_map_metadata(phase_dir / "map_metadata_before.txt")
    map_meta_after = parse_map_metadata(phase_dir / "map_metadata_after_warmup.txt")
    map_meta_end = parse_map_metadata(phase_dir / "map_metadata_end.txt")

    clock_start = parse_clock(phase_dir / "clock_start.txt")
    clock_end = parse_clock(phase_dir / "clock_end.txt")
    odom_start_stamp = parse_header_stamp(phase_dir / "odom_start.txt")
    odom_end_stamp = parse_header_stamp(phase_dir / "odom_end.txt")
    filt_start_stamp = parse_header_stamp(phase_dir / "odom_filtered_start.txt")
    filt_end_stamp = parse_header_stamp(phase_dir / "odometry_filtered_end.txt") or parse_header_stamp(phase_dir / "odom_filtered_end.txt")
    turn_deg_txt = read_text(phase_dir / "post_warmup_turn_deg_used.txt").strip()
    turn_deg_used = float(turn_deg_txt) if turn_deg_txt else None
    turn_mode_used = read_text(phase_dir / "post_warmup_turn_mode_used.txt").strip() or None

    phase = {
        "phase": phase_name,
        "nav_result": nav,
        "spin_result": spin,
        "post_warmup_turn_deg_used": turn_deg_used,
        "post_warmup_turn_mode_used": turn_mode_used,
        "clock": {
            "start": clock_start,
            "end": clock_end,
            "duration_sec": (clock_end - clock_start) if clock_start is not None and clock_end is not None else None,
        },
        "timestamps": {
            "odom_start_header": odom_start_stamp,
            "odom_end_header": odom_end_stamp,
            "odom_filtered_start_header": filt_start_stamp,
            "odom_filtered_end_header": filt_end_stamp,
            "odom_start_age_vs_clock": (clock_start - odom_start_stamp) if None not in (clock_start, odom_start_stamp) else None,
            "odom_end_age_vs_clock": (clock_end - odom_end_stamp) if None not in (clock_end, odom_end_stamp) else None,
            "odom_filtered_start_age_vs_clock": (clock_start - filt_start_stamp) if None not in (clock_start, filt_start_stamp) else None,
            "odom_filtered_end_age_vs_clock": (clock_end - filt_end_stamp) if None not in (clock_end, filt_end_stamp) else None,
        },
        "poses": {
            "start_map_tf": start_map,
            "end_map_tf": end_map,
            "start_odom_tf": start_odom_tf,
            "end_odom_tf": end_odom_tf,
            "start_gazebo": start_gz,
            "end_gazebo": end_gz,
            "start_odom_msg": odom_start,
            "end_odom_msg": odom_end,
            "start_odom_filtered_msg": filt_start,
            "end_odom_filtered_msg": filt_end,
            "goal_map": goal,
        },
        "map_metadata": {
            "before": map_meta_before,
            "after_warmup": map_meta_after,
            "end": map_meta_end,
        },
        "collision_monitor_state": collision,
        "metrics": {},
    }

    if start_map and end_map:
        mvec = dxy(start_map, end_map)
        phase["metrics"]["map_tf_delta"] = {**mvec, "distance_m": mag(mvec)}
    if start_odom_tf and end_odom_tf:
        ovec = dxy(start_odom_tf, end_odom_tf)
        phase["metrics"]["odom_tf_delta"] = {**ovec, "distance_m": mag(ovec)}
    if start_gz and end_gz:
        gvec = dxy(start_gz, end_gz)
        phase["metrics"]["gazebo_delta"] = {**gvec, "distance_m": mag(gvec)}
    if odom_start and odom_end:
        omsg = dxy(odom_start, odom_end)
        phase["metrics"]["odom_msg_delta"] = {**omsg, "distance_m": mag(omsg)}
    if filt_start and filt_end:
        fmsg = dxy(filt_start, filt_end)
        phase["metrics"]["odom_filtered_msg_delta"] = {**fmsg, "distance_m": mag(fmsg)}
    if goal and end_map:
        phase["metrics"]["goal_error_from_end_map_m"] = dist_xy({"x": goal["x"], "y": goal["y"]}, end_map)
        phase["metrics"]["goal_distance_from_start_map_m"] = dist_xy(start_map, {"x": goal["x"], "y": goal["y"]}) if start_map else None
    if goal and start_map:
        phase["metrics"]["goal_distance_from_start_map_m"] = dist_xy(start_map, {"x": goal["x"], "y": goal["y"]})

    g = phase["metrics"].get("gazebo_delta")
    m = phase["metrics"].get("map_tf_delta")
    if g and m:
        phase["metrics"]["gazebo_vs_map_displacement"] = {
            "distance_diff_m": abs(g["distance_m"] - m["distance_m"]),
            "distance_ratio": (m["distance_m"] / g["distance_m"]) if abs(g["distance_m"]) > 1e-9 else None,
            "note": "Map TF and Gazebo vectors are different frames; compare magnitudes, not dx/dy components.",
        }
    return phase

for trial_dir in sorted(diag_root.glob("trial_*")):
    trial_name = trial_dir.name
    phase_dirs = sorted([p for p in trial_dir.glob("phase_*") if p.is_dir()])
    if phase_dirs:
        trial = {"trial": trial_name, "phases": {}, "phase_comparison": {}}
        for p in phase_dirs:
            trial["phases"][p.name] = parse_phase_dir(p, p.name)
        a = trial["phases"].get("phase_a")
        b = trial["phases"].get("phase_b")
        if a and b:
            def mget(phase, key):
                return phase.get("metrics", {}).get(key)
            a_goal = mget(a, "goal_error_from_end_map_m")
            b_goal = mget(b, "goal_error_from_end_map_m")
            a_dur = a.get("clock", {}).get("duration_sec")
            b_dur = b.get("clock", {}).get("duration_sec")
            a_diff = (mget(a, "gazebo_vs_map_displacement") or {}).get("distance_diff_m")
            b_diff = (mget(b, "gazebo_vs_map_displacement") or {}).get("distance_diff_m")
            trial["phase_comparison"] = {
                "goal_error_stress_minus_easy_m": (b_goal - a_goal) if a_goal is not None and b_goal is not None else None,
                "duration_stress_minus_easy_sec": (b_dur - a_dur) if a_dur is not None and b_dur is not None else None,
                "gazebo_vs_map_disp_diff_stress_minus_easy_m": (b_diff - a_diff) if a_diff is not None and b_diff is not None else None,
                "phase_a_status": a["nav_result"]["status"],
                "phase_b_status": b["nav_result"]["status"],
            }
        report["trials"].append(trial)
    else:
        phase = parse_phase_dir(trial_dir, "single")
        report["trials"].append({"trial": trial_name, "phases": {"single": phase}, "phase_comparison": {}})


all_phases = []
for t in report["trials"]:
    for pname, p in t.get("phases", {}).items():
        all_phases.append((t["trial"], pname, p))

completed = [(tn, pn, p) for (tn, pn, p) in all_phases if p["nav_result"]["raw_available"]]
succ = [(tn, pn, p) for (tn, pn, p) in completed if p["nav_result"]["status"] == "SUCCEEDED"]

def phase_aggregate(phase_rows):
    rows = [r for r in phase_rows if r[2]["nav_result"]["raw_available"]]
    srows = [r for r in rows if r[2]["nav_result"]["status"] == "SUCCEEDED"]
    map_goal_errors = [r[2]["metrics"].get("goal_error_from_end_map_m") for r in rows if r[2]["metrics"].get("goal_error_from_end_map_m") is not None]
    map_moves = [r[2]["metrics"].get("map_tf_delta", {}).get("distance_m") for r in rows if r[2]["metrics"].get("map_tf_delta", {}).get("distance_m") is not None]
    gz_moves = [r[2]["metrics"].get("gazebo_delta", {}).get("distance_m") for r in rows if r[2]["metrics"].get("gazebo_delta", {}).get("distance_m") is not None]
    disp_diffs = [r[2]["metrics"].get("gazebo_vs_map_displacement", {}).get("distance_diff_m") for r in rows if r[2]["metrics"].get("gazebo_vs_map_displacement", {}).get("distance_diff_m") is not None]
    durations = [r[2]["clock"]["duration_sec"] for r in rows if r[2]["clock"]["duration_sec"] is not None]
    collision_changes = [r[2]["collision_monitor_state"]["changed"] for r in rows if r[2]["collision_monitor_state"]["changed"] is not None]
    return {
        "completed_phases": len(rows),
        "success_count": len(srows),
        "success_rate": (len(srows) / len(rows)) if rows else None,
        "aborted_count": sum(1 for _, _, p in rows if p["nav_result"]["status"] == "ABORTED"),
        "timeout_count": sum(1 for _, _, p in rows if p["nav_result"]["status"] == "TIMEOUT"),
        "statuses": [p["nav_result"]["status"] for _, _, p in rows],
        "error_codes": [p["nav_result"]["error_code"] for _, _, p in rows],
        "duration_sec_mean": stats.mean(durations) if durations else None,
        "duration_sec_median": stats.median(durations) if durations else None,
        "goal_error_end_map_mean_m": stats.mean(map_goal_errors) if map_goal_errors else None,
        "goal_error_end_map_median_m": stats.median(map_goal_errors) if map_goal_errors else None,
        "map_displacement_mean_m": stats.mean(map_moves) if map_moves else None,
        "gazebo_displacement_mean_m": stats.mean(gz_moves) if gz_moves else None,
        "gazebo_vs_map_disp_diff_mean_m": stats.mean(disp_diffs) if disp_diffs else None,
        "collision_monitor_state_changed_true_count": sum(1 for x in collision_changes if x is True),
        "collision_monitor_state_samples_available_count": len(collision_changes),
    }

report["aggregate"] = {
    "completed_trials": len(report["trials"]),
    "completed_phases_total": len(completed),
    "success_count_total": len(succ),
    "success_rate_total": (len(succ) / len(completed)) if completed else None,
    "phase_names_seen": sorted(set(pn for _, pn, _ in all_phases)),
    "all_phases": phase_aggregate(all_phases),
    "by_phase": {},
}
for phase_name in sorted(set(pn for _, pn, _ in all_phases)):
    report["aggregate"]["by_phase"][phase_name] = phase_aggregate([r for r in all_phases if r[1] == phase_name])

summary_path = diag_root / "summary.json"
summary_path.write_text(json.dumps(report, indent=2))

print("\n=== Nav2 Reliability Trials Summary ===")
agg = report["aggregate"]
print(f"Trials parsed: {agg['completed_trials']} / {trials_expected}")
print(f"Phases completed: {agg['completed_phases_total']}")
print(f"Total successes: {agg['success_count_total']}  Success rate: {agg['success_rate_total']}")
print("By phase:")
for pname, pagg in agg["by_phase"].items():
    print(f"- {pname}: completed={pagg['completed_phases']} success={pagg['success_count']} rate={pagg['success_rate']} "
          f"mean_dur={pagg['duration_sec_mean']} mean_goal_err={pagg['goal_error_end_map_mean_m']} "
          f"mean_|gz-map|_diff={pagg['gazebo_vs_map_disp_diff_mean_m']}")

print("\nPer-trial:")
for t in report["trials"]:
    pieces = []
    for pname, p in sorted(t.get("phases", {}).items()):
        status = p["nav_result"]["status"]
        err = p["nav_result"]["error_code"]
        spin_status = (p.get("spin_result") or {}).get("status")
        dur = p["clock"]["duration_sec"]
        goal_err = p["metrics"].get("goal_error_from_end_map_m")
        pieces.append(f"{pname}:status={status},err={err},dur={dur},goal_err={goal_err},turn_deg={p.get('post_warmup_turn_deg_used')},turn_mode={p.get('post_warmup_turn_mode_used')},spin_status={spin_status}")
    cmp = t.get("phase_comparison", {})
    cmp_txt = ""
    if cmp:
        cmp_txt = (f" | stress-easy Δgoal={cmp.get('goal_error_stress_minus_easy_m')} "
                   f"Δdur={cmp.get('duration_stress_minus_easy_sec')} "
                   f"Δ|gz-map|={cmp.get('gazebo_vs_map_disp_diff_stress_minus_easy_m')}")
    print(f"- {t['trial']}: " + " ; ".join(pieces) + cmp_txt)

print(f"\nDetailed report: {summary_path}")
print(f"Diagnostics root: {diag_root}")
PY
