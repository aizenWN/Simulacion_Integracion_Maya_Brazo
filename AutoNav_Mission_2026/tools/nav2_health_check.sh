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

WORLD="${WORLD:-random_world.sdf}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre2}"
RENDER_ENGINE_GUI="${RENDER_ENGINE_GUI:-ogre2}"
HEADLESS="${HEADLESS:-1}"
GOAL_DX="${GOAL_DX:-1.0}"
GOAL_DY="${GOAL_DY:-0.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"
GOAL_TIMEOUT_SEC="${GOAL_TIMEOUT_SEC:-90}"
STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-90}"
BW_SAMPLE_SEC="${BW_SAMPLE_SEC:-4}"
ECHO_TIMEOUT_SEC="${ECHO_TIMEOUT_SEC:-3}"

SIM_LAUNCH_CMD=(ros2 launch maya_bringup maya.launch.xml "rviz:=false")
if [[ "$HEADLESS" == "1" ]]; then
  SIM_LAUNCH_CMD+=("gz_args:=${WORLD} -s --render-engine ${RENDER_ENGINE} --render-engine-gui ${RENDER_ENGINE_GUI} -r")
else
  SIM_LAUNCH_CMD+=("world:=${WORLD}")
fi

tmpdir="$(mktemp -d)"
diag_dir="$tmpdir/diag"
ros_log_dir="$tmpdir/roslog"
mkdir -p "$diag_dir"
mkdir -p "$ros_log_dir"
export ROS_LOG_DIR="$ros_log_dir"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT -- "-$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
  echo "Diagnostics: $diag_dir"
}
trap cleanup EXIT

setsid "${SIM_LAUNCH_CMD[@]}" >"$diag_dir/launch.log" 2>&1 &
LAUNCH_PID=$!

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
            print(f"{tx} {ty} {yaw}")
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

get_start_pose_with_frame() {
  local out=""
  out="$(lookup_tf_xyyaw map base_footprint 25 || true)"
  if [[ -n "$out" ]]; then
    printf 'map %s\n' "$out"
    return 0
  fi
  out="$(lookup_tf_xyyaw odom base_footprint 25 || true)"
  if [[ -n "$out" ]]; then
    printf 'odom %s\n' "$out"
    return 0
  fi
  return 1
}

echo "[1/6] Waiting for startup topics..."
wait_for_topic /clock "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /clock" >&2; exit 1; }
wait_for_topic /scan "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /scan" >&2; exit 1; }
wait_for_topic /scan_merged "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /scan_merged" >&2; exit 1; }
wait_for_topic /odom "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /odom" >&2; exit 1; }
wait_for_topic /odometry/filtered "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /odometry/filtered" >&2; exit 1; }
wait_for_topic /map "$STARTUP_TIMEOUT_SEC" || { echo "Timed out waiting /map" >&2; exit 1; }

echo "[2/6] Waiting Nav2 lifecycle..."
wait_for_active_nav2 "$STARTUP_TIMEOUT_SEC" || { echo "Nav2 bt_navigator did not become active" >&2; exit 1; }

echo "[3/6] Capturing initial state..."
read_pose_component /clock "$diag_dir/clock_start.txt"
read_pose_component /odom "$diag_dir/odom_start.txt"
read_pose_component /odometry/filtered "$diag_dir/odom_filtered_start.txt"

xyyaw="$(lookup_tf_xyyaw map base_footprint 20 || true)"
start_pose="$(get_start_pose_with_frame || true)"
if [[ -z "$start_pose" ]]; then
  echo "Could not resolve startup TF pose (map->base_footprint or odom->base_footprint)." >&2
  exit 1
fi
read -r goal_frame start_x start_y start_yaw <<<"$start_pose"
xyyaw="${start_x} ${start_y} ${start_yaw}"
printf '%s\n' "$xyyaw" >"$diag_dir/tf_map_base_start.txt"
printf '%s\n' "$goal_frame" >"$diag_dir/goal_frame.txt"

goal_x="$(python3 - <<'PY' "$start_x" "$GOAL_DX"
import sys
print(float(sys.argv[1]) + float(sys.argv[2]))
PY
)"
goal_y="$(python3 - <<'PY' "$start_y" "$GOAL_DY"
import sys
print(float(sys.argv[1]) + float(sys.argv[2]))
PY
)"
goal_yaw="$(python3 - <<'PY' "$start_yaw" "$GOAL_YAW"
import sys
print(float(sys.argv[1]) + float(sys.argv[2]))
PY
)"

goal_qz_qw="$(python3 - <<'PY' "$goal_yaw"
import math
import sys
yaw = float(sys.argv[1])
print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))
PY
)"
read -r goal_qz goal_qw <<<"$goal_qz_qw"

goal_yaml="{pose: {header: {frame_id: ${goal_frame}}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${goal_qz}, w: ${goal_qw}}}}}"

echo "[4/6] Sending Nav2 goal..."
printf '%s\n' "$goal_yaml" >"$diag_dir/goal.yaml"
if timeout "${GOAL_TIMEOUT_SEC}s" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$goal_yaml" >"$diag_dir/nav2_goal.txt" 2>&1; then
  echo "Goal command finished."
else
  echo "Goal command timed out/failed (details in nav2_goal.txt)." >&2
fi

echo "[5/6] Capturing end state and topic bandwidth..."
read_pose_component /clock "$diag_dir/clock_end.txt"
xyyaw_end="$(lookup_tf_xyyaw map base_footprint 20 || true)"
if [[ -n "$xyyaw_end" ]]; then
  printf '%s\n' "$xyyaw_end" >"$diag_dir/tf_map_base_end.txt"
fi
read_pose_component /odom "$diag_dir/odom_end.txt"
read_pose_component /odometry/filtered "$diag_dir/odom_filtered_end.txt"

for topic in /scan /scan_merged /odom /odometry/filtered /imu /tf /map /clock; do
  safe_name="${topic////_}"
  echo "  [bw] sampling ${topic} (${BW_SAMPLE_SEC}s)..."
  timeout "${BW_SAMPLE_SEC}s" ros2 topic bw "$topic" >"$diag_dir/bw_${safe_name}.txt" 2>&1 || true
  bw_line="$(rg -m1 -N 'average:|^average|B/s|KB/s|MB/s' "$diag_dir/bw_${safe_name}.txt" || true)"
  if [[ -n "$bw_line" ]]; then
    echo "  [bw] ${topic}: ${bw_line}"
  else
    echo "  [bw] ${topic}: no bandwidth sample"
  fi
  timeout "${ECHO_TIMEOUT_SEC}s" ros2 topic echo --once "$topic" >"$diag_dir/echo_${safe_name}.txt" 2>&1 || true
done

timeout 6s ros2 run tf2_ros tf2_echo map odom >"$diag_dir/tf_map_odom.txt" 2>&1 || true
timeout 6s ros2 run tf2_ros tf2_echo odom base_footprint >"$diag_dir/tf_odom_base.txt" 2>&1 || true

python3 - <<'PY' "$diag_dir"
import json
import math
import re
import sys
from pathlib import Path

diag = Path(sys.argv[1])

BW_MIN_BPS = {
    '/scan': 1.0,
    '/scan_merged': 1.0,
    '/odom': 1.0,
    '/odometry/filtered': 1.0,
    '/imu': 1.0,
    '/tf': 1.0,
    '/clock': 1.0,
}
AGE_MAX = {
    '/scan': 0.25,
    '/scan_merged': 0.25,
    '/odom': 0.25,
    '/odometry/filtered': 0.25,
    '/imu': 0.25,
    '/map': 2.0,
}


def read_text(path):
    if not path.exists():
        return ''
    return path.read_text(encoding='utf-8', errors='ignore')


def parse_bw_bps(path):
    text = read_text(path)
    m = re.search(r'average:\s*([0-9.]+)\s*([KMG]?B/s)', text)
    if not m:
        return None
    val = float(m.group(1))
    unit = m.group(2)
    if unit == 'B/s':
        return val
    if unit == 'KB/s':
        return val * 1024.0
    if unit == 'MB/s':
        return val * 1024.0 * 1024.0
    if unit == 'GB/s':
        return val * 1024.0 * 1024.0 * 1024.0
    return None


def parse_clock(path):
    txt = read_text(path)
    ms = re.search(r'sec:\s*(\d+)', txt)
    mn = re.search(r'nanosec:\s*(\d+)', txt)
    if not ms or not mn:
        return None
    return int(ms.group(1)) + int(mn.group(1)) * 1e-9


def parse_header_stamp(path):
    txt = read_text(path)
    m = re.search(r'header:\s*\n\s*stamp:\s*\n\s*sec:\s*(\d+)\s*\n\s*nanosec:\s*(\d+)', txt)
    if not m:
        return None
    return int(m.group(1)) + int(m.group(2)) * 1e-9


def parse_odom_xy(path):
    txt = read_text(path)
    mx = re.search(r'position:\s*\n\s*x:\s*([\-0-9.eE]+)', txt)
    my = re.search(r'position:\s*\n\s*x:\s*[\-0-9.eE]+\s*\n\s*y:\s*([\-0-9.eE]+)', txt)
    if not mx or not my:
        return None
    return float(mx.group(1)), float(my.group(1))


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])

report = {
    'bandwidth_bps': {},
    'total_bandwidth_bps': 0.0,
    'ages_sec': {},
    'movement_m': {},
    'checks': []
}

for topic in BW_MIN_BPS:
    safe = topic.replace('/', '_')
    bw = parse_bw_bps(diag / f'bw_{safe}.txt')
    report['bandwidth_bps'][topic] = bw
    if bw is not None:
        report['total_bandwidth_bps'] += bw
    ok = bw is not None and bw >= BW_MIN_BPS[topic]
    report['checks'].append({
        'name': f'bandwidth {topic} > 0',
        'ok': ok,
        'value': bw,
    })

clock_end = parse_clock(diag / 'clock_end.txt')
for topic, max_age in AGE_MAX.items():
    safe = topic.replace('/', '_')
    st = parse_header_stamp(diag / f'echo_{safe}.txt')
    age = None if st is None or clock_end is None else max(0.0, clock_end - st)
    report['ages_sec'][topic] = age
    ok = age is not None and age <= max_age
    report['checks'].append({
        'name': f'age {topic} <= {max_age}s',
        'ok': ok,
        'value': age,
    })

odom_s = parse_odom_xy(diag / 'odom_start.txt')
odom_e = parse_odom_xy(diag / 'odom_end.txt')
filt_s = parse_odom_xy(diag / 'odom_filtered_start.txt')
filt_e = parse_odom_xy(diag / 'odom_filtered_end.txt')

odom_move = dist(odom_s, odom_e) if odom_s and odom_e else None
filt_move = dist(filt_s, filt_e) if filt_s and filt_e else None
report['movement_m']['odom'] = odom_move
report['movement_m']['odometry_filtered'] = filt_move

report['checks'].append({
    'name': 'movement odom >= 0.10m',
    'ok': odom_move is not None and odom_move >= 0.10,
    'value': odom_move,
})
report['checks'].append({
    'name': 'movement odometry/filtered >= 0.10m',
    'ok': filt_move is not None and filt_move >= 0.10,
    'value': filt_move,
})

nav2_text = read_text(diag / 'nav2_goal.txt')
status_match = re.search(r'Goal finished with status:\s*([A-Z_]+)', nav2_text)
error_match = re.search(r'error_code:\s*([0-9]+)', nav2_text)
nav2_status = status_match.group(1) if status_match else 'UNKNOWN'
nav2_error_code = int(error_match.group(1)) if error_match else None
nav2_ok = nav2_status == 'SUCCEEDED'
report['checks'].append({
    'name': 'nav2 goal result succeeded',
    'ok': nav2_ok,
    'value': {'status': nav2_status, 'error_code': nav2_error_code},
})

report['pass'] = all(c['ok'] for c in report['checks'])
(diag / 'summary.json').write_text(json.dumps(report, indent=2))

print('\n=== Nav2 Health Summary ===')
print(f"Overall: {'PASS' if report['pass'] else 'FAIL'}")
print(f"Total throughput: {report['total_bandwidth_bps']:.2f} B/s ({report['total_bandwidth_bps']/1024.0:.2f} KB/s)")
for c in report['checks']:
    state = 'PASS' if c['ok'] else 'FAIL'
    print(f"[{state}] {c['name']} (value={c['value']})")
print('\nsummary:', diag / 'summary.json')
PY

echo "[6/6] Done."
