#!/usr/bin/env bash
# RDK X5 ROSMASTER M1 - one-click saved-map Nav2 navigation
# Usage:
#   bash scripts/slam/nav2_oneclick_goal.sh [GOAL_X] [GOAL_Y] [GOAL_YAW_RAD]
# Example:
#   bash scripts/slam/nav2_oneclick_goal.sh 0.30 0.00 0.00
# You can also edit the DEFAULT_GOAL_* values below.

set -Eeuo pipefail

# ===================== USER EDIT AREA =====================
DEFAULT_GOAL_X="0.30"
DEFAULT_GOAL_Y="0.00"
DEFAULT_GOAL_YAW="0.00"

DEFAULT_INIT_X="0.00"
DEFAULT_INIT_Y="0.00"
DEFAULT_INIT_YAW="0.00"

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
MAP_YAML="${MAP_YAML:-$PROJECT_DIR/maps/joy_calibrated_corridor_map.yaml}"
NAV2_PARAMS="${NAV2_PARAMS:-$PROJECT_DIR/configs/nav2_params.yaml}"

# If empty, the script reads /scan.header.frame_id and uses it automatically.
LASER_FRAME="${LASER_FRAME:-}"
LASER_X="${LASER_X:-0.10}"
LASER_Y="${LASER_Y:-0.0}"
LASER_Z="${LASER_Z:-0.12}"
LASER_ROLL="${LASER_ROLL:-0.0}"
LASER_PITCH="${LASER_PITCH:-0.0}"
LASER_YAW="${LASER_YAW:-0.0}"

# Conservative real-car speed profile. Override in terminal if needed.
CHASSIS_MAX_VX="${CHASSIS_MAX_VX:-0.18}"
CHASSIS_MAX_WZ="${CHASSIS_MAX_WZ:-0.80}"
CHASSIS_PWM_MAX="${CHASSIS_PWM_MAX:-45.0}"
CHASSIS_VX_PWM_GAIN="${CHASSIS_VX_PWM_GAIN:-220.0}"
CHASSIS_WZ_PWM_GAIN="${CHASSIS_WZ_PWM_GAIN:-160.0}"
CHASSIS_CONTROL_RATE_HZ="${CHASSIS_CONTROL_RATE_HZ:-20}"

# 1 = send goal after startup; 0 = only start Nav2 and diagnostics.
AUTO_SEND_GOAL="${AUTO_SEND_GOAL:-1}"
# 1 = keep stack alive until Ctrl+C. Unset = auto: exit after nav when AUTO_SEND_GOAL=1,
# keep running when AUTO_SEND_GOAL=0 (manual goal mode).
KEEP_RUNNING_AFTER_GOAL="${KEEP_RUNNING_AFTER_GOAL:-}"
# 1 = also start Foxglove bridge if installed.
START_FOXGLOVE="${START_FOXGLOVE:-1}"
FOXGLOVE_LAYOUT="${FOXGLOVE_LAYOUT:-$PROJECT_DIR/configs/foxglove_nav2_oneclick.layout.json}"
# 1 = publish Nav2 plan/local_plan markers for Foxglove map overlay.
START_NAV2_PATH_VIZ="${START_NAV2_PATH_VIZ:-1}"

# Chassis self-check before Nav2: brief straight cmd_vel burst (0=skip, 1=run).
CHASSIS_VERIFY="${CHASSIS_VERIFY:-1}"
CHASSIS_VERIFY_VX="${CHASSIS_VERIFY_VX:-0.12}"
CHASSIS_VERIFY_SEC="${CHASSIS_VERIFY_SEC:-1.0}"
CHASSIS_VERIFY_MIN_DELTA="${CHASSIS_VERIFY_MIN_DELTA:-0.03}"

# After navigation, verify map pose vs goal (0=trust Nav2 only, 1=check TF).
GOAL_VERIFY="${GOAL_VERIFY:-1}"
GOAL_VERIFY_XY_TOL="${GOAL_VERIFY_XY_TOL:-0.15}"
GOAL_VERIFY_YAW_TOL="${GOAL_VERIFY_YAW_TOL:-0.35}"

# 1 = invert Nav2 cmd_vel (vx/wz) via oneclick-only relay during navigation.
# Does not change chassis bridge or other scripts. Default off.
NAV2_ONECLICK_CMD_VEL_INVERT="${NAV2_ONECLICK_CMD_VEL_INVERT:-0}"
NAV2_ONECLICK_RAW_CMD_VEL="${NAV2_ONECLICK_RAW_CMD_VEL:-/nav2_oneclick/raw_cmd_vel}"
# ==========================================================

GOAL_X="${1:-$DEFAULT_GOAL_X}"
GOAL_Y="${2:-$DEFAULT_GOAL_Y}"
GOAL_YAW="${3:-$DEFAULT_GOAL_YAW}"
INIT_X="${INIT_X:-$DEFAULT_INIT_X}"
INIT_Y="${INIT_Y:-$DEFAULT_INIT_Y}"
INIT_YAW="${INIT_YAW:-$DEFAULT_INIT_YAW}"

PIDS=()
LOG_DIR=""

log() { echo "[NAV2-ONECLICK] $*"; }
warn() { echo "[NAV2-ONECLICK][WARN] $*" >&2; }
fatal() { echo "[NAV2-ONECLICK][FAIL] $*" >&2; exit 1; }

source_ros() {
  set +u
  [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
  [ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
  set -u
}

start_bg() {
  local name="$1"; shift
  mkdir -p "$LOG_DIR"
  log "start $name: $*"
  "$@" > "$LOG_DIR/${name}.log" 2>&1 &
  PIDS+=("$!")
}

cleanup() {
  local code=$?
  set +e
  log "cleanup: sending zero cmd and stopping background processes..."
  python3 - <<'PY' >/dev/null 2>&1 || true
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init()
node = Node('nav2_oneclick_zero_on_exit')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
msg = Twist()
for _ in range(8):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.0)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" >/dev/null 2>&1 || true
  done
  pkill -f "nav2_bringup.*bringup_launch.py" >/dev/null 2>&1 || true
  pkill -f "amcl|map_server|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|smoother_server|waypoint_follower|lifecycle_manager" >/dev/null 2>&1 || true
  pkill -f "m1_pwm_cmd_vel_bridge.py|cmd_vel_to_rosmaster.py" >/dev/null 2>&1 || true
  pkill -f "nav2_plan_path_viz.py" >/dev/null 2>&1 || true
  pkill -f "nav2_oneclick_cmd_vel_relay.py" >/dev/null 2>&1 || true
  pkill -f "static_transform_publisher.*base_link" >/dev/null 2>&1 || true
  exit "$code"
}
trap cleanup INT TERM EXIT

require_file() {
  [ -f "$1" ] || fatal "missing file: $1"
}

require_executable_or_file() {
  [ -e "$1" ] || fatal "missing file: $1"
}

clean_conflicting_nodes() {
  log "clean old SLAM/Nav2/YOLO/Joy publishers to avoid /cmd_vel conflicts"
  pkill -f "ros2 topic pub" >/dev/null 2>&1 || true
  pkill -f "slam_toolbox|run_joy_mapping_all|run_corridor_mapping_live_foxglove" >/dev/null 2>&1 || true
  pkill -f "teleop_twist_joy|joy_node" >/dev/null 2>&1 || true
  pkill -f "shared_nav|yolov5s_bpu_web_node|run_mvp_task.py|yolo_lidar" >/dev/null 2>&1 || true
  pkill -f "nav2_bringup|amcl|map_server|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|smoother_server|waypoint_follower|lifecycle_manager" >/dev/null 2>&1 || true
  pkill -f "m1_pwm_cmd_vel_bridge.py|cmd_vel_to_rosmaster.py" >/dev/null 2>&1 || true
  pkill -f "foxglove_bridge" >/dev/null 2>&1 || true
  pkill -f "simple_scan_filter.py" >/dev/null 2>&1 || true
  pkill -f "static_transform_publisher.*base_link" >/dev/null 2>&1 || true
  sleep 2
}

probe_rosmaster_port() {
  python3 - <<'PY'
import contextlib
import glob
import io
import os
import sys
import time

try:
    from Rosmaster_Lib import Rosmaster
except Exception:
    sys.exit(1)

candidates = []
for path in ("/dev/rosmaster", "/dev/ttyACM0"):
    if os.path.exists(path):
        candidates.append(path)
for path in sorted(glob.glob("/dev/ttyUSB*")):
    if path not in candidates:
        candidates.append(path)

best_port = ""
best_voltage = 0.0
for port in candidates:
    bot = None
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            bot = Rosmaster(car_type=1, com=port)
            time.sleep(1.0)
            bot.create_receive_threading()
            time.sleep(1.5)
            voltage = float(bot.get_battery_voltage())
        if voltage > best_voltage:
            best_voltage = voltage
            best_port = port
    except Exception:
        pass
    finally:
        if bot is not None:
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    bot.__del__()
            except Exception:
                pass

if best_port and best_voltage > 3.0:
    print(best_port)
PY
}

select_chassis_dev() {
  if [ -n "${CHASSIS_DEV:-}" ]; then
    :
  elif [ -e /dev/rosmaster ]; then
    CHASSIS_DEV="/dev/rosmaster"
  elif [ -e /dev/ttyACM0 ]; then
    CHASSIS_DEV="/dev/ttyACM0"
  else
    local probed=""
    probed="$(probe_rosmaster_port 2>/dev/null || true)"
    if [ -n "$probed" ]; then
      CHASSIS_DEV="$probed"
      log "auto-detected Rosmaster chassis on $CHASSIS_DEV"
    else
      fatal "cannot find chassis device. On this robot use CHASSIS_DEV=/dev/rosmaster (not /dev/ttyUSB0, which is the lidar)."
    fi
  fi
  export CHASSIS_DEV CHASSIS_PORT="$CHASSIS_DEV"
  export CHASSIS_MAX_VX CHASSIS_MAX_WZ CHASSIS_PWM_MAX CHASSIS_VX_PWM_GAIN CHASSIS_WZ_PWM_GAIN CHASSIS_CONTROL_RATE_HZ
  log "CHASSIS_DEV=$CHASSIS_DEV max_vx=$CHASSIS_MAX_VX max_wz=$CHASSIS_MAX_WZ pwm_max=$CHASSIS_PWM_MAX"
}

verify_chassis_motion() {
  if [ "${CHASSIS_VERIFY:-1}" != "1" ]; then
    log "CHASSIS_VERIFY=0, skip chassis motion self-check"
    return 0
  fi
  local burst_vx="${CHASSIS_VERIFY_VX:-0.12}"
  local burst_sec="${CHASSIS_VERIFY_SEC:-1.0}"
  local min_delta="${CHASSIS_VERIFY_MIN_DELTA:-0.03}"
  log "verify chassis motion (straight cmd_vel vx=$burst_vx wz=0 for ${burst_sec}s)"
  python3 - <<PY
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomProbe(Node):
    def __init__(self):
        super().__init__("nav2_oneclick_odom_probe")
        self.pose = None
        self.sub = self.create_subscription(Odometry, "/odom", self._cb, 10)

    def _cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = (float(p.x), float(p.y))

def wait_pose(node, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.pose is not None:
            return node.pose
    return None

rclpy.init()
node = OdomProbe()
before = wait_pose(node)
if before is None:
    print("[FAIL] no /odom pose before chassis verify", flush=True)
    rclpy.shutdown()
    sys.exit(1)

subprocess.run(
    ["python3", "$PROJECT_DIR/scripts/slam/cmd_vel_burst.py", str($burst_vx), "0.0", str($burst_sec)],
    check=False,
)
time.sleep(0.5)
after = wait_pose(node, timeout=5.0)
rclpy.shutdown()
if after is None:
    print("[FAIL] no /odom pose after chassis verify", flush=True)
    sys.exit(1)

delta = math.hypot(after[0] - before[0], after[1] - before[1])
print(f"[INFO] odom delta={delta:.4f} m (before={before}, after={after})", flush=True)
if delta < float($min_delta):
    print(
        "[FAIL] chassis did not move enough; check CHASSIS_DEV "
        f"(expected /dev/rosmaster, not lidar /dev/ydlidar) and motor power",
        flush=True,
    )
    sys.exit(1)
print("[OK] chassis motion verified", flush=True)
PY
}

fix_map_yaml() {
  if [ ! -f "$MAP_YAML" ]; then
    if [ -f "$PROJECT_DIR/maps/joy_corridor_map.yaml" ]; then
      warn "MAP_YAML not found, fallback to joy_corridor_map.yaml"
      MAP_YAML="$PROJECT_DIR/maps/joy_corridor_map.yaml"
    else
      fatal "map yaml not found: $MAP_YAML"
    fi
  fi

  local map_dir image_base preferred_pgm
  map_dir="$(dirname "$MAP_YAML")"
  preferred_pgm="${MAP_YAML%.yaml}.pgm"

  if [ -f "$preferred_pgm" ]; then
    image_base="$(basename "$preferred_pgm")"
  else
    image_base="$(awk '/^[[:space:]]*image[[:space:]]*:/ {print $2; exit}' "$MAP_YAML" | tr -d '"')"
    if [ -z "$image_base" ] || [ ! -f "$map_dir/$image_base" ]; then
      local any_pgm
      any_pgm="$(ls -t "$map_dir"/*.pgm 2>/dev/null | head -1 || true)"
      [ -n "$any_pgm" ] || fatal "no .pgm map image found in $map_dir"
      image_base="$(basename "$any_pgm")"
    fi
  fi

  python3 - "$MAP_YAML" "$image_base" <<'PY'
import sys
from pathlib import Path
path = Path(sys.argv[1])
image = sys.argv[2]
lines = path.read_text().splitlines()
out = []
done = False
for line in lines:
    if line.strip().startswith('image:'):
        out.append(f'image: {image}')
        done = True
    else:
        out.append(line)
if not done:
    out.insert(0, f'image: {image}')
path.write_text('\n'.join(out).rstrip() + '\n')
PY
  log "MAP_YAML=$MAP_YAML"
  log "map image=$(grep -E '^image:' "$MAP_YAML" | head -1)"
  [ -f "$map_dir/$image_base" ] || fatal "map image missing after fix: $map_dir/$image_base"
}

patch_nav2_params() {
  require_file "$NAV2_PARAMS"
  python3 - "$NAV2_PARAMS" "$MAP_YAML" <<'PY'
import sys
from pathlib import Path
try:
    import yaml
except Exception as e:
    raise SystemExit(f"PyYAML is required but not importable: {e}")

path = Path(sys.argv[1])
map_yaml = sys.argv[2]
data = yaml.safe_load(path.read_text())
if not isinstance(data, dict):
    raise SystemExit("nav2_params.yaml did not parse as a mapping")

def ros_params(*keys):
    d = data
    for k in keys:
        d = d.setdefault(k, {})
    return d.setdefault('ros__parameters', {})

# map_server: avoid launch-override failures; make map path explicit.
ros_params('map_server')['yaml_filename'] = map_yaml
ros_params('map_server')['use_sim_time'] = False

# local costmap should use odom for smooth local control; global costmap stays map.
local = ros_params('local_costmap', 'local_costmap')
local['global_frame'] = 'odom'
local['robot_base_frame'] = 'base_link'
local['rolling_window'] = True
# Nav2 costmap width/height must be integers (not 3.0 doubles).
local['width'] = int(local.get('width', 3) or 3)
local['height'] = int(local.get('height', 3) or 3)
local['robot_radius'] = float(local.get('robot_radius', 0.18) or 0.18)

# Global costmap frame must remain map.
global_cm = ros_params('global_costmap', 'global_costmap')
global_cm['global_frame'] = 'map'
global_cm['robot_base_frame'] = 'base_link'
global_cm['robot_radius'] = float(global_cm.get('robot_radius', 0.18) or 0.18)

# AMCL frames and scan topic.
amcl = ros_params('amcl')
amcl['global_frame_id'] = 'map'
amcl['odom_frame_id'] = 'odom'
amcl['base_frame_id'] = 'base_link'
amcl['scan_topic'] = '/scan'
amcl['tf_broadcast'] = True
amcl['transform_tolerance'] = float(amcl.get('transform_tolerance', 2.0) or 2.0)
amcl['update_min_a'] = float(amcl.get('update_min_a', 0.05) or 0.05)
amcl['update_min_d'] = float(amcl.get('update_min_d', 0.05) or 0.05)
amcl['laser_min_range'] = float(amcl.get('laser_min_range', 0.18) or 0.18)

# DWB speed profile: allow enough speed to overcome M1 chassis dead zone, but stay safe.
controller = ros_params('controller_server')
controller['controller_frequency'] = 20.0
controller['min_x_velocity_threshold'] = 0.01
controller['min_theta_velocity_threshold'] = 0.01
pc = controller.setdefault('progress_checker', {})
pc['plugin'] = pc.get('plugin', 'nav2_controller::SimpleProgressChecker')
pc['required_movement_radius'] = 0.05
pc['movement_time_allowance'] = 60.0

gc = controller.setdefault('general_goal_checker', {})
gc['stateful'] = True
gc['plugin'] = gc.get('plugin', 'nav2_controller::SimpleGoalChecker')
gc['xy_goal_tolerance'] = 0.15
gc['yaw_goal_tolerance'] = 0.35

fp = controller.setdefault('FollowPath', {})
fp['plugin'] = 'dwb_core::DWBLocalPlanner'
fp['min_vel_x'] = 0.10
fp['max_vel_x'] = 0.18
fp['max_vel_theta'] = 0.80
fp['min_speed_xy'] = 0.08
fp['max_speed_xy'] = 0.18
fp['min_speed_theta'] = 0.0
fp['acc_lim_x'] = 0.35
fp['decel_lim_x'] = -0.35
fp['acc_lim_theta'] = 1.2
fp['decel_lim_theta'] = -1.2
fp['trans_stopped_velocity'] = 0.02
fp['xy_goal_tolerance'] = 0.15

# Recovery spin previously produced angular.z around 1.0; cap it for lab safety.
behavior = ros_params('behavior_server')
behavior['global_frame'] = 'map'
behavior['robot_base_frame'] = 'base_link'
behavior['max_rotational_vel'] = 0.6
behavior['min_rotational_vel'] = 0.15
behavior['rotational_acc_lim'] = 1.2

# Match smoother to real-car limits.
vs = ros_params('velocity_smoother')
vs['max_velocity'] = [0.18, 0.0, 0.80]
vs['min_velocity'] = [-0.05, 0.0, -0.80]
vs['max_accel'] = [0.35, 0.0, 1.2]
vs['max_decel'] = [-0.35, 0.0, -1.2]
vs['deadband_velocity'] = [0.0, 0.0, 0.0]

# Planner remains NavFn; keep map global planner behavior.
planner = ros_params('planner_server')
planner['use_sim_time'] = False
grid = planner.setdefault('GridBased', {})
grid['plugin'] = 'nav2_navfn_planner/NavfnPlanner'
grid['use_astar'] = False
grid['allow_unknown'] = True

bt = ros_params('bt_navigator')
bt['default_server_timeout'] = 60
bt['wait_for_service_timeout'] = 5000

# Costmaps expect /scan_filtered when simple_scan_filter is running.
scan_topic = '/scan_filtered'
for costmap_name in ('local_costmap', 'global_costmap'):
    cm = ros_params(costmap_name, costmap_name)
    for plugin_key in ('voxel_layer', 'obstacle_layer'):
        layer = cm.get(plugin_key)
        if not isinstance(layer, dict):
            continue
        sources = layer.get('observation_sources')
        if not sources:
            continue
        for src_name in str(sources).split():
            src = layer.get(src_name)
            if isinstance(src, dict):
                src['topic'] = scan_topic

path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
print('[OK] patched nav2 params:', path)
PY
  python3 - <<PY
import yaml
with open('$NAV2_PARAMS') as f:
    yaml.safe_load(f)
print('[OK] YAML parse:', '$NAV2_PARAMS')
PY
}

wait_topic_data() {
  local topic="$1" timeout_sec="$2" extra_args="${3:-}"
  log "wait data: $topic"
  local start now
  start="$(date +%s)"
  while true; do
    # shellcheck disable=SC2086
    if timeout 3 ros2 topic echo "$topic" --once $extra_args >/dev/null 2>&1; then
      log "data OK: $topic"
      return 0
    fi
    now="$(date +%s)"
    if [ $((now - start)) -ge "$timeout_sec" ]; then
      warn "data timeout: $topic"
      return 1
    fi
    sleep 1
  done
}

get_scan_frame() {
  local frame
  frame="$(timeout 8 ros2 topic echo /scan --once 2>/dev/null | awk '
    /frame_id:/ {gsub(/"/,"",$2); print $2; exit}
  ' || true)"
  if [ -z "$frame" ]; then
    warn "cannot read /scan frame_id; fallback to laser"
    frame="laser"
  fi
  echo "$frame"
}

wait_tf() {
  local parent="$1"
  local child="$2"
  local timeout_s="${3:-12}"

  log "wait TF: ${parent} -> ${child}"

  local deadline=$(( $(date +%s) + timeout_s ))
  local tmp

  while [ "$(date +%s)" -lt "$deadline" ]; do
    tmp="$(mktemp)"

    # tf2_echo keeps running, so timeout may return non-zero even after success.
    # Judge success by whether transform content was printed.
    timeout 2 ros2 run tf2_ros tf2_echo "$parent" "$child" > "$tmp" 2>&1 || true

    if grep -q "Translation:" "$tmp"; then
      rm -f "$tmp"
      log "TF OK: ${parent} -> ${child}"
      return 0
    fi

    rm -f "$tmp"
    sleep 0.5
  done

  warn "TF timeout: ${parent} -> ${child}"
  return 1
}

lifecycle_get() {
  timeout 5 ros2 lifecycle get "$1" 2>/dev/null || true
}

# Nav2 autostart uses lifecycle_manager; only poll state, never manual configure/activate.
wait_lifecycle_active() {
  local node="$1"
  local timeout_sec="${2:-120}"
  local start now state

  log "wait lifecycle active: $node (timeout=${timeout_sec}s)"
  start="$(date +%s)"
  while true; do
    state="$(lifecycle_get "$node")"
    if echo "$state" | grep -q "active"; then
      log "$node active: ${state}"
      return 0
    fi
    now="$(date +%s)"
    if [ $((now - start)) -ge "$timeout_sec" ]; then
      warn "$node not active after ${timeout_sec}s (last: ${state:-NO RESPONSE})"
      return 1
    fi
    if [ -n "$state" ]; then
      log "$node pending: $state"
    else
      log "$node pending: waiting for lifecycle service..."
    fi
    sleep 2
  done
}

wait_map_topic() {
  local timeout_sec="${1:-60}"
  log "wait map topic with transient_local QoS (/map)"
  python3 - "$timeout_sec" <<'PY'
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

timeout = float(sys.argv[1])
rclpy.init()
node = Node("nav2_oneclick_wait_map")
qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
got = {"ok": False, "w": 0, "h": 0}

def cb(msg: OccupancyGrid) -> None:
    if msg.info.width > 0 and msg.info.height > 0:
        got["ok"] = True
        got["w"] = msg.info.width
        got["h"] = msg.info.height

node.create_subscription(OccupancyGrid, "/map", cb, qos)
start = time.time()
while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.5)
    if got["ok"]:
        print(f"[OK] /map received: {got['w']} x {got['h']}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    time.sleep(0.2)

node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
PY
}

publish_initial_pose() {
  local x="$1" y="$2" yaw="$3"
  log "publish initial pose: x=$x y=$y yaw=$yaw"
  python3 - "$x" "$y" "$yaw" <<'PY'
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

x, y, yaw = map(float, sys.argv[1:4])
rclpy.init()
node = Node("nav2_oneclick_initial_pose")
qos = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
)
pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.pose.pose.position.x = x
msg.pose.pose.position.y = y
msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
msg.pose.covariance = [0.0] * 36
msg.pose.covariance[0] = 0.25
msg.pose.covariance[7] = 0.25
msg.pose.covariance[35] = 0.0685
# Keep publishing until AMCL/DDS has time to discover the publisher.
for _ in range(30):
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.2)
node.destroy_node()
rclpy.shutdown()
PY
}

wait_for_localization() {
  local x="$1" y="$2" yaw="$3"
  local timeout_sec="${4:-90}"
  log "wait AMCL localization (map->odom / map->base_link), timeout=${timeout_sec}s"
  python3 - "$x" "$y" "$yaw" "$timeout_sec" <<'PY'
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

x, y, yaw = map(float, sys.argv[1:4])
timeout = float(sys.argv[4])

rclpy.init()
node = Node("nav2_oneclick_wait_localization")
qos = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
)
pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)
tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
listener = TransformListener(tf_buffer, node, spin_thread=False)

msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.pose.pose.position.x = x
msg.pose.pose.position.y = y
msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
msg.pose.covariance = [0.0] * 36
msg.pose.covariance[0] = 0.25
msg.pose.covariance[7] = 0.25
msg.pose.covariance[35] = 0.0685

start = time.time()
last_pub = 0.0
map_odom_ok = False
map_base_ok = False

while time.time() - start < timeout:
    now = time.time()
    if now - last_pub >= 0.5:
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        last_pub = now

    rclpy.spin_once(node, timeout_sec=0.1)

    try:
        if not map_odom_ok:
            tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(), timeout=Duration(seconds=0.2))
            map_odom_ok = True
            print("[OK] TF map -> odom available", flush=True)
        if not map_base_ok:
            tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.2))
            map_base_ok = True
            print("[OK] TF map -> base_link available", flush=True)
            break
    except Exception:
        pass

node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if map_base_ok else 1)
PY
}

verify_goal_reached() {
  local gx="$1" gy="$2" gyaw="$3"
  if [ "${GOAL_VERIFY:-1}" != "1" ]; then
    log "GOAL_VERIFY=0, skip post-navigation pose check"
    return 0
  fi
  log "verify goal reached: target=($gx, $gy, yaw=$gyaw) xy_tol=${GOAL_VERIFY_XY_TOL:-0.15}"
  python3 - <<PY
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros

gx = float("$gx")
gy = float("$gy")
gyaw = float("$gyaw")
xy_tol = float("${GOAL_VERIFY_XY_TOL:-0.15}")
yaw_tol = float("${GOAL_VERIFY_YAW_TOL:-0.35}")

def yaw_from_quat(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

rclpy.init()
node = Node("nav2_oneclick_goal_verify")
tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
tf_listener = tf2_ros.TransformListener(tf_buffer, node)

pose = None
end = time.time() + 8.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        tf = tf_buffer.lookup_transform(
            "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.3)
        )
        t = tf.transform.translation
        q = tf.transform.rotation
        pose = (float(t.x), float(t.y), yaw_from_quat(q.x, q.y, q.z, q.w))
        break
    except Exception:
        pass

node.destroy_node()
rclpy.shutdown()

if pose is None:
    print("[FAIL] cannot read map->base_link TF for goal verify", flush=True)
    sys.exit(1)

x, y, yaw = pose
xy_err = math.hypot(gx - x, gy - y)
yaw_err = abs(math.atan2(math.sin(yaw - gyaw), math.cos(yaw - gyaw)))
print(
    f"[INFO] pose=({x:.3f}, {y:.3f}, yaw={yaw:.3f}) "
    f"xy_err={xy_err:.3f}m yaw_err={math.degrees(yaw_err):.1f}deg",
    flush=True,
)
if xy_err > xy_tol:
    print(
        f"[FAIL] goal verify: xy error {xy_err:.3f}m > tol {xy_tol:.3f}m "
        f"(Nav2 may have reported false success)",
        flush=True,
    )
    sys.exit(1)
if yaw_err > yaw_tol:
    print(
        f"[FAIL] goal verify: yaw error {math.degrees(yaw_err):.1f}deg > "
        f"tol {math.degrees(yaw_tol):.1f}deg",
        flush=True,
    )
    sys.exit(1)
print("[OK] goal verify passed", flush=True)
PY
}


write_nav2_invert_bringup_launch() {
  local launch_py="$LOG_DIR/nav2_oneclick_bringup.launch.py"
  cat >"$launch_py" <<'LAUNCHPY'
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        GroupAction([
            SetRemap(src='/cmd_vel', dst='/nav2_oneclick/raw_cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('params_file'),
                    'use_composition': LaunchConfiguration('use_composition'),
                }.items(),
            ),
        ]),
    ])
LAUNCHPY
  echo "$launch_py"
}

start_cmd_vel_invert_relay() {
  require_file "$PROJECT_DIR/ros2_bridge/nav2_oneclick_cmd_vel_relay.py"
  log "cmd_vel invert relay ON: ${NAV2_ONECLICK_RAW_CMD_VEL} -> /cmd_vel (nav only)"
  start_bg cmd_vel_relay python3 "$PROJECT_DIR/ros2_bridge/nav2_oneclick_cmd_vel_relay.py"     --in-topic "$NAV2_ONECLICK_RAW_CMD_VEL"     --out-topic /cmd_vel
  sleep 1
}

send_goal() {
  local x="$1" y="$2" yaw="$3"
  local qz qw
  read -r qz qw < <(python3 - <<PY
import math
yaw = float('$yaw')
print(math.sin(yaw/2.0), math.cos(yaw/2.0))
PY
)
  log "wait action server /navigate_to_pose"
  local start now
  start="$(date +%s)"
  while true; do
    if ros2 action list 2>/dev/null | grep -qx "/navigate_to_pose"; then
      break
    fi
    now="$(date +%s)"
    if [ $((now - start)) -ge 60 ]; then
      fatal "action server /navigate_to_pose not available"
    fi
    sleep 1
  done
  log "send NavigateToPose goal: x=$x y=$y yaw=$yaw qz=$qz qw=$qw"
  python3 - <<PY >/dev/null 2>&1 || true
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math

rclpy.init()
node = rclpy.create_node("nav2_oneclick_goal_pose_pub")
pub = node.create_publisher(PoseStamped, "/goal_pose", 10)
msg = PoseStamped()
msg.header.frame_id = "map"
msg.pose.position.x = float("$x")
msg.pose.position.y = float("$y")
msg.pose.orientation.z = float("$qz")
msg.pose.orientation.w = float("$qw")
for _ in range(5):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_node()
rclpy.shutdown()
PY
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: map
  pose:
    position:
      x: ${x}
      y: ${y}
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: ${qz}
      w: ${qw}
behavior_tree: ''
" --feedback
}

print_foxglove_help() {
  local host
  host="$(hostname -I 2>/dev/null | awk '{print $1}')"
  [ -n "$host" ] || host="127.0.0.1"
  echo "========== FOXGLOVE NAV2 PATH VIZ =========="
  echo "Connect: ws://${host}:8765"
  if [ -f "$FOXGLOVE_LAYOUT" ]; then
    echo "Layout -> Import -> $FOXGLOVE_LAYOUT"
  fi
  echo "Path topics on map:"
  echo "  /plan                 global plan (green)"
  echo "  /local_plan           local plan (blue)"
  echo "  /nav2_plan_markers    MarkerArray overlay"
  echo "  /nav2_viz/global_plan /nav2_viz/local_plan  republished paths"
  echo "  /goal_pose            navigation target (red sphere)"
  echo "============================================="
}

print_status() {
  echo "========== NAV2 ONECLICK STATUS =========="
  ros2 node list | grep -E "amcl|map_server|controller|planner|bt_navigator|lifecycle|m1_pwm|ydlidar|static_transform" || true
  echo "---------- lifecycle ----------"
  for n in /map_server /amcl /controller_server /planner_server /bt_navigator; do
    echo "===== $n ====="
    timeout 3 ros2 lifecycle get "$n" || echo "TIMEOUT OR NO RESPONSE"
  done
  echo "---------- topics ----------"
  ros2 topic list | grep -E "^/map$|^/scan$|^/odom$|^/tf$|^/tf_static$|^/cmd_vel$|^/amcl_pose$" || true
  echo "---------- cmd_vel endpoints ----------"
  ros2 topic info /cmd_vel -v 2>/dev/null | grep -E "Publisher count|Subscription count|Node name|Endpoint type" || true
}

main() {
  source_ros
  cd "$PROJECT_DIR" || fatal "cannot cd PROJECT_DIR=$PROJECT_DIR"
  LOG_DIR="$PROJECT_DIR/logs/nav2_oneclick_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$LOG_DIR"

  log "GOAL=($GOAL_X, $GOAL_Y, $GOAL_YAW), INIT=($INIT_X, $INIT_Y, $INIT_YAW)"
  log "logs=$LOG_DIR"

  require_executable_or_file "$PROJECT_DIR/scripts/lidar/start_lidar_only.sh"
  require_file "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
  require_file "$PROJECT_DIR/ros2_bridge/m1_pwm_cmd_vel_bridge.py"
  require_file "$PROJECT_DIR/ros2_bridge/simple_scan_filter.py"

  clean_conflicting_nodes
  select_chassis_dev
  fix_map_yaml
  patch_nav2_params

  log "start lidar first, then auto-detect /scan frame_id"
  start_bg lidar bash "$PROJECT_DIR/scripts/lidar/start_lidar_only.sh"
  wait_topic_data /scan 90 || fatal "no /scan data; check lidar log: $LOG_DIR/lidar.log"

  if [ -z "$LASER_FRAME" ]; then
    LASER_FRAME="$(get_scan_frame)"
  fi
  export LASER_FRAME
  log "using LASER_FRAME=$LASER_FRAME"

  log "start scan filter: /scan -> /scan_filtered"
  start_bg scan_filter python3 "$PROJECT_DIR/ros2_bridge/simple_scan_filter.py" \
    --in-topic /scan \
    --out-topic /scan_filtered \
    --min-range 0.18 \
    --max-range "${SCAN_FILTER_MAX_RANGE:-4.0}" \
    --isolated-window "${SCAN_FILTER_ISOLATED_WINDOW:-2}" \
    --isolated-delta "${SCAN_FILTER_ISOLATED_DELTA:-0.25}" \
    --min-support-neighbors "${SCAN_FILTER_MIN_SUPPORT:-1}" \
    --stats-every 50
  wait_topic_data /scan_filtered 30 || fatal "no /scan_filtered; check $LOG_DIR/scan_filter.log"

  start_bg static_tf ros2 run tf2_ros static_transform_publisher \
  "$LASER_X" "$LASER_Y" "$LASER_Z" \
  "$LASER_YAW" "$LASER_PITCH" "$LASER_ROLL" \
  base_link "$LASER_FRAME"

  if [ -f "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh" ]; then
    # Save oneclick nav limits before mvp_tune (mapping profile) overwrites them.
    local _oc_max_vx="$CHASSIS_MAX_VX" _oc_max_wz="$CHASSIS_MAX_WZ" _oc_pwm_max="$CHASSIS_PWM_MAX"
    local _oc_vx_gain="$CHASSIS_VX_PWM_GAIN" _oc_wz_gain="$CHASSIS_WZ_PWM_GAIN" _oc_rate="$CHASSIS_CONTROL_RATE_HZ"
    set +u
    source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh" || true
    set -u
    export CHASSIS_MAX_VX="$_oc_max_vx" CHASSIS_MAX_WZ="$_oc_max_wz" CHASSIS_PWM_MAX="$_oc_pwm_max"
    export CHASSIS_VX_PWM_GAIN="$_oc_vx_gain" CHASSIS_WZ_PWM_GAIN="$_oc_wz_gain"
    export CHASSIS_CONTROL_RATE_HZ="$_oc_rate"
  fi
  export CHASSIS_VX_PWM_DEADBAND="${CHASSIS_VX_PWM_DEADBAND:-6.0}"
  export CHASSIS_WZ_PWM_DEADBAND="${CHASSIS_WZ_PWM_DEADBAND:-8.0}"
  log "chassis nav profile: max_vx=$CHASSIS_MAX_VX max_wz=$CHASSIS_MAX_WZ pwm_max=$CHASSIS_PWM_MAX"
  source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
  run_chassis_bridge "$LOG_DIR/chassis_bridge.log"

  if [ "$START_FOXGLOVE" = "1" ] && ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    start_bg foxglove ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
  fi

  wait_topic_data /odom 90 || fatal "no /odom data; check chassis bridge: $LOG_DIR/chassis_bridge.log"
  verify_chassis_motion || fatal "chassis not moving; see $LOG_DIR/chassis_bridge.log (wrong serial port is the usual cause)"
  wait_tf odom base_link 20 || warn "odom->base_link not ready yet; continuing for Nav2 startup"
  wait_tf base_link "$LASER_FRAME" 20 || fatal "base_link->$LASER_FRAME TF missing"

  if [ "$NAV2_ONECLICK_CMD_VEL_INVERT" = "1" ]; then
    _oneclick_launch="$(write_nav2_invert_bringup_launch)"
    log "launch Nav2 bringup (cmd_vel -> ${NAV2_ONECLICK_RAW_CMD_VEL}, invert relay during nav)"
    start_bg nav2 ros2 launch "$_oneclick_launch" \
      use_sim_time:=False \
      autostart:=True \
      map:="$MAP_YAML" \
      params_file:="$NAV2_PARAMS" \
      use_composition:=False
  else
    log "launch Nav2 bringup"
    start_bg nav2 ros2 launch nav2_bringup bringup_launch.py \
      use_sim_time:=False \
      autostart:=True \
      map:="$MAP_YAML" \
      params_file:="$NAV2_PARAMS" \
      use_composition:=False
  fi

  # Localization stack comes up first; navigation stack needs map->base_link TF.
  wait_lifecycle_active /map_server 120 || fatal "/map_server not active; see $LOG_DIR/nav2.log"
  wait_lifecycle_active /amcl 120 || fatal "/amcl not active; see $LOG_DIR/nav2.log"
  wait_map_topic 60 || fatal "no /map data; see $LOG_DIR/nav2.log"
  sleep 2

  # AMCL needs repeated /initialpose while DDS + laser TF settle.
  wait_for_localization "$INIT_X" "$INIT_Y" "$INIT_YAW" 90 \
    || fatal "AMCL localization failed (no map->base_link); check $LOG_DIR/nav2.log and laser TF"

  # After map TF exists, navigation lifecycle can finish activating.
  wait_lifecycle_active /controller_server 120 || fatal "/controller_server not active; see $LOG_DIR/nav2.log"
  wait_lifecycle_active /planner_server 120 || fatal "/planner_server not active; see $LOG_DIR/nav2.log"
  wait_lifecycle_active /bt_navigator 120 || fatal "/bt_navigator not active; see $LOG_DIR/nav2.log"

  if [ "$NAV2_ONECLICK_CMD_VEL_INVERT" = "1" ]; then
    start_cmd_vel_invert_relay
  fi

  if [ "$START_NAV2_PATH_VIZ" = "1" ]; then
    start_bg path_viz python3 "$PROJECT_DIR/ros2_bridge/nav2_plan_path_viz.py"
    log "Nav2 path viz started -> /nav2_plan_markers, /nav2_viz/global_plan, /nav2_viz/local_plan"
  fi

  if ! wait_topic_data /amcl_pose 15; then
    warn "/amcl_pose not echoed yet, but map->base_link TF is OK; continuing"
  fi

  print_status
  if [ "$START_FOXGLOVE" = "1" ]; then
    print_foxglove_help
  fi

  if [ "$AUTO_SEND_GOAL" = "1" ]; then
    set +e
    send_goal "$GOAL_X" "$GOAL_Y" "$GOAL_YAW"
    local nav_exit=$?
    set -e
    if [ "$nav_exit" -ne 0 ]; then
      warn "Nav2 action exit code=$nav_exit (see terminal feedback above)"
    fi
    verify_goal_reached "$GOAL_X" "$GOAL_Y" "$GOAL_YAW" \
      || fatal "goal verify failed: robot not at ($GOAL_X, $GOAL_Y) within tolerance"
    log "navigation finished (Nav2 exit=$nav_exit, goal verify OK)"
  else
    log "AUTO_SEND_GOAL=0, stack is ready. Send goal manually with scripts/slam/nav_goal.sh or ros2 action."
  fi

  local keep_running="${KEEP_RUNNING_AFTER_GOAL:-}"
  if [ -z "$keep_running" ]; then
    if [ "$AUTO_SEND_GOAL" = "1" ]; then
      keep_running=0
    else
      keep_running=1
    fi
  fi
  if [ "$keep_running" = "1" ]; then
    log "stack still running. Press Ctrl+C here to stop all Nav2/lidar/chassis processes."
    while true; do sleep 5; done
  fi
  log "done. cleaning up Nav2/lidar/chassis processes..."
}

main "$@"
