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
# 1 = keep stack alive after goal command returns; Ctrl+C to stop.
KEEP_RUNNING_AFTER_GOAL="${KEEP_RUNNING_AFTER_GOAL:-1}"
# 1 = also start Foxglove bridge if installed.
START_FOXGLOVE="${START_FOXGLOVE:-1}"
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
  pkill -f "static_transform_publisher.*base_link" >/dev/null 2>&1 || true
  sleep 2
}

select_chassis_dev() {
  if [ -n "${CHASSIS_DEV:-}" ]; then
    :
  elif [ -e /dev/ttyACM0 ]; then
    CHASSIS_DEV="/dev/ttyACM0"
  elif [ -e /dev/ttyUSB0 ]; then
    CHASSIS_DEV="/dev/ttyUSB0"
  else
    fatal "cannot find chassis device. Set CHASSIS_DEV=/dev/ttyUSB0 or /dev/ttyACM0"
  fi
  export CHASSIS_DEV CHASSIS_PORT="$CHASSIS_DEV"
  export CHASSIS_MAX_VX CHASSIS_MAX_WZ CHASSIS_PWM_MAX CHASSIS_VX_PWM_GAIN CHASSIS_WZ_PWM_GAIN CHASSIS_CONTROL_RATE_HZ
  log "CHASSIS_DEV=$CHASSIS_DEV max_vx=$CHASSIS_MAX_VX max_wz=$CHASSIS_MAX_WZ pwm_max=$CHASSIS_PWM_MAX"
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

  cp "$MAP_YAML" "$MAP_YAML.bak_$(date +%Y%m%d_%H%M%S)" || true
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
  cp "$NAV2_PARAMS" "$NAV2_PARAMS.bak_$(date +%Y%m%d_%H%M%S)" || true
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
local['width'] = float(local.get('width', 3.0) or 3.0)
local['height'] = float(local.get('height', 3.0) or 3.0)
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
amcl['transform_tolerance'] = float(amcl.get('transform_tolerance', 1.0) or 1.0)

# DWB speed profile: allow enough speed to overcome M1 chassis dead zone, but stay safe.
controller = ros_params('controller_server')
controller['controller_frequency'] = 20.0
controller['min_x_velocity_threshold'] = 0.01
controller['min_theta_velocity_threshold'] = 0.01
pc = controller.setdefault('progress_checker', {})
pc['plugin'] = pc.get('plugin', 'nav2_controller::SimpleProgressChecker')
pc['required_movement_radius'] = 0.05
pc['movement_time_allowance'] = 20.0

gc = controller.setdefault('general_goal_checker', {})
gc['stateful'] = True
gc['plugin'] = gc.get('plugin', 'nav2_controller::SimpleGoalChecker')
gc['xy_goal_tolerance'] = 0.15
gc['yaw_goal_tolerance'] = 0.35

fp = controller.setdefault('FollowPath', {})
fp['plugin'] = 'dwb_core::DWBLocalPlanner'
fp['min_vel_x'] = 0.0
fp['max_vel_x'] = 0.18
fp['max_vel_theta'] = 0.80
fp['min_speed_xy'] = 0.0
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
  local parent="$1" child="$2" timeout_sec="$3"
  log "wait TF: $parent -> $child"
  local start now
  start="$(date +%s)"
  while true; do
    if timeout 3 ros2 run tf2_ros tf2_echo "$parent" "$child" >/dev/null 2>&1; then
      log "TF OK: $parent -> $child"
      return 0
    fi
    now="$(date +%s)"
    if [ $((now - start)) -ge "$timeout_sec" ]; then
      warn "TF timeout: $parent -> $child"
      return 1
    fi
    sleep 1
  done
}

lifecycle_get() {
  timeout 3 ros2 lifecycle get "$1" 2>/dev/null || true
}

ensure_active() {
  local node="$1"
  local state
  state="$(lifecycle_get "$node")"
  log "$node state: ${state:-NO RESPONSE}"
  if echo "$state" | grep -q "active"; then
    return 0
  fi
  if echo "$state" | grep -q "unconfigured"; then
    timeout 8 ros2 lifecycle set "$node" configure || true
    sleep 0.5
  fi
  state="$(lifecycle_get "$node")"
  if echo "$state" | grep -q "inactive"; then
    timeout 8 ros2 lifecycle set "$node" activate || true
    sleep 0.5
  fi
  state="$(lifecycle_get "$node")"
  log "$node after ensure: ${state:-NO RESPONSE}"
  echo "$state" | grep -q "active"
}

publish_initial_pose() {
  local x="$1" y="$2" yaw="$3"
  log "publish initial pose: x=$x y=$y yaw=$yaw"
  python3 - "$x" "$y" "$yaw" <<'PY'
import math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
x, y, yaw = map(float, sys.argv[1:4])
rclpy.init()
node = Node('nav2_oneclick_initial_pose')
pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = 'map'
msg.pose.pose.position.x = x
msg.pose.pose.position.y = y
msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
msg.pose.covariance = [0.0] * 36
msg.pose.covariance[0] = 0.25
msg.pose.covariance[7] = 0.25
msg.pose.covariance[35] = 0.0685
for i in range(8):
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.15)
node.destroy_node()
rclpy.shutdown()
PY
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
  log "send NavigateToPose goal: x=$x y=$y yaw=$yaw qz=$qz qw=$qw"
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

  start_bg static_tf ros2 run tf2_ros static_transform_publisher \
    --x "$LASER_X" --y "$LASER_Y" --z "$LASER_Z" \
    --roll "$LASER_ROLL" --pitch "$LASER_PITCH" --yaw "$LASER_YAW" \
    --frame-id base_link \
    --child-frame-id "$LASER_FRAME"

  if [ -f "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh" ]; then
    # May define tuning variables; safe if absent.
    set +u
    source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh" || true
    set -u
  fi
  source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
  run_chassis_bridge "$LOG_DIR/chassis_bridge.log"

  if [ "$START_FOXGLOVE" = "1" ] && ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    start_bg foxglove ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
  fi

  wait_topic_data /odom 90 || fatal "no /odom data; check chassis bridge: $LOG_DIR/chassis_bridge.log"
  wait_tf odom base_link 20 || warn "odom->base_link not ready yet; continuing for Nav2 startup"
  wait_tf base_link "$LASER_FRAME" 20 || fatal "base_link->$LASER_FRAME TF missing"

  log "launch Nav2 bringup"
  start_bg nav2 ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=False \
    autostart:=True \
    map:="$MAP_YAML" \
    params_file:="$NAV2_PARAMS" \
    use_composition:=False

  sleep 8
  ensure_active /map_server || fatal "/map_server failed to become active; see $LOG_DIR/nav2.log"
  ensure_active /amcl || fatal "/amcl failed to become active; see $LOG_DIR/nav2.log"
  ensure_active /controller_server || warn "/controller_server is not active"
  ensure_active /planner_server || warn "/planner_server is not active"
  ensure_active /bt_navigator || warn "/bt_navigator is not active"

  wait_topic_data /map 30 "--qos-durability transient_local" || fatal "no /map data; map_server active but map not published"

  publish_initial_pose "$INIT_X" "$INIT_Y" "$INIT_YAW"
  wait_topic_data /amcl_pose 20 || fatal "no /amcl_pose after initialpose; check /map /scan /odom and TF"
  wait_tf map base_link 20 || fatal "map->base_link missing after AMCL pose"

  print_status

  if [ "$AUTO_SEND_GOAL" = "1" ]; then
    send_goal "$GOAL_X" "$GOAL_Y" "$GOAL_YAW"
  else
    log "AUTO_SEND_GOAL=0, stack is ready. Send goal manually with scripts/slam/nav_goal.sh or ros2 action."
  fi

  if [ "$KEEP_RUNNING_AFTER_GOAL" = "1" ]; then
    log "stack still running. Press Ctrl+C here to stop all Nav2/lidar/chassis processes."
    while true; do sleep 5; done
  fi
}

main "$@"
