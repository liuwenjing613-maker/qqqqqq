#!/usr/bin/env bash
set -Eeo pipefail

# ROS setup.bash may read unset variables; disable nounset while sourcing.
set +u
source /opt/ros/humble/setup.bash
[ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
set -u

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
MAP_YAML="${MAP_YAML:-$PROJECT_DIR/maps/joy_corridor_map.yaml}"
NAV2_PARAMS="${NAV2_PARAMS:-$PROJECT_DIR/configs/nav2_params.yaml}"
MVP_TUNE="${MVP_TUNE:-$PROJECT_DIR/configs/mvp_tune.yaml}"

# 如果你的雷达 frame_id 不是 laser，运行脚本时用：
# LASER_FRAME=xxx bash scripts/slam/run_nav2_saved_map.sh
LASER_FRAME="${LASER_FRAME:-laser}"

# 雷达相对 base_link 的安装位置，后续需要按实物校准
LASER_X="${LASER_X:-0.10}"
LASER_Y="${LASER_Y:-0.0}"
LASER_Z="${LASER_Z:-0.12}"
LASER_YAW="${LASER_YAW:-0.0}"

# 自动判断底盘串口
if [ -n "${CHASSIS_DEV:-}" ]; then
  :
elif [ -e /dev/ttyACM0 ]; then
  CHASSIS_DEV="/dev/ttyACM0"
elif [ -e /dev/ttyUSB0 ]; then
  CHASSIS_DEV="/dev/ttyUSB0"
else
  echo "[NAV2] ERROR: cannot find chassis serial device. Set CHASSIS_DEV=/dev/xxx"
  exit 1
fi

LOG_DIR="$PROJECT_DIR/logs/nav2_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

PIDS=()

log() {
  echo "[NAV2] $*"
}

start_bg() {
  local name="$1"
  shift
  log "start $name: $*"
  "$@" > "$LOG_DIR/${name}.log" 2>&1 &
  PIDS+=("$!")
}

zero_cmd() {
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
}

cleanup() {
  log "cleanup..."
  zero_cmd
  sleep 0.2
  zero_cmd
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" >/dev/null 2>&1 || true
  done
}
trap cleanup INT TERM EXIT

wait_topic_exists() {
  local topic="$1"
  local timeout_sec="$2"
  local start
  start="$(date +%s)"
  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      log "topic OK: $topic"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_sec" ]; then
      log "ERROR: topic not found: $topic"
      return 1
    fi
    sleep 1
  done
}

if [ ! -f "$MAP_YAML" ]; then
  log "ERROR: map yaml not found: $MAP_YAML"
  log "先建图并保存地图，或者用 MAP_YAML=/path/to/map.yaml 指定地图。"
  exit 1
fi

if [ ! -f "$NAV2_PARAMS" ]; then
  log "ERROR: nav2 params not found: $NAV2_PARAMS"
  exit 1
fi

log "MAP_YAML=$MAP_YAML"
log "NAV2_PARAMS=$NAV2_PARAMS"
log "CHASSIS_DEV=$CHASSIS_DEV"
log "LASER_FRAME=$LASER_FRAME"
log "logs=$LOG_DIR"

# 避免建图/手柄节点和导航抢 /cmd_vel 或 map->odom
pkill -f "slam_toolbox|teleop_twist_joy|joy_node|run_joy_mapping_all|run_corridor_mapping_live_foxglove" 2>/dev/null || true
sleep 1
zero_cmd

# 1. 启动雷达
if [ -x "$PROJECT_DIR/scripts/lidar/start_lidar_only.sh" ]; then
  start_bg lidar bash "$PROJECT_DIR/scripts/lidar/start_lidar_only.sh"
else
  log "ERROR: lidar script not found or not executable: $PROJECT_DIR/scripts/lidar/start_lidar_only.sh"
  exit 1
fi

# 2. 启动 base_link -> laser 静态 TF
start_bg static_tf ros2 run tf2_ros static_transform_publisher \
  --x "$LASER_X" --y "$LASER_Y" --z "$LASER_Z" \
  --roll 0.0 --pitch 0.0 --yaw "$LASER_YAW" \
  --frame-id base_link \
  --child-frame-id "$LASER_FRAME"

# 3. 启动 PWM 底盘桥：订阅 /cmd_vel（无 /odom）
source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
export CHASSIS_PORT="$CHASSIS_DEV"
run_chassis_bridge "$LOG_DIR/chassis_bridge.log"

# 4. Foxglove 可视化
if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  start_bg foxglove ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
else
  log "WARN: foxglove_bridge not found, skip foxglove."
fi

# 5. 等基础 topic
wait_topic_exists /scan 90 || exit 1
wait_topic_exists /odom 90 || exit 1
wait_topic_exists /tf 40 || exit 1

# 6. 启动 Nav2
log "launch Nav2..."
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  autostart:=True \
  map:="$MAP_YAML" \
  params_file:="$NAV2_PARAMS" \
  use_composition:=False
