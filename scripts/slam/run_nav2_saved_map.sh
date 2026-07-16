#!/usr/bin/env bash
set -Eeo pipefail

# ROS setup.bash may read unset variables; disable nounset while sourcing.
set +u
source /opt/ros/humble/setup.bash
[ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
set -u

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
source "${PROJECT_DIR}/scripts/lib/project_dir.sh"
source "${PROJECT_DIR}/scripts/lib/cleanup_lidar_slam_nav.sh"
source "${PROJECT_DIR}/scripts/lib/lidar_frame_config.sh"
source "${PROJECT_DIR}/scripts/lib/nav2_stack_reuse.sh"
source "${PROJECT_DIR}/scripts/lib/nav2_localization_bootstrap.sh"
export_ros_dds_env
cleanup_ros2_fastrtps_shm
# 与 calibrated 建图一致：底盘口 /dev/rosmaster + odom 校准参数
if [ -f "${PROJECT_DIR}/scripts/lib/slam_calibrated_env.sh" ]; then
  # shellcheck source=scripts/lib/slam_calibrated_env.sh
  source "${PROJECT_DIR}/scripts/lib/slam_calibrated_env.sh"
fi
# Nav-only PWM smoothing: more responsive steps for closed-loop follow.
export CHASSIS_PWM_SMOOTH_ALPHA="${CHASSIS_NAV_PWM_SMOOTH_ALPHA:-0.35}"
export CHASSIS_MAX_PWM_DELTA="${CHASSIS_NAV_MAX_PWM_DELTA:-3.0}"
# Use calibrated PWM (mvp_tune breakaway ~0.055 m/s at vx_db=10 gain=200).
export CHASSIS_NAV_VX_PWM_DEADBAND="${CHASSIS_NAV_VX_PWM_DEADBAND:-${CHASSIS_VX_PWM_DEADBAND:-10.0}}"
export CHASSIS_NAV_WZ_PWM_DEADBAND="${CHASSIS_NAV_WZ_PWM_DEADBAND:-${CHASSIS_WZ_PWM_DEADBAND:-10.0}}"
export CHASSIS_NAV_VX_PWM_GAIN="${CHASSIS_NAV_VX_PWM_GAIN:-${CHASSIS_VX_PWM_GAIN:-200.0}}"
export CHASSIS_NAV_PWM_MAX="${CHASSIS_NAV_PWM_MAX:-${CHASSIS_PWM_MAX:-40.0}}"
export CHASSIS_NAV_CMD_WZ_DEADZONE="${CHASSIS_NAV_CMD_WZ_DEADZONE:-0.015}"
export CHASSIS_NAV_CONTROL_RATE_HZ="${CHASSIS_NAV_CONTROL_RATE_HZ:-10.0}"
# Nav2 velocity limits must match chassis; keep calibrated caps after mvp_tune load.
_NAV_CHASSIS_MAX_VX="${CHASSIS_MAX_VX:-0.06}"
_NAV_CHASSIS_MAX_WZ="${CHASSIS_MAX_WZ:-0.24}"
_NAV_CHASSIS_MOTOR_TRIMS="${CHASSIS_MOTOR_TRIMS:-1.15,1.15,1.0,1.0}"

MAP_YAML="${MAP_YAML:-$PROJECT_DIR/maps/joy_calibrated_corridor_map.yaml}"
NAV2_PARAMS="${NAV2_PARAMS:-$PROJECT_DIR/configs/nav2_params.yaml}"
MVP_TUNE="${MVP_TUNE:-$PROJECT_DIR/configs/mvp_tune.yaml}"
NAV2_STOP_CONFLICTS="${NAV2_STOP_CONFLICTS:-0}"
NAV2_REUSE_EXISTING="${NAV2_REUSE_EXISTING:-1}"

# 自动判断底盘串口（slam_calibrated_env 已设 CHASSIS_DEV 时优先沿用）
if [ -n "${CHASSIS_DEV:-}" ] && [ -e "${CHASSIS_DEV}" ]; then
  :
elif [ -e /dev/rosmaster ]; then
  CHASSIS_DEV="/dev/rosmaster"
elif [ -e /dev/ttyACM0 ]; then
  CHASSIS_DEV="/dev/ttyACM0"
elif [ -e /dev/ttyUSB0 ]; then
  CHASSIS_DEV="/dev/ttyUSB0"
else
  echo "[NAV2] ERROR: cannot find chassis serial device. Set CHASSIS_DEV=/dev/xxx"
  exit 1
fi

LOG_DIR="$PROJECT_DIR/logs/nav2_$(date +%Y%m%d_%H%M%S)"
STATE_DIR="$PROJECT_DIR/state"
POSE_STATE_FILE="${POSE_STATE_FILE:-$STATE_DIR/last_pose_map.json}"
mkdir -p "$LOG_DIR" "$STATE_DIR"

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
  timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
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
    # ros2 daemon 在 SLAM→Nav2 切换后 topic list 常滞后；用 hz 确认真实发布。
    if topic_is_publishing "$topic"; then
      log "topic OK (publishing): $topic"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_sec" ]; then
      log "ERROR: topic not found: $topic"
      return 1
    fi
    sleep 1
  done
}

wait_topic_publishing() {
  local topic="$1"
  local timeout_sec="$2"
  local start now hits=0
  start="$(date +%s)"
  log "wait topic publishing: $topic (timeout=${timeout_sec}s) ..."
  while true; do
    if topic_is_publishing "$topic"; then
      hits=$((hits + 1))
      if (( hits >= 2 )); then
        log "topic publishing OK: $topic"
        return 0
      fi
    else
      hits=0
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_sec )); then
      log "ERROR: topic not publishing: $topic"
      return 1
    fi
    sleep 2
  done
}

ensure_bg_process_alive() {
  local name="$1"
  local pid="$2"
  local logfile="$3"
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
    log "ERROR: $name exited during startup (pid=${pid:-none})"
    if [[ -f "$logfile" ]]; then
      tail -20 "$logfile" 2>/dev/null || true
    fi
    return 1
  fi
  return 0
}

wait_initialpose_subscriber() {
  local timeout_sec="${1:-60}"
  local start
  start="$(date +%s)"
  while true; do
    local sub_count
    sub_count="$(ros2 topic info /initialpose -v 2>/dev/null | awk '/Subscription count:/{print $3; exit}')"
    if [ "${sub_count:-0}" -ge 1 ]; then
      log "initialpose subscriber OK (count=${sub_count})"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_sec" ]; then
      log "WARN: no /initialpose subscriber yet; pose_memory will publish anyway"
      return 0
    fi
    sleep 1
  done
}

if [ ! -f "$MAP_YAML" ] && [ -f "$PROJECT_DIR/maps/joy_corridor_map.yaml" ]; then
  log "WARN: MAP_YAML not found, fallback to joy_corridor_map.yaml"
  MAP_YAML="$PROJECT_DIR/maps/joy_corridor_map.yaml"
fi

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
log "POSE_STATE_FILE=$POSE_STATE_FILE"
log "NAV2_STOP_CONFLICTS=$NAV2_STOP_CONFLICTS NAV2_REUSE_EXISTING=$NAV2_REUSE_EXISTING"
log "logs=$LOG_DIR"

if slam_toolbox_running; then
  log "WARN: slam_toolbox still running; stopping SLAM before saved-map Nav2."
  pkill -f "async_slam_toolbox_node|sync_slam_toolbox_node|slam_toolbox online_async_launch.py" 2>/dev/null || true
  sleep 2
fi

if [ "$NAV2_STOP_CONFLICTS" = "1" ]; then
  log "NAV2_STOP_CONFLICTS=1: stopping mapping/joystick/semantic conflicts..."
  pkill -f "teleop_twist_joy|joy_node|run_joy_mapping_all|run_joy_mapping_calibrated|run_corridor_mapping_live_foxglove|run_slam_calibrated" 2>/dev/null || true
  pkill -f "run_shared_nav_semantic_explore|semantic_mapper_node|explore_goal_selector" 2>/dev/null || true
  sleep 1
  pkill -9 -f "run_shared_nav_semantic_explore|semantic_mapper_node|explore_goal_selector" 2>/dev/null || true
  cleanup_stale_nav2_processes
  cleanup_lidar_slam_nav_processes
  cleanup_ros2_fastrtps_shm
  timeout 5 ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 1
else
  log "NAV2_STOP_CONFLICTS=0: skip external pkill; reuse sensors when already running"
fi
zero_cmd

# 1. 启动雷达（若已有 /scan 则复用）
if [ "$NAV2_REUSE_EXISTING" = "1" ] && topic_is_publishing /scan; then
  log "reuse existing /scan publisher (skip lidar start)"
elif [ -x "$PROJECT_DIR/scripts/lidar/start_lidar_only.sh" ]; then
  start_bg lidar bash "$PROJECT_DIR/scripts/lidar/start_lidar_only.sh"
  LIDAR_PID="${PIDS[-1]}"
  sleep 6
  ensure_bg_process_alive "lidar" "$LIDAR_PID" "$LOG_DIR/lidar.log" || exit 1
else
  log "ERROR: lidar script not found or not executable: $PROJECT_DIR/scripts/lidar/start_lidar_only.sh"
  exit 1
fi

# 2. 启动 scan filter -> /scan_filtered
if [ "$NAV2_REUSE_EXISTING" = "1" ] && topic_is_publishing /scan_filtered; then
  log "reuse existing /scan_filtered publisher (skip scan_filter start)"
else
  start_bg scan_filter python3 "${PROJECT_DIR}/ros2_bridge/simple_scan_filter.py" \
    --in-topic /scan \
    --out-topic /scan_filtered \
    --min-range "${SCAN_FILTER_MIN_RANGE:-0.22}" \
    --max-range "${SCAN_FILTER_MAX_RANGE:-4.0}" \
    --isolated-window "${SCAN_FILTER_ISOLATED_WINDOW:-2}" \
    --isolated-delta "${SCAN_FILTER_ISOLATED_DELTA:-0.25}" \
    --min-support-neighbors "${SCAN_FILTER_MIN_SUPPORT:-1}" \
    --stats-every 50
  SCAN_FILTER_PID="${PIDS[-1]}"
  sleep 6
  ensure_bg_process_alive "scan_filter" "$SCAN_FILTER_PID" "$LOG_DIR/scan_filter.log" || exit 1
fi

# 3. 启动 base_link -> laser 静态 TF
if [ "$NAV2_REUSE_EXISTING" = "1" ] && laser_static_tf_ready "${LASER_FRAME}"; then
  log "reuse existing base_link->${LASER_FRAME} TF (skip static_tf start)"
else
  start_bg static_tf ros2 run tf2_ros static_transform_publisher \
    --x "${LASER_X}" --y "${LASER_Y}" --z "${LASER_Z}" \
    --roll "${LASER_ROLL}" --pitch "${LASER_PITCH}" --yaw "${LASER_YAW}" \
    --frame-id base_link \
    --child-frame-id "${LASER_FRAME}"
fi

# 4. 启动 PWM 底盘桥：必须能发布 odom->base_link TF
source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"
export CHASSIS_MAX_VX="${_NAV_CHASSIS_MAX_VX}"
export CHASSIS_MAX_WZ="${_NAV_CHASSIS_MAX_WZ}"
export CHASSIS_MOTOR_TRIMS="${_NAV_CHASSIS_MOTOR_TRIMS}"
export CHASSIS_VX_PWM_DEADBAND="${CHASSIS_NAV_VX_PWM_DEADBAND}"
export CHASSIS_WZ_PWM_DEADBAND="${CHASSIS_NAV_WZ_PWM_DEADBAND}"
export CHASSIS_VX_PWM_GAIN="${CHASSIS_NAV_VX_PWM_GAIN}"
export CHASSIS_PWM_MAX="${CHASSIS_NAV_PWM_MAX}"
export CHASSIS_CMD_WZ_DEADZONE="${CHASSIS_NAV_CMD_WZ_DEADZONE}"
export CHASSIS_CMD_VX_DEADZONE="${CHASSIS_NAV_CMD_VX_DEADZONE:-0.0}"
export CHASSIS_CONTROL_RATE_HZ="${CHASSIS_NAV_CONTROL_RATE_HZ}"
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
export CHASSIS_PORT="$CHASSIS_DEV"
export CHASSIS_REUSE_IF_RUNNING=0
if [ "$NAV2_REUSE_EXISTING" = "1" ] && odom_base_link_tf_ready; then
  export CHASSIS_REUSE_IF_RUNNING=1
  log "reuse existing chassis bridge (odom->base_link TF OK)"
else
  log "start fresh chassis bridge (need odom->base_link TF)"
  pkill -f "m1_pwm_cmd_vel_bridge.py|cmd_vel_to_rosmaster.py" 2>/dev/null || true
  sleep 1
fi
run_chassis_bridge "$LOG_DIR/chassis_bridge.log"
sleep 3
if grep -q "motor_trims=\[1.0, 1.0, 1.0, 1.0\]" "$LOG_DIR/chassis_bridge.log" 2>/dev/null; then
  log "WARN: chassis using default motor_trims 1.0; expected ${CHASSIS_MOTOR_TRIMS}"
fi

# 5. Foxglove 可视化
if [ "$NAV2_REUSE_EXISTING" = "1" ] && foxglove_bridge_running; then
  log "reuse existing foxglove_bridge (skip start)"
elif ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  start_bg foxglove ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
else
  log "WARN: foxglove_bridge not found, skip foxglove."
fi

# 6. 等基础 topic（必须真实在发数据，不能只信 topic list）
wait_topic_publishing /scan 120 || exit 1
wait_topic_publishing /scan_filtered 120 || exit 1
wait_topic_publishing /odom 90 || exit 1
wait_topic_exists /tf 40 || exit 1

if ! wait_odom_base_link_tf 60; then
  log "ERROR: odom->base_link TF not ready; check chassis bridge on ${CHASSIS_DEV}"
  exit 1
fi
log "TF OK: odom -> base_link"

# 7. 启动 Nav2（后台），完成 AMCL 定位后再启动 pose_memory
# Custom bringup: behavior_server cmd_vel -> cmd_vel_nav (avoids 5-way /cmd_vel conflict).
NAV2_BRINGUP_LAUNCH="${NAV2_BRINGUP_LAUNCH:-$PROJECT_DIR/configs/nav2_click_nav_bringup_launch.py}"
log "launch Nav2 (bringup=$NAV2_BRINGUP_LAUNCH)..."
start_bg nav2 ros2 launch "$NAV2_BRINGUP_LAUNCH" \
  use_sim_time:=False \
  autostart:=True \
  map:="$MAP_YAML" \
  params_file:="$NAV2_PARAMS" \
  use_composition:=False

sleep 15
if ! wait_map_topic_data 120; then
  log "WARN: /map 未收到，刷新 ros2 daemon 后重试 ..."
  ros2 daemon stop 2>/dev/null || true
  sleep 2
  ros2 daemon start 2>/dev/null || true
  sleep 2
  wait_map_topic_data 120 || exit 1
fi
wait_lifecycle_active /amcl 90 || exit 1
sleep 3

if ! wait_topic_publishing /scan_filtered 45; then
  log "WARN: /scan_filtered 未稳定发布，重启 scan_filter ..."
  pkill -f "simple_scan_filter.py" 2>/dev/null || true
  sleep 1
  start_bg scan_filter python3 "${PROJECT_DIR}/ros2_bridge/simple_scan_filter.py" \
    --in-topic /scan \
    --out-topic /scan_filtered \
    --min-range "${SCAN_FILTER_MIN_RANGE:-0.22}" \
    --max-range "${SCAN_FILTER_MAX_RANGE:-4.0}" \
    --isolated-window "${SCAN_FILTER_ISOLATED_WINDOW:-2}" \
    --isolated-delta "${SCAN_FILTER_ISOLATED_DELTA:-0.25}" \
    --min-support-neighbors "${SCAN_FILTER_MIN_SUPPORT:-1}" \
    --stats-every 50
  SCAN_FILTER_PID="${PIDS[-1]}"
  sleep 6
  ensure_bg_process_alive "scan_filter" "$SCAN_FILTER_PID" "$LOG_DIR/scan_filter.log" || exit 1
  wait_topic_publishing /scan_filtered 60 || exit 1
fi

print_pose_state_summary "$POSE_STATE_FILE" || true

if ! bootstrap_amcl_from_state_file "$POSE_STATE_FILE" 120; then
  log "WARN: AMCL bootstrap from $POSE_STATE_FILE failed."
  log "Set initial pose in Foxglove: Publish -> 2D Pose estimate -> /initialpose"
  log "Align laser scan with map walls, then click-nav goals."
  if ! wait_map_base_link_tf 60; then
    log "ERROR: map->base_link still missing after bootstrap"
    log "HINT: saved pose may be stale; use /initialpose in Foxglove then restart."
    exit 1
  fi
else
  log "TF OK: map -> base_link (AMCL localized)"
fi
if ! wait_amcl_localization_settle 60; then
  log "WARN: AMCL not fully settled; align scan/map in Foxglove before click-nav."
  log "Use Publish -> 2D Pose estimate -> /initialpose if walls do not match scan."
fi

if ! wait_nav_actions_ready 180; then
  cleanup_ros2_fastrtps_shm
  sleep 2
  retry_navigation_bringup 90 || exit 1
fi
log "Nav2 navigation stack active"

touch "$LOG_DIR/ready"
log "READY file: $LOG_DIR/ready"

start_bg pose_memory python3 "$PROJECT_DIR/scripts/slam/pose_memory_node.py" \
  --state-file "$POSE_STATE_FILE" \
  --map-frame map \
  --base-frame base_link \
  --fallback-base-frame base_footprint \
  --save-period 1.0 \
  --initial-delay 5.0 \
  --initial-interval 0.5

wait
