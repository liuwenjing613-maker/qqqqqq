#!/usr/bin/env bash
# Live corridor SLAM stack:
# LiDAR + odom + static TF + slam_toolbox + optional Foxglove.
# Does NOT auto-drive the robot. Manual /cmd_vel only.

set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
source "${PROJECT_DIR}/scripts/lib/cleanup_lidar_slam_nav.sh"
source "${PROJECT_DIR}/scripts/lib/lidar_frame_config.sh"

LOG_DIR="${PROJECT_DIR}/logs/slam_live"
SLAM_CONFIG="${PROJECT_DIR}/configs/slam_toolbox.yaml"
LIDAR_DEV="/dev/ydlidar"
CHASSIS_DEV="/dev/rosmaster"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

PIDS=()
FOXGLOVE_STARTED=0

set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f "${HOME}/ydlidar_ws/install/setup.bash" ]; then
  source "${HOME}/ydlidar_ws/install/setup.bash"
fi
set -u

log() {
  echo "[$(date +%H:%M:%S)] $*"
}

publish_zero_cmd() {
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
}

cleanup() {
  echo ""
  log "[cleanup] publishing zero /cmd_vel..."
  publish_zero_cmd

  log "[cleanup] stopping processes started by this script..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  pkill -f "async_slam_toolbox_node" 2>/dev/null || true
  pkill -f "sync_slam_toolbox_node" 2>/dev/null || true
  pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
  cleanup_lidar_slam_nav_processes
  pkill -f "ydlidar_ros2_driver_node" 2>/dev/null || true
  pkill -f "start_lidar_only.sh" 2>/dev/null || true
}

# Do not trap INT: when started under setsid from run_joy_mapping_all.sh,
# parent Ctrl+C should not tear down slam_toolbox before map save.
trap cleanup EXIT TERM

ensure_slam_config() {
  if [ -f "$SLAM_CONFIG" ]; then
    log "SLAM config OK: $SLAM_CONFIG"
    return 0
  fi

  mkdir -p "$(dirname "$SLAM_CONFIG")"

  cat > "$SLAM_CONFIG" <<'YAML'
slam_toolbox:
  ros__parameters:
    use_sim_time: false

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan_filtered

    mode: mapping
    resolution: 0.05
    max_laser_range: 4.0

    minimum_time_interval: 0.1
    transform_timeout: 1.0
    tf_buffer_duration: 60.0
    map_update_interval: 1.0
    throttle_scans: 1
    transform_publish_period: 0.02

    debug_logging: false
    enable_interactive_mode: true
    stack_size_to_use: 40000000
YAML

  log "SLAM config ready: $SLAM_CONFIG"
}

start_background() {
  local name="$1"
  shift
  local log_file="${LOG_DIR}/${name}.log"

  log "Starting ${name} -> ${log_file}"
  "$@" > "$log_file" 2>&1 &
  PIDS+=("$!")
  sleep 0.5
}

wait_topic_exists() {
  local topic="$1"
  local timeout_sec="${2:-60}"

  log "Waiting for ${topic} ..."
  for _ in $(seq 1 "$timeout_sec"); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      log "OK: ${topic}"
      return 0
    fi
    sleep 1
  done

  log "FAIL: timeout waiting for ${topic}"
  return 1
}

show_status() {
  echo ""
  echo "========== ROS TOPICS =========="
  ros2 topic list | sort | egrep "scan|odom|tf|map|cmd_vel|joy|chassis" || true

  echo ""
  echo "========== /scan info =========="
  ros2 topic info /scan -v 2>/dev/null | head -40 || true

  echo ""
  echo "========== /odom info =========="
  ros2 topic info /odom -v 2>/dev/null | head -40 || true

  echo ""
  echo "========== /map_metadata once =========="
  timeout 5 ros2 topic echo /map_metadata --once 2>/dev/null | head -25 || true

  echo ""
  echo "========== TF odom -> laser =========="
  timeout 8 ros2 run tf2_ros tf2_echo odom laser 2>/dev/null | head -25 || true

  echo ""
  echo "========== Logs if something is missing =========="
  echo "LiDAR log:         ${LOG_DIR}/lidar.log"
  echo "Scan filter log:   ${LOG_DIR}/scan_filter.log"
  echo "Chassis log:       ${LOG_DIR}/chassis_bridge.log"
  echo "SLAM log:          ${LOG_DIR}/slam_toolbox.log"
  echo "Foxglove log:      ${LOG_DIR}/foxglove_bridge.log"
}

main() {
  log "===== Corridor SLAM Live (Foxglove, known-good style) ====="
  log "PROJECT_DIR=$PROJECT_DIR"
  log "No auto motion. Drive manually via joystick /cmd_vel."

  mkdir -p "$LOG_DIR"

  if [ ! -e "$LIDAR_DEV" ]; then
    echo "FAIL: missing $LIDAR_DEV" >&2
    ls -l /dev/ydlidar /dev/rosmaster /dev/ttyUSB* 2>/dev/null || true
    exit 1
  fi

  if [ ! -e "$CHASSIS_DEV" ]; then
    echo "FAIL: missing $CHASSIS_DEV" >&2
    ls -l /dev/ydlidar /dev/rosmaster /dev/ttyUSB* 2>/dev/null || true
    exit 1
  fi

  ensure_slam_config

  log "Stopping old SLAM-related processes..."
  publish_zero_cmd
  cleanup_lidar_slam_nav_processes
  pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
  pkill -f "ydlidar_ros2_driver_node" 2>/dev/null || true
  pkill -f "start_lidar_only.sh" 2>/dev/null || true
  sleep 1

  log "[1/6] LiDAR -> /scan"
  start_background lidar bash "${PROJECT_DIR}/scripts/lidar/start_lidar_only.sh"
  sleep 6

  wait_topic_exists /scan 30 || {
    log "FAIL: /scan not found"
    exit 1
  }

  log "[scan_filter] Start /scan -> /scan_filtered"
  start_background scan_filter python3 "${PROJECT_DIR}/ros2_bridge/simple_scan_filter.py" \
    --in-topic /scan \
    --out-topic /scan_filtered \
    --min-range 0.18 \
    --max-range "${SCAN_FILTER_MAX_RANGE:-4.0}" \
    --isolated-window "${SCAN_FILTER_ISOLATED_WINDOW:-2}" \
    --isolated-delta "${SCAN_FILTER_ISOLATED_DELTA:-0.25}" \
    --min-support-neighbors "${SCAN_FILTER_MIN_SUPPORT:-1}" \
    --stats-every 50
  sleep 2

  wait_topic_exists /scan_filtered 30 || {
    log "FAIL: /scan_filtered not found"
    exit 1
  }

  log "[2/6] Chassis PWM bridge + /odom"
  source "${PROJECT_DIR}/scripts/lib/load_mvp_tune.sh"
  source "${PROJECT_DIR}/scripts/lib/run_chassis_bridge.sh"
  export CHASSIS_PORT="${CHASSIS_DEV}"
  run_chassis_bridge "${LOG_DIR}/chassis_bridge.log"
  sleep 4

  log "[3/6] Static TF base_link -> ${LASER_FRAME}"
  start_background static_tf \
    ros2 run tf2_ros static_transform_publisher \
    --x "${LASER_X}" \
    --y "${LASER_Y}" \
    --z "${LASER_Z}" \
    --roll "${LASER_ROLL}" \
    --pitch "${LASER_PITCH}" \
    --yaw "${LASER_YAW}" \
    --frame-id base_link \
    --child-frame-id "${LASER_FRAME}"
  sleep 2

  log "[4/6] slam_toolbox online_async (scan_topic=/scan_filtered)"
  start_background slam_toolbox \
    ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:="${SLAM_CONFIG}"
  sleep 8

  if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    log "[5/6] foxglove_bridge port ${FOXGLOVE_PORT}"
    start_background foxglove_bridge \
      ros2 launch foxglove_bridge foxglove_bridge_launch.xml "port:=${FOXGLOVE_PORT}"
    FOXGLOVE_STARTED=1
    sleep 3
  else
    log "[5/6] foxglove_bridge not installed, skipping"
  fi

  show_status

  log "===== SLAM live stack started ====="
  log "Do NOT close this terminal."
  if [ "$FOXGLOVE_STARTED" = "1" ]; then
    BOARD_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
    log "Foxglove: ws://${BOARD_IP}:${FOXGLOVE_PORT}"
  fi
  log "Now open terminal 2: joy_node."
  log "Then terminal 3: teleop_twist_joy."
  log "Terminal 4: monitoring and map saving."

  while true; do
    sleep 3600
  done
}

main "$@"
