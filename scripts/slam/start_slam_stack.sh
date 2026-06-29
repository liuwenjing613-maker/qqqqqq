#!/usr/bin/env bash
# Live SLAM stack: /scan + /odom + TF + slam_toolbox /map + Foxglove :8765
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
source "${PROJECT_DIR}/scripts/lib/cleanup_lidar_slam_nav.sh"
source "${PROJECT_DIR}/scripts/lib/lidar_frame_config.sh"

LOG_DIR="${PROJECT_DIR}/logs/slam_stack"
MAP_DIR="${PROJECT_DIR}/maps"
SLAM_CONFIG="${PROJECT_DIR}/configs/slam_toolbox.yaml"
LIDAR_DEV="/dev/ydlidar"
CHASSIS_DEV="/dev/rosmaster"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

PIDS=()

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

kill_stack_processes() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  cleanup_lidar_slam_nav_processes
  pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
  pkill -f "ydlidar_ros2_driver_node" 2>/dev/null || true
}

cleanup() {
  echo ""
  log "cleanup: publishing zero /cmd_vel..."
  publish_zero_cmd
  log "cleanup: stopping SLAM stack..."
  kill_stack_processes
}

trap cleanup EXIT INT TERM

start_background() {
  local name="$1"
  shift
  local log_file="${LOG_DIR}/${name}.log"
  log "Starting ${name} -> ${log_file}"
  "$@" >"$log_file" 2>&1 &
  PIDS+=("$!")
  sleep 0.3
}

wait_for_topic_msgs() {
  local topic="$1"
  local timeout_sec="${2:-15}"
  python3 "${PROJECT_DIR}/scripts/lib/wait_ros_topic.py" \
    --topic "$topic" \
    --timeout "$timeout_sec" \
    --min-msgs 1
}

check_topic_hz() {
  local topic="$1"
  local label="$2"
  local out
  out="$(timeout 8 ros2 topic hz "$topic" 2>&1 | head -5 || true)"
  if echo "$out" | grep -q "average rate"; then
    log "OK ${label}: $(echo "$out" | grep 'average rate' | head -1)"
    return 0
  fi
  echo "FAIL: ${label} (${topic}) has no publish rate within 8s" >&2
  echo "$out" >&2
  return 1
}

check_tf_chain() {
  log "Checking TF odom -> laser ..."
  local tf_out
  tf_out="$(timeout 8 ros2 run tf2_ros tf2_echo odom laser 2>&1 | head -20 || true)"
  if echo "$tf_out" | grep -q "Translation"; then
    log "OK TF odom -> laser is available"
    return 0
  fi
  echo "FAIL: could not lookup transform odom -> laser" >&2
  echo "$tf_out" >&2
  return 1
}

preflight_checks() {
  log "Preflight checks..."
  if [ ! -e "$LIDAR_DEV" ]; then
    echo "FAIL: LiDAR device not found: $LIDAR_DEV" >&2
    exit 1
  fi
  if [ ! -e "$CHASSIS_DEV" ]; then
    echo "FAIL: chassis device not found: $CHASSIS_DEV" >&2
    exit 1
  fi
  if [ ! -f "$SLAM_CONFIG" ]; then
    echo "FAIL: SLAM config not found: $SLAM_CONFIG" >&2
    exit 1
  fi
  mkdir -p "$LOG_DIR" "$MAP_DIR"
  ls -l "$LIDAR_DEV" "$CHASSIS_DEV"
}

post_start_checks() {
  log "Post-start health checks..."
  wait_for_topic_msgs /scan 20
  wait_for_topic_msgs /scan_filtered 20
  wait_for_topic_msgs /odom 20
  check_topic_hz /scan "LiDAR /scan"
  check_topic_hz /scan_filtered "Filtered /scan_filtered"
  check_topic_hz /odom "Odometry /odom"
  check_tf_chain
  wait_for_topic_msgs /map 30
  log "OK /map is publishing"
}

main() {
  log "===== SLAM Stack (Foxglove live view) ====="
  log "PROJECT_DIR=$PROJECT_DIR"

  preflight_checks

  log "Stopping previous SLAM-related processes..."
  kill_stack_processes
  sleep 0.5

  log "[1/6] LiDAR driver -> /scan"
  start_background lidar bash "${PROJECT_DIR}/scripts/lidar/start_lidar_only.sh"
  sleep 3

  log "[2/6] Scan filter /scan -> /scan_filtered"
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

  log "[3/6] Chassis PWM bridge + /odom"
  source "${PROJECT_DIR}/scripts/lib/load_mvp_tune.sh"
  source "${PROJECT_DIR}/scripts/lib/run_chassis_bridge.sh"
  export CHASSIS_PORT="${CHASSIS_DEV}"
  run_chassis_bridge "${LOG_DIR}/chassis_bridge.log"
  sleep 2

  log "[4/6] Static TF base_link -> ${LASER_FRAME}"
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
  sleep 1

  log "[5/6] slam_toolbox -> /map"
  start_background slam_toolbox \
    ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:="${SLAM_CONFIG}"
  sleep 4

  log "[6/6] foxglove_bridge -> ws://<host>:${FOXGLOVE_PORT}"
  start_background foxglove \
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml "port:=${FOXGLOVE_PORT}"
  sleep 2

  post_start_checks

  log "===== SLAM stack ready ====="
  log "Topics: /scan  /odom  /map  /tf"
  log "Foxglove Studio: ws://$(hostname -I 2>/dev/null | awk '{print $1}'):${FOXGLOVE_PORT}"
  log "Logs: ${LOG_DIR}/"
  log "Manual drive: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ..."
  log "Save map: ros2 run nav2_map_server map_saver_cli -f ${MAP_DIR}/my_map"
  log "Press Ctrl+C to stop."

  while true; do sleep 3600; done
}

main "$@"
