#!/usr/bin/env bash
# One-shot live joystick mapping:
# Start SLAM/Foxglove + joy_node + teleop.
# Drive manually with joystick.
# Press Ctrl+C to save map and stop.

set -u

cd ~/rdk_x5_vln_robot

LOG_DIR="$PWD/logs/joy_mapping"
MAP_DIR="$PWD/maps"
MAP_NAME="${MAP_NAME:-joy_corridor_map}"

mkdir -p "$LOG_DIR" "$MAP_DIR"

PIDS=()
SAVED=0

source_ros() {
  # ROS setup.bash may reference unset variables such as AMENT_TRACE_SETUP_FILES.
  # Temporarily disable nounset while sourcing ROS environments.
  set +u

  if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
  elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
  fi

  if [ -f "$HOME/ydlidar_ws/install/setup.bash" ]; then
    source "$HOME/ydlidar_ws/install/setup.bash"
  fi

  set -u
}
log() {
  echo "[$(date +%H:%M:%S)] $*"
}

zero_cmd() {
  source_ros
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
}

kill_children() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done

  pkill -f "teleop_twist_joy" 2>/dev/null || true
  pkill -f "joy_node" 2>/dev/null || true
  pkill -f "game_controller_node" 2>/dev/null || true
  pkill -f "cmd_vel_kickstart.py" 2>/dev/null || true
  pkill -f "cmd_vel_pulse_crawl.py" 2>/dev/null || true
}

save_map() {
  if [ "$SAVED" = "1" ]; then
    return 0
  fi

  source_ros
  zero_cmd
  sleep 1

  log "Saving map to ${MAP_DIR}/${MAP_NAME} ..."

  if ros2 topic list | grep -qx "/map"; then
    ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}" \
      > "${LOG_DIR}/map_saver.log" 2>&1

    if [ -f "${MAP_DIR}/${MAP_NAME}.yaml" ] && [ -f "${MAP_DIR}/${MAP_NAME}.pgm" ]; then
      log "Map saved:"
      ls -lh "${MAP_DIR}/${MAP_NAME}.yaml" "${MAP_DIR}/${MAP_NAME}.pgm"
      SAVED=1
    else
      log "WARN: map_saver_cli ran but map file not found. See ${LOG_DIR}/map_saver.log"
      tail -80 "${LOG_DIR}/map_saver.log" 2>/dev/null || true
    fi
  else
    log "WARN: /map does not exist, skip map saving."
  fi
}

cleanup() {
  echo
  log "Ctrl+C/exit detected: stop robot, save map, cleanup..."
  zero_cmd
  save_map
  kill_children
  log "Done."
}

trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"
  shift

  log "Starting ${name} ..."
  "$@" > "${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
  sleep 1
}

wait_topic_exists() {
  local topic="$1"
  local timeout_sec="${2:-60}"

  log "Waiting for ${topic} ..."
  for _ in $(seq 1 "$timeout_sec"); do
    if ros2 topic list | grep -qx "$topic"; then
      log "OK: ${topic}"
      return 0
    fi
    sleep 1
  done

  log "WARN: timeout waiting for ${topic}"
  return 1
}

show_status() {
  echo
  echo "========== ROS topics =========="
  ros2 topic list | sort | egrep "joy|cmd_vel|scan|odom|tf|map|chassis" || true

  echo
  echo "========== /cmd_vel info =========="
  ros2 topic info /cmd_vel -v 2>/dev/null | head -80 || true

  echo
  echo "========== Foxglove =========="
  local ip
  ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo "Connect in Foxglove: ws://${ip}:8765"

  echo
  echo "========== How to use =========="
  echo "1. Open Foxglove and connect to ws://${ip}:8765"
  echo "2. In 3D panel: 固定参考系=map"
  echo "3. Enable /map, /scan, /tf, /tf_static, /odom"
  echo "4. Drive slowly with joystick"
  echo "5. Press Ctrl+C in this terminal to save map and stop"
  echo
}

main() {
  source_ros

  log "===== One-click joystick SLAM mapping ====="
  log "Project: $PWD"
  log "Map output: ${MAP_DIR}/${MAP_NAME}.yaml/.pgm"

  log "Stopping old joystick/control nodes first..."
  zero_cmd
  pkill -f "teleop_twist_joy" 2>/dev/null || true
  pkill -f "joy_node" 2>/dev/null || true
  pkill -f "game_controller_node" 2>/dev/null || true
  pkill -f "cmd_vel_kickstart.py" 2>/dev/null || true
  pkill -f "cmd_vel_pulse_crawl.py" 2>/dev/null || true
  sleep 1

  log "[1/3] Start SLAM + Foxglove live stack"
  start_bg live_stack bash scripts/slam/run_corridor_mapping_live_foxglove.sh

  wait_topic_exists /scan 90 || true
  wait_topic_exists /odom 90 || true
  wait_topic_exists /map 90 || true
  wait_topic_exists /tf 40 || true

  log "[2/3] Start joystick /joy"
  start_bg joy_node ros2 run joy joy_node --ros-args \
    -p dev:=/dev/input/js0 \
    -p deadzone:=0.08 \
    -p autorepeat_rate:=20.0

  wait_topic_exists /joy 30 || true

  log "[3/3] Start teleop /joy -> /cmd_vel"
  start_bg teleop ros2 run teleop_twist_joy teleop_node --ros-args \
    -p require_enable_button:=false \
    -p axis_linear.x:=1 \
    -p scale_linear.x:=0.025 \
    -p axis_angular.yaw:=0 \
    -p scale_angular.yaw:=0.12

  sleep 2
  show_status

  log "System is running. Do NOT close this terminal."
  log "Press Ctrl+C when you want to save map and stop."

  while true; do
    sleep 3600
  done
}

main "$@"
