#!/usr/bin/env bash
# Helpers for run_nav2_saved_map.sh: reuse live topics/processes without killing unrelated stacks.

_ROS_TOPIC_PROBE="${PROJECT_DIR:-/root/rdk_x5_vln_robot}/scripts/lib/ros_topic_probe.py"
_LIDAR_DRIVER_LOG="${PROJECT_DIR:-/root/rdk_x5_vln_robot}/logs/lidar_driver.log"

_topic_probe_qos_flags() {
  case "$1" in
    /scan|/scan_filtered) echo --sensor-qos ;;
    *) echo --reliable ;;
  esac
}

topic_is_publishing() {
  local topic="$1"
  local min_samples="${2:-1}"
  local timeout_sec="${3:-10}"
  local qos_flags
  qos_flags="$(_topic_probe_qos_flags "$topic")"
  python3 "$_ROS_TOPIC_PROBE" has-samples "$topic" "$min_samples" "$timeout_sec" $qos_flags >/dev/null 2>&1
}

wait_topic_publishing() {
  local topic="$1"
  local timeout_sec="$2"
  local min_hits="${3:-2}"
  local min_samples="${4:-1}"
  local start now hits=0
  local qos_flags
  qos_flags="$(_topic_probe_qos_flags "$topic")"
  start="$(date +%s)"
  echo "[NAV2] wait topic publishing: $topic (timeout=${timeout_sec}s, rclpy) ..."
  while true; do
    if python3 "$_ROS_TOPIC_PROBE" has-samples "$topic" "$min_samples" 12 $qos_flags; then
      hits=$((hits + 1))
      if (( hits >= min_hits )); then
        echo "[NAV2] topic publishing OK: $topic"
        return 0
      fi
    else
      hits=0
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_sec )); then
      echo "[NAV2] ERROR: topic not publishing: $topic"
      return 1
    fi
    sleep 1
  done
}

scan_driver_alive() {
  pgrep -f "ydlidar_ros2_driver_node" >/dev/null 2>&1
}

scan_filter_running() {
  pgrep -f "simple_scan_filter.py" >/dev/null 2>&1
}

scan_stack_reusable() {
  if ! scan_driver_alive; then
    return 1
  fi
  topic_is_publishing /scan 1 12
}

wait_lidar_driver_scanning() {
  local timeout_sec="${1:-90}"
  local logfile="${2:-$_LIDAR_DRIVER_LOG}"
  local start
  start="$(date +%s)"
  echo "[NAV2] wait lidar driver log: Now lidar is scanning (timeout=${timeout_sec}s) ..."
  while true; do
    if [[ -f "$logfile" ]] && grep -q "Now lidar is scanning" "$logfile" 2>/dev/null; then
      echo "[NAV2] lidar driver scanning OK"
      return 0
    fi
    if scan_driver_alive && topic_is_publishing /scan 1 8; then
      echo "[NAV2] lidar driver scanning OK (rclpy /scan)"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_sec" ]; then
      echo "[NAV2] WARN: lidar driver log not ready after ${timeout_sec}s"
      return 1
    fi
    sleep 1
  done
}

restart_lidar_driver() {
  local project_dir="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
  echo "[NAV2] restart ydlidar driver (stale DDS publisher recovery) ..."
  pkill -f "ydlidar_ros2_driver_node|start_lidar_only.sh" 2>/dev/null || true
  sleep 4
  if [ -x "$project_dir/scripts/lidar/start_lidar_only.sh" ]; then
    bash "$project_dir/scripts/lidar/start_lidar_only.sh" &
    local pid=$!
    sleep 4
    wait_lidar_driver_scanning 90 "$project_dir/logs/lidar_driver.log" || true
    if kill -0 "$pid" 2>/dev/null || scan_driver_alive; then
      return 0
    fi
  fi
  return 1
}

ensure_scan_publishing() {
  local timeout_sec="${1:-120}"
  if wait_topic_publishing /scan "$timeout_sec"; then
    return 0
  fi
  if scan_driver_alive; then
    echo "[NAV2] WARN: ydlidar alive but /scan silent; restarting driver once"
    restart_lidar_driver || true
    wait_topic_publishing /scan "$timeout_sec"
    return $?
  fi
  return 1
}

laser_static_tf_ready() {
  local laser_frame="$1"
  timeout 2 ros2 run tf2_ros tf2_echo base_link "$laser_frame" 2>/dev/null \
    | head -5 | grep -q "Translation"
}

chassis_stack_ready() {
  if ros2 topic list 2>/dev/null | grep -qx /chassis_bridge_state; then
    return 0
  fi
  if pgrep -f "m1_pwm_cmd_vel_bridge.py|cmd_vel_to_rosmaster.py" >/dev/null 2>&1 \
    && topic_is_publishing /odom 1 8; then
    return 0
  fi
  return 1
}

foxglove_bridge_running() {
  pgrep -f "foxglove_bridge" >/dev/null 2>&1
}

slam_toolbox_running() {
  pgrep -f "slam_toolbox" >/dev/null 2>&1
}

odom_base_link_tf_ready() {
  timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null \
    | head -8 | grep -q "Translation:" \
    || topic_is_publishing /odom 1 8
}

map_base_link_tf_ready() {
  timeout 4 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null \
    | head -10 | grep -q "Translation:" \
    || timeout 4 ros2 topic echo /tf --once 2>/dev/null \
    | awk '/frame_id: map/{m=1} m && /child_frame_id: base_link/{exit 0} END{exit 1}'
}

wait_map_base_link_tf() {
  local timeout_sec="${1:-45}"
  local start
  start="$(date +%s)"
  while true; do
    if map_base_link_tf_ready; then
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_sec" ]; then
      return 1
    fi
    sleep 1
  done
}
