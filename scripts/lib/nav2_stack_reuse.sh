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

nav2_servers_running() {
  pgrep -f "map_server|planner_server|controller_server|bt_navigator|amcl" >/dev/null 2>&1
}

stop_stale_nav2_servers_only() {
  echo "[NAV2_REUSE] stop stale Nav2 servers (keep lidar/chassis) ..."
  pkill -f "map_server|amcl|planner_server|controller_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|waypoint_follower|lifecycle_manager|nav2_click_nav_bringup_launch.py|run_nav2_saved_map.sh" 2>/dev/null || true
  sleep 1
}

_chassis_bridge_pids() {
  pgrep -f "m1_pwm_cmd_vel_bridge.py|cmd_vel_to_rosmaster.py" 2>/dev/null || true
}

_serial_holders() {
  local port="${1:-${CHASSIS_PORT:-${CHASSIS_DEV:-/dev/rosmaster}}}"
  if command -v fuser >/dev/null 2>&1; then
    fuser "$port" 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+$' || true
    return 0
  fi
  if command -v lsof >/dev/null 2>&1; then
    lsof -t "$port" 2>/dev/null || true
    return 0
  fi
  return 0
}

_cmd_vel_unexpected_publishers() {
  # Return non-empty lines for unexpected /cmd_vel publishers (teleop etc.)
  ros2 topic info /cmd_vel -v 2>/dev/null | awk '
    /Publisher count:/{next}
    /Node name:/{node=$3}
    /Topic type:/{next}
    /Endpoint type: PUBLISHER/{print node}
  ' | while read -r node; do
    case "$node" in
      ""|*chassis*|*pwm*|*rosmaster*|*velocity_smoother*|*controller_server*|*behavior_server*)
        ;;
      *teleop*|*joy*)
        echo "$node"
        ;;
      *)
        # treat other publishers as unexpected during handoff
        echo "$node"
        ;;
    esac
  done
}

check_fast_nav_reusable_stack() {
  local label="${1:-FAST_NAV}"
  local fail=0
  local bridge_pids bridge_count holders unexpected

  _pass() { echo "[$label] PASS $1"; }
  _fail() { echo "[$label] FAIL $1"; fail=1; }

  if topic_is_publishing /scan 1 8; then _pass "scan"; else _fail "scan"; fi
  if topic_is_publishing /scan_filtered 1 8; then _pass "scan_filtered"; else _fail "scan_filtered"; fi
  if topic_is_publishing /odom 1 8; then _pass "odom"; else _fail "odom"; fi
  if odom_base_link_tf_ready; then _pass "odom_base_link_tf"; else _fail "odom_base_link_tf"; fi
  if laser_static_tf_ready "${LASER_FRAME:-laser}"; then _pass "base_link_laser_tf"; else _fail "base_link_laser_tf"; fi

  if chassis_stack_ready; then
    _pass "chassis_stack_ready"
  else
    _fail "chassis_stack_ready"
  fi

  bridge_pids="$(_chassis_bridge_pids | tr '\n' ' ' | xargs || true)"
  bridge_count=0
  if [[ -n "${bridge_pids:-}" ]]; then
    bridge_count="$(echo "$bridge_pids" | wc -w | tr -d ' ')"
  fi
  if [[ "$bridge_count" -eq 1 ]]; then
    _pass "chassis_owner pid=${bridge_pids}"
  else
    _fail "chassis_owner count=${bridge_count} pids=${bridge_pids:-none}"
  fi

  holders="$(_serial_holders | tr '\n' ' ' | xargs || true)"
  if [[ -n "$holders" ]]; then
    # If tools available, require serial owned by the single bridge pid
    if [[ "$bridge_count" -eq 1 ]] && echo " $holders " | grep -q " ${bridge_pids} "; then
      _pass "chassis_serial_owner pid=${bridge_pids}"
    elif [[ "$bridge_count" -eq 1 ]]; then
      _fail "chassis_serial holders=${holders} expected_bridge=${bridge_pids}"
    else
      _fail "chassis_serial holders=${holders}"
    fi
  else
    _pass "chassis_serial (fuser/lsof unavailable or empty; skipped hard owner check)"
  fi

  if pgrep -f "teleop_twist_joy|joy_node" >/dev/null 2>&1; then
    _fail "teleop still running"
  else
    _pass "teleop stopped"
  fi

  unexpected="$(_cmd_vel_unexpected_publishers | tr '\n' ',' | sed 's/,$//')"
  if [[ -n "$unexpected" ]]; then
    _fail "cmd_vel unexpected publisher=${unexpected}"
  else
    _pass "cmd_vel"
  fi

  if slam_toolbox_running; then
    _fail "slam_toolbox still running"
  else
    _pass "slam_toolbox stopped"
  fi

  if nav2_servers_running; then
    _fail "old Nav2 nodes still present"
  else
    _pass "no_old_nav2"
  fi

  return "$fail"
}
