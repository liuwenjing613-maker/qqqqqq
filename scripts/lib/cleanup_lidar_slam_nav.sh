#!/usr/bin/env bash
# Stop stale LiDAR/SLAM/Nav helper processes before starting a fresh stack.

foxglove_port_listening() {
  local port="${1:-8765}"
  if command -v ss >/dev/null 2>&1; then
    ss -tln 2>/dev/null | grep -q ":${port} "
    return $?
  fi
  if command -v netstat >/dev/null 2>&1; then
    netstat -tln 2>/dev/null | grep -q ":${port} "
    return $?
  fi
  return 1
}

# Free ws://*:8765 so mapping/nav Foxglove serves /scan_filtered from THIS stack.
ensure_foxglove_port_free() {
  local port="${1:-8765}"
  local timeout_sec="${2:-10}"
  local waited=0

  if ! foxglove_port_listening "$port"; then
    return 0
  fi

  echo "[cleanup] WARN: port ${port} in use; stopping stale foxglove_bridge..."
  pkill -f "foxglove_bridge" 2>/dev/null || true
  sleep 1
  pkill -9 -f "foxglove_bridge" 2>/dev/null || true

  while foxglove_port_listening "$port" && [ "$waited" -lt "$timeout_sec" ]; do
    sleep 1
    waited=$((waited + 1))
  done

  if foxglove_port_listening "$port"; then
    echo "[cleanup] ERROR: port ${port} still busy after ${timeout_sec}s"
    echo "[cleanup] HINT: stop nav2/semantic/mapping Foxglove owner, then retry"
    ss -tlnp 2>/dev/null | grep ":${port} " || true
    return 1
  fi

  echo "[cleanup] OK: port ${port} is free"
  return 0
}

foxglove_bridge_log_looks_healthy() {
  local log_file="${1:-}"
  if [ -z "$log_file" ] || [ ! -f "$log_file" ]; then
    return 1
  fi
  if grep -q "Bind Error" "$log_file" 2>/dev/null; then
    return 1
  fi
  if grep -q "process has died" "$log_file" 2>/dev/null; then
    return 1
  fi
  if grep -q "Server listening on port" "$log_file" 2>/dev/null; then
    return 0
  fi
  return 1
}

cleanup_lidar_slam_nav_processes() {
  pkill -f "slam_toolbox" 2>/dev/null || true
  pkill -f "static_transform_publisher.*base_link.*laser" 2>/dev/null || true
  pkill -f "simple_scan_filter.py" 2>/dev/null || true
  pkill -f "foxglove_bridge" 2>/dev/null || true
  sleep 1
}

cleanup_stale_nav2_processes() {
  for pattern in \
    "nav2_bringup bringup_launch.py" \
    "planner_server" \
    "controller_server" \
    "bt_navigator" \
    "behavior_server" \
    "velocity_smoother" \
    "smoother_server" \
    "waypoint_follower" \
    "lifecycle_manager" \
    "map_server" \
    "amcl"
  do
    pkill -f "$pattern" 2>/dev/null || true
  done
}

cleanup_ros2_fastrtps_shm() {
  # Leftover Fast DDS shared-memory segments cause port lock failures and slow Nav2 bringup.
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
}

# Kill processes matching pattern but never the caller shell or its parent.
_click_nav_safe_pkill() {
  local pattern="$1"
  local pid self_pids
  self_pids=" $$ ${PPID:-} "
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    if [[ "$self_pids" == *" $pid "* ]]; then
      continue
    fi
    kill "$pid" 2>/dev/null || true
  done
}

_click_nav_safe_pkill_wait() {
  local pattern="$1"
  local wait_sec="${2:-2}"
  _click_nav_safe_pkill "$pattern"
  sleep "$wait_sec"
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    if [[ " $$ ${PPID:-} " == *" $pid "* ]]; then
      continue
    fi
    kill -9 "$pid" 2>/dev/null || true
  done
}

# Full teardown for click-nav / saved-map Nav2 (used by Ctrl+C and stop script).
cleanup_click_nav_stack_processes() {
  local label="${1:-CLEANUP}"
  local log_fn="${2:-echo}"

  "$log_fn" "[$label] zero /cmd_vel..."
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
  sleep 0.2
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true

  local pattern
  for pattern in \
    "foxglove_click_goal_bridge.py" \
    "run_nav2_saved_map.sh" \
    "nav2_bringup bringup_launch.py" \
    "pose_memory_node.py" \
    "simple_scan_filter.py" \
    "foxglove_bridge" \
    "ydlidar_ros2_driver" \
    "start_lidar_only.sh" \
    "static_transform_publisher.*base_link.*laser" \
    "m1_pwm_cmd_vel_bridge.py" \
    "cmd_vel_to_rosmaster.py" \
    "run_chassis_bridge.sh" \
    "planner_server" \
    "controller_server" \
    "bt_navigator" \
    "behavior_server" \
    "velocity_smoother" \
    "smoother_server" \
    "waypoint_follower" \
    "lifecycle_manager" \
    "map_server" \
    "amcl" \
    "global_costmap" \
    "local_costmap" \
    "slam_toolbox" \
    "teleop_twist_joy" \
    "joy_node"
  do
    if pgrep -f "$pattern" >/dev/null 2>&1; then
      "$log_fn" "[$label] stop: $pattern"
      _click_nav_safe_pkill_wait "$pattern" 1
    fi
  done

  cleanup_ros2_fastrtps_shm
  sleep 1

  local remaining
  remaining="$(pgrep -af 'nav2_bringup|foxglove_click|pose_memory|run_nav2_saved_map|m1_pwm|ydlidar|simple_scan_filter|controller_server|planner_server|bt_navigator|amcl|map_server|lifecycle_manager' 2>/dev/null \
    | grep -v "pgrep -af" \
    | grep -v " $$ " || true)"
  if [ -n "$remaining" ]; then
    "$log_fn" "[$label] WARN: some processes still running:"
    echo "$remaining" | while IFS= read -r line; do
      "$log_fn" "[$label]   $line"
    done
  else
    "$log_fn" "[$label] all click-nav processes stopped"
  fi
}
