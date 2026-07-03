#!/usr/bin/env bash
# Stop click-navigation / saved-map Nav2 stack and SLAM conflicts (project scope only).
set -u

log() { echo "[STOP_NAV] $*"; }

log "zero /cmd_vel..."
timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  >/dev/null 2>&1 || true

for pattern in \
  "run_nav2_foxglove_click_goal.sh" \
  "run_nav2_saved_map.sh" \
  "nav2_bringup bringup_launch.py" \
  "foxglove_click_goal_bridge.py" \
  "pose_memory_node.py" \
  "run_joy_mapping_calibrated.sh" \
  "run_joy_mapping_all.sh" \
  "run_slam_calibrated.sh" \
  "run_corridor_mapping_live_foxglove.sh" \
  "async_slam_toolbox_node" \
  "sync_slam_toolbox_node" \
  "slam_toolbox online_async_launch.py" \
  "teleop_twist_joy" \
  "joy_node" \
  "simple_scan_filter.py" \
  "foxglove_bridge" \
  "ydlidar_ros2_driver" \
  "m1_pwm_cmd_vel_bridge.py" \
  "cmd_vel_to_rosmaster.py" \
  "planner_server" \
  "amcl" \
  "bt_navigator" \
  "controller_server" \
  "lifecycle_manager" \
  "map_server" \
  "behavior_server" \
  "velocity_smoother" \
  "waypoint_follower" \
  "smoother_server" \
  "global_costmap" \
  "local_costmap"
do
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    log "pkill: $pattern"
    pkill -f "$pattern" 2>/dev/null || true
  fi
done

sleep 2
log "done. Remaining robot-related processes:"
pgrep -af 'slam_toolbox|nav2_bringup|foxglove_click|pose_memory|m1_pwm|ydlidar' 2>/dev/null || log "(none)"
