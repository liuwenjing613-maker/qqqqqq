#!/usr/bin/env bash
# Helpers for run_nav2_saved_map.sh: reuse live topics/processes without killing unrelated stacks.

topic_is_publishing() {
  local topic="$1"
  timeout 2.5 ros2 topic hz "$topic" 2>/dev/null | grep -q "average rate"
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
    && topic_is_publishing /odom; then
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
    || timeout 8 ros2 topic hz /odom --window 5 2>/dev/null | grep -q "average rate"
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
