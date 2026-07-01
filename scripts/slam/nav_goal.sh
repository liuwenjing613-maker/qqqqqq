#!/usr/bin/env bash
set -eo pipefail

# ROS setup scripts may reference unset variables internally.
set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
set -u

X="${1:?usage: bash scripts/slam/nav_goal.sh X Y YAW_RAD}"
Y="${2:?usage: bash scripts/slam/nav_goal.sh X Y YAW_RAD}"
YAW="${3:-0.0}"

read QZ QW < <(python3 - <<PY
import math
yaw = float("${YAW}")
print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))
PY
)

echo "[NAV_GOAL] x=${X}, y=${Y}, yaw=${YAW}, qz=${QZ}, qw=${QW}"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: map
  pose:
    position:
      x: ${X}
      y: ${Y}
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: ${QZ}
      w: ${QW}
behavior_tree: ''
" --feedback
