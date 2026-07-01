#!/usr/bin/env bash
set -eo pipefail

# ROS setup scripts may reference unset variables internally.
# Do not enable "set -u" before sourcing ROS environments.
set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
set -u

X="${1:?usage: bash scripts/slam/set_initial_pose.sh X Y YAW_RAD}"
Y="${2:?usage: bash scripts/slam/set_initial_pose.sh X Y YAW_RAD}"
YAW="${3:?usage: bash scripts/slam/set_initial_pose.sh X Y YAW_RAD}"

read QZ QW < <(python3 - <<PY
import math
yaw = float("${YAW}")
print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))
PY
)

echo "[INITIAL_POSE] x=${X}, y=${Y}, yaw=${YAW}, qz=${QZ}, qw=${QW}"

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: map
pose:
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
  covariance:
    [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
"
