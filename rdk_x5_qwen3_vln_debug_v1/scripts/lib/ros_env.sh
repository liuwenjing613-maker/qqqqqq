#!/usr/bin/env bash
set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "[ERROR] ROS2 environment not found" >&2
  return 1 2>/dev/null || exit 1
fi
if [ -f "$HOME/ydlidar_ws/install/setup.bash" ]; then
  source "$HOME/ydlidar_ws/install/setup.bash"
fi
set -u
