#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
[ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash || true
[ -f "$HOME/ydlidar_ws/install/setup.bash" ] && source "$HOME/ydlidar_ws/install/setup.bash" || true
set -u

X="${1:-0.5}"
Y="${2:-0.0}"

echo "[TEST] Publish single-click point to /foxglove_goal_point"
echo "[TEST] x=$X y=$Y"

ros2 topic pub --once /foxglove_goal_point geometry_msgs/msg/PointStamped "{
  header: {
    frame_id: 'map'
  },
  point: {
    x: $X,
    y: $Y,
    z: 0.0
  }
}"
