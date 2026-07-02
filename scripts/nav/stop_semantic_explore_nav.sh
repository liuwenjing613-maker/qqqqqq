#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
set -u

echo "[stop_semantic_explore_nav] stopping explore stack..."

timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
  >/dev/null 2>&1 || true

pkill -f run_shared_nav_semantic_explore.py 2>/dev/null || true
pkill -f explore_goal_selector.py 2>/dev/null || true
pkill -f "semantic_mapper_node.py.*semantic_explore" 2>/dev/null || true
pkill -f semantic_mapper_node.py 2>/dev/null || true

echo "[stop_semantic_explore_nav] done (SLAM/foxglove left running)"
