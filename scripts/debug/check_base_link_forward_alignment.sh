#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"

set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
set -u

echo "[CHECK] This script does not move the robot and does not kill unrelated processes."
echo "[CHECK] odom -> base_link:"
timeout 3 ros2 run tf2_ros tf2_echo odom base_link || true

echo ""
echo "[MANUAL TEST]"
echo "1. Open Foxglove."
echo "2. Set Fixed Frame = map or odom."
echo "3. Show TF axes."
echo "4. Confirm base_link red +X axis points to the real car front."
echo "5. If real car front is left but base_link +X points up, try:"
echo "   CHASSIS_BASE_YAW_OFFSET=1.5708"
echo "6. If it points the opposite way, try:"
echo "   CHASSIS_BASE_YAW_OFFSET=-1.5708"
echo ""
echo "[NAV2 REQUIREMENT]"
echo "base_link +X must equal physical forward direction."
