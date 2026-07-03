#!/usr/bin/env bash
# Detect Foxglove mouse clicks on /foxglove_goal_point (no navigation).
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
POINT_TOPIC="${POINT_TOPIC:-/foxglove_goal_point}"
POSE_TOPIC="${POSE_TOPIC:-/foxglove_goal_pose}"
TIMEOUT_SEC="${TIMEOUT_SEC:-0}"

set +u
source /opt/ros/humble/setup.bash
[ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash || true
[ -f "$HOME/ydlidar_ws/install/setup.bash" ] && source "$HOME/ydlidar_ws/install/setup.bash" || true
set -u

cd "$PROJECT_DIR"
chmod +x scripts/slam/test_foxglove_mouse_click_detect.py

BOARD_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"

echo "============================================================"
echo "[CLICK_DETECT] Foxglove mouse click detector"
echo "[CLICK_DETECT] Connect Foxglove: ws://${BOARD_IP}:8765"
echo "[CLICK_DETECT] Import layout: ${PROJECT_DIR}/configs/foxglove_click_goal_nav.layout.json"
echo "[CLICK_DETECT] Required Foxglove settings:"
echo "  Panel: 3D"
echo "  Fixed frame: map"
echo "  Publish tool: 2D point  (NOT Pan / Orbit)"
echo "  Topic: ${POINT_TOPIC}"
echo "  Type: geometry_msgs/msg/PointStamped"
echo ""
echo "[CLICK_DETECT] Wrong topic examples that will NOT work:"
echo "  xglove_goal_point, /clicked_point, /move_base_simple/goal"
echo "============================================================"

if ! pgrep -f "foxglove_bridge" >/dev/null 2>&1; then
  echo "[CLICK_DETECT] WARN: foxglove_bridge not running."
  echo "[CLICK_DETECT] Start nav stack first:"
  echo "  bash scripts/slam/run_nav2_foxglove_click_goal.sh"
  echo "[CLICK_DETECT] Or only foxglove:"
  echo "  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
fi

echo "[CLICK_DETECT] Waiting for clicks on ${POINT_TOPIC} ..."
echo "[CLICK_DETECT] Click the map in Foxglove now. Press Ctrl+C to stop."
echo ""

exec python3 "$PROJECT_DIR/scripts/slam/test_foxglove_mouse_click_detect.py" \
  --point-topic "$POINT_TOPIC" \
  --pose-topic "$POSE_TOPIC" \
  --timeout-sec "$TIMEOUT_SEC"
