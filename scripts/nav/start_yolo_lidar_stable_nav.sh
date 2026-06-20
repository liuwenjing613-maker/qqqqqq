#!/usr/bin/env bash
# 稳定版 YOLO + LiDAR 导航启动脚本（连续控制 + kick start + 可观测底盘桥）
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

export CHASSIS_PORT="${CHASSIS_PORT:-/dev/ttyUSB1}"
export STABLE_NAV=1

echo "===== Stable YOLO + LiDAR Navigation ====="
echo "CHASSIS_PORT=$CHASSIS_PORT (override: CHASSIS_PORT=/dev/myserial ...)"

bash "$PROJECT_DIR/scripts/nav/start_yolo_lidar_failsafe_nav.sh" "$@"

sleep 3

echo ""
echo "[stable] post-start checks..."

if ros2 topic info /cmd_vel 2>/dev/null | grep -q "Subscription count"; then
  ros2 topic info /cmd_vel 2>/dev/null | grep -E "Publisher|Subscription" || true
fi

if timeout 3 ros2 topic echo /cmd_vel_sent --once >/dev/null 2>&1; then
  echo "  /cmd_vel_sent OK"
else
  echo "  WARN: /cmd_vel_sent not publishing yet (check logs/yolo_failsafe_chassis.log)"
fi

if timeout 3 ros2 topic echo /chassis_bridge_state --once >/dev/null 2>&1; then
  echo "  /chassis_bridge_state OK"
else
  echo "  WARN: /chassis_bridge_state not publishing yet"
fi

echo ""
echo "Foxglove 必看 topic:"
echo "  /scan  /failsafe_nav/markers  /failsafe_nav/debug_image"
echo "  /failsafe_nav_state  /chassis_bridge_state"
echo "  Plot: /cmd_vel.linear.x  /cmd_vel_sent.linear.x"
echo "  Plot: /cmd_vel.angular.z  /cmd_vel_sent.angular.z"
echo ""
echo "Chassis log: tail -f logs/yolo_failsafe_chassis.log"
echo "Nav log:     tail -f logs/yolo_lidar_failsafe_nav.log"
echo "Test guide:  docs/STABLE_YOLO_LIDAR_NAV_TEST.md"
