#!/usr/bin/env bash
# 仅停止 failsafe 导航节点；相机/雷达/YOLO/底盘桥/Foxglove 保持运行。
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

set +u
source /opt/tros/humble/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
set -u

echo "[STOP] publish zero /cmd_vel..."
timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  >/dev/null 2>&1 || true

echo "[STOP] stop failsafe nav node only..."
pkill -f run_yolo_lidar_failsafe_nav.py 2>/dev/null || true
sleep 0.5

if pgrep -f run_yolo_lidar_failsafe_nav.py >/dev/null 2>&1; then
  echo "[WARN] nav node still running"
  exit 1
fi

echo "[STOP] done (sensors/YOLO/chassis/foxglove still running)."
