#!/usr/bin/env bash
set +e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
LOG_DIR="$PROJECT_DIR/logs"
PID_FILE="$LOG_DIR/red_servo_pids.txt"
TROS_SETUP="/opt/tros/humble/setup.bash"

echo "============================================================"
echo " Red Visual Servo 一键停止"
echo "============================================================"

if [ -f "$TROS_SETUP" ]; then
  source "$TROS_SETUP"
fi

echo "[INFO] 发布 0 速度停车命令..."

for i in 1 2 3; do
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  >/dev/null 2>&1 || true
  sleep 0.2
done

echo "[INFO] 根据 PID 文件停止进程..."

if [ -f "$PID_FILE" ]; then
  source "$PID_FILE"

  if [ -n "$SERVO_PID" ]; then
    kill "$SERVO_PID" 2>/dev/null || true
  fi

  if [ -n "$BRIDGE_PID" ]; then
    kill "$BRIDGE_PID" 2>/dev/null || true
  fi

  if [ -n "$CAMERA_PID" ]; then
    kill "$CAMERA_PID" 2>/dev/null || true
  fi
fi

sleep 1

echo "[INFO] 清理可能残留的相关进程..."

pkill -f "red_target_servo_ros.py" 2>/dev/null || true
pkill -f "red_target_servo_compressed_ros.py" 2>/dev/null || true
pkill -f "red_target_servo_auto_ros.py" 2>/dev/null || true
pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
pkill -f "hobot_usb_cam" 2>/dev/null || true

echo "[INFO] 再次发布 0 速度停车命令..."

for i in 1 2 3; do
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  >/dev/null 2>&1 || true
  sleep 0.2
done

rm -f "$PID_FILE"

echo "[OK] 已停止红色视觉伺服完整链路。"
echo "如果小车仍异常运动，请立即关闭小车电源。"
echo "============================================================"
