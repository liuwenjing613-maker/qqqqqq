#!/usr/bin/env bash
set -e

# ============================================================
# 一键启动红色视觉伺服完整链路
#
# 启动内容：
# 1. USB 摄像头节点 hobot_usb_cam，发布 /image
# 2. 底盘桥 cmd_vel_to_rosmaster.py，订阅 /cmd_vel
# 3. 红色视觉伺服 red_target_servo_auto_ros.py，订阅 /image，发布 /cmd_vel
#
# 默认参数：
# CAMERA_DEV=/dev/video0
# CHASSIS_PORT=/dev/ttyUSB0
# IMAGE_TOPIC=/image
#
# 如果你的设备不同，可以这样运行：
# CAMERA_DEV=/dev/video8 CHASSIS_PORT=/dev/ttyUSB0 ./scripts/start_red_servo.sh
# CAMERA_DEV=/dev/video0 CHASSIS_PORT=/dev/myserial ./scripts/start_red_servo.sh
# ============================================================

PROJECT_DIR="${PROJECT_DIR:-$HOME/rdk_x5_vln_robot}"
CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
CHASSIS_PORT="${CHASSIS_PORT:-/dev/ttyUSB0}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/image}"

BRIDGE_MAX_VX="${BRIDGE_MAX_VX:-0.06}"
BRIDGE_MAX_WZ="${BRIDGE_MAX_WZ:-0.18}"

KICK_VX="${KICK_VX:-0.11}"
KICK_DURATION="${KICK_DURATION:-0.22}"
MIN_DRIVE_VX="${MIN_DRIVE_VX:-0.055}"
KICK_MAX_WZ="${KICK_MAX_WZ:-0.12}"

SERVO_MAX_VX="${SERVO_MAX_VX:-0.04}"
SERVO_MAX_WZ="${SERVO_MAX_WZ:-0.08}"
KP_TURN="${KP_TURN:-0.1}"
CENTER_THRESHOLD="${CENTER_THRESHOLD:-0.22}"
ARRIVE_AREA_RATIO="${ARRIVE_AREA_RATIO:-0.30}"

LOG_DIR="$PROJECT_DIR/logs"
PID_FILE="$LOG_DIR/red_servo_pids.txt"

CAMERA_LOG="$LOG_DIR/red_servo_camera.log"
BRIDGE_LOG="$LOG_DIR/red_servo_bridge.log"
SERVO_LOG="$LOG_DIR/red_servo_servo.log"

TROS_SETUP="/opt/tros/humble/setup.bash"

echo "============================================================"
echo " Red Visual Servo 一键启动"
echo "============================================================"
echo "PROJECT_DIR         = $PROJECT_DIR"
echo "CAMERA_DEV          = $CAMERA_DEV"
echo "CHASSIS_PORT        = $CHASSIS_PORT"
echo "IMAGE_TOPIC         = $IMAGE_TOPIC"
echo "BRIDGE_MAX_VX       = $BRIDGE_MAX_VX"
echo "BRIDGE_MAX_WZ       = $BRIDGE_MAX_WZ"
echo "SERVO_MAX_VX        = $SERVO_MAX_VX"
echo "SERVO_MAX_WZ        = $SERVO_MAX_WZ"
echo "KP_TURN             = $KP_TURN"
echo "CENTER_THRESHOLD    = $CENTER_THRESHOLD"
echo "ARRIVE_AREA_RATIO   = $ARRIVE_AREA_RATIO"
echo "============================================================"

mkdir -p "$LOG_DIR"
mkdir -p "$PROJECT_DIR/data/images/red_servo_auto_debug"

# ---------- 基础检查 ----------
if [ ! -f "$TROS_SETUP" ]; then
  echo "[ERROR] 找不到 $TROS_SETUP"
  echo "请确认阶段 3 的 TROS/ROS2 环境已经安装。"
  exit 1
fi

if [ ! -e "$CAMERA_DEV" ]; then
  echo "[ERROR] 找不到相机设备：$CAMERA_DEV"
  echo "请先执行："
  echo "  ls /dev/video*"
  echo "  v4l2-ctl --list-devices"
  exit 1
fi

if [ ! -e "$CHASSIS_PORT" ]; then
  echo "[ERROR] 找不到底盘串口：$CHASSIS_PORT"
  echo "请先执行："
  echo "  ls -l /dev/myserial"
  echo "  ls -l /dev/ttyUSB*"
  echo "如果你实际是 /dev/myserial，请这样运行："
  echo "  CHASSIS_PORT=/dev/myserial ./scripts/start_red_servo.sh"
  exit 1
fi

if [ ! -f "$PROJECT_DIR/ros2_bridge/cmd_vel_to_rosmaster.py" ]; then
  echo "[ERROR] 找不到底盘桥脚本：$PROJECT_DIR/ros2_bridge/cmd_vel_to_rosmaster.py"
  exit 1
fi

if [ ! -f "$PROJECT_DIR/visual_servo/red_target_servo_auto_ros.py" ]; then
  echo "[ERROR] 找不到视觉伺服脚本：$PROJECT_DIR/visual_servo/red_target_servo_auto_ros.py"
  exit 1
fi

source "$TROS_SETUP"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 命令不可用，请检查 TROS 环境。"
  exit 1
fi

# ---------- 清理旧节点 ----------
echo "[INFO] 清理旧的红色视觉伺服相关进程..."

pkill -f "red_target_servo_ros.py" 2>/dev/null || true
pkill -f "red_target_servo_compressed_ros.py" 2>/dev/null || true
pkill -f "red_target_servo_auto_ros.py" 2>/dev/null || true
pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
pkill -f "hobot_usb_cam" 2>/dev/null || true

sleep 1

# ---------- 发布停车命令，防止旧速度残留 ----------
echo "[INFO] 尝试发布 0 速度停车命令，最多等待 2 秒..."
timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
>/dev/null 2>&1 || true

# ---------- 启动相机 ----------
echo "[INFO] 启动相机节点 hobot_usb_cam..."
echo "[INFO] 日志：$CAMERA_LOG"

bash -lc "
source $TROS_SETUP
cd $PROJECT_DIR/perception
ros2 launch $PROJECT_DIR/perception/launch/usb_cam.launch.py usb_video_device:=$CAMERA_DEV
" > "$CAMERA_LOG" 2>&1 &

CAMERA_PID=$!
echo "[INFO] CAMERA_PID=$CAMERA_PID"

# ---------- 等待 /image 出现 ----------
echo "[INFO] 等待图像话题 $IMAGE_TOPIC 出现..."

IMAGE_READY=0
for i in $(seq 1 15); do
  if ros2 topic list 2>/dev/null | grep -qx "$IMAGE_TOPIC"; then
    IMAGE_READY=1
    break
  fi
  sleep 1
done

if [ "$IMAGE_READY" -ne 1 ]; then
  echo "[ERROR] 等待 $IMAGE_TOPIC 超时。"
  echo "请查看相机日志："
  echo "  tail -n 80 $CAMERA_LOG"
  exit 1
fi

echo "[OK] 已检测到图像话题：$IMAGE_TOPIC"

echo "[INFO] 图像话题类型："
ros2 topic type "$IMAGE_TOPIC" || true

# ---------- 启动底盘桥 ----------
echo "[INFO] 启动底盘桥 cmd_vel_to_rosmaster.py..."
echo "[INFO] 日志：$BRIDGE_LOG"

bash -lc "
source $TROS_SETUP
cd $PROJECT_DIR/ros2_bridge
python3 cmd_vel_to_rosmaster.py \
  --port $CHASSIS_PORT \
  --max-vx $BRIDGE_MAX_VX \
  --max-wz $BRIDGE_MAX_WZ \
  --kick-vx $KICK_VX \
  --kick-duration $KICK_DURATION \
  --min-drive-vx $MIN_DRIVE_VX \
  --kick-max-wz $KICK_MAX_WZ \
  --debug
" > "$BRIDGE_LOG" 2>&1 &

BRIDGE_PID=$!
echo "[INFO] BRIDGE_PID=$BRIDGE_PID"

# ---------- 等待 /cmd_vel 订阅者出现 ----------
echo "[INFO] 等待底盘桥订阅 /cmd_vel..."

CMD_SUB_READY=0
for i in $(seq 1 10); do
  INFO_OUT="$(ros2 topic info /cmd_vel 2>/dev/null || true)"
  echo "$INFO_OUT" | grep -q "Subscription count: 1" && CMD_SUB_READY=1 && break
  sleep 1
done

if [ "$CMD_SUB_READY" -ne 1 ]; then
  echo "[WARN] 没有确认到 /cmd_vel 的订阅者。"
  echo "请查看底盘桥日志："
  echo "  tail -n 80 $BRIDGE_LOG"
else
  echo "[OK] /cmd_vel 已有订阅者。"
fi

# ---------- 启动红色视觉伺服 ----------
echo "[INFO] 启动红色视觉伺服 red_target_servo_auto_ros.py..."
echo "[INFO] 日志：$SERVO_LOG"

bash -lc "
source $TROS_SETUP
cd $PROJECT_DIR/visual_servo
python3 red_target_servo_auto_ros.py \
  --image-topic $IMAGE_TOPIC \
  --max-vx $SERVO_MAX_VX \
  --max-wz $SERVO_MAX_WZ \
  --kp-turn $KP_TURN \
  --center-threshold $CENTER_THRESHOLD \
  --arrive-area-ratio $ARRIVE_AREA_RATIO \
  --save-debug
" > "$SERVO_LOG" 2>&1 &

SERVO_PID=$!
echo "[INFO] SERVO_PID=$SERVO_PID"

# ---------- 保存 PID ----------
cat > "$PID_FILE" <<EOF
CAMERA_PID=$CAMERA_PID
BRIDGE_PID=$BRIDGE_PID
SERVO_PID=$SERVO_PID
EOF

echo "============================================================"
echo "[OK] 红色视觉伺服完整链路已经启动。"
echo "============================================================"
echo "现在请做 3 件事："
echo
echo "1. 把红色目标放到摄像头前方。"
echo
echo "2. 观察 /cmd_vel："
echo "   source /opt/tros/humble/setup.bash"
echo "   ros2 topic echo /cmd_vel"
echo
echo "3. 观察视觉状态："
echo "   source /opt/tros/humble/setup.bash"
echo "   ros2 topic echo /red_servo_state"
echo
echo "查看日志："
echo "   tail -f $SERVO_LOG"
echo "   tail -f $BRIDGE_LOG"
echo "   tail -f $CAMERA_LOG"
echo
echo "停止完整链路："
echo "   cd $PROJECT_DIR"
echo "   ./scripts/stop_red_servo.sh"
echo "============================================================"
