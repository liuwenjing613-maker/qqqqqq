#!/usr/bin/env bash
set -e

PROJECT_DIR=~/rdk_x5_vln_robot
CAMERA_DEV=/dev/video0
CHASSIS_PORT=/dev/ttyUSB0

COMPRESSED_IMAGE_TOPIC=/image
RAW_IMAGE_TOPIC=/image_raw
CMD_TOPIC=/cmd_vel

# 启动死区补偿（落地启动困难时调大 kick-vx / kick-duration / min-drive-vx）
KICK_VX="${KICK_VX:-0.11}"
KICK_DURATION="${KICK_DURATION:-0.22}"
MIN_DRIVE_VX="${MIN_DRIVE_VX:-0.01}"
KICK_MAX_WZ="${KICK_MAX_WZ:-0.12}"

echo "============================================================"
echo " RDK X5 VLN Robot MVP - Red Target with Raw Image Bridge"
echo "============================================================"
echo "PROJECT_DIR              = $PROJECT_DIR"
echo "CAMERA_DEV               = $CAMERA_DEV"
echo "CHASSIS_PORT             = $CHASSIS_PORT"
echo "COMPRESSED_IMAGE_TOPIC   = $COMPRESSED_IMAGE_TOPIC"
echo "RAW_IMAGE_TOPIC          = $RAW_IMAGE_TOPIC"
echo "CMD_TOPIC                = $CMD_TOPIC"
echo "============================================================"

cd $PROJECT_DIR
mkdir -p logs data/images/mvp_debug

echo "[0/5] stop old processes..."
bash scripts/stop_all_safe.sh || true
sleep 1

echo "[1/5] start camera: hobot_usb_cam -> $COMPRESSED_IMAGE_TOPIC"
cd $PROJECT_DIR/perception
source /opt/tros/humble/setup.bash
ros2 launch $PROJECT_DIR/perception/launch/usb_cam.launch.py usb_video_device:=$CAMERA_DEV \
  > $PROJECT_DIR/logs/mvp_camera_compressed.log 2>&1 &
sleep 3

echo "[2/5] start image bridge: $COMPRESSED_IMAGE_TOPIC -> $RAW_IMAGE_TOPIC"
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash
python3 src/perception/compressed_to_raw_image.py \
  --in-topic $COMPRESSED_IMAGE_TOPIC \
  --out-topic $RAW_IMAGE_TOPIC \
  > $PROJECT_DIR/logs/mvp_image_raw_bridge.log 2>&1 &
sleep 2

echo "[3/5] check image topics..."
source /opt/tros/humble/setup.bash
ros2 topic info $COMPRESSED_IMAGE_TOPIC || true
ros2 topic info $RAW_IMAGE_TOPIC || true

echo "[4/5] start chassis bridge: $CMD_TOPIC -> M1"
cd $PROJECT_DIR/ros2_bridge
source /opt/tros/humble/setup.bash
python3 cmd_vel_to_rosmaster.py \
  --port $CHASSIS_PORT \
  --max-vx 0.10 \
  --max-wz 0.20 \
  --kick-vx $KICK_VX \
  --kick-duration $KICK_DURATION \
  --min-drive-vx $MIN_DRIVE_VX \
  --kick-max-wz $KICK_MAX_WZ \
  --debug \
  > $PROJECT_DIR/logs/mvp_chassis_bridge.log 2>&1 &
sleep 2

echo "[5/5] start MVP task: subscribe $RAW_IMAGE_TOPIC, publish $CMD_TOPIC"
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash
python3 src/apps/run_mvp_task.py \
  --instruction "find the red backpack" \
  --backend red \
  --image-topic $RAW_IMAGE_TOPIC \
  --image-width 1280 \
  --image-height 720 \
  --cmd-topic $CMD_TOPIC \
  --save-debug \
  2>&1 | tee $PROJECT_DIR/logs/mvp_task.log
