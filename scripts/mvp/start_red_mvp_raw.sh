#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
CAMERA_DEV=/dev/video0

source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

COMPRESSED_IMAGE_TOPIC=/image
RAW_IMAGE_TOPIC=/image_raw
CMD_TOPIC=/cmd_vel

echo "============================================================"
echo " RDK X5 VLN Robot MVP - Red Target with Raw Image Bridge"
echo " Tune file: $MVP_TUNE_FILE"
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
bash scripts/system/stop_all_safe.sh || true
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
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
run_chassis_bridge "$PROJECT_DIR/logs/mvp_chassis_bridge.log"
sleep 2

echo "[5/5] start MVP task: subscribe $RAW_IMAGE_TOPIC, publish $CMD_TOPIC"
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash
python3 src/apps/run_mvp_task.py \
  --mvp-tune-config "$MVP_TUNE_FILE" \
  --instruction "find the red backpack" \
  --backend red \
  --image-topic $RAW_IMAGE_TOPIC \
  --image-width 1280 \
  --image-height 720 \
  --cmd-topic $CMD_TOPIC \
  --save-debug \
  2>&1 | tee $PROJECT_DIR/logs/mvp_task.log
