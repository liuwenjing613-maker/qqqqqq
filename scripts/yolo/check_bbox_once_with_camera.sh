#!/usr/bin/env bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
# 自带相机启动：单独跑 check_bbox_once.sh 会卡在 waiting /image_raw，
# 因为该脚本不会启动相机，需要先有 /image_raw 话题。
set -e

PROJECT_DIR=~/rdk_x5_vln_robot
CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
COMPRESSED_IMAGE_TOPIC=/image
RAW_IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
SAVE_DIR="${SAVE_DIR:-$PROJECT_DIR/check_bbox}"
KEEP_CAMERA="${KEEP_CAMERA:-0}"

echo "============================================================"
echo " Check Bbox Once (with camera)"
echo " Red HSV detection - verifies camera + red target"
echo "============================================================"
echo "CAMERA_DEV  = $CAMERA_DEV"
echo "IMAGE_TOPIC = $RAW_IMAGE_TOPIC"
echo "SAVE_DIR    = $SAVE_DIR"
echo "============================================================"

cd "$PROJECT_DIR"
mkdir -p logs "$SAVE_DIR"

source /opt/tros/humble/setup.bash

PUBLISHER_COUNT=$(ros2 topic info "$RAW_IMAGE_TOPIC" 2>/dev/null | awk '/Publisher count:/ {print $3}' || echo 0)
if [ "${PUBLISHER_COUNT:-0}" -lt 1 ]; then
  echo "[1/3] no publisher on $RAW_IMAGE_TOPIC, starting camera..."
  bash scripts/system/stop_all_safe.sh || true
  sleep 1

  cd "$PROJECT_DIR/perception"
  ros2 launch "$PROJECT_DIR/perception/launch/usb_cam.launch.py" usb_video_device:="$CAMERA_DEV" \
    > "$PROJECT_DIR/logs/check_bbox_camera.log" 2>&1 &
  sleep 3

  cd "$PROJECT_DIR"
  python3 src/perception/compressed_to_raw_image.py \
    --in-topic "$COMPRESSED_IMAGE_TOPIC" \
    --out-topic "$RAW_IMAGE_TOPIC" \
    > "$PROJECT_DIR/logs/check_bbox_image_bridge.log" 2>&1 &
  sleep 2
  STARTED_CAMERA=1
else
  echo "[1/3] $RAW_IMAGE_TOPIC already has publisher, skip camera start"
  STARTED_CAMERA=0
fi

echo "[2/3] grab one frame and run Red HSV detection..."
IMAGE_TOPIC="$RAW_IMAGE_TOPIC" SAVE_DIR="$SAVE_DIR" bash "$PROJECT_DIR/scripts/yolo/check_bbox_once.sh"

echo "[3/3] done. check images in: $SAVE_DIR"

if [ "$STARTED_CAMERA" = "1" ] && [ "$KEEP_CAMERA" != "1" ]; then
  echo "[cleanup] stopping camera started by this script..."
  pkill -f "hobot_usb_cam" 2>/dev/null || true
  pkill -f "compressed_to_raw_image.py" 2>/dev/null || true
elif [ "$STARTED_CAMERA" = "1" ]; then
  echo "[info] camera left running (KEEP_CAMERA=1)"
fi
