#!/usr/bin/env bash
set -e

PROJECT_DIR=~/rdk_x5_vln_robot
IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
SAVE_DIR="${SAVE_DIR:-$PROJECT_DIR/check_bbox}"

echo "============================================================"
echo " Check Bbox Once - grab one frame during navigation"
echo "============================================================"
echo "IMAGE_TOPIC = $IMAGE_TOPIC"
echo "SAVE_DIR    = $SAVE_DIR"
echo "============================================================"

mkdir -p "$SAVE_DIR"

source /opt/tros/humble/setup.bash
python3 "$PROJECT_DIR/debug_tools/check_bbox_once.py" \
  --image-topic "$IMAGE_TOPIC" \
  --save-dir "$SAVE_DIR"

echo "[DONE] check images in: $SAVE_DIR"
