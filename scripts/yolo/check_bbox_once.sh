#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
SAVE_DIR="${SAVE_DIR:-$PROJECT_DIR/check_bbox}"

echo "============================================================"
echo " Check Bbox Once - grab one frame during navigation"
echo " NOTE: requires /image_raw already publishing."
echo "       If nothing is running, use:"
echo "       bash scripts/yolo/check_bbox_once_with_camera.sh"
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
