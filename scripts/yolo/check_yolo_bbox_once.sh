#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
DET_TOPIC="${DET_TOPIC:-/hobot_yolo_world}"
SAVE_DIR="${SAVE_DIR:-$PROJECT_DIR/check_bbox}"
TARGET_CLASSES="${TARGET_CLASSES:-}"
NO_RED_VERIFY="${NO_RED_VERIFY:-1}"
TIMEOUT="${TIMEOUT:-0}"

echo "============================================================"
echo " Check YOLO Bbox Once - wait for detection then save"
echo "============================================================"
echo "IMAGE_TOPIC     = $IMAGE_TOPIC"
echo "DET_TOPIC       = $DET_TOPIC"
echo "SAVE_DIR        = $SAVE_DIR"
echo "TARGET_CLASSES  = ${TARGET_CLASSES:-(all)}"
echo "MIN_SCORE       = $MIN_SCORE (from $MVP_TUNE_FILE)"
echo "MAX_AREA_RATIO  = $MAX_AREA_RATIO"
echo "NO_RED_VERIFY   = $NO_RED_VERIFY"
echo "TIMEOUT         = $TIMEOUT"
echo "============================================================"

mkdir -p "$SAVE_DIR"

source /opt/tros/humble/setup.bash

ARGS=(
  --image-topic "$IMAGE_TOPIC"
  --det-topic "$DET_TOPIC"
  --save-dir "$SAVE_DIR"
  --target-classes "$TARGET_CLASSES"
  --min-score "$MIN_SCORE"
  --max-area-ratio "$MAX_AREA_RATIO"
)

if [ "$NO_RED_VERIFY" = "1" ]; then
  ARGS+=(--no-red-verify)
fi

if [ "$TIMEOUT" != "0" ]; then
  ARGS+=(--timeout "$TIMEOUT")
fi

python3 "$PROJECT_DIR/debug_tools/check_yolo_bbox_once.py" "${ARGS[@]}"

echo "[DONE] check images in: $SAVE_DIR"
