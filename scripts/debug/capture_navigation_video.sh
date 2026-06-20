#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
SAVE_DIR="${SAVE_DIR:-$PROJECT_DIR/capture_video}"

echo "============================================================"
echo " Navigation Video Capture"
echo "============================================================"
echo "1) Run this script first and wait"
echo "2) Start navigation in another terminal"
echo "3) Recording begins automatically when navigation starts"
echo "4) Stops on task SUCCESS or Ctrl+C (video still saved)"
echo "============================================================"
echo "IMAGE_TOPIC = $IMAGE_TOPIC"
echo "CMD_TOPIC   = $CMD_TOPIC"
echo "SAVE_DIR    = $SAVE_DIR"
echo "============================================================"

mkdir -p "$SAVE_DIR"

source /opt/tros/humble/setup.bash
python3 "$PROJECT_DIR/debug_tools/capture_navigation_video.py" \
  --image-topic "$IMAGE_TOPIC" \
  --cmd-topic "$CMD_TOPIC" \
  --save-dir "$SAVE_DIR" \
  --image-width 1280 \
  --image-height 720
