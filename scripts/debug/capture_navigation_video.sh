#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
NAV_STATE_TOPIC="${NAV_STATE_TOPIC:-/nav_state}"
BBOX_TOPIC="${BBOX_TOPIC:-/target_bbox_json}"
POINT_TOPIC="${POINT_TOPIC:-/nav_target_point}"
SAVE_DIR="${SAVE_DIR:-$PROJECT_DIR/capture_video}"
RECORD_FPS="${RECORD_FPS:-20}"
RECORD_IMMEDIATELY="${RECORD_IMMEDIATELY:-0}"

echo "============================================================"
echo " V2 Navigation Video Capture"
echo "============================================================"
echo "1) Run this script first and wait"
echo "2) Start navigation in another terminal"
echo "3) Recording begins when nav leaves BOOT/WAIT_SENSORS or cmd_vel moves"
echo "4) Stops on SUCCESS or Ctrl+C (video still saved)"
echo "============================================================"
echo "Overlay: [1 TARGET] [2 STATE] [3 UV] [4 VEL]"
echo "IMAGE_TOPIC     = $IMAGE_TOPIC"
echo "NAV_STATE_TOPIC = $NAV_STATE_TOPIC"
echo "BBOX_TOPIC      = $BBOX_TOPIC"
echo "CMD_TOPIC       = $CMD_TOPIC"
echo "POINT_TOPIC     = $POINT_TOPIC"
echo "SAVE_DIR        = $SAVE_DIR"
echo "RECORD_FPS      = $RECORD_FPS"
echo "RECORD_IMMEDIATELY = $RECORD_IMMEDIATELY"
echo "============================================================"

mkdir -p "$SAVE_DIR"

set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
set -u

ARGS=(
  --image-topic "$IMAGE_TOPIC"
  --cmd-topic "$CMD_TOPIC"
  --nav-state-topic "$NAV_STATE_TOPIC"
  --bbox-topic "$BBOX_TOPIC"
  --point-topic "$POINT_TOPIC"
  --save-dir "$SAVE_DIR"
  --record-fps "$RECORD_FPS"
)

if [ "$RECORD_IMMEDIATELY" = "1" ]; then
  ARGS+=(--record-immediately)
fi

python3 "$PROJECT_DIR/debug_tools/capture_navigation_video.py" "${ARGS[@]}"
