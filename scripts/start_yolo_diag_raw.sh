#!/usr/bin/env bash
set -e

PROJECT_DIR=~/rdk_x5_vln_robot
CAMERA_DEV=/dev/video0

COMPRESSED_IMAGE_TOPIC=/image
RAW_IMAGE_TOPIC=/image_raw
YOLO_IMAGE_TOPIC=/image_raw
TARGET_WORDS_TOPIC=/target_words
DET_TOPIC=/hobot_yolo_world

# 只用 offline_vocabulary 内词
TARGET_WORDS="${TARGET_WORDS:-backpack,handbag,suitcase}"
TARGET_CLASSES="${TARGET_CLASSES:-backpack,handbag,suitcase}"
SCORE_THRESHOLD="${SCORE_THRESHOLD:-0.001}"
MIN_SCORE="${MIN_SCORE:-0.002}"
MIN_RED_RATIO="${MIN_RED_RATIO:-0.06}"
MAX_AREA_RATIO="${MAX_AREA_RATIO:-0.15}"
RAW_MIN_SCORE="${RAW_MIN_SCORE:-0.0}"
SAVE_INTERVAL="${SAVE_INTERVAL:-15}"
SAVE_DIR="${SAVE_DIR:-$PROJECT_DIR/check_bbox}"

echo "============================================================"
echo " RDK X5 VLN Robot - YOLO-World Diagnostic Mode"
echo " (camera + detection only, NO chassis / NO /cmd_vel)"
echo " See yolo_world/README.md for inference vs post-process roles"
echo "============================================================"
echo "PROJECT_DIR            = $PROJECT_DIR"
echo "CAMERA_DEV             = $CAMERA_DEV"
echo "YOLO_IMAGE_TOPIC       = $YOLO_IMAGE_TOPIC"
echo "DET_TOPIC              = $DET_TOPIC"
echo "TARGET_WORDS_TOPIC     = $TARGET_WORDS_TOPIC"
echo "TARGET_WORDS           = $TARGET_WORDS"
echo "TARGET_CLASSES         = $TARGET_CLASSES"
echo "SCORE_THRESHOLD (node) = $SCORE_THRESHOLD"
echo "MIN_SCORE (MVP filter) = $MIN_SCORE"
echo "MIN_RED_RATIO (HSV)    = $MIN_RED_RATIO"
echo "MAX_AREA_RATIO         = $MAX_AREA_RATIO"
echo "RAW_MIN_SCORE (draw)   = $RAW_MIN_SCORE"
echo "SAVE_DIR               = $SAVE_DIR"
echo "SAVE_INTERVAL          = $SAVE_INTERVAL"
echo "============================================================"
echo "Legend in saved images:"
echo "  GREEN  = final MVP target (only one box shown by default)"
echo "  RED    = best rejected candidate when no MVP (debug)"
echo "  Use SHOW_ALL_BOXES=1 for full raw overlay"
echo "============================================================"

cd "$PROJECT_DIR"
mkdir -p logs "$SAVE_DIR"

echo "[0/6] stop old processes..."
bash scripts/stop_all_safe.sh || true
sleep 1

echo "[1/6] start camera: hobot_usb_cam -> $COMPRESSED_IMAGE_TOPIC"
cd "$PROJECT_DIR/perception"
source /opt/tros/humble/setup.bash
ros2 launch "$PROJECT_DIR/perception/launch/usb_cam.launch.py" usb_video_device:="$CAMERA_DEV" \
  > "$PROJECT_DIR/logs/yolo_diag_camera_compressed.log" 2>&1 &
sleep 3

echo "[2/6] start image bridge: $COMPRESSED_IMAGE_TOPIC -> $RAW_IMAGE_TOPIC"
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
python3 src/perception/compressed_to_raw_image.py \
  --in-topic "$COMPRESSED_IMAGE_TOPIC" \
  --out-topic "$RAW_IMAGE_TOPIC" \
  > "$PROJECT_DIR/logs/yolo_diag_image_raw_bridge.log" 2>&1 &
sleep 2

echo "[3/6] publish target words BEFORE YOLO -> $TARGET_WORDS_TOPIC"
source /opt/tros/humble/setup.bash
ros2 topic pub -r 1 "$TARGET_WORDS_TOPIC" std_msgs/msg/String "{data: '$TARGET_WORDS'}" \
  > "$PROJECT_DIR/logs/yolo_diag_target_words.log" 2>&1 &
sleep 2

echo "[4/6] start YOLO-World (diag thresholds): $YOLO_IMAGE_TOPIC -> $DET_TOPIC"
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source "$PROJECT_DIR/source_stage10.sh"
fi

ros2 run hobot_yolo_world hobot_yolo_world \
  --ros-args \
  -p feed_type:=1 \
  -p ros_img_sub_topic_name:="$YOLO_IMAGE_TOPIC" \
  -p ros_string_sub_topic_name:="$TARGET_WORDS_TOPIC" \
  -p ai_msg_pub_topic_name:="$DET_TOPIC" \
  -p texts:="$TARGET_WORDS" \
  -p score_threshold:="$SCORE_THRESHOLD" \
  -p iou_threshold:=0.45 \
  --ros-args --log-level info \
  > "$PROJECT_DIR/logs/yolo_diag_yolo_world.log" 2>&1 &
sleep 4

echo "[5/6] check topics..."
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source "$PROJECT_DIR/source_stage10.sh"
fi

ros2 topic info "$COMPRESSED_IMAGE_TOPIC" || true
ros2 topic info "$RAW_IMAGE_TOPIC" || true
ros2 topic info "$TARGET_WORDS_TOPIC" || true
ros2 topic info "$DET_TOPIC" || true

echo "[6/6] start diagnostic preview (foreground, Ctrl+C to stop)..."
echo "  snapshots -> $SAVE_DIR"
echo "  yolo log    -> $PROJECT_DIR/logs/yolo_diag_yolo_world.log"
echo "  preview log -> $PROJECT_DIR/logs/yolo_diag_preview.log"

cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source "$PROJECT_DIR/source_stage10.sh"
fi

python3 debug_tools/yolo_world_diag_preview.py \
  --image-topic "$RAW_IMAGE_TOPIC" \
  --det-topic "$DET_TOPIC" \
  --save-dir "$SAVE_DIR" \
  --target-classes "$TARGET_CLASSES" \
  --image-width 1280 \
  --image-height 720 \
  --min-score "$MIN_SCORE" \
  --raw-min-score "$RAW_MIN_SCORE" \
  --min-red-ratio "$MIN_RED_RATIO" \
  --max-area-ratio "$MAX_AREA_RATIO" \
  --save-interval "$SAVE_INTERVAL" \
  ${SHOW_ALL_BOXES:+--show-all-boxes} \
  2>&1 | tee "$PROJECT_DIR/logs/yolo_diag_preview.log"
