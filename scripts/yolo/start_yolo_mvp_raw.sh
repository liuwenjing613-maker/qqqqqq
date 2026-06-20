#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
CAMERA_DEV=/dev/video0

source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

COMPRESSED_IMAGE_TOPIC=/image
RAW_IMAGE_TOPIC=/image_raw

# YOLO-World 输入话题：
# 优先使用 /image_raw，保持“算法实际输入原图”的统一原则。
YOLO_IMAGE_TOPIC=/image_raw

CMD_TOPIC=/cmd_vel
TARGET_WORDS_TOPIC=/target_words
DET_TOPIC=/hobot_yolo_world

INSTRUCTION="find the red backpack"
TARGET_WORDS="${TARGET_WORDS:-backpack,handbag,suitcase}"
SAVE_BBOX_DIR="${SAVE_BBOX_DIR:-$PROJECT_DIR/check_bbox}"
SAVE_BBOX_INTERVAL="${SAVE_BBOX_INTERVAL:-15}"

echo "============================================================"
echo " RDK X5 VLN Robot MVP - YOLO-World Red Backpack"
echo " Tune file: $MVP_TUNE_FILE"
echo " See yolo_world/README.md: hobot_yolo_world must run before post-process"
echo "============================================================"
echo "PROJECT_DIR              = $PROJECT_DIR"
echo "CAMERA_DEV               = $CAMERA_DEV"
echo "CHASSIS_PORT             = $CHASSIS_PORT"
echo "COMPRESSED_IMAGE_TOPIC   = $COMPRESSED_IMAGE_TOPIC"
echo "RAW_IMAGE_TOPIC          = $RAW_IMAGE_TOPIC"
echo "YOLO_IMAGE_TOPIC         = $YOLO_IMAGE_TOPIC"
echo "CMD_TOPIC                = $CMD_TOPIC"
echo "TARGET_WORDS_TOPIC       = $TARGET_WORDS_TOPIC"
echo "DET_TOPIC                = $DET_TOPIC"
echo "INSTRUCTION              = $INSTRUCTION"
echo "TARGET_WORDS             = $TARGET_WORDS"
echo "yolo_min_score           = $MIN_SCORE"
echo "max_vx                   = $MAX_VX"
echo "MIN_RED_RATIO            = $MIN_RED_RATIO"
echo "SAVE_BBOX_DIR            = $SAVE_BBOX_DIR"
echo "SAVE_BBOX_INTERVAL       = $SAVE_BBOX_INTERVAL"
echo "KICK_VX                  = $KICK_VX"
echo "KICK_WZ                  = $KICK_WZ"
echo "KICK_DURATION            = $KICK_DURATION"
echo "MIN_DRIVE_VX             = $MIN_DRIVE_VX"
echo "============================================================"

cd $PROJECT_DIR
mkdir -p logs data/images/mvp_debug "$SAVE_BBOX_DIR"

echo "[0/6] stop old processes..."
bash scripts/system/stop_all_safe.sh || true
sleep 1

echo "[1/6] start camera: hobot_usb_cam -> $COMPRESSED_IMAGE_TOPIC"
cd $PROJECT_DIR/perception
source /opt/tros/humble/setup.bash
ros2 launch $PROJECT_DIR/perception/launch/usb_cam.launch.py usb_video_device:=$CAMERA_DEV \
  > $PROJECT_DIR/logs/yolo_red_backpack_camera_compressed.log 2>&1 &
sleep 3

echo "[2/6] start image bridge: $COMPRESSED_IMAGE_TOPIC -> $RAW_IMAGE_TOPIC"
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash
python3 src/perception/compressed_to_raw_image.py \
  --in-topic $COMPRESSED_IMAGE_TOPIC \
  --out-topic $RAW_IMAGE_TOPIC \
  > $PROJECT_DIR/logs/yolo_red_backpack_image_raw_bridge.log 2>&1 &
sleep 2

echo "[3/6] publish target words BEFORE YOLO -> $TARGET_WORDS_TOPIC"
source /opt/tros/humble/setup.bash
ros2 topic pub -r 1 $TARGET_WORDS_TOPIC std_msgs/msg/String "{data: '$TARGET_WORDS'}" \
  > $PROJECT_DIR/logs/yolo_red_backpack_target_words.log 2>&1 &
sleep 2

echo "[4/6] start YOLO-World: $YOLO_IMAGE_TOPIC -> $DET_TOPIC"
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash

if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source $PROJECT_DIR/source_stage10.sh
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
  --ros-args --log-level warn \
  > $PROJECT_DIR/logs/yolo_red_backpack_yolo_world.log 2>&1 &
sleep 4

echo "[5/6] start chassis bridge: $CMD_TOPIC -> M1"
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
run_chassis_bridge "$PROJECT_DIR/logs/yolo_red_backpack_chassis_bridge.log"
sleep 2

echo "[5.5/6] check topics..."
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash
if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source $PROJECT_DIR/source_stage10.sh
fi

ros2 topic info $COMPRESSED_IMAGE_TOPIC || true
ros2 topic info $RAW_IMAGE_TOPIC || true
ros2 topic info $TARGET_WORDS_TOPIC || true
ros2 topic info $DET_TOPIC || true
ros2 topic info $CMD_TOPIC || true

echo "[6/6] start MVP task: YOLO backend, subscribe $RAW_IMAGE_TOPIC + $DET_TOPIC, publish $CMD_TOPIC"
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash
if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source $PROJECT_DIR/source_stage10.sh
fi

python3 src/apps/run_mvp_task.py \
  --mvp-tune-config "$MVP_TUNE_FILE" \
  --instruction "$INSTRUCTION" \
  --backend yolo_world \
  --image-topic $RAW_IMAGE_TOPIC \
  --det-topic $DET_TOPIC \
  --cmd-topic $CMD_TOPIC \
  --target-words-topic $TARGET_WORDS_TOPIC \
  --save-bbox-dir "$SAVE_BBOX_DIR" \
  --save-bbox-interval "$SAVE_BBOX_INTERVAL" \
  --save-debug \
  2>&1 | tee $PROJECT_DIR/logs/yolo_red_backpack_mvp_task.log
