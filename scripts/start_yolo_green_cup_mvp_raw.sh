#!/usr/bin/env bash
set -e

PROJECT_DIR=~/rdk_x5_vln_robot
CAMERA_DEV=/dev/video0

# 统一调参：改 configs/mvp_tune.yaml 即可
source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

COMPRESSED_IMAGE_TOPIC=/image
RAW_IMAGE_TOPIC=/image_raw

YOLO_IMAGE_TOPIC=/image_raw

CMD_TOPIC=/cmd_vel
TARGET_WORDS_TOPIC=/target_words
DET_TOPIC=/hobot_yolo_world

INSTRUCTION="find the green bottle"
TARGET_WORDS="${TARGET_WORDS:-green bottle,bottle,cup}"
echo "============================================================"
echo " RDK X5 VLN Robot MVP - YOLO-World Green Cup"
echo " Tune file: $MVP_TUNE_FILE"
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
echo "lost_frames_limit        = $LOST_FRAMES_LIMIT"
echo "MAX_AREA_RATIO           = $MAX_AREA_RATIO"
echo "KICK_VX                  = $KICK_VX"
echo "KICK_WZ                  = $KICK_WZ"
echo "KICK_DURATION            = $KICK_DURATION"
echo "MIN_DRIVE_VX             = $MIN_DRIVE_VX"
echo "ENABLE_KICK_START        = $ENABLE_KICK_START"
echo "CMD_SMOOTH_ALPHA         = $CMD_SMOOTH_ALPHA"
echo "MAX_VX_DELTA             = $MAX_VX_DELTA"
echo "MAX_WZ_DELTA             = $MAX_WZ_DELTA"
echo "RECOVERY_SCAN_WZ         = $RECOVERY_SCAN_WZ"
echo "LOST_HOLD_FRAMES         = $LOST_HOLD_FRAMES"
echo "LOST_OBSERVE_FRAMES      = $LOST_OBSERVE_FRAMES"
echo "RECOVERY_SCAN_MAX_FRAMES = $RECOVERY_SCAN_MAX_FRAMES"
echo "TURN_THRESHOLD           = $TURN_THRESHOLD"
echo "FORWARD_THRESHOLD        = $FORWARD_THRESHOLD"
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
  > $PROJECT_DIR/logs/yolo_green_cup_camera_compressed.log 2>&1 &
sleep 3

echo "[2/5] start image bridge: $COMPRESSED_IMAGE_TOPIC -> $RAW_IMAGE_TOPIC"
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash
python3 src/perception/compressed_to_raw_image.py \
  --in-topic $COMPRESSED_IMAGE_TOPIC \
  --out-topic $RAW_IMAGE_TOPIC \
  > $PROJECT_DIR/logs/yolo_green_cup_image_raw_bridge.log 2>&1 &
sleep 2

echo "[3/5] start YOLO-World: $YOLO_IMAGE_TOPIC -> $DET_TOPIC"
echo "  target_words: -p texts only (run_mvp_task.py publishes $TARGET_WORDS_TOPIC @ 1Hz)"
cd /opt/tros/humble/lib/hobot_yolo_world
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
  > $PROJECT_DIR/logs/yolo_green_cup_yolo_world.log 2>&1 &
sleep 4

echo "[4/5] start chassis bridge: $CMD_TOPIC -> M1"
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
run_chassis_bridge "$PROJECT_DIR/logs/yolo_green_cup_chassis_bridge.log"
sleep 2

echo "[4.5/5] check topics..."
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

echo "[5/5] start MVP task: YOLO backend, green cup, no red verify"
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
  --no-red-verify \
  2>&1 | tee $PROJECT_DIR/logs/yolo_green_cup_mvp_task.log
