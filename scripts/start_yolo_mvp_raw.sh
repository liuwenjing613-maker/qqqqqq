#!/usr/bin/env bash
set -e

PROJECT_DIR=~/rdk_x5_vln_robot
CAMERA_DEV=/dev/video0
CHASSIS_PORT=/dev/ttyUSB0

COMPRESSED_IMAGE_TOPIC=/image
RAW_IMAGE_TOPIC=/image_raw

# YOLO-World 输入话题：
# 优先使用 /image_raw，保持“算法实际输入原图”的统一原则。
# 如果实测 hobot_yolo_world 订阅 /image_raw 没输出，再把这里改成 /image。
YOLO_IMAGE_TOPIC=/image_raw

CMD_TOPIC=/cmd_vel
TARGET_WORDS_TOPIC=/target_words
DET_TOPIC=/hobot_yolo_world

INSTRUCTION="find the red backpack"
TARGET_WORDS="${TARGET_WORDS:-red backpack,backpack,bag}"

# 启动死区补偿（落地启动困难时调大 kick-vx / kick-duration / min-drive-vx）
KICK_VX="${KICK_VX:-0.11}"
KICK_DURATION="${KICK_DURATION:-0.22}"
MIN_DRIVE_VX="${MIN_DRIVE_VX:-0.01}"
KICK_MAX_WZ="${KICK_MAX_WZ:-0.12}"

echo "============================================================"
echo " RDK X5 VLN Robot MVP - YOLO-World Red Backpack"
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
echo "KICK_VX                  = $KICK_VX"
echo "KICK_DURATION            = $KICK_DURATION"
echo "MIN_DRIVE_VX             = $MIN_DRIVE_VX"
echo "KICK_MAX_WZ              = $KICK_MAX_WZ"
echo "============================================================"

cd $PROJECT_DIR
mkdir -p logs data/images/mvp_debug

echo "[0/6] stop old processes..."
bash scripts/stop_all_safe.sh || true
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

echo "[3/6] start YOLO-World: $YOLO_IMAGE_TOPIC -> $DET_TOPIC"
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash

# 如果存在 source_stage10.sh，说明你之前编译过 hobot_yolo_world，就加载它。
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
  -p score_threshold:=0.04 \
  -p iou_threshold:=0.45 \
  --ros-args --log-level warn \
  > $PROJECT_DIR/logs/yolo_red_backpack_yolo_world.log 2>&1 &
sleep 4

echo "[3.5/6] publish YOLO target words -> $TARGET_WORDS_TOPIC"
source /opt/tros/humble/setup.bash
ros2 topic pub -r 1 $TARGET_WORDS_TOPIC std_msgs/msg/String "{data: '$TARGET_WORDS'}" \
  > $PROJECT_DIR/logs/yolo_red_backpack_target_words.log 2>&1 &
sleep 1

echo "[4/6] start chassis bridge: $CMD_TOPIC -> M1"
cd $PROJECT_DIR/ros2_bridge
source /opt/tros/humble/setup.bash
python3 cmd_vel_to_rosmaster.py \
  --port $CHASSIS_PORT \
  --max-vx 0.10 \
  --max-wz 0.20 \
  --kick-vx $KICK_VX \
  --kick-duration $KICK_DURATION \
  --min-drive-vx $MIN_DRIVE_VX \
  --kick-max-wz $KICK_MAX_WZ \
  --debug \
  > $PROJECT_DIR/logs/yolo_red_backpack_chassis_bridge.log 2>&1 &
sleep 2

echo "[5/6] check topics..."
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
  --instruction "$INSTRUCTION" \
  --backend yolo_world \
  --image-topic $RAW_IMAGE_TOPIC \
  --det-topic $DET_TOPIC \
  --cmd-topic $CMD_TOPIC \
  --target-words-topic $TARGET_WORDS_TOPIC \
  --min-score 0.08 \
  --image-width 1280 \
  --image-height 720 \
  --save-debug \
  2>&1 | tee $PROJECT_DIR/logs/yolo_red_backpack_mvp_task.log
