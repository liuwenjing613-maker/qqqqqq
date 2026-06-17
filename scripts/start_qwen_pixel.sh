#!/usr/bin/env bash
set -e

PROJECT_DIR=~/rdk_x5_vln_robot
TUNE_FILE="$PROJECT_DIR/configs/qwen_pixel_tune.yaml"
CAMERA_DEV=/dev/video0

COMPRESSED_IMAGE_TOPIC=/image
RAW_IMAGE_TOPIC=/image_raw
CMD_TOPIC=/cmd_vel

INSTRUCTION="${1:-find the bottle}"

echo "============================================================"
echo " RDK X5 Qwen Point Servo Task"
echo " Tune: $TUNE_FILE"
echo " Instruction: $INSTRUCTION"
echo "============================================================"

cd "$PROJECT_DIR"
mkdir -p logs data/images/qwen_pixel_debug

echo "[0/7] Ollama prep + warmup (text + vision)..."
bash scripts/ollama_prep_infer.sh qwen2.5vl:3b

echo "[1/7] stop old processes..."
bash scripts/stop_all_safe.sh || true
sleep 1

echo "[2/7] start camera..."
cd "$PROJECT_DIR/perception"
source /opt/tros/humble/setup.bash
ros2 launch "$PROJECT_DIR/perception/launch/usb_cam.launch.py" usb_video_device:=$CAMERA_DEV \
  > "$PROJECT_DIR/logs/qwen_pixel_camera.log" 2>&1 &
sleep 3

echo "[3/7] start image bridge..."
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
python3 src/perception/compressed_to_raw_image.py \
  --in-topic "$COMPRESSED_IMAGE_TOPIC" \
  --out-topic "$RAW_IMAGE_TOPIC" \
  > "$PROJECT_DIR/logs/qwen_pixel_image_bridge.log" 2>&1 &
sleep 2

echo "[4/7] start chassis bridge..."
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
run_chassis_bridge "$PROJECT_DIR/logs/qwen_pixel_chassis.log"
sleep 2

echo "[5/7] start Qwen point servo task (model already warm)..."
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
python3 src/apps/run_qwen_pixel_task.py \
  --qwen-pixel-tune-config "$TUNE_FILE" \
  --instruction "$INSTRUCTION" \
  --image-topic "$RAW_IMAGE_TOPIC" \
  --cmd-topic "$CMD_TOPIC" \
  --no-warmup \
  --save-debug \
  2>&1 | tee "$PROJECT_DIR/logs/qwen_pixel_task.log"
