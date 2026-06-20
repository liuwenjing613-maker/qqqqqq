#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
CAMERA_DEV=/dev/video0

source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

echo "============================================================"
echo " RDK X5 VLN Robot MVP - Red Target"
echo " Tune file: $MVP_TUNE_FILE"
echo "============================================================"
echo "PROJECT_DIR  = $PROJECT_DIR"
echo "CAMERA_DEV   = $CAMERA_DEV"
echo "CHASSIS_PORT = $CHASSIS_PORT"
echo "============================================================"

cd $PROJECT_DIR
mkdir -p logs data/images/mvp_debug

echo "[1/4] stop old processes..."
bash scripts/system/stop_all.sh || true
sleep 1

echo "[2/4] start camera..."
cd $PROJECT_DIR/perception
source /opt/tros/humble/setup.bash
ros2 launch $PROJECT_DIR/perception/launch/usb_cam.launch.py usb_video_device:=$CAMERA_DEV \
  > $PROJECT_DIR/logs/mvp_camera.log 2>&1 &
sleep 3

echo "[2.5/4] start compressed2raw"
cd $PROJECT_DIR/perception
source /opt/tros/humble/setup.bash
python3 compressed_to_raw.py --in-topic /image --out-topic /image_raw
sleep 3

echo "[3/4] start chassis bridge..."
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
run_chassis_bridge "$PROJECT_DIR/logs/mvp_chassis_bridge.log"
sleep 2

echo "[4/4] start MVP task..."
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash
python3 src/apps/run_mvp_task.py \
  --mvp-tune-config "$MVP_TUNE_FILE" \
  --instruction "find the red backpack" \
  --backend red \
  --image-topic /image_raw \
  --image-width 1280 \
  --image-height 720 \
  --cmd-topic /cmd_vel \
  --save-debug \
  2>&1 | tee $PROJECT_DIR/logs/mvp_task.log
