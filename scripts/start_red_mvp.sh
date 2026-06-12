#!/usr/bin/env bash
set -e

PROJECT_DIR=~/rdk_x5_vln_robot
CAMERA_DEV=/dev/video0
CHASSIS_PORT=/dev/myserial

KICK_VX="${KICK_VX:-0.11}"
KICK_DURATION="${KICK_DURATION:-0.22}"
MIN_DRIVE_VX="${MIN_DRIVE_VX:-0.01}"
KICK_MAX_WZ="${KICK_MAX_WZ:-0.12}"

echo "============================================================"
echo " RDK X5 VLN Robot MVP - Red Target"
echo "============================================================"
echo "PROJECT_DIR  = $PROJECT_DIR"
echo "CAMERA_DEV   = $CAMERA_DEV"
echo "CHASSIS_PORT = $CHASSIS_PORT"
echo "============================================================"

cd $PROJECT_DIR
mkdir -p logs data/images/mvp_debug

echo "[1/4] stop old processes..."
bash scripts/stop_all.sh || true
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
  > $PROJECT_DIR/logs/mvp_chassis_bridge.log 2>&1 &
sleep 2

echo "[4/4] start MVP task..."
cd $PROJECT_DIR
source /opt/tros/humble/setup.bash
python3 src/apps/run_mvp_task.py \
  --instruction "find the red backpack" \
  --backend red \
  --image-topic /image_raw \
  --image-width 1280 \
  --image-height 720 \
  --cmd-topic /cmd_vel \
  --save-debug \
  2>&1 | tee $PROJECT_DIR/logs/mvp_task.log
