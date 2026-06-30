#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
CONFIG=$PROJECT_DIR/configs/qwen_lidar_nav.yaml
INSTRUCTION="${1:-find the bottle}"
CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
CHASSIS_PORT="${CHASSIS_PORT:-/dev/ttyUSB0}"
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
mkdir -p logs data/images/qwen_lidar_debug

echo "===== Qwen-only + LiDAR Navigation ====="
echo "INSTRUCTION=$INSTRUCTION CAMERA_DEV=$CAMERA_DEV CHASSIS_PORT=$CHASSIS_PORT"

echo "[1/8] stop old related processes..."
pkill -f run_qwen_lidar_nav.py || true
pkill -f run_qwen_pixel_task.py || true
pkill -f compressed_to_raw_image.py || true
pkill -f m1_pwm_cmd_vel_bridge.py || true
pkill -f cmd_vel_to_rosmaster.py || true
pkill -f hobot_yolo_world || true

echo "[2/8] publish zero cmd_vel once..."
timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 >/dev/null 2>&1 || true

echo "[3/8] ensure Ollama API is up..."
if ! curl -sf --max-time 3 http://127.0.0.1:11434/api/tags >/dev/null; then
  systemctl start ollama || true
  sleep 3
fi

echo "[4/8] start camera..."
ros2 launch hobot_usb_cam hobot_usb_cam.launch.py \
  usb_video_device:="$CAMERA_DEV" \
  usb_image_width:=640 \
  usb_image_height:=480 \
  usb_framerate:=10 \
  > logs/qwen_lidar_camera.log 2>&1 &
sleep 3

echo "[5/8] start compressed -> raw image bridge..."
python3 src/perception/compressed_to_raw_image.py \
  --in-topic /image \
  --out-topic /image_raw \
  --max-fps 2 \
  > logs/qwen_lidar_image_raw.log 2>&1 &
sleep 2

echo "[6/8] start lidar..."
ros2 launch /root/rdk_x5_vln_robot/lidar/launch/tmini_plus.launch.py > logs/qwen_lidar_scan.log 2>&1 &
sleep 3

echo "[7/8] wait for /image_raw and /scan..."
for topic in /image_raw /scan; do
  echo "waiting $topic..."
  ok=0
  for i in $(seq 1 15); do
    if timeout 3 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      echo "  $topic OK"
      ok=1
      break
    fi
    echo "  waiting $topic... ($i/15)"
    sleep 2
  done
  if [ "$ok" -ne 1 ]; then
    echo "ERROR: $topic not available. Check logs/."
    exit 1
  fi
done

echo "[8/8] start chassis bridge and Qwen LiDAR nav..."
source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
run_chassis_bridge "$PROJECT_DIR/logs/qwen_lidar_chassis.log"
sleep 1
python3 src/apps/run_qwen_lidar_nav.py --config "$CONFIG" --instruction "$INSTRUCTION" > logs/qwen_lidar_nav.log 2>&1 &

echo "Started. tail -f logs/qwen_lidar_nav.log"
