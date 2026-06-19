#!/usr/bin/env bash
set -e

PROJECT_DIR=/root/rdk_x5_vln_robot
CONFIG=$PROJECT_DIR/configs/qwen_lidar_nav.yaml
INSTRUCTION="${1:-find the bottle}"

CAM_W=$(python3 -c "import yaml; c=yaml.safe_load(open('$CONFIG')); print(c.get('camera_width', 640))")
CAM_H=$(python3 -c "import yaml; c=yaml.safe_load(open('$CONFIG')); print(c.get('camera_height', 480))")

echo "============================================================"
echo " Qwen-only + LiDAR Navigation (camera-on fast path)"
echo "============================================================"
echo "PROJECT_DIR = $PROJECT_DIR"
echo "CONFIG      = $CONFIG"
echo "INSTRUCTION = $INSTRUCTION"
echo "CAMERA      = ${CAM_W}x${CAM_H} (lower res saves RAM for Ollama)"
echo ""
echo "Flow: camera -> bridge -> nav waits first frame -> aligned warmup -> infer"
echo "Do NOT run ollama_prep_infer.sh before this; warmup happens with camera on."
echo "============================================================"

cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash

mkdir -p logs data/images/qwen_lidar_debug

echo "[1/7] stop old related processes..."
pkill -f run_qwen_lidar_nav.py || true
pkill -f run_qwen_pixel_task.py || true
pkill -f compressed_to_raw_image.py || true
pkill -f chassis_cmdvel_bridge.py || true
pkill -f "ros2 launch hobot_usb_cam" || true
sleep 1

echo "[2/7] ensure Ollama is up (no model unload prep)..."
if ! curl -sf --max-time 3 http://127.0.0.1:11434/api/tags >/dev/null; then
  echo "[WARN] Ollama not reachable; starting service..."
  systemctl start ollama || true
  sleep 2
fi

echo "[3/7] start camera at ${CAM_W}x${CAM_H}..."
ros2 launch hobot_usb_cam hobot_usb_cam.launch.py \
  usb_video_device:=/dev/video0 \
  usb_image_width:=${CAM_W} \
  usb_image_height:=${CAM_H} \
  usb_framerate:=15 \
  > logs/qwen_lidar_camera.log 2>&1 &

sleep 3

echo "[4/7] start compressed -> raw image bridge (10 fps, depth=1)..."
python3 src/perception/compressed_to_raw_image.py \
  --in-topic /image \
  --out-topic /image_raw \
  --max-fps 10 \
  > logs/qwen_lidar_image_raw.log 2>&1 &

sleep 2

echo "[5/7] wait for /image_raw..."
for i in $(seq 1 15); do
  if timeout 3 ros2 topic hz /image_raw --window 3 2>/dev/null | grep -q "average rate"; then
    echo "  /image_raw publishing"
    break
  fi
  echo "  waiting... (${i}/15)"
  sleep 2
done

free -h | sed 's/^/  /'

echo "[6/7] start chassis cmd_vel bridge..."
python3 ros2_bridge/chassis_cmdvel_bridge.py \
  --port /dev/ttyUSB0 \
  --cmd-topic /cmd_vel \
  > logs/qwen_lidar_chassis.log 2>&1 &

sleep 1

echo "[7/7] start Qwen LiDAR nav (aligned warmup on first camera frame)..."
python3 src/apps/run_qwen_lidar_nav.py \
  --config "$CONFIG" \
  --instruction "$INSTRUCTION" \
  > logs/qwen_lidar_nav.log 2>&1 &

echo "============================================================"
echo "Started."
echo "Check:"
echo "  tail -f logs/qwen_lidar_nav.log"
echo "  ros2 topic echo /qwen_nav_json"
echo ""
echo "Expected timing (7GB board, camera on):"
echo "  aligned warmup (first time): 1-8 min"
echo "  each infer after warmup:     ~7-10s (check prompt_eval_ms < 1000)"
echo "============================================================"
