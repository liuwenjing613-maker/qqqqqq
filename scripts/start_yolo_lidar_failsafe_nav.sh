#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$PROJECT_DIR"

if [ -f /opt/tros/humble/setup.bash ]; then
  set +u
  source /opt/tros/humble/setup.bash
  set -u
else
  source /opt/ros/humble/setup.bash
fi

source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

CONFIG="${1:-configs/yolo_lidar_failsafe_nav.yaml}"
INSTRUCTION="${2:-bottle}"
NAV_ONLY="${NAV_ONLY:-0}"

CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
CHASSIS_PORT="${CHASSIS_PORT:-/dev/ttyUSB0}"

TARGET_WORDS="${TARGET_WORDS:-bottle,water bottle,cup}"
TARGET_CLASSES="${TARGET_CLASSES:-bottle,cup}"
DET_TOPIC="${DET_TOPIC:-/hobot_yolo_world}"
YOLO_BRIDGE_MIN_SCORE="${YOLO_BRIDGE_MIN_SCORE:-0.002}"
YOLO_BRIDGE_MAX_AREA_RATIO="${YOLO_BRIDGE_MAX_AREA_RATIO:-0.24}"

mkdir -p logs data/images/yolo_lidar_failsafe_debug

echo "===== P0 YOLO + LiDAR Failsafe Navigation ====="
echo "PROJECT_DIR=$PROJECT_DIR"
echo "CONFIG=$CONFIG INSTRUCTION=$INSTRUCTION NAV_ONLY=$NAV_ONLY"
echo "TARGET_WORDS=$TARGET_WORDS TARGET_CLASSES=$TARGET_CLASSES"
echo "SCORE_THRESHOLD (node)=$SCORE_THRESHOLD"

echo "[P0] stopping old nav/qwen/preview processes..."
pkill -f run_qwen_lidar_nav.py || true
pkill -f run_yolo_lidar_failsafe_nav.py || true
pkill -f yolo_world_to_bbox_json.py || true
pkill -f yolo_live_browser_preview.py || true
pkill -f test_qwen_ollama_image.py || true
pkill -f bench_qwen_ollama.py || true

if [ "$NAV_ONLY" = "1" ]; then
  echo "[P0] NAV_ONLY=1: starting failsafe nav only (assume sensors/YOLO already running)"
  python3 src/apps/run_yolo_lidar_failsafe_nav.py \
    --config "$CONFIG" \
    --instruction "$INSTRUCTION" \
    > logs/yolo_lidar_failsafe_nav.log 2>&1 &
  echo "[P0] started nav pid=$!"
  echo "  tail -f logs/yolo_lidar_failsafe_nav.log"
  exit 0
fi

echo "[P0] not starting Qwen/Ollama."

echo "[1/7] publish zero cmd_vel once..."
timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
  >/dev/null 2>&1 || true

echo "[2/7] start camera + image bridge..."
pkill -f compressed_to_raw_image.py || true
ros2 launch hobot_usb_cam hobot_usb_cam.launch.py \
  usb_video_device:="$CAMERA_DEV" \
  usb_image_width:=640 \
  usb_image_height:=480 \
  usb_framerate:=10 \
  > logs/yolo_failsafe_camera.log 2>&1 &
sleep 3

python3 src/perception/compressed_to_raw_image.py \
  --in-topic /image \
  --out-topic /image_raw \
  --max-fps 2 \
  > logs/yolo_failsafe_image_raw.log 2>&1 &
sleep 2

echo "[3/7] start lidar..."
ros2 launch "$PROJECT_DIR/lidar/launch/tmini_plus.launch.py" > logs/yolo_failsafe_scan.log 2>&1 &
sleep 3

echo "[4/7] start YOLO-World (aligned with start_yolo_live_preview.sh)..."
pkill -f hobot_yolo_world || true
sleep 1

if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source "$PROJECT_DIR/source_stage10.sh"
fi

ros2 run hobot_yolo_world hobot_yolo_world \
  --ros-args \
  -p feed_type:=1 \
  -p ros_img_sub_topic_name:=/image_raw \
  -p ros_string_sub_topic_name:=/target_words \
  -p ai_msg_pub_topic_name:="$DET_TOPIC" \
  -p texts:="$TARGET_WORDS" \
  -p score_threshold:="$SCORE_THRESHOLD" \
  -p iou_threshold:=0.45 \
  > logs/yolo_failsafe_yolo_world.log 2>&1 &
sleep 4

echo "[5/7] start yolo_world_to_bbox_json bridge..."
python3 src/perception/yolo_world_to_bbox_json.py \
  --det-topic "$DET_TOPIC" \
  --target-classes "$TARGET_CLASSES" \
  --min-score "$YOLO_BRIDGE_MIN_SCORE" \
  --max-area-ratio "$YOLO_BRIDGE_MAX_AREA_RATIO" \
  > logs/yolo_failsafe_bbox_bridge.log 2>&1 &
sleep 2

echo "[6/7] start chassis bridge..."
pkill -f cmd_vel_to_rosmaster.py || true
python3 ros2_bridge/cmd_vel_to_rosmaster.py \
  --port "$CHASSIS_PORT" \
  --max-vx 0.08 \
  --max-wz 0.35 \
  --watchdog-timeout 0.5 \
  > logs/yolo_failsafe_chassis.log 2>&1 &
sleep 1

echo "[7/7] wait for /image_raw and /scan..."
for topic in /image_raw /scan; do
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

echo "[P0] starting failsafe nav..."
python3 src/apps/run_yolo_lidar_failsafe_nav.py \
  --config "$CONFIG" \
  --instruction "$INSTRUCTION" \
  > logs/yolo_lidar_failsafe_nav.log 2>&1 &

echo "[P0] started."
echo "Watch logs:"
echo "  tail -f logs/yolo_lidar_failsafe_nav.log"
echo "Watch state:"
echo "  ros2 topic echo /failsafe_nav_state"
echo "Watch point:"
echo "  ros2 topic echo /failsafe_nav_point"
echo "Watch cmd:"
echo "  ros2 topic echo /cmd_vel"
echo "Watch bbox:"
echo "  ros2 topic echo /target_bbox_json"
