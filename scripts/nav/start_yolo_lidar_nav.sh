#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

source_ros_env() {
  set +u
  if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
  elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
  fi
  set -u
}

source_stage10_env() {
  if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
    set +u
    source "$PROJECT_DIR/source_stage10.sh"
    set -u
  fi
}

source_ros_env

CONFIG="${1:-configs/nav_yolo_lidar.yaml}"
USER_INSTRUCTION="${2:-}"
NAV_ONLY="${NAV_ONLY:-0}"
_CAMERA_OVERRIDE="${CAMERA_DEV:-}"
_CHASSIS_OVERRIDE="${CHASSIS_PORT:-}"

eval "$(python3 "$PROJECT_DIR/src/config/failsafe_nav_launch.py" --config "$CONFIG" --shell-export)"
if [ -n "$USER_INSTRUCTION" ]; then
  export INSTRUCTION="$USER_INSTRUCTION"
fi
if [ -n "$_CAMERA_OVERRIDE" ]; then
  export CAMERA_DEV="$_CAMERA_OVERRIDE"
fi
if [ -n "$_CHASSIS_OVERRIDE" ]; then
  export CHASSIS_PORT="$_CHASSIS_OVERRIDE"
fi

mkdir -p logs

echo "===== V2 yolo_lidar_nav ====="
echo "CONFIG=$CONFIG INSTRUCTION=$INSTRUCTION NAV_ONLY=$NAV_ONLY"
echo "CAMERA_DEV=$CAMERA_DEV CHASSIS_PORT=$CHASSIS_PORT"
echo "TARGET_WORDS=$TARGET_WORDS TARGET_CLASSES=$TARGET_CLASSES SCORE_THRESHOLD=$SCORE_THRESHOLD"
echo "[V2] not starting Qwen/Ollama."

pkill -f run_shared_nav.py || true
pkill -f yolo_world_to_bbox_json.py || true
pkill -f hobot_yolo_world || true
pkill -f compressed_to_raw_image.py || true
pkill -f m1_pwm_cmd_vel_bridge.py || true
timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
  >/dev/null 2>&1 || true

if [ "$NAV_ONLY" = "1" ]; then
  echo "[V2] NAV_ONLY=1: starting shared nav only."
  python3 "$PROJECT_DIR/src/apps/run_shared_nav.py" \
    --config "$CONFIG" \
    --instruction "$INSTRUCTION" \
    > "$PROJECT_DIR/logs/yolo_lidar_nav.log" 2>&1 &
  echo "[V2] started nav pid=$!"
  exit 0
fi

echo "[1/6] start camera + image bridge..."
ros2 launch "$PROJECT_DIR/perception/launch/usb_cam.launch.py" \
  usb_video_device:="$CAMERA_DEV" \
  > logs/yolo_lidar_camera.log 2>&1 &
sleep 5

python3 "$PROJECT_DIR/src/perception/compressed_to_raw_image.py" \
  --in-topic "$CAMERA_COMPRESSED_TOPIC" \
  --out-topic "$IMAGE_RAW_TOPIC" \
  --max-fps "$IMAGE_RAW_MAX_FPS" \
  > logs/yolo_lidar_image_raw.log 2>&1 &
sleep 2

echo "[2/6] start lidar..."
pkill -f ydlidar_ros2_driver_node 2>/dev/null || true
source "$PROJECT_DIR/scripts/lidar/source_ydlidar.sh"
ros2 launch "$PROJECT_DIR/lidar/launch/tmini_plus.launch.py" > logs/yolo_lidar_scan.log 2>&1 &
sleep 5

echo "[3/6] start YOLO-World..."
pkill -f hobot_yolo_world || true
sleep 1
source_stage10_env
ros2 run hobot_yolo_world hobot_yolo_world \
  --ros-args \
  -p feed_type:="$YOLO_FEED_TYPE" \
  -p ros_img_sub_topic_name:="$YOLO_IMAGE_TOPIC" \
  -p ros_string_sub_topic_name:=/target_words \
  -p ai_msg_pub_topic_name:="$DET_TOPIC" \
  -p texts:="$TARGET_WORDS" \
  -p score_threshold:="$SCORE_THRESHOLD" \
  -p iou_threshold:="$YOLO_IOU_THRESHOLD" \
  > logs/yolo_lidar_yolo_world.log 2>&1 &
sleep 4

echo "[4/6] start bbox JSON bridge..."
python3 "$PROJECT_DIR/src/perception/yolo_world_to_bbox_json.py" \
  --config "$CONFIG" \
  > logs/yolo_lidar_bbox_bridge.log 2>&1 &
sleep 2

echo "[5/6] start chassis bridge..."
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
run_chassis_bridge "$PROJECT_DIR/logs/yolo_lidar_chassis.log"
sleep 1

echo "[6/6] start shared nav..."
python3 "$PROJECT_DIR/src/apps/run_shared_nav.py" \
  --config "$CONFIG" \
  --instruction "$INSTRUCTION" \
  > "$PROJECT_DIR/logs/yolo_lidar_nav.log" 2>&1 &

if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  pkill -f "foxglove_bridge" 2>/dev/null || true
  bash "$PROJECT_DIR/scripts/lidar/start_foxglove.sh" \
    > "$PROJECT_DIR/logs/yolo_lidar_foxglove_bridge.log" 2>&1 &
fi

echo "[yolo_lidar_nav] started pid=$!"
echo "  tail -f logs/yolo_lidar_nav.log"
echo "  ros2 topic echo /nav_state"
echo "  ros2 topic echo /target_bbox_json"
echo "  ros2 topic echo /cmd_vel"
