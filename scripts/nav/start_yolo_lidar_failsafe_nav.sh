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
    # source_stage10.sh also sources ROS setup.bash; must run under set +u
    source "$PROJECT_DIR/source_stage10.sh"
    set -u
  fi
}

source_ros_env

CONFIG="${1:-configs/yolo_lidar_failsafe_nav.yaml}"
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

mkdir -p logs data/images/yolo_lidar_failsafe_debug

echo "===== P0 YOLO + LiDAR Failsafe Navigation ====="
echo "PROJECT_DIR=$PROJECT_DIR"
echo "CONFIG=$CONFIG INSTRUCTION=$INSTRUCTION NAV_ONLY=$NAV_ONLY"
echo "FSM: min_state_frames from yaml, emergency -> EMERGENCY_STOP immediate"
echo "CAMERA_DEV=$CAMERA_DEV CHASSIS_PORT=$CHASSIS_PORT"
echo "TARGET_WORDS=$TARGET_WORDS TARGET_CLASSES=$TARGET_CLASSES SCORE_THRESHOLD=$SCORE_THRESHOLD"

echo "[P0] stopping old nav/qwen/preview processes..."
pkill -f run_qwen_lidar_nav.py || true
pkill -f run_yolo_lidar_failsafe_nav.py || true
pkill -f yolo_world_to_bbox_json.py || true
pkill -f yolo_live_browser_preview.py || true
pkill -f test_qwen_ollama_image.py || true
pkill -f bench_qwen_ollama.py || true
pkill -f compressed_to_raw_image.py || true
pkill -f "hobot_usb_cam" || true
pkill -f "ros2 launch hobot_usb_cam" || true
sleep 2

wait_ros_topic() {
  local topic="$1"
  local label="${2:-$topic}"
  local tries="${3:-20}"
  local per_try_sec="${4:-5}"
  local ok=0
  for i in $(seq 1 "$tries"); do
    if timeout "$per_try_sec" ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      echo "  $label OK"
      ok=1
      break
    fi
    if [ "$label" = "/image" ] || [ "$label" = "camera /image" ]; then
      if ! pgrep -f "hobot_usb_cam" >/dev/null 2>&1; then
        echo "  ERROR: hobot_usb_cam not running (see logs/yolo_failsafe_camera.log)"
        tail -5 logs/yolo_failsafe_camera.log 2>/dev/null || true
        return 1
      fi
    fi
    echo "  waiting $label... ($i/$tries)"
    sleep 1
  done
  if [ "$ok" -ne 1 ]; then
    echo "ERROR: $label not available after $tries attempts."
    return 1
  fi
  return 0
}

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
echo "  CAMERA_DEV=$CAMERA_DEV"
if [ ! -e "$CAMERA_DEV" ]; then
  echo "ERROR: camera device not found: $CAMERA_DEV"
  echo "  try: ls -l /dev/video*"
  echo "  or:  CAMERA_DEV=/dev/video1 bash scripts/nav/start_yolo_lidar_failsafe_nav.sh"
  exit 1
fi

# Use project launch (1280x720 MJPEG) — direct 640x480 launch may crash on some boards.
ros2 launch "$PROJECT_DIR/perception/launch/usb_cam.launch.py" \
  usb_video_device:="$CAMERA_DEV" \
  > logs/yolo_failsafe_camera.log 2>&1 &
sleep 5

if ! wait_ros_topic /image "camera /image" 12 5; then
  echo "Camera failed. Last log lines:"
  tail -15 logs/yolo_failsafe_camera.log 2>/dev/null || true
  exit 1
fi

python3 src/perception/compressed_to_raw_image.py \
  --in-topic "$CAMERA_COMPRESSED_TOPIC" \
  --out-topic "$IMAGE_RAW_TOPIC" \
  --max-fps "$IMAGE_RAW_MAX_FPS" \
  > logs/yolo_failsafe_image_raw.log 2>&1 &
sleep 3

if ! wait_ros_topic "$IMAGE_RAW_TOPIC" "$IMAGE_RAW_TOPIC" 10 5; then
  echo "Image bridge failed. Last log lines:"
  tail -10 logs/yolo_failsafe_image_raw.log 2>/dev/null || true
  exit 1
fi

echo "[3/7] start lidar..."
ros2 launch "$PROJECT_DIR/lidar/launch/tmini_plus.launch.py" > logs/yolo_failsafe_scan.log 2>&1 &
sleep 3

echo "[4/7] start YOLO-World (aligned with start_yolo_live_preview.sh)..."
pkill -f hobot_yolo_world || true
sleep 1

if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source_stage10_env
fi

ros2 run hobot_yolo_world hobot_yolo_world \
  --ros-args \
  -p feed_type:="$YOLO_FEED_TYPE" \
  -p ros_img_sub_topic_name:="$YOLO_IMAGE_TOPIC" \
  -p ros_string_sub_topic_name:=/target_words \
  -p ai_msg_pub_topic_name:="$DET_TOPIC" \
  -p texts:="$TARGET_WORDS" \
  -p score_threshold:="$SCORE_THRESHOLD" \
  -p iou_threshold:="$YOLO_IOU_THRESHOLD" \
  > logs/yolo_failsafe_yolo_world.log 2>&1 &
sleep 4

echo "[5/7] start yolo_world_to_bbox_json bridge..."
python3 src/perception/yolo_world_to_bbox_json.py \
  --config "$CONFIG" \
  > logs/yolo_failsafe_bbox_bridge.log 2>&1 &
sleep 2

echo "[6/7] start chassis bridge..."
pkill -f cmd_vel_to_rosmaster.py || true
python3 ros2_bridge/cmd_vel_to_rosmaster.py \
  --port "$CHASSIS_PORT" \
  --max-vx "$CHASSIS_MAX_VX" \
  --max-wz "$CHASSIS_MAX_WZ" \
  --watchdog-timeout "$CHASSIS_WATCHDOG_TIMEOUT" \
  $( [ "$CHASSIS_ENABLE_KICK_START" = "1" ] && echo "--enable-kick-start" || echo "--no-enable-kick-start" ) \
  --kick-vx "$CHASSIS_KICK_VX" \
  --kick-wz "$CHASSIS_KICK_WZ" \
  --kick-duration "$CHASSIS_KICK_DURATION" \
  --kick-cooldown "$CHASSIS_KICK_COOLDOWN" \
  --cmd-wz-deadzone "$CHASSIS_CMD_WZ_DEADZONE" \
  --cmd-smooth-alpha "$CHASSIS_CMD_SMOOTH_ALPHA" \
  --max-vx-delta "$CHASSIS_MAX_VX_DELTA" \
  --max-wz-delta "$CHASSIS_MAX_WZ_DELTA" \
  --control-rate-hz "$CHASSIS_CONTROL_RATE_HZ" \
  > logs/yolo_failsafe_chassis.log 2>&1 &
sleep 1

echo "[7/7] wait for /scan..."
if ! wait_ros_topic /scan "/scan" 15 5; then
  echo "Check logs/yolo_failsafe_scan.log"
  exit 1
fi

echo "[P0] starting failsafe nav..."
python3 src/apps/run_yolo_lidar_failsafe_nav.py \
  --config "$CONFIG" \
  --instruction "$INSTRUCTION" \
  > logs/yolo_lidar_failsafe_nav.log 2>&1 &

echo "[P0] starting Foxglove viz bridge..."
python3 src/apps/failsafe_nav_foxglove_viz.py \
  --config "$CONFIG" \
  > logs/failsafe_nav_foxglove_viz.log 2>&1 &

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
echo "Foxglove topics:"
echo "  /scan  /failsafe_nav/markers  /failsafe_nav/debug_image"
echo "  /failsafe_nav_state  /target_bbox_json"
echo "See docs/FOXGLOVE_FAILSAFE_NAV.md"
