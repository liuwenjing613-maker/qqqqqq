#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
cd "$PROJECT_DIR"

set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source "$PROJECT_DIR/source_stage10.sh" || true
fi
set -u

CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
MODEL="${MODEL:-$PROJECT_DIR/models/yolov5s.onnx}"
CLASSES="${CLASSES:-cup,bottle,backpack}"
CONF="${CONF:-0.25}"
IMGSZ="${IMGSZ:-640}"
MAX_FPS="${MAX_FPS:-4.0}"
DEBUG_SCALE="${DEBUG_SCALE:-0.45}"
JPEG_QUALITY="${JPEG_QUALITY:-55}"
OPENCV_THREADS="${OPENCV_THREADS:-4}"
PUBLISH_RAW_DEBUG="${PUBLISH_RAW_DEBUG:-0}"
WEB_PORT="${WEB_PORT:-8765}"
DETECTOR_WAIT_SEC="${DETECTOR_WAIT_SEC:-45}"
DEBUG_TOPIC="/yolov5s/debug_image/compressed"

mkdir -p logs

# pipefail + "ros2 topic list | grep -q" breaks: grep exits early -> ros2 gets SIGPIPE (141).
ros2_has_topic() {
  local topic="$1"
  grep -Fxq "$topic" < <(ros2 topic list 2>/dev/null || true)
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="${2:-30}"
  local start=$SECONDS
  while (( SECONDS - start < timeout_sec )); do
    if ros2_has_topic "$topic"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_process() {
  local pattern="$1"
  local timeout_sec="${2:-30}"
  local start=$SECONDS
  while (( SECONDS - start < timeout_sec )); do
    if pgrep -f "$pattern" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

echo "===== YOLOv5s ONNX + Foxglove realtime test ====="
echo "PROJECT_DIR=$PROJECT_DIR"
echo "CAMERA_DEV=$CAMERA_DEV"
echo "MODEL=$MODEL"
echo "CLASSES=$CLASSES"
echo "CONF=$CONF"
echo "IMGSZ=$IMGSZ"
echo "MAX_FPS=$MAX_FPS"
echo "DEBUG_SCALE=$DEBUG_SCALE"
echo "JPEG_QUALITY=$JPEG_QUALITY"
echo "OPENCV_THREADS=$OPENCV_THREADS"

echo "[0] stop old visual/nav processes..."
pkill -f run_yolo_lidar_failsafe_nav.py || true
pkill -f yolo_world_to_bbox_json.py || true
pkill -f yolo_live_browser_preview.py || true
pkill -f yolov5s_onnx_ros.py || true
pkill -f failsafe_nav_foxglove_viz.py || true
pkill -f "hobot_yolo_world" || true
pkill -f "hobot_usb_cam" || true
pkill -f "compressed_to_raw_image.py" || true
pkill -f "foxglove_bridge" || true

# 不让车动，安全一点。毕竟轮子不会因为你只是测试视觉就自觉克制。
timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
  >/dev/null 2>&1 || true

sleep 2

if [ ! -e "$CAMERA_DEV" ]; then
  echo "ERROR: camera device not found: $CAMERA_DEV"
  echo "try: ls -l /dev/video*"
  exit 1
fi

if [ ! -f "$MODEL" ]; then
  echo "ERROR: model not found: $MODEL"
  echo "put your model at: $PROJECT_DIR/models/yolov5s.onnx"
  exit 1
fi

echo "[1] start camera /image ..."
ros2 launch "$PROJECT_DIR/perception/launch/usb_cam.launch.py" \
  usb_video_device:="$CAMERA_DEV" \
  > logs/yolov5s_camera.log 2>&1 &

echo "[2] wait for /image ..."
if ! wait_for_topic "/image" 20; then
  echo "ERROR: /image not ready within 20s. camera log:"
  tail -30 logs/yolov5s_camera.log || true
  exit 1
fi
if ! timeout 8 ros2 topic echo --once /image >/dev/null 2>&1; then
  echo "ERROR: /image advertised but no data yet. camera log:"
  tail -30 logs/yolov5s_camera.log || true
  exit 1
fi
echo "OK: /image active"

echo "[3] start yolov5s onnx detector ..."
DETECTOR_ARGS=(
  "$PROJECT_DIR/src/perception/yolov5s_onnx_ros.py"
  --model "$MODEL"
  --image-topic /image
  --image-type compressed
  --classes "$CLASSES"
  --conf "$CONF"
  --imgsz "$IMGSZ"
  --max-fps "$MAX_FPS"
  --debug-scale "$DEBUG_SCALE"
  --jpeg-quality "$JPEG_QUALITY"
  --opencv-threads "$OPENCV_THREADS"
)
if [ "$PUBLISH_RAW_DEBUG" = "1" ]; then
  DETECTOR_ARGS+=(--publish-raw-debug)
  DEBUG_TOPIC="/yolov5s/debug_image"
fi
python3 "${DETECTOR_ARGS[@]}" \
  > logs/yolov5s_onnx_ros.log 2>&1 &

echo "[4] wait for detector process and topics ..."
if ! wait_for_process "yolov5s_onnx_ros.py" 10; then
  echo "ERROR: yolov5s_onnx_ros.py did not start. log:"
  tail -30 logs/yolov5s_onnx_ros.log || true
  exit 1
fi
if ! wait_for_topic "$DEBUG_TOPIC" "$DETECTOR_WAIT_SEC"; then
  echo "ERROR: $DEBUG_TOPIC not advertised within ${DETECTOR_WAIT_SEC}s. log:"
  tail -30 logs/yolov5s_onnx_ros.log || true
  exit 1
fi
if ! timeout 12 ros2 topic echo --once "$DEBUG_TOPIC" >/dev/null 2>&1; then
  echo "WARN: $DEBUG_TOPIC advertised but first frame not yet (inference may be slow)."
  echo "      check: tail -f logs/yolov5s_onnx_ros.log"
fi
echo "OK: detector topics ready"
grep -E "^/yolov5s/|^/target_bbox_json$" < <(ros2 topic list 2>/dev/null || true) || true

FAILSAFE_CFG="$PROJECT_DIR/configs/yolo_lidar_failsafe_nav.yaml"
if [ ! -f "$FAILSAFE_CFG" ] && [ -f "$PROJECT_DIR/configs/archive_legacy/yolo_lidar_failsafe_nav.yaml" ]; then
  FAILSAFE_CFG="$PROJECT_DIR/configs/archive_legacy/yolo_lidar_failsafe_nav.yaml"
fi

echo "[5] start optional failsafe Foxglove viz bridge ..."
if [ -f "$FAILSAFE_CFG" ]; then
  python3 "$PROJECT_DIR/src/apps/failsafe_nav_foxglove_viz.py" \
    --config "$FAILSAFE_CFG" \
    > logs/yolov5s_failsafe_viz.log 2>&1 &
else
  echo "SKIP: failsafe viz config not found (optional for yolov5s test)"
fi

sleep 2

echo "[6] start foxglove_bridge ws://:${WEB_PORT} ..."
if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:="${WEB_PORT}" \
    topic_whitelist:="['.*']" \
    send_buffer_limit:=10000000 \
    max_qos_depth:=10 \
    > logs/yolov5s_foxglove_bridge.log 2>&1 &
else
  echo "ERROR: foxglove_bridge not installed."
  echo "Install:"
  echo "  sudo apt update"
  echo "  sudo apt install -y ros-humble-foxglove-bridge"
  exit 1
fi

sleep 3
if ! wait_for_process "foxglove_bridge" 15; then
  echo "ERROR: foxglove_bridge did not start. log:"
  tail -30 logs/yolov5s_foxglove_bridge.log || true
  exit 1
fi

BOARD_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"

echo ""
echo "===== STARTED ====="
echo "Foxglove connect:"
echo "  ws://${BOARD_IP}:${WEB_PORT}"
echo ""
echo "Foxglove panels:"
echo "  Image: /yolov5s/debug_image/compressed"
echo "  Image raw optional: /yolov5s/debug_image  (set PUBLISH_RAW_DEBUG=1)"
echo "  Image optional: /failsafe_nav/debug_image"
echo "  Raw: /yolov5s/stability"
echo "  Raw: /target_bbox_json"
echo "  Plot: /yolov5s/fps"
echo "  Plot: /yolov5s/infer_ms"
echo "  Plot: /yolov5s/visible_rate"
echo "  Plot: /yolov5s/score"
echo "  Plot: /yolov5s/jitter_px"
echo ""
echo "Logs:"
echo "  tail -f logs/yolov5s_onnx_ros.log"
echo ""
echo "Speed knobs:"
echo "  faster preview: DEBUG_SCALE=0.35 JPEG_QUALITY=45 OPENCV_THREADS=4 bash scripts/yolo/start_yolov5s_foxglove_test.sh"
echo "  clearer preview: DEBUG_SCALE=0.6 JPEG_QUALITY=70 bash scripts/yolo/start_yolov5s_foxglove_test.sh"
echo "  note: IMGSZ must stay 640 for bundled yolov5s.onnx"
echo ""
echo "Stop:"
echo "  pkill -f yolov5s_onnx_ros.py; pkill -f hobot_usb_cam; pkill -f foxglove_bridge"
