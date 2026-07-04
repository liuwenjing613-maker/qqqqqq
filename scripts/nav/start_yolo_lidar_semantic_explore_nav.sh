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
  if [ -f "$HOME/ydlidar_ws/install/setup.bash" ]; then
    source "$HOME/ydlidar_ws/install/setup.bash"
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

CONFIG="${1:-configs/nav_yolo_lidar_semantic_explore.yaml}"
USER_INSTRUCTION="${2:-}"
NAV_ONLY="${NAV_ONLY:-0}"
QWEN_TEXT_ENABLED="${QWEN_TEXT_ENABLED:-}"
_CAMERA_OVERRIDE="${CAMERA_DEV:-}"
_MAP_NAME="${MAP_NAME:-semantic_explore_live}"

eval "$(python3 "$PROJECT_DIR/src/config/failsafe_nav_launch.py" --config "$CONFIG" --shell-export)"
if [ -n "$USER_INSTRUCTION" ]; then
  export INSTRUCTION="$USER_INSTRUCTION"
fi
if [ -n "$_CAMERA_OVERRIDE" ]; then
  export CAMERA_DEV="$_CAMERA_OVERRIDE"
fi

SEMANTIC_EXPLORE_ENABLED="$(python3 - "$CONFIG" <<'PY'
import sys, yaml
with open(sys.argv[1], encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}
print("1" if cfg.get("semantic_explore", {}).get("enabled", True) else "0")
PY
)"

mkdir -p logs

echo "===== semantic_explore_nav ====="
echo "CONFIG=$CONFIG"
echo "INSTRUCTION=$INSTRUCTION"
echo "NAV_ONLY=$NAV_ONLY"
echo "SEMANTIC_EXPLORE_ENABLED=$SEMANTIC_EXPLORE_ENABLED"

stop_stale_live_stack() {
  echo "[semantic_explore] stopping stale SLAM live stack..."
  pkill -TERM -f "run_slam_calibrated.sh" 2>/dev/null || true
  pkill -TERM -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
  sleep 2
  pkill -KILL -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
  pkill -KILL -f "run_slam_calibrated.sh" 2>/dev/null || true
  sleep 1
}

wait_topic_exists() {
  local topic="$1"
  local timeout_sec="${2:-60}"
  echo "[semantic_explore] waiting for ${topic} ..."
  for _ in $(seq 1 "$timeout_sec"); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      echo "[semantic_explore] OK: ${topic}"
      return 0
    fi
    sleep 1
  done
  echo "[semantic_explore] FAIL: timeout waiting for ${topic}"
  return 1
}

ensure_foxglove_bridge() {
  local port="${FOXGLOVE_PORT:-8765}"
  if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    echo "[foxglove] WARN: foxglove_bridge not installed"
    return 1
  fi
  if ss -tln 2>/dev/null | grep -q ":${port} "; then
    echo "[foxglove] OK: listening on ${port}"
    return 0
  fi
  pkill -f "foxglove_bridge" 2>/dev/null || true
  sleep 1
  bash "$PROJECT_DIR/scripts/lidar/start_foxglove.sh" \
    > "$PROJECT_DIR/logs/semantic_explore_foxglove_bridge.log" 2>&1 &
  for _ in $(seq 1 15); do
    if ss -tln 2>/dev/null | grep -q ":${port} "; then
      echo "[foxglove] OK: listening on ${port}"
      return 0
    fi
    sleep 1
  done
  echo "[foxglove] WARN: bridge failed to start on ${port}"
  return 1
}

pkill -f run_shared_nav_semantic_explore.py || true
pkill -f explore_goal_selector.py || true
pkill -f semantic_mapper_node.py || true
pkill -f yolov5s_bpu_web_node.py || true
pkill -f yolo_world_to_bbox_json.py || true
pkill -f hobot_yolo_world || true
pkill -f compressed_to_raw_image.py || true

timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
  >/dev/null 2>&1 || true

if [ "$NAV_ONLY" = "1" ]; then
  echo "[semantic_explore] NAV_ONLY=1: selector + semantic nav only"
  if [ "$SEMANTIC_EXPLORE_ENABLED" = "1" ]; then
    python3 "$PROJECT_DIR/src/planning/explore_goal_selector.py" \
      --config "$CONFIG" \
      --instruction "$INSTRUCTION" \
      > "$PROJECT_DIR/logs/semantic_explore_selector.log" 2>&1 &
  fi
  python3 "$PROJECT_DIR/src/apps/run_shared_nav_semantic_explore.py" \
    --config "$CONFIG" \
    --instruction "$INSTRUCTION" \
    > "$PROJECT_DIR/logs/semantic_explore_nav.log" 2>&1 &
  echo "[semantic_explore] started nav-only stack"
  exit 0
fi

stop_stale_live_stack

# shellcheck source=scripts/lib/slam_calibrated_env.sh
source "${PWD}/scripts/lib/slam_calibrated_env.sh"
export SLAM_USE_CALIBRATION=1

echo "[1/8] Start calibrated SLAM live stack (joy style, includes lidar+chassis+slam)..."
setsid bash "$PROJECT_DIR/scripts/slam/run_slam_calibrated.sh" \
  > "$PROJECT_DIR/logs/semantic_explore_slam.log" 2>&1 &

wait_topic_exists /scan 90 || exit 1
wait_topic_exists /scan_filtered 90 || exit 1
wait_topic_exists /odom 90 || exit 1
wait_topic_exists /map 90 || exit 1
wait_topic_exists /tf 40 || exit 1

# shellcheck source=scripts/lib/camera_stack.sh
source "${PWD}/scripts/lib/camera_stack.sh"

echo "[2/8] Start camera + image bridge..."
ensure_camera_image_stream logs/semantic_explore_camera.log || {
  echo "[semantic_explore] ERROR: camera failed; see logs/semantic_explore_camera.log"
  exit 1
}
ensure_image_raw_stream logs/semantic_explore_image_raw.log || {
  echo "[semantic_explore] ERROR: image bridge failed; see logs/semantic_explore_image_raw.log"
  camera_stack_diagnose logs/semantic_explore_camera.log logs/semantic_explore_image_raw.log
  exit 1
}

echo "[3/8] Start detector..."
if [ "${DETECTOR_BACKEND:-yolov5s_bpu}" = "yolov5s_bpu" ]; then
  python3 "$PROJECT_DIR/src/perception/yolov5s_bpu_web_node.py" \
    --model "$YOLOV5S_MODEL" \
    --runtime-dir "$YOLOV5S_RUNTIME_DIR" \
    --zoo-root "$YOLOV5S_ZOO_ROOT" \
    --input-type "$YOLOV5S_INPUT_TYPE" \
    --image-topic "$YOLOV5S_IMAGE_TOPIC" \
    --out-topic "$YOLOV5S_OUT_TOPIC" \
    --target-words-topic "$YOLOV5S_TARGET_WORDS_TOPIC" \
    --target-classes "$TARGET_CLASSES" \
    --score-thres "$SCORE_THRESHOLD" \
    --nms-thres "$YOLOV5S_NMS_THRESHOLD" \
    --max-hz "$YOLOV5S_MAX_HZ" \
    --semantic-depth-overlay \
    --semantic-obs-topic /semantic_observations \
    > logs/semantic_explore_yolov5s_bpu.log 2>&1 &
else
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
    > logs/semantic_explore_yolo_world.log 2>&1 &
  sleep 4
  python3 "$PROJECT_DIR/src/perception/yolo_world_to_bbox_json.py" \
    --config "$CONFIG" \
    > logs/semantic_explore_bbox_bridge.log 2>&1 &
fi
sleep 4

echo "[4/8] Start semantic mapper..."
python3 "$PROJECT_DIR/src/mapping/semantic_mapper_node.py" \
  --config "$CONFIG" \
  --map-name "$_MAP_NAME" \
  > logs/semantic_explore_mapper.log 2>&1 &
wait_topic_exists /semantic_map_json 60 || {
  echo "[semantic_explore] ERROR: /semantic_map_json not published"
  tail -n 80 logs/semantic_explore_mapper.log 2>/dev/null || true
  exit 1
}

if [ "$SEMANTIC_EXPLORE_ENABLED" = "1" ]; then
  echo "[5/8] Start explore goal selector..."
  python3 "$PROJECT_DIR/src/planning/explore_goal_selector.py" \
    --config "$CONFIG" \
    --instruction "$INSTRUCTION" \
    > logs/semantic_explore_selector.log 2>&1 &
  sleep 2
else
  echo "[5/8] semantic_explore.enabled=false, skip selector"
fi

echo "[6/8] Start semantic explore nav (no separate chassis; SLAM stack owns it)..."
python3 "$PROJECT_DIR/src/apps/run_shared_nav_semantic_explore.py" \
  --config "$CONFIG" \
  --instruction "$INSTRUCTION" \
  > logs/semantic_explore_nav.log 2>&1 &

echo "[7/8] Foxglove bridge..."
ensure_foxglove_bridge || true

echo "[8/8] semantic_explore_nav started"
echo "  tail -f logs/semantic_explore_nav.log"
echo "  tail -f logs/semantic_explore_selector.log"
echo "  ros2 topic echo /explore_goal_hint"
echo "  ros2 topic echo /explore_state_json"
echo "  ros2 topic echo /nav_state"
echo "  Foxglove 3D topics:"
echo "    /explore_candidate_goals  (yellow/orange/green candidates + rank labels)"
echo "    /explore_selected_goal    (green selected + look-at arrow)"
echo "    /explore_selection_process (robot cyan, link line, pending orange, status text)"
echo "    /explore_astar_markers    (cyan A* path + waypoints)"
echo "    /explore_path             (nav_msgs/Path)"
echo "    /explore_frontiers        (blue frontier clusters)"
echo "  Layout: ${PROJECT_DIR}/configs/foxglove_semantic_explore_nav.layout.json"
