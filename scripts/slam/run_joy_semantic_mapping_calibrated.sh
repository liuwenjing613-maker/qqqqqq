#!/usr/bin/env bash
# Calibrated joystick SLAM + semantic mapping overlay.
# Drive manually with joystick. Ctrl+C saves semantic map then SLAM map.

set -u

cd ~/rdk_x5_vln_robot

# shellcheck source=scripts/lib/slam_calibrated_env.sh
source "${PWD}/scripts/lib/slam_calibrated_env.sh"
export SLAM_USE_CALIBRATION=1

LOG_DIR="$PWD/logs/joy_semantic_mapping_calibrated"
MAP_DIR="$PWD/maps"
MAP_NAME="${MAP_NAME:-joy_semantic_calibrated_map}"
SEMANTIC_CONFIG="${SEMANTIC_CONFIG:-configs/semantic_mapping.yaml}"
NAV_DIRECTION_CONFIG="${NAV_DIRECTION_CONFIG:-configs/nav_yolo_lidar.yaml}"
SEMANTIC_CLASSES="${SEMANTIC_CLASSES:-bottle,cup,backpack,chair,dining table,book,potted plant,cell phone,couch}"
CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"

YOLOV5S_MODEL="${YOLOV5S_MODEL:-/root/rdk_model_zoo/samples/vision/yolov5/model/yolov5s_tag_v7.0_detect_640x640_bayese_nv12.bin}"
YOLOV5S_RUNTIME_DIR="${YOLOV5S_RUNTIME_DIR:-/root/rdk_model_zoo/samples/vision/yolov5/runtime/python}"
YOLOV5S_ZOO_ROOT="${YOLOV5S_ZOO_ROOT:-/root/rdk_model_zoo}"

mkdir -p "$LOG_DIR" "$MAP_DIR"
PIDS=()
SAVED=0
CLEANUP_DONE=0
ROS_ENV_READY=0

source_ros() {
  if [ "$ROS_ENV_READY" = "1" ]; then
    return 0
  fi
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
  ROS_ENV_READY=1
}

log() {
  echo "[$(date +%H:%M:%S)] $*"
}

load_nav_lidar_direction() {
  # Keep camera<->LiDAR bearing aligned with scripts/nav/start_yolo_lidar_nav.sh
  local nav_cfg="${PWD}/${NAV_DIRECTION_CONFIG}"
  if [ ! -f "$nav_cfg" ]; then
    log "WARN: NAV_DIRECTION_CONFIG not found: ${nav_cfg}"
    return 0
  fi
  eval "$(python3 - "$nav_cfg" <<'PY'
import shlex
import sys

import yaml

with open(sys.argv[1], encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}

mapping = {
    "SEMANTIC_CAMERA_TO_LASER_YAW_DEG": cfg.get("camera_lidar_yaw_offset_deg", 0.0),
    "SEMANTIC_CAMERA_HFOV_DEG": cfg.get("camera_hfov_deg", 70.0),
    "SEMANTIC_LIDAR_FRONT_DEG": cfg.get("lidar_front_deg", 30.0),
}
for key, value in mapping.items():
    print(f"export {key}={shlex.quote(str(value))}")
PY
)"
  log "LiDAR direction aligned with ${NAV_DIRECTION_CONFIG}:"
  log "  camera_to_laser_yaw_deg=${SEMANTIC_CAMERA_TO_LASER_YAW_DEG:-?}"
  log "  camera_hfov_deg=${SEMANTIC_CAMERA_HFOV_DEG:-?}"
  log "  lidar_front_deg=${SEMANTIC_LIDAR_FRONT_DEG:-?}"
}

start_bg() {
  local name="$1"
  shift
  log "Starting ${name} ..."
  "$@" > "${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
  sleep 1
}

zero_cmd() {
  source_ros
  timeout 1.2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
    >/dev/null 2>&1 || true
}

wait_topic_exists() {
  local topic="$1"
  local timeout_sec="${2:-60}"
  log "Waiting for ${topic} ..."
  for _ in $(seq 1 "$timeout_sec"); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      log "OK: ${topic}"
      return 0
    fi
    sleep 1
  done
  log "FAIL: timeout waiting for ${topic}"
  return 1
}

stop_stale_live_stack() {
  log "Stopping stale SLAM live stack processes ..."
  pkill -TERM -f "run_slam_calibrated.sh" 2>/dev/null || true
  pkill -TERM -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
  sleep 2
  pkill -KILL -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
  pkill -KILL -f "run_slam_calibrated.sh" 2>/dev/null || true
  sleep 1
}

ensure_foxglove_bridge() {
  local port="${FOXGLOVE_PORT:-8765}"
  source_ros
  if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    log "WARN: foxglove_bridge not installed"
    return 1
  fi
  if ss -tln 2>/dev/null | grep -q ":${port} "; then
    log "OK: Foxglove bridge listening on ${port}"
    return 0
  fi
  log "WARN: Foxglove not listening on ${port}; restarting bridge ..."
  pkill -f "foxglove_bridge" 2>/dev/null || true
  sleep 1
  start_bg foxglove_bridge bash "$PWD/scripts/lidar/start_foxglove.sh"
  for _ in $(seq 1 15); do
    if ss -tln 2>/dev/null | grep -q ":${port} "; then
      log "OK: Foxglove bridge listening on ${port}"
      return 0
    fi
    sleep 1
  done
  log "ERROR: Foxglove bridge failed to start on ${port}"
  tail -n 40 "${LOG_DIR}/foxglove_bridge.log" 2>/dev/null || true
  return 1
}

trigger_semantic_save() {
  source_ros
  log "Trigger semantic map save ..."
  timeout 2 ros2 topic pub --once /semantic_map/save std_msgs/msg/String \
    "{data: 'final_save'}" >/dev/null 2>&1 || true
  sleep 2
}

save_map() {
  source_ros
  local MAP_OUT="${MAP_DIR}/${MAP_NAME}"
  local MAP_TMP="${MAP_OUT}.tmp_$(date +%Y%m%d_%H%M%S)"
  log "Saving SLAM map to ${MAP_OUT} ..."
  timeout 30 ros2 run nav2_map_server map_saver_cli \
    -t /map \
    -f "$MAP_TMP" \
    --ros-args -p save_map_timeout:=20.0 \
    >> "${LOG_DIR}/map_saver.log" 2>&1

  if [ -f "${MAP_TMP}.pgm" ] && [ -f "${MAP_TMP}.yaml" ]; then
    mv "${MAP_TMP}.pgm" "${MAP_OUT}.pgm"
    mv "${MAP_TMP}.yaml" "${MAP_OUT}.yaml"
    log "Map saved: ${MAP_OUT}.yaml/.pgm"
    SAVED=1
  else
    log "ERROR: map save failed. See ${LOG_DIR}/map_saver.log"
    return 1
  fi
}

stop_joystick_nodes() {
  pkill -f "teleop_twist_joy" 2>/dev/null || true
  pkill -f "joy_node" 2>/dev/null || true
}

stop_semantic_stack() {
  pkill -f "semantic_mapper_node.py" 2>/dev/null || true
  pkill -f "yolov5s_bpu_web_node.py" 2>/dev/null || true
  pkill -f "compressed_to_raw_image.py" 2>/dev/null || true
  pkill -f "hobot_usb_cam" 2>/dev/null || true
}

stop_live_stack() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  pkill -TERM -f "run_slam_calibrated.sh" 2>/dev/null || true
  pkill -TERM -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
  pkill -TERM -f "async_slam_toolbox_node" 2>/dev/null || true
  pkill -TERM -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -TERM -f "ydlidar_ros2_driver_node" 2>/dev/null || true
  pkill -TERM -f "foxglove_bridge" 2>/dev/null || true
  pkill -TERM -f "simple_scan_filter.py" 2>/dev/null || true
  sleep 1
}

cleanup() {
  if [ "$CLEANUP_DONE" = "1" ]; then
    return 0
  fi
  CLEANUP_DONE=1

  echo
  log "Ctrl+C: save semantic map and SLAM map, then stop nodes..."

  stop_joystick_nodes
  zero_cmd
  trigger_semantic_save
  save_map || true
  zero_cmd

  stop_semantic_stack
  stop_live_stack

  log "Done."
  exit 0
}
trap cleanup INT TERM

main() {
  source_ros
  log "===== Joystick SLAM + Semantic Mapping ====="
  log "MAP_NAME=${MAP_NAME}"
  log "SEMANTIC_CONFIG=${SEMANTIC_CONFIG}"
  log "SEMANTIC_CLASSES=${SEMANTIC_CLASSES}"
  load_nav_lidar_direction

  zero_cmd
  stop_joystick_nodes
  stop_stale_live_stack
  sleep 1

  log "[1/7] Start calibrated SLAM live stack"
  start_bg live_stack setsid bash scripts/slam/run_slam_calibrated.sh

  wait_topic_exists /scan 90 || exit 1
  wait_topic_exists /scan_filtered 90 || exit 1
  wait_topic_exists /odom 90 || exit 1
  wait_topic_exists /map 90 || exit 1
  wait_topic_exists /tf 40 || exit 1

  log "[2/7] Start camera"
  start_bg camera ros2 launch "$PWD/perception/launch/usb_cam.launch.py" \
    usb_video_device:="$CAMERA_DEV"
  sleep 5

  log "[3/7] Start image bridge /image -> /image_raw"
  start_bg image_raw python3 "$PWD/src/perception/compressed_to_raw_image.py" \
    --in-topic /image \
    --out-topic /image_raw \
    --max-fps 8
  wait_topic_exists /image_raw 30 || exit 1

  log "[4/7] Start YOLOv5s-BPU semantic detector"
  start_bg yolo_bpu python3 "$PWD/src/perception/yolov5s_bpu_web_node.py" \
    --model "$YOLOV5S_MODEL" \
    --runtime-dir "$YOLOV5S_RUNTIME_DIR" \
    --zoo-root "$YOLOV5S_ZOO_ROOT" \
    --input-type raw \
    --image-topic /image_raw \
    --out-topic /target_bbox_json \
    --target-classes "$SEMANTIC_CLASSES" \
    --score-thres 0.20 \
    --nms-thres 0.45 \
    --max-hz 6.0 \
    --jpeg-quality 35 \
    --semantic-depth-overlay \
    --semantic-obs-topic /semantic_observations
  wait_topic_exists /target_bbox_json 40 || exit 1

  log "[5/7] Start semantic mapper"
  start_bg semantic_mapper python3 "$PWD/src/mapping/semantic_mapper_node.py" \
    --config "$SEMANTIC_CONFIG" \
    --map-name "$MAP_NAME"
  wait_topic_exists /semantic_map_json 30 || {
    log "ERROR: semantic mapper did not publish /semantic_map_json"
    tail -n 120 "${LOG_DIR}/semantic_mapper.log" || true
    exit 1
  }

  log "[6/7] Start joystick"
  start_bg joy_node ros2 run joy joy_node --ros-args \
    -p dev:="$JOY_DEV" \
    -p deadzone:=0.15 \
    -p autorepeat_rate:=20.0
  wait_topic_exists /joy 30 || exit 1

  log "[7/7] Start teleop"
  start_bg teleop ros2 run teleop_twist_joy teleop_node --ros-args \
    -p require_enable_button:=false \
    -p axis_linear.x:=1 \
    -p scale_linear.x:="$JOY_SCALE_LINEAR_X" \
    -p axis_angular.yaw:=0 \
    -p scale_angular.yaw:="$JOY_SCALE_ANGULAR_YAW"

  ensure_foxglove_bridge || true

  local ip
  ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo
  echo "========== Running =========="
  echo "Foxglove: ws://${ip}:8765"
  echo "Layout: ${PWD}/configs/foxglove_joy_semantic_mapping.layout.json"
  echo "View topics: /map /scan_filtered /tf /semantic_landmarks /semantic_viewpoints /semantic_observed_cones"
  echo "YOLO preview: http://${ip}:8088/"
  echo "Press Ctrl+C to save semantic map + SLAM map."
  echo

  while true; do
    sleep 3600
  done
}

main "$@"
