#!/usr/bin/env bash
# USB camera + /image -> /image_raw bridge helpers.
# NOTE: hobot_usb_cam publishes with sensor/BEST_EFFORT QoS.
#       `ros2 topic hz` defaults to RELIABLE and will falsely report no data.
# Requires: ros2 on PATH, PROJECT_DIR set, caller has sourced ROS env.

camera_process_alive() {
  pgrep -f "hobot_usb_cam" >/dev/null 2>&1
}

stop_camera_stack() {
  pkill -f hobot_usb_cam 2>/dev/null || true
  pkill -f compressed_to_raw_image.py 2>/dev/null || true
  sleep 2
}

start_usb_camera() {
  local dev="${1:-${CAMERA_DEV:-/dev/video0}}"
  local log_file="${2:-logs/semantic_explore_camera.log}"

  echo "[camera] launch ${dev} 1280x720@20 mjpeg -> ${log_file}"
  ros2 launch "$PROJECT_DIR/perception/launch/usb_cam.launch.py" \
    usb_video_device:="${dev}" \
    > "${log_file}" 2>&1 &
}

start_image_bridge() {
  local in_topic="${1:-${CAMERA_COMPRESSED_TOPIC:-/image}}"
  local out_topic="${2:-${IMAGE_RAW_TOPIC:-/image_raw}}"
  local max_fps="${3:-${IMAGE_RAW_MAX_FPS:-8}}"
  local log_file="${4:-logs/semantic_explore_image_raw.log}"

  : > "${log_file}"
  python3 "$PROJECT_DIR/src/perception/compressed_to_raw_image.py" \
    --in-topic "${in_topic}" \
    --out-topic "${out_topic}" \
    --max-fps "${max_fps}" \
    > "${log_file}" 2>&1 &
}

wait_camera_process() {
  local timeout_sec="${1:-20}"
  echo "[camera] waiting for hobot_usb_cam process ..."
  for _ in $(seq 1 "$timeout_sec"); do
    if camera_process_alive; then
      echo "[camera] OK: hobot_usb_cam running"
      return 0
    fi
    sleep 1
  done
  echo "[camera] FAIL: hobot_usb_cam not running"
  return 1
}

wait_bridge_publishing() {
  local log_file="${1:-logs/semantic_explore_image_raw.log}"
  local timeout_sec="${2:-35}"
  echo "[camera] waiting for bridge to publish /image_raw (check ${log_file}) ..."
  for _ in $(seq 1 "$timeout_sec"); do
    if grep -q "published /image_raw" "${log_file}" 2>/dev/null; then
      echo "[camera] OK: /image_raw bridge publishing"
      return 0
    fi
    if grep -qi "convert failed\|imdecode returned None" "${log_file}" 2>/dev/null; then
      echo "[camera] FAIL: bridge decode error"
      tail -n 10 "${log_file}" 2>/dev/null || true
      return 1
    fi
    if ! camera_process_alive; then
      echo "[camera] FAIL: camera died while waiting for bridge"
      return 1
    fi
    sleep 1
  done
  echo "[camera] FAIL: no /image_raw output within ${timeout_sec}s"
  tail -n 15 "${log_file}" 2>/dev/null || true
  return 1
}

camera_stack_diagnose() {
  local cam_log="${1:-logs/semantic_explore_camera.log}"
  local bridge_log="${2:-logs/semantic_explore_image_raw.log}"
  echo "[camera] --- diagnose ---"
  if camera_process_alive; then
    echo "[camera] hobot_usb_cam: running"
  else
    echo "[camera] hobot_usb_cam: NOT running"
  fi
  tail -n 12 "${cam_log}" 2>/dev/null || true
  tail -n 8 "${bridge_log}" 2>/dev/null || true
}

ensure_camera_image_stream() {
  local cam_log="${1:-logs/semantic_explore_camera.log}"
  local dev="${CAMERA_DEV:-/dev/video0}"

  stop_camera_stack
  start_usb_camera "${dev}" "${cam_log}"
  sleep 5
  if ! wait_camera_process 15; then
    camera_stack_diagnose "${cam_log}"
    return 1
  fi
  if grep -qi "Select timeout\|process has died\|terminate called" "${cam_log}" 2>/dev/null; then
    echo "[camera] FAIL: camera error in log"
    camera_stack_diagnose "${cam_log}"
    return 1
  fi
  # Camera opened successfully; give it a moment before bridge subscribes.
  sleep 2
  return 0
}

ensure_image_raw_stream() {
  local bridge_log="${1:-logs/semantic_explore_image_raw.log}"
  start_image_bridge "" "" "" "${bridge_log}"
  sleep 1
  wait_bridge_publishing "${bridge_log}" 35
}
