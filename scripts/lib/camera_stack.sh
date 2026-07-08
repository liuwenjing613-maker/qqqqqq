#!/usr/bin/env bash
# USB camera + /image -> /image_raw bridge helpers.
# NOTE: hobot_usb_cam publishes with sensor/BEST_EFFORT QoS.
#       `ros2 topic hz` defaults to RELIABLE and will falsely report no data.
# Requires: ros2 on PATH, PROJECT_DIR set, caller has sourced ROS env.

camera_process_alive() {
  pgrep -f "hobot_usb_cam" >/dev/null 2>&1
}

stop_camera_stack() {
  pkill -TERM -f "usb_cam.launch.py" 2>/dev/null || true
  pkill -TERM -f "hobot_usb_cam" 2>/dev/null || true
  pkill -TERM -f "compressed_to_raw_image.py" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "usb_cam.launch.py" 2>/dev/null || true
  pkill -KILL -f "hobot_usb_cam" 2>/dev/null || true
  pkill -KILL -f "compressed_to_raw_image.py" 2>/dev/null || true
  sleep 1
  local dev="${CAMERA_DEV:-/dev/video0}"
  for _ in $(seq 1 10); do
    if ! fuser "$dev" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  echo "[camera] WARN: ${dev} still in use after stop"
}

start_usb_camera() {
  local dev="${1:-${CAMERA_DEV:-/dev/video0}}"
  local log_file="${2:-logs/semantic_explore_camera.log}"
  local width="${3:-${CAMERA_WIDTH:-1280}}"
  local height="${4:-${CAMERA_HEIGHT:-720}}"
  local fps="${5:-${CAMERA_FPS:-20}}"

  echo "[camera] launch ${dev} ${width}x${height}@${fps} mjpeg -> ${log_file}"
  : > "${log_file}"
  ros2 launch "$PROJECT_DIR/perception/launch/usb_cam.launch.py" \
    usb_video_device:="${dev}" \
    usb_image_width:="${width}" \
    usb_image_height:="${height}" \
    usb_framerate:="${fps}" \
    > "${log_file}" 2>&1 &
  echo $! > "${log_file}.pid"
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

camera_log_has_fatal_error() {
  local cam_log="$1"
  grep -qi "process has died\|terminate called\|Select timeout\|Failed to open\|can't open" "${cam_log}" 2>/dev/null
}

wait_camera_process() {
  local cam_log="${1:-logs/semantic_explore_camera.log}"
  local timeout_sec="${2:-20}"
  echo "[camera] waiting for hobot_usb_cam process ..."
  for _ in $(seq 1 "$timeout_sec"); do
    if camera_log_has_fatal_error "${cam_log}"; then
      echo "[camera] FAIL: camera error in log"
      return 1
    fi
    if camera_process_alive; then
      if grep -q "process started with pid" "${cam_log}" 2>/dev/null; then
        echo "[camera] OK: hobot_usb_cam running"
        return 0
      fi
    fi
    sleep 1
  done
  echo "[camera] FAIL: hobot_usb_cam not running"
  return 1
}

_try_start_camera_profile() {
  local cam_log="$1"
  local dev="$2"
  local width="$3"
  local height="$4"
  local fps="$5"

  stop_camera_stack
  start_usb_camera "${dev}" "${cam_log}" "${width}" "${height}" "${fps}"
  sleep 4
  wait_camera_process "${cam_log}" 15 || return 1
  if camera_log_has_fatal_error "${cam_log}"; then
    return 1
  fi
  sleep 2
  return 0
}

ensure_camera_image_stream() {
  local cam_log="${1:-logs/semantic_explore_camera.log}"
  local dev="${CAMERA_DEV:-/dev/video0}"

  if _try_start_camera_profile "${cam_log}" "${dev}" 1280 720 20; then
    return 0
  fi
  echo "[camera] WARN: 1280x720@20 failed, retry 640x480@15 ..."
  camera_stack_diagnose "${cam_log}"
  if _try_start_camera_profile "${cam_log}" "${dev}" 640 480 15; then
    return 0
  fi
  echo "[camera] FAIL: camera could not start"
  camera_stack_diagnose "${cam_log}"
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

ensure_image_raw_stream() {
  local bridge_log="${1:-logs/semantic_explore_image_raw.log}"
  start_image_bridge "" "" "" "${bridge_log}"
  sleep 1
  wait_bridge_publishing "${bridge_log}" 35
}
