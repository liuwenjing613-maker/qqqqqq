#!/usr/bin/env bash
# Project camera chain:
# hobot_usb_cam -> /image (CompressedImage)
# bridge        -> /image_raw (Image bgr8, RELIABLE)
#
# Failure modes this file defends against:
# 1) `ros2 topic info` can hang under a busy DDS graph -> always wrap with timeout.
# 2) Killing only the ros2 launch PID orphans hobot_usb_cam on /dev/video0;
#    the next launch then falls through to /dev/video1 and aborts (throw char*).

_ros2_topic_info() {
  # Soft timeout: never block the startup state machine on a stuck CLI.
  timeout 4 ros2 topic info "$1" 2>/dev/null || true
}

camera_topic_has_publisher() {
  local topic="$1"
  _ros2_topic_info "$topic" | grep -Eq 'Publisher count: [1-9][0-9]*'
}

camera_topic_has_frame() {
  local topic="$1"
  # Camera topics use sensor-data / BEST_EFFORT QoS. The default ros2 echo
  # reliability can miss frames even while the publisher is healthy.
  timeout 3 ros2 topic echo "$topic" --once --qos-profile sensor_data >/dev/null 2>&1
}

opencv_camera_process_alive() {
  local pid="${1:-${CAMERA_PID:-}}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    return 0
  fi
  pgrep -f '[p]ython3? -u .*/opencv_compressed_cam\.py' >/dev/null 2>&1
}

camera_log_shows_live_frames() {
  # Under Nav2+Qwen load, ros2 topic type/echo often return empty even while
  # opencv_compressed_cam is publishing. Trust a freshly updated frame log.
  local log_file="${1:-}"
  local max_age_sec="${2:-8}"
  [[ -n "$log_file" && -f "$log_file" ]] || return 1
  grep -Eq 'published [0-9]+ frames' "$log_file" 2>/dev/null || return 1
  # Reject stale logs left by a previous camera instance.
  local now mtime age
  now="$(date +%s)"
  mtime="$(stat -c %Y "$log_file" 2>/dev/null || echo 0)"
  age=$((now - mtime))
  [[ "$age" -le "$max_age_sec" ]]
}

camera_ready() {
  local topic="$1"
  # Publisher count alone is unreliable on RDK: hobot_usb_cam can crash with
  # "Unable to queue image buffer" while DDS still advertises a publisher.
  # Require at least one real frame before treating the camera as ready.
  camera_topic_has_frame "$topic"
}

camera_launch_crashed() {
  local log_file="$1"
  [[ -f "$log_file" ]] || return 1
  grep -Eq 'Unable to queue image buffer|process has died|terminate called|context is invalid' "$log_file"
}

wait_topic_publisher() {
  local topic="$1" timeout_sec="${2:-40}" log_file="${3:-}" camera_pid="${4:-}"
  local i
  for i in $(seq 1 "$timeout_sec"); do
    if [[ -n "$camera_pid" ]] && ! kill -0 "$camera_pid" 2>/dev/null; then
      echo "[wait] $topic aborted: camera launch pid=$camera_pid exited"
      return 1
    fi
    if [[ -n "$log_file" ]] && camera_launch_crashed "$log_file"; then
      echo "[wait] $topic aborted: camera crash in $(basename "$log_file")"
      return 1
    fi
    if [[ -n "$log_file" ]] && grep -Eq 'published [0-9]+ frames|published /image_raw:' "$log_file" 2>/dev/null; then
      echo "[wait] $topic ready from frame log (${i}s)"
      return 0
    fi
    if camera_ready "$topic"; then
      echo "[wait] $topic ready (${i}s)"
      return 0
    fi
    if [ $((i % 5)) -eq 0 ]; then
      local pub_line
      pub_line="$(_ros2_topic_info "$topic" | grep -E 'Publisher count:' || echo 'topic absent / no pub line')"
      echo "[wait] $topic... ${i}/${timeout_sec}s (${pub_line})"
    fi
    sleep 1
  done
  return 1
}

camera_topic_type() {
  timeout 4 ros2 topic type "$1" 2>/dev/null | head -n 1 || true
}

video_device_busy() {
  local dev="${1:-${CAMERA_DEV:-/dev/video0}}"
  fuser "$dev" >/dev/null 2>&1
}

stop_camera_tree() {
  # Stop launch parent + children, then any leftover hobot/opencv camera publishers.
  local pid="${1:-${CAMERA_PID:-}}"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    local child
    for child in $(pgrep -P "$pid" 2>/dev/null || true); do
      kill "$child" 2>/dev/null || true
    done
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
  local orphan
  for orphan in $(pgrep -x hobot_usb_cam 2>/dev/null || true); do
    kill "$orphan" 2>/dev/null || true
  done
  for orphan in $(pgrep -f '[p]ython3? -u .*/opencv_compressed_cam.py' 2>/dev/null || true); do
    kill "$orphan" 2>/dev/null || true
  done
  CAMERA_PID=""
  export CAMERA_PID
  local dev="${CAMERA_DEV:-/dev/video0}"
  local i
  for i in $(seq 1 10); do
    if ! video_device_busy "$dev"; then
      return 0
    fi
    sleep 0.5
  done
  echo "[camera] WARN: $dev still busy after stop_camera_tree: $(fuser -v "$dev" 2>&1 | tr '\n' ' ')" >&2
  return 1
}

start_opencv_usb_camera() {
  local package_root="$1" log_file="$2" width="$3" height="$4" fps="$5"
  local dev="${CAMERA_DEV:-/dev/video0}"
  local topic="${CAMERA_COMPRESSED_TOPIC:-/image}"
  local script="$package_root/src/perception/opencv_compressed_cam.py"
  if [ ! -f "$script" ]; then
    echo "[camera] ERROR: missing $script" >&2
    return 1
  fi
  if video_device_busy "$dev"; then
    echo "[camera] $dev busy before opencv launch; clearing stale holders"
    stop_camera_tree "" || true
    sleep 0.5
  fi
  if video_device_busy "$dev"; then
    echo "[camera] ERROR: $dev is busy before opencv launch: $(fuser -v "$dev" 2>&1 | tr '\n' ' ')" >&2
    return 1
  fi
  mkdir -p "$(dirname "$log_file")"
  : >"$log_file"
  # Proven profile used by start_live_servo_voice.sh (OpenCV MJPG USB).
  echo "[camera] start OpenCV USB camera: $dev ${width}x${height}@${fps} -> $topic"
  python3 -u "$script" \
    --device "$dev" \
    --topic "$topic" \
    --width "$width" \
    --height "$height" \
    --fps "$fps" \
    >"$log_file" 2>&1 &
  CAMERA_PID=$!
  export CAMERA_PID
}

start_project_usb_camera_profile() {
  local package_root="$1" log_file="$2" width="$3" height="$4" fps="$5"
  local backend="${CAMERA_BACKEND:-opencv}"
  if [ "$backend" = "opencv" ]; then
    start_opencv_usb_camera "$package_root" "$log_file" "$width" "$height" "$fps"
    return $?
  fi
  local project_dir="${ROBOT_PROJECT_DIR:-$(cd "$package_root/.." && pwd)}"
  local launch_file="$project_dir/perception/launch/usb_cam.launch.py"
  local dev="${CAMERA_DEV:-/dev/video0}"
  if [ ! -f "$launch_file" ]; then
    echo "[camera] ERROR: missing $launch_file" >&2
    echo "[camera] Set ROBOT_PROJECT_DIR=/root/rdk_x5_vln_robot" >&2
    return 1
  fi
  if video_device_busy "$dev"; then
    echo "[camera] ERROR: $dev is busy before launch: $(fuser -v "$dev" 2>&1 | tr '\n' ' ')" >&2
    return 1
  fi
  mkdir -p "$(dirname "$log_file")"
  : >"$log_file"
  echo "[camera] start hobot USB camera: $dev ${width}x${height}@${fps}"
  ros2 launch "$launch_file" \
    usb_video_device:="$dev" \
    usb_image_width:="$width" \
    usb_image_height:="$height" \
    usb_framerate:="$fps" \
    >"$log_file" 2>&1 &
  CAMERA_PID=$!
  export CAMERA_PID
}

ensure_compressed_camera() {
  local package_root="$1" log_file="$2"
  local compressed_topic="${CAMERA_COMPRESSED_TOPIC:-/image}"
  local dev="${CAMERA_DEV:-/dev/video0}"

  if camera_ready "$compressed_topic"; then
    echo "[camera] reuse existing $compressed_topic publisher"
  else
    if [ "${START_CAMERA:-auto}" = "0" ]; then
      echo "[camera] ERROR: $compressed_topic has no publisher and START_CAMERA=0" >&2
      return 1
    fi

    # Stale orphan from a previous failed run will block /dev/video0.
    if video_device_busy "$dev"; then
      echo "[camera] $dev busy without usable $compressed_topic; clearing stale camera"
      stop_camera_tree "" || true
    fi

    # Default 1280x720@15 (camera native 16:9). Prefer OpenCV backend:
    # hobot_usb_cam often advertises /image under a busy Nav2 graph but never
    # delivers frames.
    local width="${CAMERA_WIDTH:-1280}" height="${CAMERA_HEIGHT:-720}" fps="${CAMERA_FPS:-15}"
    start_project_usb_camera_profile "$package_root" "$log_file" "$width" "$height" "$fps" || return 1
    if ! wait_topic_publisher "$compressed_topic" 25 "$log_file" "$CAMERA_PID"; then
      echo "[camera] WARN: primary camera backend not ready; falling back to OpenCV 1280x720@15"
      stop_camera_tree "$CAMERA_PID" || true
      sleep 1
      if video_device_busy "$dev"; then
        echo "[camera] ERROR: cannot fallback because $dev is still busy" >&2
        fuser -v "$dev" >&2 || true
        return 1
      fi
      if [ -f "$log_file" ]; then
        cp -f "$log_file" "${log_file}.prev" 2>/dev/null || true
      fi
      CAMERA_BACKEND=opencv start_opencv_usb_camera "$package_root" "$log_file" 1280 720 15 || return 1
      wait_topic_publisher "$compressed_topic" 25 "$log_file" "$CAMERA_PID" || {
        echo "[camera] ERROR: $compressed_topic did not start; tail $log_file" >&2
        tail -n 60 "$log_file" 2>/dev/null || true
        return 1
      }
    fi
  fi
  # Type probe can briefly return empty after DDS graph churn / shm wipe while
  # the camera is still publishing frames. Retry, then accept live evidence:
  #   1) ros2 echo frame  2) fresh camera.log frames  3) publisher count + alive proc
  local actual_type=""
  local attempt
  for attempt in 1 2 3 4 5 6 7 8 9 10; do
    actual_type="$(camera_topic_type "$compressed_topic")"
    if [ "$actual_type" = "sensor_msgs/msg/CompressedImage" ]; then
      break
    fi
    if camera_topic_has_frame "$compressed_topic"; then
      echo "[camera] WARN: $compressed_topic type probe='$actual_type' but live frames OK; continuing"
      actual_type="sensor_msgs/msg/CompressedImage"
      break
    fi
    if opencv_camera_process_alive "${CAMERA_PID:-}" && camera_log_shows_live_frames "$log_file" 10; then
      echo "[camera] WARN: $compressed_topic type probe='$actual_type' but opencv log still publishing; continuing"
      actual_type="sensor_msgs/msg/CompressedImage"
      break
    fi
    if opencv_camera_process_alive "${CAMERA_PID:-}" && camera_topic_has_publisher "$compressed_topic"; then
      echo "[camera] WARN: $compressed_topic type probe='$actual_type' but publisher+process alive; continuing"
      actual_type="sensor_msgs/msg/CompressedImage"
      break
    fi
    if [ -n "${CAMERA_PID:-}" ] && ! kill -0 "$CAMERA_PID" 2>/dev/null; then
      echo "[camera] ERROR: camera pid=$CAMERA_PID died during type check (type='$actual_type')" >&2
      return 1
    fi
    if ! opencv_camera_process_alive "${CAMERA_PID:-}" && ! camera_topic_has_publisher "$compressed_topic"; then
      # No process and no publisher — keep retrying briefly in case of spawn lag.
      :
    fi
    sleep 0.6
  done
  if [ "$actual_type" != "sensor_msgs/msg/CompressedImage" ]; then
    echo "[camera] ERROR: $compressed_topic type is '$actual_type', expected sensor_msgs/msg/CompressedImage" >&2
    if opencv_camera_process_alive "${CAMERA_PID:-}"; then
      echo "[camera] HINT: opencv process still alive; check DDS/load. last log:" >&2
      tail -n 15 "$log_file" 2>/dev/null || true
    fi
    return 1
  fi
}

start_raw_bridge() {
  local package_root="$1" log_file="$2"
  local compressed_topic="${CAMERA_COMPRESSED_TOPIC:-/image}"
  local raw_topic="${IMAGE_RAW_TOPIC:-/image_raw}"
  local max_fps="${IMAGE_RAW_MAX_FPS:-8}"
  if camera_topic_has_publisher "$raw_topic"; then
    local actual_type
    actual_type="$(camera_topic_type "$raw_topic")"
    if [ "$actual_type" != "sensor_msgs/msg/Image" ]; then
      echo "[bridge] ERROR: existing $raw_topic type is '$actual_type', expected sensor_msgs/msg/Image" >&2
      return 1
    fi
    echo "[bridge] reuse existing $raw_topic publisher"
    return 0
  fi
  echo "[bridge] $compressed_topic -> $raw_topic, max_fps=$max_fps"
  mkdir -p "$(dirname "$log_file")"
  python3 -u "$package_root/src/perception/compressed_to_raw_image.py" \
    --in-topic "$compressed_topic" \
    --out-topic "$raw_topic" \
    --max-fps "$max_fps" \
    >"$log_file" 2>&1 &
  BRIDGE_PID=$!
  export BRIDGE_PID
  wait_topic_publisher "$raw_topic" 25 "$log_file" "$BRIDGE_PID" || {
    echo "[bridge] ERROR: $raw_topic publisher not created" >&2
    tail -n 40 "$log_file" 2>/dev/null || true
    return 1
  }
  local _
  for _ in $(seq 1 20); do
    if grep -q "published .*approx_fps=" "$log_file" 2>/dev/null; then
      echo "[bridge] raw frames confirmed"
      return 0
    fi
    if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
      echo "[bridge] ERROR: bridge process exited" >&2
      tail -n 40 "$log_file" 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
  echo "[bridge] WARN: publisher exists but no decoded-frame log yet"
  return 0
}
