#!/usr/bin/env bash
# Standalone: USB camera -> /image -> /image_viz, viewable in Foxglove.
#
# Does NOT start SLAM / lidar / nav. Only camera preview for Foxglove.
#
# Usage:
#   bash scripts/camera/start_foxglove_camera.sh
#   bash scripts/camera/start_foxglove_camera.sh --no-bridge   # camera+/image_viz only
#   bash scripts/camera/start_foxglove_camera.sh --fg          # keep running in foreground
#
# Foxglove Studio:
#   Open connection -> ws://<RDK_IP>:8765
#   Image panel -> topic /image_viz  (NOT /image or /image_raw)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
export PROJECT_DIR

START_BRIDGE=1
FOREGROUND=0
for arg in "$@"; do
  case "$arg" in
    --no-bridge) START_BRIDGE=0 ;;
    --fg|--foreground) FOREGROUND=1 ;;
    -h|--help)
      sed -n '2,16p' "$0"
      exit 0
      ;;
    *)
      echo "[foxglove_cam] unknown arg: $arg" >&2
      exit 2
      ;;
  esac
done

set +u
if [[ -f /opt/tros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
set -u

# shellcheck disable=SC1091
source "${PROJECT_DIR}/scripts/lib/camera_stack.sh"

FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
LOG_DIR="${PROJECT_DIR}/logs/foxglove_camera"
CAM_LOG="${LOG_DIR}/usb_cam.log"
THROTTLE_LOG="${LOG_DIR}/image_throttle.log"
BRIDGE_LOG="${LOG_DIR}/foxglove_bridge.log"
THROTTLE_SCRIPT="${PROJECT_DIR}/scripts/lidar/foxglove_image_throttle.py"
VIZ_MAX_FPS="${FOXGLOVE_VIZ_MAX_FPS:-5}"
VIZ_MAX_WIDTH="${FOXGLOVE_VIZ_MAX_WIDTH:-640}"
VIZ_JPEG_Q="${FOXGLOVE_VIZ_JPEG_QUALITY:-50}"

mkdir -p "${LOG_DIR}"

board_ip() {
  hostname -I 2>/dev/null | awk '{print $1}'
}

port_listening() {
  local port="$1"
  command -v ss >/dev/null 2>&1 && ss -tln 2>/dev/null | grep -q ":${port} "
}

topic_has_publisher() {
  local topic="$1"
  ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"
}

wait_topic_publisher() {
  local topic="$1"
  local timeout_sec="${2:-25}"
  local i
  echo "[foxglove_cam] waiting for publisher on ${topic} ..."
  for i in $(seq 1 "${timeout_sec}"); do
    if topic_has_publisher "${topic}"; then
      echo "[foxglove_cam] OK: ${topic} has publisher (${i}s)"
      return 0
    fi
    sleep 1
  done
  echo "[foxglove_cam] FAIL: no publisher on ${topic} within ${timeout_sec}s" >&2
  return 1
}

start_throttle() {
  if pgrep -f "foxglove_image_throttle.py" >/dev/null 2>&1; then
    echo "[foxglove_cam] image throttle already running"
    return 0
  fi
  if [[ ! -f "${THROTTLE_SCRIPT}" ]]; then
    echo "[foxglove_cam] ERROR: missing ${THROTTLE_SCRIPT}" >&2
    return 1
  fi
  echo "[foxglove_cam] starting /image -> /image_viz (${VIZ_MAX_FPS}fps, w<=${VIZ_MAX_WIDTH}, q=${VIZ_JPEG_Q})"
  : > "${THROTTLE_LOG}"
  python3 -u "${THROTTLE_SCRIPT}" \
    --in-topic /image \
    --out-topic /image_viz \
    --max-fps "${VIZ_MAX_FPS}" \
    --max-width "${VIZ_MAX_WIDTH}" \
    --jpeg-quality "${VIZ_JPEG_Q}" \
    >"${THROTTLE_LOG}" 2>&1 &
  echo $! > "${THROTTLE_LOG}.pid"
  sleep 1
  if ! kill -0 "$(cat "${THROTTLE_LOG}.pid")" 2>/dev/null; then
    echo "[foxglove_cam] ERROR: throttle exited; see ${THROTTLE_LOG}" >&2
    tail -n 20 "${THROTTLE_LOG}" 2>/dev/null || true
    return 1
  fi
}

ensure_bridge() {
  if port_listening "${FOXGLOVE_PORT}"; then
    echo "[foxglove_cam] foxglove_bridge already on :${FOXGLOVE_PORT}"
    return 0
  fi
  if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    echo "[foxglove_cam] ERROR: foxglove_bridge not installed" >&2
    return 1
  fi
  echo "[foxglove_cam] starting foxglove_bridge on :${FOXGLOVE_PORT}"
  : > "${BRIDGE_LOG}"
  # Reuse project bridge (whitelist includes /image_viz). Skip its own throttle
  # by pre-starting ours above; start_foxglove.sh will pkill+restart throttle — OK.
  FOXGLOVE_PORT="${FOXGLOVE_PORT}" \
  FOXGLOVE_THROTTLE_LOG="${THROTTLE_LOG}" \
  FOXGLOVE_VIZ_MAX_FPS="${VIZ_MAX_FPS}" \
  FOXGLOVE_VIZ_MAX_WIDTH="${VIZ_MAX_WIDTH}" \
  FOXGLOVE_VIZ_JPEG_QUALITY="${VIZ_JPEG_Q}" \
  PROJECT_DIR="${PROJECT_DIR}" \
    bash "${PROJECT_DIR}/scripts/lidar/start_foxglove.sh" \
    >"${BRIDGE_LOG}" 2>&1 &
  echo $! > "${BRIDGE_LOG}.pid"

  local waited=0
  local max_wait=30
  while [[ "${waited}" -lt "${max_wait}" ]]; do
    if port_listening "${FOXGLOVE_PORT}"; then
      echo "[foxglove_cam] OK: foxglove_bridge listening (${waited}s)"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "[foxglove_cam] ERROR: foxglove_bridge failed; see ${BRIDGE_LOG}" >&2
  tail -n 30 "${BRIDGE_LOG}" 2>/dev/null || true
  return 1
}

echo "[foxglove_cam] ===== Foxglove camera preview ====="
echo "[foxglove_cam] device=${CAMERA_DEV} logs=${LOG_DIR}"

if camera_process_alive && topic_has_publisher /image; then
  echo "[foxglove_cam] camera already publishing /image"
else
  ensure_camera_image_stream "${CAM_LOG}" || {
    echo "[foxglove_cam] camera start failed; see ${CAM_LOG}" >&2
    exit 1
  }
fi

wait_topic_publisher /image 25 || {
  camera_stack_diagnose "${CAM_LOG}"
  exit 1
}

start_throttle || exit 1
wait_topic_publisher /image_viz 15 || {
  echo "[foxglove_cam] throttle log:" >&2
  tail -n 30 "${THROTTLE_LOG}" 2>/dev/null || true
  exit 1
}

if [[ "${START_BRIDGE}" -eq 1 ]]; then
  ensure_bridge || exit 1
fi

IP="$(board_ip)"
echo ""
echo "[foxglove_cam] ========== READY =========="
echo "[foxglove_cam] ROS topics:  /image  (full)   /image_viz  (Foxglove preview)"
if [[ "${START_BRIDGE}" -eq 1 ]]; then
  echo "[foxglove_cam] Foxglove:    ws://${IP:-<board_ip>}:${FOXGLOVE_PORT}"
  echo "[foxglove_cam] Image panel: subscribe to /image_viz"
else
  echo "[foxglove_cam] bridge skipped (--no-bridge); start foxglove separately if needed"
fi
echo "[foxglove_cam] stop:        bash scripts/camera/stop_foxglove_camera.sh"
echo "[foxglove_cam] logs:        ${LOG_DIR}/"
echo ""

if [[ "${FOREGROUND}" -eq 1 ]]; then
  echo "[foxglove_cam] foreground mode — Ctrl+C to stop camera preview stack"
  cleanup() {
    bash "${SCRIPT_DIR}/stop_foxglove_camera.sh" || true
  }
  trap cleanup INT TERM
  while true; do
    if ! camera_process_alive; then
      echo "[foxglove_cam] camera process died" >&2
      exit 1
    fi
    sleep 2
  done
fi
