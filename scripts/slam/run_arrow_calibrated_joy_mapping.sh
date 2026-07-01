#!/usr/bin/env bash
# Non-destructive arrow-calibrated joystick mapping.
#
# Flow:
# 1) Start the existing SLAM live stack without modifying it.
# 2) Drive forward slowly using /cmd_vel until /odom displacement reaches TARGET_DISTANCE.
# 3) Compute a visualization-only arrow yaw offset and save configs/arrow_calibration.env.
# 4) Start calibrated display arrows: /arrow_calibration_markers + base_forward_calibrated + laser_forward_calibrated.
# 5) Start joystick + teleop for normal mapping.
# 6) Ctrl+C saves map, stops robot, and stops nodes.
#
# Safety and scope:
# - Does NOT edit old mapping scripts.
# - Does NOT edit lidar config.
# - Does NOT edit m1_pwm_cmd_vel_bridge.py or run_chassis_bridge.sh.
# - Does NOT change /odom or base_link->laser used by SLAM.
set -u

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
cd "${PROJECT_DIR}" || exit 1

LOG_DIR="${PROJECT_DIR}/logs/arrow_calibration"
MAP_DIR="${PROJECT_DIR}/maps"
MAP_NAME="${MAP_NAME:-joy_corridor_map_arrow_calibrated}"
ARROW_CONFIG_FILE="${ARROW_CONFIG_FILE:-${PROJECT_DIR}/configs/arrow_calibration.env}"
TARGET_DISTANCE="${TARGET_DISTANCE:-1.0}"
CALIB_SPEED="${CALIB_SPEED:-0.04}"
CALIB_MAX_TIME="${CALIB_MAX_TIME:-35.0}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

# Pick your existing known-good stack. Default: live stack without joystick.
BASE_STACK="${SLAM_BASE_SCRIPT:-${PROJECT_DIR}/scripts/slam/run_corridor_mapping_live_foxglove.sh}"
if [ ! -f "${BASE_STACK}" ]; then
  echo "[ERROR] Missing base stack: ${BASE_STACK}" >&2
  exit 1
fi

mkdir -p "${LOG_DIR}" "${MAP_DIR}" "$(dirname "${ARROW_CONFIG_FILE}")"
PIDS=()
STACK_PID=""
SAVED=0
CLEANUP_DONE=0
ROS_ENV_READY=0

log() { echo "[$(date +%H:%M:%S)] $*"; }

source_ros() {
  if [ "${ROS_ENV_READY}" = "1" ]; then return 0; fi
  set +u
  if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
  elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
  fi
  if [ -f "${HOME}/ydlidar_ws/install/setup.bash" ]; then
    source "${HOME}/ydlidar_ws/install/setup.bash"
  fi
  set -u
  ROS_ENV_READY=1
}

zero_cmd() {
  source_ros
  timeout 1.5 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
    >/dev/null 2>&1 || true
}

stop_joystick_nodes() {
  pkill -f "teleop_twist_joy" 2>/dev/null || true
  pkill -f "joy_node" 2>/dev/null || true
  pkill -f "game_controller_node" 2>/dev/null || true
}

stop_arrow_nodes() {
  pkill -f "arrow_display_markers.py" 2>/dev/null || true
  pkill -f "arrow_calibration_probe.py" 2>/dev/null || true
}

start_bg() {
  local name="$1"
  shift
  local log_file="${LOG_DIR}/${name}.log"
  log "Starting ${name} -> ${log_file}"
  "$@" > "${log_file}" 2>&1 &
  PIDS+=("$!")
  sleep 0.8
}

wait_topic_exists() {
  local topic="$1"
  local timeout_sec="${2:-60}"
  log "Waiting for ${topic} ..."
  for _ in $(seq 1 "${timeout_sec}"); do
    if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      log "OK: ${topic}"
      return 0
    fi
    sleep 1
  done
  log "FAIL: timeout waiting for ${topic}"
  return 1
}

maybe_convert_png() {
  local pgm="${MAP_DIR}/${MAP_NAME}.pgm"
  local png="${MAP_DIR}/${MAP_NAME}.png"
  if [ ! -f "${pgm}" ]; then return 0; fi
  python3 - "${pgm}" "${png}" <<'PY' || true
import sys
from pathlib import Path
try:
    from PIL import Image
except ImportError:
    print("[info] Pillow not installed; skip PNG preview")
    sys.exit(0)
pgm = Path(sys.argv[1])
png = Path(sys.argv[2])
if pgm.is_file():
    Image.open(pgm).save(png)
    print(f"[info] Wrote {png}")
PY
  [ -f "${png}" ] && ls -lh "${png}" || true
}

save_map() {
  if [ "${SAVED}" = "1" ]; then return 0; fi
  source_ros
  log "Saving map to ${MAP_DIR}/${MAP_NAME} ..."
  if ros2 topic list 2>/dev/null | grep -qx "/map"; then
    ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}" \
      > "${LOG_DIR}/map_saver.log" 2>&1 || true
    if [ -f "${MAP_DIR}/${MAP_NAME}.yaml" ] && [ -f "${MAP_DIR}/${MAP_NAME}.pgm" ]; then
      log "Map saved:"
      ls -lh "${MAP_DIR}/${MAP_NAME}.yaml" "${MAP_DIR}/${MAP_NAME}.pgm"
      maybe_convert_png
      SAVED=1
    else
      log "WARN: map_saver_cli did not create map files. See ${LOG_DIR}/map_saver.log"
      tail -80 "${LOG_DIR}/map_saver.log" 2>/dev/null || true
    fi
  else
    log "WARN: /map does not exist; skip map saving."
  fi
}

cleanup() {
  if [ "${CLEANUP_DONE}" = "1" ]; then return 0; fi
  CLEANUP_DONE=1
  echo
  log "Cleanup: stop robot, save map, stop nodes..."
  zero_cmd
  stop_joystick_nodes
  save_map
  zero_cmd
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then kill -TERM "${pid}" 2>/dev/null || true; fi
  done
  if [ -n "${STACK_PID}" ] && kill -0 "${STACK_PID}" 2>/dev/null; then
    kill -TERM "${STACK_PID}" 2>/dev/null || true
  fi
  stop_arrow_nodes
  pkill -TERM -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
  log "Done."
  exit 0
}
trap cleanup INT TERM

show_status() {
  echo
  echo "========== ARROW CALIBRATION =========="
  [ -f "${ARROW_CONFIG_FILE}" ] && cat "${ARROW_CONFIG_FILE}" || true
  echo
  echo "========== ROS topics =========="
  ros2 topic list | sort | egrep "arrow|joy|cmd_vel|scan|odom|tf|map|chassis" || true
  echo
  echo "========== Foxglove =========="
  local ip
  ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo "Connect in Foxglove: ws://${ip}:${FOXGLOVE_PORT}"
  echo "Enable: /map, /scan_filtered, /tf, /tf_static, /odom, /arrow_calibration_markers"
  echo "Extra TF frames: base_forward_calibrated, laser_forward_calibrated"
  echo
}

main() {
  source_ros
  log "===== Arrow-calibrated joystick SLAM mapping ====="
  log "Project: ${PROJECT_DIR}"
  log "Base stack: ${BASE_STACK}"
  log "Map output: ${MAP_DIR}/${MAP_NAME}.yaml/.pgm/.png"
  log "Arrow config: ${ARROW_CONFIG_FILE}"
  log "Stopping old joystick/arrow nodes first..."
  zero_cmd
  stop_joystick_nodes
  stop_arrow_nodes
  sleep 1

  log "[1/5] Start existing SLAM live stack"
  setsid bash "${BASE_STACK}" > "${LOG_DIR}/base_stack.log" 2>&1 &
  STACK_PID="$!"

  wait_topic_exists /scan 90 || { log "FAIL: /scan not found"; cleanup; }
  wait_topic_exists /scan_filtered 90 || log "WARN: /scan_filtered not found yet; continuing only if your base script uses /scan directly."
  wait_topic_exists /odom 90 || { log "FAIL: /odom not found"; cleanup; }
  wait_topic_exists /tf 40 || { log "FAIL: /tf not found"; cleanup; }
  wait_topic_exists /map 90 || log "WARN: /map not found yet; slam_toolbox may still be initializing."

  log "[2/5] Auto probe: command forward and compute visualization arrow offset"
  log "Robot will move slowly for up to ${TARGET_DISTANCE}m. Clear the path before continuing."
  python3 "${PROJECT_DIR}/ros2_bridge/arrow_calibration_probe.py" \
    --target-distance "${TARGET_DISTANCE}" \
    --speed "${CALIB_SPEED}" \
    --max-time "${CALIB_MAX_TIME}" \
    --output "${ARROW_CONFIG_FILE}" \
    2>&1 | tee "${LOG_DIR}/arrow_probe.log"
  local probe_status=${PIPESTATUS[0]}
  zero_cmd
  if [ "${probe_status}" != "0" ]; then
    log "FAIL: arrow probe failed with status ${probe_status}. Keeping SLAM running for manual inspection."
    show_status
    while true; do sleep 3600; done
  fi

  log "[3/5] Start visualization-only calibrated arrows"
  start_bg arrow_display python3 "${PROJECT_DIR}/ros2_bridge/arrow_display_markers.py" \
    --config "${ARROW_CONFIG_FILE}"

  log "[4/5] Start joystick /joy"
  start_bg joy_node ros2 run joy joy_node --ros-args \
    -p dev:=/dev/input/js0 \
    -p deadzone:=0.15 \
    -p autorepeat_rate:=20.0
  wait_topic_exists /joy 30 || log "WARN: /joy not found; joystick may be disconnected."

  log "[5/5] Start teleop /joy -> /cmd_vel"
  start_bg teleop ros2 run teleop_twist_joy teleop_node --ros-args \
    -p require_enable_button:=false \
    -p axis_linear.x:=1 \
    -p scale_linear.x:=0.06 \
    -p axis_angular.yaw:=0 \
    -p scale_angular.yaw:=0.24

  sleep 2
  show_status
  log "System is running. Drive slowly with joystick. Press Ctrl+C here to save map and stop."
  while true; do sleep 3600; done
}

main "$@"
