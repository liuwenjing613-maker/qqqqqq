#!/usr/bin/env bash
# One-shot calibrated joystick SLAM mapping:
# Calibrated SLAM/Foxglove + parameter verification + joy_node + teleop.
# Drive manually with joystick. Press Ctrl+C to save map and stop.

set -u

cd ~/rdk_x5_vln_robot

# shellcheck source=scripts/lib/slam_calibrated_env.sh
source "${PWD}/scripts/lib/slam_calibrated_env.sh"
export SLAM_USE_CALIBRATION=1

LOG_DIR="$PWD/logs/joy_mapping_calibrated"
MAP_DIR="$PWD/maps"
MAP_NAME="${MAP_NAME:-joy_calibrated_corridor_map}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"

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

stop_joystick_nodes() {
  pkill -f "simple_scan_filter.py" 2>/dev/null || true
  pkill -f "teleop_twist_joy" 2>/dev/null || true
  pkill -f "joy_node" 2>/dev/null || true
  pkill -f "game_controller_node" 2>/dev/null || true
  pkill -f "cmd_vel_kickstart.py" 2>/dev/null || true
  pkill -f "cmd_vel_pulse_crawl.py" 2>/dev/null || true
}

zero_cmd() {
  source_ros
  timeout 1.2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
    >/dev/null 2>&1 || true
}

maybe_convert_png() {
  local pgm="${MAP_DIR}/${MAP_NAME}.pgm"
  local png="${MAP_DIR}/${MAP_NAME}.png"

  if [ ! -f "$pgm" ]; then
    return 1
  fi

  if python3 - "$pgm" "$png" <<'PY'
import sys
from pathlib import Path

pgm = Path(sys.argv[1])
png = Path(sys.argv[2])
if not pgm.is_file():
    sys.exit(1)
try:
    from PIL import Image
except ImportError:
    print("[WARN] Pillow not installed; skip PNG preview", file=sys.stderr)
    sys.exit(1)

img = Image.open(pgm)
img.save(png)
print(f"[info] Wrote {png}")
PY
  then
    log "Preview PNG:"
    ls -lh "$png"
    return 0
  fi

  log "WARN: PNG preview conversion failed. See above for details."
  return 1
}

save_map() {
  local MAP_OUT="${MAP_DIR}/${MAP_NAME}"
  local MAP_TMP
  local save_start_epoch
  local saver_rc=0
  local pub_count
  local file_epoch

  if [ "$SAVED" = "1" ]; then
    return 0
  fi

  source_ros
  save_start_epoch="$(date +%s)"
  MAP_TMP="${MAP_OUT}.tmp_$(date +%Y%m%d_%H%M%S)"

  log "Saving map to ${MAP_OUT} ..."

  if ! ros2 topic list 2>/dev/null | grep -qx "/map"; then
    log "ERROR: /map does not exist, skip map saving."
    return 1
  fi

  {
    echo "===== /map topic info ====="
    ros2 topic info /map -v 2>&1 || true
    echo
  } > "${LOG_DIR}/map_saver.log"

  pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
  if [ "${pub_count:-0}" = "0" ]; then
    log "ERROR: /map has no publisher; slam_toolbox may have exited."
    return 1
  fi

  set +e
  timeout 30 ros2 run nav2_map_server map_saver_cli \
    -t /map \
    -f "$MAP_TMP" \
    --ros-args \
    -p save_map_timeout:=20.0 \
    >> "${LOG_DIR}/map_saver.log" 2>&1
  saver_rc=$?
  set -u

  if [ "$saver_rc" -ne 0 ]; then
    log "ERROR: map_saver_cli failed with exit code ${saver_rc}. See ${LOG_DIR}/map_saver.log"
    tail -40 "${LOG_DIR}/map_saver.log" 2>/dev/null || true
    rm -f "${MAP_TMP}.pgm" "${MAP_TMP}.yaml" 2>/dev/null || true
    return 1
  fi

  if [ ! -f "${MAP_TMP}.pgm" ] || [ ! -f "${MAP_TMP}.yaml" ]; then
    log "ERROR: map_saver_cli exited 0 but temp map files not found: ${MAP_TMP}.pgm/.yaml"
    tail -40 "${LOG_DIR}/map_saver.log" 2>/dev/null || true
    rm -f "${MAP_TMP}.pgm" "${MAP_TMP}.yaml" 2>/dev/null || true
    return 1
  fi

  file_epoch="$(stat -c %Y "${MAP_TMP}.pgm" 2>/dev/null || echo 0)"
  if [ "$file_epoch" -lt "$save_start_epoch" ]; then
    log "ERROR: temp map file mtime (${file_epoch}) is older than save start (${save_start_epoch})."
    rm -f "${MAP_TMP}.pgm" "${MAP_TMP}.yaml" 2>/dev/null || true
    return 1
  fi

  file_epoch="$(stat -c %Y "${MAP_TMP}.yaml" 2>/dev/null || echo 0)"
  if [ "$file_epoch" -lt "$save_start_epoch" ]; then
    log "ERROR: temp map yaml mtime (${file_epoch}) is older than save start (${save_start_epoch})."
    rm -f "${MAP_TMP}.pgm" "${MAP_TMP}.yaml" 2>/dev/null || true
    return 1
  fi

  mv "${MAP_TMP}.pgm" "${MAP_OUT}.pgm"
  mv "${MAP_TMP}.yaml" "${MAP_OUT}.yaml"

  log "Map saved:"
  ls -lh "${MAP_OUT}.yaml" "${MAP_OUT}.pgm"
  SAVED=1
  maybe_convert_png || true
  return 0
}

stop_live_stack() {
  local pid
  local still_alive=0

  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done

  for _ in $(seq 1 10); do
    still_alive=0
    for pid in "${PIDS[@]:-}"; do
      if kill -0 "$pid" 2>/dev/null; then
        still_alive=1
        break
      fi
    done
    if [ "$still_alive" = "0" ]; then
      return 0
    fi
    sleep 0.5
  done

  log "WARN: live_stack still running, force stopping SLAM stack..."
  pkill -TERM -f "run_slam_calibrated.sh" 2>/dev/null || true
  pkill -TERM -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
  pkill -TERM -f "async_slam_toolbox_node" 2>/dev/null || true
  pkill -TERM -f "sync_slam_toolbox_node" 2>/dev/null || true
  pkill -TERM -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -TERM -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
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
  log "Ctrl+C detected: save map first, then stop all nodes..."

  stop_joystick_nodes

  if save_map; then
    log "OK: map saved."
  else
    log "ERROR: map save failed. Old map files were not overwritten."
  fi

  zero_cmd
  stop_live_stack

  log "Done."
  exit 0
}

trap cleanup INT TERM

start_bg() {
  local name="$1"
  shift

  log "Starting ${name} ..."
  "$@" > "${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
  sleep 1
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

  log "WARN: timeout waiting for ${topic}"
  return 1
}

wait_topic_hz() {
  local topic="$1"
  local timeout_sec="${2:-8}"

  log "Checking ${topic} publish rate ..."
  if timeout "${timeout_sec}s" ros2 topic hz "$topic" 2>/dev/null | grep -q "average rate"; then
    log "OK: ${topic} is publishing"
    return 0
  fi

  log "WARN: ${topic} not publishing within ${timeout_sec}s"
  return 1
}

verify_calibrated_inputs() {
  log "===== Calibrated input verification ====="

  echo "===== /cmd_vel (pre-joystick) ====="
  ros2 topic info /cmd_vel -v 2>/dev/null | head -40 || true

  wait_topic_hz /odom 8 || return 1
  wait_topic_hz /scan_filtered 8 || return 1

  log "Checking /chassis_bridge_state calibration fields ..."
  local state_msg
  state_msg="$(timeout 8s ros2 topic echo /chassis_bridge_state --once --full-length 2>/dev/null || true)"
  if ! printf '%s\n' "$state_msg" | python3 "${PWD}/scripts/slam/verify_calibration_state.py" "${LOG_DIR}/calibration_check.log"; then
    log "FAIL: /chassis_bridge_state calibration check failed"
    log "See ${LOG_DIR}/calibration_check.log (if written)"
    return 1
  fi

  log "Checking TF odom -> base_link ..."
  if ! timeout 4s ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -5 | grep -q "Translation"; then
    log "FAIL: TF odom -> base_link not available"
    return 1
  fi
  log "OK: TF odom -> base_link"

  log "Checking /scan_filtered frame_id ..."
  local frame_id
  frame_id="$(timeout 4s ros2 topic echo /scan_filtered --once 2>/dev/null | awk '/frame_id:/{print $2; exit}')"
  if [ "$frame_id" != "laser" ]; then
    log "FAIL: /scan_filtered frame_id=${frame_id:-<empty>}, expected laser"
    return 1
  fi
  log "OK: /scan_filtered frame_id=laser"

  log "===== Calibration verification passed ====="
  return 0
}

show_status() {
  echo
  echo "========== Calibration =========="
  echo "motor_trims=${CHASSIS_MOTOR_TRIMS}"
  echo "odom_vx_scale=${CHASSIS_ODOM_VX_SCALE}"
  echo "odom_wz_scale=${CHASSIS_ODOM_WZ_SCALE}"
  echo "odom_use_vy=${CHASSIS_ODOM_USE_VY}"
  echo "max_vx=${CHASSIS_MAX_VX} max_wz=${CHASSIS_MAX_WZ}"
  echo "joy_scale_linear=${JOY_SCALE_LINEAR_X} joy_scale_angular=${JOY_SCALE_ANGULAR_YAW}"

  echo
  echo "========== ROS topics =========="
  ros2 topic list | sort | egrep "joy|cmd_vel|scan|odom|tf|map|chassis" || true
  echo ""
  echo "========== /scan_filtered info =========="
  ros2 topic info /scan_filtered -v 2>/dev/null | head -40 || true

  echo
  echo "========== /cmd_vel info =========="
  ros2 topic info /cmd_vel -v 2>/dev/null | head -80 || true

  echo
  echo "========== Foxglove (IMPORTANT) =========="
  local ip
  ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo "Connect: ws://${ip}:8765"
  echo ""
  echo "DO NOT use Image panel (图像面板) for /map or /scan!"
  echo "/map is OccupancyGrid -> need 3D panel."
  echo "/scan_filtered is LaserScan -> need 3D panel."
  echo ""
  echo "Quick fix:"
  echo "  1. Foxglove menu: Layout -> Import layout"
  echo "  2. Select: ${PWD}/configs/foxglove_slam_mapping.layout.json"
  echo "  3. In 3D panel settings: Fixed frame = map"
  echo "  4. Enable layers: /map + /scan_filtered + TF"
  echo ""
  echo "If topics show red ! in sidebar, wrong panel type is selected."
  echo ""
  echo "========== How to use =========="
  echo "1. Import layout (above) then connect ws://${ip}:8765"
  echo "2. Drive slowly with joystick (forward/back + turn only)"
  echo "3. Press Ctrl+C in this terminal to save map and stop"
  echo
}

main() {
  source_ros

  log "===== One-click CALIBRATED joystick SLAM mapping ====="
  log "Project: $PWD"
  log "Map output: ${MAP_DIR}/${MAP_NAME}.yaml/.pgm/.png"
  log "Calibration: motor_trims=${CHASSIS_MOTOR_TRIMS} odom_vx=${CHASSIS_ODOM_VX_SCALE} odom_wz=${CHASSIS_ODOM_WZ_SCALE}"

  if [ ! -e "$JOY_DEV" ]; then
    log "WARN: joystick device ${JOY_DEV} not found; joy_node may fail."
    ls -l /dev/input/js* 2>/dev/null || true
  fi

  log "Stopping old joystick/control nodes first..."
  zero_cmd
  stop_joystick_nodes
  sleep 1

  log "[1/4] Start calibrated SLAM + Foxglove live stack"
  start_bg live_stack setsid bash scripts/slam/run_slam_calibrated.sh

  wait_topic_exists /scan 90 || {
    log "FAIL: /scan not found"
    stop_live_stack 2>/dev/null || true
    exit 1
  }

  wait_topic_exists /scan_filtered 90 || {
    log "FAIL: /scan_filtered not found"
    stop_live_stack 2>/dev/null || true
    exit 1
  }

  wait_topic_exists /odom 90 || {
    log "FAIL: /odom not found"
    stop_live_stack 2>/dev/null || true
    exit 1
  }

  wait_topic_exists /chassis_bridge_state 30 || {
    log "FAIL: /chassis_bridge_state not found"
    stop_live_stack 2>/dev/null || true
    exit 1
  }

  wait_topic_exists /map 90 || {
    log "FAIL: /map not found"
    stop_live_stack 2>/dev/null || true
    exit 1
  }

  wait_topic_exists /tf 40 || {
    log "FAIL: /tf not found"
    stop_live_stack 2>/dev/null || true
    exit 1
  }

  log "[2/4] Verify calibrated parameters and sensor chain"
  verify_calibrated_inputs || {
    log "FAIL: calibration verification failed; NOT starting joystick."
    stop_live_stack 2>/dev/null || true
    exit 1
  }

  log "[3/4] Start joystick /joy (dev=${JOY_DEV})"
  start_bg joy_node ros2 run joy joy_node --ros-args \
    -p dev:="${JOY_DEV}" \
    -p deadzone:=0.15 \
    -p autorepeat_rate:=20.0

  wait_topic_exists /joy 30 || {
    log "FAIL: /joy not found"
    stop_live_stack 2>/dev/null || true
    exit 1
  }

  log "[4/4] Start teleop /joy -> /cmd_vel"
  start_bg teleop ros2 run teleop_twist_joy teleop_node --ros-args \
    -p require_enable_button:=false \
    -p axis_linear.x:=1 \
    -p scale_linear.x:="${JOY_SCALE_LINEAR_X}" \
    -p axis_angular.yaw:=0 \
    -p scale_angular.yaw:="${JOY_SCALE_ANGULAR_YAW}"

  sleep 2
  show_status

  log "System is running. Do NOT close this terminal."
  log "Press Ctrl+C when you want to save map and stop."

  while true; do
    sleep 3600
  done
}

main "$@"
