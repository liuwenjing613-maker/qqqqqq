#!/usr/bin/env bash
# One-shot live joystick mapping:
# Start SLAM/Foxglove + joy_node + teleop.
# Drive manually with joystick.
# Press Ctrl+C to save map and stop.

set -u

cd ~/rdk_x5_vln_robot

LOG_DIR="$PWD/logs/joy_mapping"
MAP_DIR="$PWD/maps"
MAP_NAME="${MAP_NAME:-joy_corridor_map}"

mkdir -p "$LOG_DIR" "$MAP_DIR"

PIDS=()
SAVED=0
CLEANUP_DONE=0
ROS_ENV_READY=0

source_ros() {
  if [ "$ROS_ENV_READY" = "1" ]; then
    return 0
  fi

  # ROS setup.bash may reference unset variables such as AMENT_TRACE_SETUP_FILES.
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
    return 0
  fi

  python3 - "$pgm" "$png" <<'PY' || true
import sys
from pathlib import Path

pgm = Path(sys.argv[1])
png = Path(sys.argv[2])
if not pgm.is_file():
    sys.exit(0)
try:
    from PIL import Image
except ImportError:
    print("[info] Pillow not installed; skip PNG preview")
    sys.exit(0)

img = Image.open(pgm)
img.save(png)
print(f"[info] Wrote {png}")
PY

  if [ -f "$png" ]; then
    log "Preview PNG:"
    ls -lh "$png"
  fi
}

save_map() {
  if [ "$SAVED" = "1" ]; then
    return 0
  fi

  log "Saving map to ${MAP_DIR}/${MAP_NAME} ..."

  if ros2 topic list 2>/dev/null | grep -qx "/map"; then
    ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}" \
      > "${LOG_DIR}/map_saver.log" 2>&1

    if [ -f "${MAP_DIR}/${MAP_NAME}.yaml" ] && [ -f "${MAP_DIR}/${MAP_NAME}.pgm" ]; then
      log "Map saved:"
      ls -lh "${MAP_DIR}/${MAP_NAME}.yaml" "${MAP_DIR}/${MAP_NAME}.pgm"
      maybe_convert_png
      SAVED=1
    else
      log "WARN: map_saver_cli ran but map file not found. See ${LOG_DIR}/map_saver.log"
      tail -80 "${LOG_DIR}/map_saver.log" 2>/dev/null || true
    fi
  else
    log "WARN: /map does not exist, skip map saving."
  fi
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

  # 1) Stop joystick publishers so they cannot overwrite /cmd_vel during save.
  stop_joystick_nodes

  # 2) Save map while SLAM stack is still alive (live_stack runs under setsid).
  save_map

  # 3) Stop robot motion, then tear down SLAM stack.
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

show_status() {
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
  echo "========== Foxglove =========="
  local ip
  ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo "Connect in Foxglove: ws://${ip}:8765"

  echo
  echo "========== How to use =========="
  echo "1. Open Foxglove and connect to ws://${ip}:8765"
  echo "2. In 3D panel: 固定参考系=map"
  echo "3. Enable /map, /scan, /scan_filtered, /tf, /tf_static, /odom"
  echo "4. Drive slowly with joystick"
  echo "5. Press Ctrl+C in this terminal to save map and stop"
  echo
}

main() {
  source_ros

  log "===== One-click joystick SLAM mapping ====="
  log "Project: $PWD"
  log "Map output: ${MAP_DIR}/${MAP_NAME}.yaml/.pgm/.png"

  log "Stopping old joystick/control nodes first..."
  zero_cmd
  stop_joystick_nodes
  sleep 1

  log "[1/3] Start SLAM + Foxglove live stack"
  # setsid: keep SLAM alive when this script receives Ctrl+C, so save_map can run first.
  start_bg live_stack setsid bash scripts/slam/run_corridor_mapping_live_foxglove.sh

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

  log "[2/3] Start joystick /joy"
  start_bg joy_node ros2 run joy joy_node --ros-args \
    -p dev:=/dev/input/js0 \
    -p deadzone:=0.15 \
    -p autorepeat_rate:=20.0

  wait_topic_exists /joy 30 || { log "FAIL: /joy not found"; stop_live_stack; exit 1; }

  log "[3/3] Start teleop /joy -> /cmd_vel"
  start_bg teleop ros2 run teleop_twist_joy teleop_node --ros-args \
    -p require_enable_button:=false \
    -p axis_linear.x:=1 \
    -p scale_linear.x:=0.06 \
    -p axis_angular.yaw:=0 \
    -p scale_angular.yaw:=0.24

  sleep 2
  show_status

  log "System is running. Do NOT close this terminal."
  log "Press Ctrl+C when you want to save map and stop."

  while true; do
    sleep 3600
  done
}

main "$@"
