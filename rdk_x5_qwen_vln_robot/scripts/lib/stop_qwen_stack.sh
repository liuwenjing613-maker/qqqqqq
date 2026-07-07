#!/usr/bin/env bash
# Stop all processes started by start_qwen_api_lidar_nav.sh and zero the chassis.

# Resolve PROJECT_DIR and RDK_ORIGINAL_ROOT relative to this script's location.
# This ensures correct paths even if the qwen project folder was moved.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/project_dir.sh"

stop_qwen_stack() {
  if [ "${_QWEN_STACK_CLEANUP_DONE:-0}" = "1" ]; then
    return 0
  fi
  _QWEN_STACK_CLEANUP_DONE=1

  echo "[cleanup] stopping Qwen API + LiDAR stack..."

  # Best-effort zero cmd before killing bridge (needs ROS env).
  set +u
  if [ -f /opt/tros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/tros/humble/setup.bash 2>/dev/null || true
  elif [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash 2>/dev/null || true
  fi
  set -u
  timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
    >/dev/null 2>&1 || true

  pkill -TERM -f "run_qwen_api_lidar_nav.py" 2>/dev/null || true
  sleep 0.3
  pkill -KILL -f "run_qwen_api_lidar_nav.py" 2>/dev/null || true

  pkill -TERM -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -TERM -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
  sleep 0.3
  pkill -KILL -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -KILL -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true

  pkill -TERM -f "tmini_plus.launch.py" 2>/dev/null || true
  pkill -TERM -f "ydlidar_ros2_driver" 2>/dev/null || true
  sleep 0.2
  pkill -KILL -f "tmini_plus.launch.py" 2>/dev/null || true
  pkill -KILL -f "ydlidar_ros2_driver" 2>/dev/null || true

  pkill -TERM -f "hobot_usb_cam" 2>/dev/null || true
  pkill -TERM -f "compressed_to_raw_image.py" 2>/dev/null || true
  sleep 0.2
  pkill -KILL -f "hobot_usb_cam" 2>/dev/null || true
  pkill -KILL -f "compressed_to_raw_image.py" 2>/dev/null || true

  local motor_port="${CHASSIS_PORT:-/dev/rosmaster}"
  if [ -e "$motor_port" ]; then
    python3 - "$motor_port" <<'PY' 2>/dev/null || true
import sys
from Rosmaster_Lib import Rosmaster
bot = Rosmaster(com=sys.argv[1])
bot.set_motor(0, 0, 0, 0)
print(f"[cleanup] motor zero via {sys.argv[1]}")
PY
  fi

  echo "[cleanup] done."
}
