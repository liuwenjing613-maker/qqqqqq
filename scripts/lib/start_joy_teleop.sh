#!/usr/bin/env bash
# Start joy_node + teleop_twist_joy -> /cmd_vel_joy (does not touch /cmd_vel).
# Requires caller to have sourced ROS and slam_calibrated_env.sh for JOY_* defaults.

#!/usr/bin/env bash
# Start joy_node + teleop_twist_joy -> /cmd_vel_joy (does not touch /cmd_vel).
# Requires caller to have sourced ROS and slam_calibrated_env.sh for JOY_* defaults.

resolve_joy_device() {
  local requested="${1:-${JOY_DEV:-/dev/input/js0}}"
  if [ -e "$requested" ]; then
    echo "$requested"
    return 0
  fi
  local js
  for js in /dev/input/js*; do
    if [ -e "$js" ]; then
      echo "[joy] WARN: ${requested} not found, fallback to ${js}" >&2
      echo "$js"
      return 0
    fi
  done
  echo "[joy] ERROR: no joystick device under /dev/input/js*" >&2
  return 1
}

start_joy_teleop() {
  local log_file="${1:-logs/joy_teleop.log}"
  local joy_dev
  joy_dev="$(resolve_joy_device "${JOY_DEV:-/dev/input/js0}")" || return 1
  export JOY_DEV="$joy_dev"
  local joy_deadzone="${JOY_DEADZONE:-0.08}"
  local axis_linear="${JOY_AXIS_LINEAR:-1}"
  local axis_angular="${JOY_AXIS_ANGULAR:-0}"
  local scale_linear="${JOY_SCALE_LINEAR_X:-0.06}"
  local scale_angular="${JOY_SCALE_ANGULAR_YAW:-0.20}"
  local joy_cmd_topic="${CMD_VEL_JOY_TOPIC:-/cmd_vel_joy}"

  mkdir -p "$(dirname "$log_file")"

  echo "[joy] using device ${joy_dev}"

  pkill -f "teleop_twist_joy" 2>/dev/null || true
  pkill -f "joy_node" 2>/dev/null || true
  sleep 0.5

  echo "[joy] starting joy_node dev=${joy_dev}"
  ros2 run joy joy_node --ros-args \
    -p "dev:=${joy_dev}" \
    -p "deadzone:=${joy_deadzone}" \
    -p autorepeat_rate:=20.0 \
    >>"$log_file" 2>&1 &

  for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -qx "/joy"; then
      break
    fi
    if ! pgrep -f "joy_node" >/dev/null 2>&1; then
      echo "[joy] ERROR: joy_node exited early; see ${log_file}"
      tail -n 8 "${log_file}" 2>/dev/null || true
      return 1
    fi
    sleep 0.5
  done

  if ! ros2 topic list 2>/dev/null | grep -qx "/joy"; then
    echo "[joy] ERROR: /joy not available"
    return 1
  fi

  echo "[joy] starting teleop -> ${joy_cmd_topic}"
  ros2 run teleop_twist_joy teleop_node --ros-args \
    -r cmd_vel:="${joy_cmd_topic}" \
    -p require_enable_button:=false \
    -p "axis_linear.x:=${axis_linear}" \
    -p "scale_linear.x:=${scale_linear}" \
    -p "axis_angular.yaw:=${axis_angular}" \
    -p "scale_angular.yaw:=${scale_angular}" \
    >>"$log_file" 2>&1 &

  sleep 1
  echo "[joy] OK: /joy + teleop -> ${joy_cmd_topic}"
  return 0
}

stop_joy_teleop() {
  pkill -f "teleop_twist_joy" 2>/dev/null || true
  pkill -f "joy_node" 2>/dev/null || true
  pkill -f "cmd_vel_priority_mux.py" 2>/dev/null || true
}
