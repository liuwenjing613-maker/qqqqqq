#!/usr/bin/env bash
# 启动 M1 PWM 底盘桥 (m1_pwm_cmd_vel_bridge.py)
# 用法: source scripts/lib/run_chassis_bridge.sh && run_chassis_bridge <log_file>

kill_chassis_bridge() {
  pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
}

run_chassis_bridge() {
  local log_file="$1"
  if [ -z "$log_file" ]; then
    echo "[ERROR] run_chassis_bridge: missing log file path"
    return 1
  fi

  kill_chassis_bridge

  cd "${PROJECT_DIR}/ros2_bridge"
  set +u
  source /opt/tros/humble/setup.bash
  set -u
  python3 m1_pwm_cmd_vel_bridge.py \
    --port "${CHASSIS_PORT:-${CHASSIS_DEV:-/dev/ttyUSB0}}" \
    --max-vx "${CHASSIS_MAX_VX:-0.06}" \
    --max-wz "${CHASSIS_MAX_WZ:-0.06}" \
    --watchdog-timeout "${CHASSIS_WATCHDOG_TIMEOUT:-0.5}" \
    --control-rate-hz "${CHASSIS_CONTROL_RATE_HZ:-20}" \
    --vx-pwm-deadband "${CHASSIS_VX_PWM_DEADBAND:-6.0}" \
    --wz-pwm-deadband "${CHASSIS_WZ_PWM_DEADBAND:-8.0}" \
    --pwm-max "${CHASSIS_PWM_MAX:-30.0}" \
    --vx-pwm-gain "${CHASSIS_VX_PWM_GAIN:-180.0}" \
    --wz-pwm-gain "${CHASSIS_WZ_PWM_GAIN:-120.0}" \
    --smooth-alpha "${CHASSIS_PWM_SMOOTH_ALPHA:-0.35}" \
    --max-pwm-delta "${CHASSIS_MAX_PWM_DELTA:-3.0}" \
    --wheel-layout "${CHASSIS_PWM_WHEEL_LAYOUT:-fl-rl-fr-rr}" \
    --motor-signs "${CHASSIS_MOTOR_SIGNS:-1,1,1,1}" \
    --publish-odom \
    --odom-topic /odom \
    --odom-frame odom \
    --base-frame base_link \
    --odom-rate-hz 30.0 \
    --odom-vxy-deadzone 0.003 \
    --odom-wz-deadzone 0.015 \
    $( [ "${CHASSIS_DEBUG:-0}" = "1" ] && echo "--debug" ) \
    > "${log_file}" 2>&1 &
}
