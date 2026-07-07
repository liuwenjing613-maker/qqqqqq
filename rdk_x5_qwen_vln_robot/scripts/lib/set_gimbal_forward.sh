#!/usr/bin/env bash
# Set camera gimbal to forward-facing pose before chassis bridge grabs the serial port.
# Uses Rosmaster PWM servos on the same port as the chassis (/dev/ttyUSB0).

set_gimbal_forward() {
  local config_file="${1:-}"
  local port="${GIMBAL_PORT:-${CHASSIS_PORT:-/dev/rosmaster}}"
  local yaw_id="${GIMBAL_YAW_ID:-2}"
  local pitch_id="${GIMBAL_PITCH_ID:-3}"
  local yaw="${GIMBAL_YAW_ANGLE:-65}"
  local pitch="${GIMBAL_PITCH_ANGLE:-130}"
  local enabled="${GIMBAL_ENABLE:-1}"

  if [ -n "$config_file" ] && [ -f "$config_file" ]; then
    eval "$(python3 - "$config_file" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1])) or {}
g = cfg.get("gimbal") or {}
if g:
    for k, env in [
        ("enabled", "GIMBAL_ENABLE"),
        ("port", "GIMBAL_PORT"),
        ("yaw_id", "GIMBAL_YAW_ID"),
        ("pitch_id", "GIMBAL_PITCH_ID"),
        ("yaw_angle", "GIMBAL_YAW_ANGLE"),
        ("pitch_angle", "GIMBAL_PITCH_ANGLE"),
    ]:
        if k in g and g[k] is not None:
            v = g[k]
            if isinstance(v, bool):
                v = 1 if v else 0
            print(f'export {env}="{v}"')
PY
)"
    port="${GIMBAL_PORT:-$port}"
    yaw_id="${GIMBAL_YAW_ID:-$yaw_id}"
    pitch_id="${GIMBAL_PITCH_ID:-$pitch_id}"
    yaw="${GIMBAL_YAW_ANGLE:-$yaw}"
    pitch="${GIMBAL_PITCH_ANGLE:-$pitch}"
    enabled="${GIMBAL_ENABLE:-$enabled}"
  fi

  if [ "$enabled" != "1" ]; then
    echo "[gimbal] disabled (GIMBAL_ENABLE=$enabled)"
    return 0
  fi

  # Chassis bridge holds the serial port; stop it before moving servos.
  pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  sleep 0.5

  echo "[gimbal] set forward pose port=$port yaw(S${yaw_id})=${yaw} pitch(S${pitch_id})=${pitch}"
  python3 "${RDK_ORIGINAL_ROOT}/gimbal/gimbal_tune.py" \
    --port "$port" \
    --yaw-id "$yaw_id" \
    --pitch-id "$pitch_id" \
    --yaw "$yaw" \
    --pitch "$pitch"
}
