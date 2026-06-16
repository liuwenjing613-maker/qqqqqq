#!/usr/bin/env bash
# 从 mvp_tune 环境变量启动底盘桥。
# 用法: source scripts/lib/run_chassis_bridge.sh && run_chassis_bridge <log_file>

run_chassis_bridge() {
  local log_file="$1"
  if [ -z "$log_file" ]; then
    echo "[ERROR] run_chassis_bridge: missing log file path"
    return 1
  fi

  local kick_vx_flag="--no-enable-kick-start"
  if [ "${ENABLE_KICK_START:-0}" = "1" ]; then
    kick_vx_flag="--enable-kick-start"
  fi

  cd "${PROJECT_DIR}/ros2_bridge"
  source /opt/tros/humble/setup.bash
  python3 cmd_vel_to_rosmaster.py \
    --mvp-tune-config "${MVP_TUNE_FILE}" \
    --port "${CHASSIS_PORT}" \
    --max-vx "${CHASSIS_MAX_VX}" \
    --max-wz "${CHASSIS_MAX_WZ}" \
    --kick-vx "${KICK_VX}" \
    --kick-wz "${KICK_WZ}" \
    --kick-duration "${KICK_DURATION}" \
    --kick-cooldown "${KICK_COOLDOWN}" \
    --cmd-wz-deadzone "${CMD_WZ_DEADZONE}" \
    --cmd-smooth-alpha "${CMD_SMOOTH_ALPHA}" \
    --max-vx-delta "${MAX_VX_DELTA}" \
    --max-wz-delta "${MAX_WZ_DELTA}" \
    "${kick_vx_flag}" \
    --control-rate-hz "${CONTROL_RATE_HZ}" \
    --debug \
    > "${log_file}" 2>&1 &
}
