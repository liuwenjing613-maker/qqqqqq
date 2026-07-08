#!/usr/bin/env bash
# Export chassis bridge env vars from qwen_api_lidar_nav.yaml chassis block.
# Then caller should source original load_mvp_tune.sh (PWM base); yaml values win if set before bridge start.

load_chassis_from_config() {
  local config_path="${1:-$PROJECT_DIR/configs/qwen_api_lidar_nav.yaml}"
  if [ ! -f "$config_path" ]; then
    echo "[chassis] WARN: config not found: $config_path"
    return 1
  fi
  eval "$(python3 - "$config_path" <<'PY'
import sys, yaml, shlex

path = sys.argv[1]
with open(path, encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}
ch = cfg.get("chassis") or {}
mapping = {
    "port": "CHASSIS_PORT",
    "max_vx": "CHASSIS_MAX_VX",
    "max_wz": "CHASSIS_MAX_WZ",
    "watchdog_timeout": "CHASSIS_WATCHDOG_TIMEOUT",
    "control_rate_hz": "CHASSIS_CONTROL_RATE_HZ",
    "wheel_layout": "CHASSIS_PWM_WHEEL_LAYOUT",
    "motor_signs": "CHASSIS_MOTOR_SIGNS",
    "vx_pwm_deadband": "CHASSIS_VX_PWM_DEADBAND",
    "wz_pwm_deadband": "CHASSIS_WZ_PWM_DEADBAND",
    "pwm_max": "CHASSIS_PWM_MAX",
    "vx_pwm_gain": "CHASSIS_VX_PWM_GAIN",
    "wz_pwm_gain": "CHASSIS_WZ_PWM_GAIN",
    "pwm_smooth_alpha": "CHASSIS_PWM_SMOOTH_ALPHA",
    "max_pwm_delta": "CHASSIS_MAX_PWM_DELTA",
    "debug": "CHASSIS_DEBUG",
}
for key, env_name in mapping.items():
    if key not in ch:
        continue
    val = ch[key]
    if isinstance(val, bool):
        val = "1" if val else "0"
    print(f"export {env_name}={shlex.quote(str(val))}")
kick = ch.get("kick_start") or {}
kick_map = {
    "enable": "CHASSIS_KICK_ENABLE",
    "kick_vx": "CHASSIS_KICK_VX",
    "kick_wz": "CHASSIS_KICK_WZ",
    "kick_duration": "CHASSIS_KICK_DURATION",
    "kick_cooldown": "CHASSIS_KICK_COOLDOWN",
}
for key, env_name in kick_map.items():
    if key not in kick:
        continue
    val = kick[key]
    if isinstance(val, bool):
        val = "1" if val else "0"
    print(f"export {env_name}={shlex.quote(str(val))}")
odom = ch.get("odom") or {}
odom_map = {
    "vx_scale": "CHASSIS_ODOM_VX_SCALE",
    "wz_scale": "CHASSIS_ODOM_WZ_SCALE",
    "vy_scale": "CHASSIS_ODOM_VY_SCALE",
    "vxy_deadzone": "CHASSIS_ODOM_VXY_DEADZONE",
    "wz_deadzone": "CHASSIS_ODOM_WZ_DEADZONE",
    "xy_yaw_offset": "CHASSIS_ODOM_XY_YAW_OFFSET",
    "base_yaw_offset": "CHASSIS_BASE_YAW_OFFSET",
    "use_vy": "CHASSIS_ODOM_USE_VY",
}
for key, env_name in odom_map.items():
    if key not in odom:
        continue
    val = odom[key]
    if isinstance(val, bool):
        val = "1" if val else "0"
    print(f"export {env_name}={shlex.quote(str(val))}")
PY
)"
}
