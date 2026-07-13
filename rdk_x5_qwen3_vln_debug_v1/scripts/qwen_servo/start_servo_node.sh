#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ -f "$ROOT/scripts/lib/ros_env.sh" ]]; then
  # Installed into rdk_x5_qwen3_vln_debug_v1.
  source "$ROOT/scripts/lib/ros_env.sh"
elif [[ -f /opt/tros/humble/setup.bash ]]; then
  source /opt/tros/humble/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: ROS2 environment not found" >&2
  exit 1
fi

CONFIG="${SERVO_CONFIG:-$ROOT/configs/qwen3_vln_servo.yaml}"
ARGS=(--config "$CONFIG")
if [[ "${MOTION_ENABLED:-0}" == "1" ]]; then
  ARGS+=(--enable-motion)
fi

exec python3 -u "$ROOT/src/apps/qwen_visual_servo_node.py" "${ARGS[@]}"
