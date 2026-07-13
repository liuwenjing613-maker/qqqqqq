#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$ROOT/.env.local" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/.env.local"
fi
source_ros() {
  local candidates=("/opt/tros/humble/setup.bash" "/opt/ros/humble/setup.bash" "/opt/ros/foxy/setup.bash")
  for setup_file in "${candidates[@]}"; do
    if [[ -f "$setup_file" ]]; then
      source "$setup_file"
      return 0
    fi
  done
  echo "ERROR: ROS2/TROS setup.bash not found." >&2
  return 1
}
source_ros
export PYTHONPATH="$ROOT/src:${PYTHONPATH:-}"
CONFIG="${QWEN_VLN_CONFIG:-$ROOT/configs/qwen3_vln_debug.yaml}"
INSTRUCTION="${1:-${QWEN_INSTRUCTION:-find the bottle}}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
IMAGE_TRANSPORT="${IMAGE_TRANSPORT:-raw}"
if [[ -z "${DASHSCOPE_API_KEY:-}" ]]; then
  echo "ERROR: DASHSCOPE_API_KEY is not set." >&2
  exit 2
fi
echo "===== Qwen3-VL Debug V1 ====="
echo "CONFIG=$CONFIG"
echo "INSTRUCTION=$INSTRUCTION"
echo "IMAGE_TOPIC=$IMAGE_TOPIC"
echo "IMAGE_TRANSPORT=$IMAGE_TRANSPORT"
echo "QWEN_MODEL=${QWEN_MODEL:-qwen3-vl-flash}"
echo "NOTE: this V1 does not publish /cmd_vel"
exec python3 "$ROOT/src/apps/qwen_vln_debug_node.py" --config "$CONFIG" --instruction "$INSTRUCTION" --image-topic "$IMAGE_TOPIC" --image-transport "$IMAGE_TRANSPORT"
