#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/lib/ros_env.sh"
# Prefer QWEN_CONFIG; keep QWEN_VLN_CONFIG as a backward-compatible alias.
CONFIG="${QWEN_CONFIG:-${QWEN_VLN_CONFIG:-$ROOT/configs/qwen3_vln_debug.yaml}}"
INSTRUCTION="${1:-find the bottle}"
: "${DASHSCOPE_API_KEY:?Please export DASHSCOPE_API_KEY first}"

ARGS=(
  --config "$CONFIG"
  --instruction "$INSTRUCTION"
)
if [[ -n "${IMAGE_TOPIC:-}" ]]; then
  ARGS+=(--image-topic "$IMAGE_TOPIC")
fi
if [[ -n "${IMAGE_TRANSPORT:-}" ]]; then
  ARGS+=(--image-transport "$IMAGE_TRANSPORT")
fi

exec python3 -u "$ROOT/src/apps/qwen_vln_debug_node.py" "${ARGS[@]}"
