#!/usr/bin/env bash
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../lib/qwen_ready.sh"

check_topic() {
  local name="$1"
  if _qwen_debug_topic_ready "$name"; then
    echo "[OK] publisher $name"
  else
    echo "[MISS] publisher $name"
  fi
}
check_subscriber() {
  local name="$1"
  if _qwen_ros2_topic_info "$name" | grep -Eq 'Subscription count: [1-9][0-9]*'; then
    echo "[OK] subscriber $name"
  else
    echo "[MISS] subscriber $name"
  fi
}
check_topic /image_raw
check_topic /qwen_vln/state
check_topic /qwen_vln/result_json
check_topic /scan_filtered
check_topic /qwen_vln/servo/status
check_subscriber /cmd_vel

echo "--- current servo status ---"
timeout 3 ros2 topic echo --once /qwen_vln/servo/status 2>/dev/null || true
