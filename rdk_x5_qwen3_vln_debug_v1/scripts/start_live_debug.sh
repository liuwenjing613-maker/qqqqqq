#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/lib/ros_env.sh"
source "$ROOT/scripts/lib/camera_stack.sh"
mkdir -p "$ROOT/logs"
INSTRUCTION="${1:-find the bottle}"
: "${DASHSCOPE_API_KEY:?Please export DASHSCOPE_API_KEY first}"
CAMERA_PID=""; BRIDGE_PID=""; QWEN_PID=""; FOXGLOVE_PID=""
cleanup() {
  echo "[cleanup] stopping only processes started by this script"
  for pid in "$QWEN_PID" "$BRIDGE_PID" "$FOXGLOVE_PID"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then kill "$pid" 2>/dev/null || true; fi
  done
  # Camera launch leaves hobot_usb_cam children; stop the whole tree / orphans.
  stop_camera_tree "${CAMERA_PID:-}" || true
}
trap cleanup EXIT INT TERM

echo "===== Qwen3-VL live camera debug V1.1 ====="
echo "compressed=${CAMERA_COMPRESSED_TOPIC:-/image} raw=${IMAGE_RAW_TOPIC:-/image_raw}"
ensure_compressed_camera "$ROOT" "$ROOT/logs/camera.log"
start_raw_bridge "$ROOT" "$ROOT/logs/image_raw_bridge.log"

if [ "${START_FOXGLOVE:-1}" = "1" ]; then
  port="${FOXGLOVE_PORT:-8765}"
  whitelist="${FOXGLOVE_TOPIC_WHITELIST:-['^/image$','^/camera_info$','^/qwen_vln/annotated_image/compressed$','^/qwen_vln/servo/.*','^/qwen_vln/(command|state|result_json|latency_ms|pixel_point|prompt_text)$','^/third_view/.*','^/map_qwen_plan/(backend_debug|bridge_status|status|candidate_summary)$','^/tf$','^/tf_static$','^/scan_filtered$','^/odom$','^/map$','^/map_metadata$']}"
  project_dir="$(cd "$ROOT/.." && pwd)"
  if ss -tln 2>/dev/null | grep -q ":${port} "; then
    echo "[foxglove] reuse port $port"
  elif ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    FOXGLOVE_PORT="$port" FOXGLOVE_TOPIC_WHITELIST="$whitelist" \
      bash "$project_dir/scripts/lidar/start_foxglove.sh" \
      >"$ROOT/logs/foxglove_bridge.log" 2>&1 &
    FOXGLOVE_PID=$!
    echo "[foxglove] starting on ws://<RDK-IP>:$port whitelist=$whitelist"
  else
    echo "[foxglove] WARN: foxglove_bridge package not installed"
  fi
fi

python3 -u "$ROOT/src/apps/qwen_vln_debug_node.py" \
  --config "$ROOT/configs/qwen3_vln_debug.yaml" \
  --instruction "$INSTRUCTION" \
  >"$ROOT/logs/qwen_vln_debug.log" 2>&1 &
QWEN_PID=$!

for _ in $(seq 1 20); do
  if ros2 topic info /qwen_vln/state 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then break; fi
  if ! kill -0 "$QWEN_PID" 2>/dev/null; then
    echo "[qwen] ERROR: node exited"
    tail -n 80 "$ROOT/logs/qwen_vln_debug.log" || true
    exit 1
  fi
  sleep 1
done

echo "[READY] Foxglove image: /qwen_vln/annotated_image/compressed"
echo "[READY] Raw camera:     ${IMAGE_RAW_TOPIC:-/image_raw}"
echo "[READY] Bridge status:  /image_raw_bridge/status"
echo "[READY] Qwen state:     /qwen_vln/state"
echo "[READY] Result JSON:    /qwen_vln/result_json"
echo "[READY] Logs: tail -f $ROOT/logs/qwen_vln_debug.log"
echo "Press Ctrl+C to stop this V1.1 stack."
wait "$QWEN_PID"
