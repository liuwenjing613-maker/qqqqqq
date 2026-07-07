#!/usr/bin/env bash
# Foxglove visualization for Qwen API + LiDAR nav (run alongside start_qwen_api_lidar_nav.sh).
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

CONFIG="${CONFIG:-$PROJECT_DIR/configs/qwen_api_lidar_nav.yaml}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
RDK_ORIGINAL_ROOT="${RDK_ORIGINAL_ROOT:-/root/rdk_x5_vln_robot}"

set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$HOME/ydlidar_ws/install/setup.bash" ]; then
  source "$HOME/ydlidar_ws/install/setup.bash"
fi
set -u

mkdir -p logs

ensure_foxglove_bridge() {
  if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    echo "[foxglove] ERROR: foxglove_bridge not installed"
    echo "  Install ROS package foxglove_bridge or use Foxglove Studio native ROS connection."
    return 1
  fi
  if command -v ss >/dev/null 2>&1 && ss -tln 2>/dev/null | grep -q ":${FOXGLOVE_PORT} "; then
    echo "[foxglove] OK: already listening on ws port ${FOXGLOVE_PORT}"
    return 0
  fi
  echo "[foxglove] starting bridge on port ${FOXGLOVE_PORT}..."
  bash "$RDK_ORIGINAL_ROOT/scripts/lidar/start_foxglove.sh" \
    > "$PROJECT_DIR/logs/qwen_api_lidar_foxglove_bridge.log" 2>&1 &
  for _ in $(seq 1 20); do
    if ss -tln 2>/dev/null | grep -q ":${FOXGLOVE_PORT} "; then
      echo "[foxglove] OK: listening on ${FOXGLOVE_PORT}"
      return 0
    fi
    sleep 1
  done
  echo "[foxglove] ERROR: bridge failed; see $PROJECT_DIR/logs/qwen_api_lidar_foxglove_bridge.log"
  tail -20 "$PROJECT_DIR/logs/qwen_api_lidar_foxglove_bridge.log" 2>/dev/null || true
  return 1
}

cleanup() {
  if [ -n "${VIZ_PID:-}" ] && kill -0 "$VIZ_PID" 2>/dev/null; then
    kill "$VIZ_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "===== Qwen API LiDAR Foxglove Viz ====="
echo "PROJECT_DIR=$PROJECT_DIR"
echo "CONFIG=$CONFIG"
echo "Foxglove WebSocket: ws://<robot-ip>:${FOXGLOVE_PORT}"
echo ""
echo "Prerequisite: nav stack running (start_qwen_api_lidar_nav.sh) with /image_raw /scan /qwen_api_*"

ensure_foxglove_bridge

echo "[viz] starting qwen_api_lidar_foxglove_viz..."
python3 "$PROJECT_DIR/src/apps/qwen_api_lidar_foxglove_viz.py" \
  --config "$CONFIG" \
  > "$PROJECT_DIR/logs/qwen_api_lidar_foxglove_viz.log" 2>&1 &
VIZ_PID=$!
sleep 1
if ! kill -0 "$VIZ_PID" 2>/dev/null; then
  echo "[viz] ERROR: viz node exited; see logs/qwen_api_lidar_foxglove_viz.log"
  tail -30 "$PROJECT_DIR/logs/qwen_api_lidar_foxglove_viz.log" 2>/dev/null || true
  exit 1
fi

echo "Started viz pid=$VIZ_PID"
echo ""
echo "Foxglove panels (suggested layout):"
echo "  1) Image     -> /qwen_api_viz/image/compressed"
echo "  2) 3D        -> Fixed frame: laser (or scan frame_id)"
echo "                 -> /qwen_api_viz/markers"
echo "                 -> /scan"
echo "  3) Raw JSON  -> /qwen_api_viz/hud  (or /qwen_api_json / /qwen_api_state)"
echo "  4) Optional  -> /image_raw (raw camera)"
echo ""
echo "  tail -f $PROJECT_DIR/logs/qwen_api_lidar_foxglove_viz.log"
echo "Press Ctrl+C to stop viz node (foxglove bridge keeps running if shared)."

wait "$VIZ_PID"
