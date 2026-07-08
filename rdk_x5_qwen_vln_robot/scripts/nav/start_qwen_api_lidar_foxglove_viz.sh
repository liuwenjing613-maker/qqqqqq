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

# shellcheck source=scripts/lib/ensure_foxglove_bridge.sh
source "$PROJECT_DIR/scripts/lib/ensure_foxglove_bridge.sh"

mkdir -p logs

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

ensure_foxglove_bridge "$PROJECT_DIR/logs/qwen_api_lidar_foxglove_bridge.log"

start_qwen_api_foxglove_viz_node "$CONFIG" "$PROJECT_DIR/logs/qwen_api_lidar_foxglove_viz.log" || exit 1
VIZ_PID=$FOXGLOVE_VIZ_PID
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
