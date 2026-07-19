#!/usr/bin/env bash
set -Eeuo pipefail
REPO_ROOT="${ROBOT_PROJECT_DIR:-/root/rdk_x5_vln_robot}"
HOST="${VOICE_DEMO_WEB_HOST:-0.0.0.0}"
PORT="${VOICE_DEMO_WEB_PORT:-8090}"
FPS="${VOICE_DEMO_WEB_FPS:-8}"

usage() {
  cat <<EOF
Usage: bash scripts/demo/start_voice_demo_hub_web_view_v1.sh [--port N] [--fps N]

Composite live webpage:
  camera + SLAM map/scan/odom (+ qwen/hub status when present)
  http://<board_ip>:${PORT}/
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="${2:?}"; shift 2 ;;
    --host) HOST="${2:?}"; shift 2 ;;
    --fps) FPS="${2:?}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

cd "$REPO_ROOT"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$REPO_ROOT/install/setup.bash"
fi
set -u

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "[web-view] open: http://${IP:-<board_ip>}:${PORT}/"
echo "[web-view] foxglove (3D): ws://${IP:-<board_ip>}:8765"
exec python3 -u "$REPO_ROOT/scripts/camera/robot_web_dashboard.py" \
  --host "$HOST" --port "$PORT" --fps "$FPS" "$@"
