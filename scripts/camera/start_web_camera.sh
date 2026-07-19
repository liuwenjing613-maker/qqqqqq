#!/usr/bin/env bash
# ROS camera web stream — subscribes to /qwen_vln/annotated_image/compressed only.
#
# Prerequisite: Qwen node publishing annotated frames.
#
# Usage:
#   bash scripts/camera/start_web_camera.sh
#   bash scripts/camera/start_web_camera.sh --port 8090 --fps 10
#
# Open: http://<RDK_IP>:8090/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

set +u
if [[ -f /opt/tros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
set -u

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "[web_cam] annotated camera stream (ROS subscribe only)"
echo "[web_cam] open: http://${IP:-<board_ip>}:${WEB_PORT:-8090}/"
echo "[web_cam] topic: /qwen_vln/annotated_image/compressed"

exec python3 -u "${SCRIPT_DIR}/robot_web_dashboard.py" "$@"
