#!/usr/bin/env bash
# Stop standalone Foxglove camera preview (camera + /image_viz throttle).
# Does NOT kill an already-running foxglove_bridge from other stacks unless
# FOXGLOVE_CAM_KILL_BRIDGE=1.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
export PROJECT_DIR

set +u
if [[ -f /opt/tros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
set -u

# shellcheck disable=SC1091
source "${PROJECT_DIR}/scripts/lib/camera_stack.sh"

echo "[foxglove_cam] stopping image throttle"
pkill -TERM -f "foxglove_image_throttle.py" 2>/dev/null || true
sleep 0.3
pkill -KILL -f "foxglove_image_throttle.py" 2>/dev/null || true

echo "[foxglove_cam] stopping USB camera stack"
stop_camera_stack

if [[ "${FOXGLOVE_CAM_KILL_BRIDGE:-0}" == "1" ]]; then
  echo "[foxglove_cam] stopping foxglove_bridge (FOXGLOVE_CAM_KILL_BRIDGE=1)"
  pkill -TERM -f "foxglove_bridge" 2>/dev/null || true
  sleep 0.5
  pkill -KILL -f "foxglove_bridge" 2>/dev/null || true
else
  echo "[foxglove_cam] leaving foxglove_bridge running (set FOXGLOVE_CAM_KILL_BRIDGE=1 to kill it)"
fi

echo "[foxglove_cam] stopped"
