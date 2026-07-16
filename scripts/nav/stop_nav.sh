#!/usr/bin/env bash
# Stop mapping + saved-map Nav2 stacks. Prefer this over partial pkill lists.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
# shellcheck source=scripts/lib/ros_dds_env.sh
source "${PROJECT_DIR}/scripts/lib/ros_dds_env.sh"

set +u
if [[ -f /opt/tros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${HOME}/ydlidar_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/ydlidar_ws/install/setup.bash"
fi
set -u
prepare_ros_dds_env

cleanup_click_nav_stack_processes "STOP_NAV" echo
pkill -9 -f "run_joy_mapping_calibrated.sh" 2>/dev/null || true
pkill -9 -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
pkill -9 -f "run_slam_calibrated.sh" 2>/dev/null || true
timeout 5 ros2 daemon stop >/dev/null 2>&1 || true

echo "[stop_nav] mapping + Nav2 stacks stopped."
