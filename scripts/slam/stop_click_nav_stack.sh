#!/usr/bin/env bash
# Stop click-navigation / saved-map Nav2 stack and SLAM conflicts (project scope only).
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/cleanup_lidar_slam_nav.sh
source "${SCRIPT_DIR}/../lib/cleanup_lidar_slam_nav.sh"

log() { echo "[STOP_NAV] $*"; }

# Stop other wrapper instances, then tear down the full stack.
if pgrep -f "run_nav2_foxglove_click_goal.sh" >/dev/null 2>&1; then
  log "stop other run_nav2_foxglove_click_goal.sh instances"
  _click_nav_safe_pkill_wait "run_nav2_foxglove_click_goal.sh" 2
fi

cleanup_click_nav_stack_processes "STOP_NAV" log

log "done."
