#!/usr/bin/env bash
set -u
RUNTIME_DIR="${FULLFLOW_RUNTIME_DIR:-/tmp/rdk_x5_vln_fullflow_v2_${USER:-robot}}"
PID_FILE="$RUNTIME_DIR/pids.env"
if [[ ! -f "$PID_FILE" ]]; then
  echo "[FULLFLOW_V2] no PID file; trying named-process cleanup"
  pkill -TERM -f 'online_map_qwen_nav_backend_v2.py' 2>/dev/null || true
  pkill -TERM -f 'nav2_online_slam_fusion_navigation_launch_v2.py' 2>/dev/null || true
  pkill -TERM -f 'start_v1_map_qwen_fullflow_v2.sh' 2>/dev/null || true
  exit 0
fi
# shellcheck disable=SC1090
source "$PID_FILE"
for pid in "${FUSION_PID:-}" "${BACKEND_PID:-}" "${NAV2_PID:-}"; do
  [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null || true
done
if [[ "${STARTED_SLAM:-0}" == 1 && -n "${SLAM_PID:-}" ]]; then
  kill -TERM -- "-$SLAM_PID" 2>/dev/null || true
fi
rm -f "$PID_FILE"
echo "[FULLFLOW_V2] stop signals sent"
