#!/usr/bin/env bash
# Stop only qwen_region_selector by verified PID.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNTIME_DIR="$PROJECT_DIR/runtime/qwen_region_selector"
META_FILE="$RUNTIME_DIR/process_meta.json"

source_ros_environment() {
  local had_nounset=0
  case "$-" in *u*) had_nounset=1 ;; esac
  set +u
  if [[ -f /opt/tros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/tros/humble/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [[ "$had_nounset" -eq 1 ]]; then set -u; fi
}

if [[ ! -f "$META_FILE" ]]; then
  echo "[STOP][FAIL] process_meta.json missing"
  exit 1
fi

PID="$(python3 - <<PY
import json
from pathlib import Path
print(int(json.loads(Path("$META_FILE").read_text())["pid"]))
PY
)"

if [[ ! -d "/proc/$PID" ]]; then
  echo "[STOP][PASS] already stopped pid=$PID"
  rm -f "$META_FILE"
  exit 0
fi

CMDLINE="$(tr '\0' ' ' < "/proc/$PID/cmdline")"
CWD="$(readlink -f "/proc/$PID/cwd")"
SCRIPT_NAME_MATCH=false
CWD_MATCH=false
ROS_NODE_MATCH=false

[[ "$CMDLINE" == *"qwen_region_selector_node.py"* ]] && SCRIPT_NAME_MATCH=true
[[ "$CWD" == "$PROJECT_DIR" ]] && CWD_MATCH=true

source_ros_environment
ros2 node list 2>/dev/null | grep -qx '/qwen_region_selector' && ROS_NODE_MATCH=true

echo "[PROCESS_IDENTITY]"
echo "pid=$PID"
echo "cmdline=$CMDLINE"
echo "cwd=$CWD"
echo "script_name_match=$SCRIPT_NAME_MATCH"
echo "cwd_match=$CWD_MATCH"
echo "ros_node_match=$ROS_NODE_MATCH"

if [[ "$SCRIPT_NAME_MATCH" != true ]] || [[ "$CWD_MATCH" != true ]] || [[ "$ROS_NODE_MATCH" != true ]]; then
  echo "[STOP][FAIL] decision=DENY_EXACT_STOP"
  exit 2
fi

echo "decision=ALLOW_EXACT_STOP"
kill -INT "$PID" 2>/dev/null || true
for _ in $(seq 1 20); do
  kill -0 "$PID" 2>/dev/null || break
  sleep 0.25
done
if kill -0 "$PID" 2>/dev/null; then
  kill -TERM "$PID" 2>/dev/null || true
  sleep 1
fi
if kill -0 "$PID" 2>/dev/null; then
  echo "[STOP][FAIL] still running"
  exit 3
fi

rm -f "$META_FILE"
echo "[STOP][PASS] debug_node_stopped=true broad_kill_used=false"
