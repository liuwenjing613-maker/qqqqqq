#!/usr/bin/env bash
# Stop only the frontier region debug node by verified PID.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNTIME_DIR="$PROJECT_DIR/runtime/qwen_region_debug"
META_FILE="$RUNTIME_DIR/process_meta.json"
PID_FILE="$RUNTIME_DIR/frontier_region_debug.pid"

source_ros_environment() {
  local had_nounset=0
  case "$-" in
    *u*) had_nounset=1 ;;
  esac
  set +u
  if [[ -f /opt/tros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/tros/humble/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "$HOME/ydlidar_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/ydlidar_ws/install/setup.bash"
  fi
  if [[ "$had_nounset" -eq 1 ]]; then
    set -u
  fi
}

if [[ ! -f "$META_FILE" ]]; then
  echo "[STOP][FAIL] process_meta.json missing: $META_FILE"
  exit 1
fi

PID="$(python3 - <<PY
import json
from pathlib import Path
meta = json.loads(Path("$META_FILE").read_text(encoding="utf-8"))
print(int(meta["pid"]))
PY
)"

if [[ -z "$PID" ]] || ! [[ "$PID" =~ ^[0-9]+$ ]]; then
  echo "[STOP][FAIL] invalid pid in process_meta.json"
  exit 1
fi

if [[ ! -d "/proc/$PID" ]]; then
  echo "[STOP][PASS] debug node already stopped (pid=$PID not running)"
  rm -f "$PID_FILE" "$META_FILE"
  exit 0
fi

CMDLINE="$(tr '\0' ' ' < "/proc/$PID/cmdline")"
CWD="$(readlink -f "/proc/$PID/cwd")"
SCRIPT_NAME_MATCH=false
CWD_MATCH=false
ROS_NODE_MATCH=false

if [[ "$CMDLINE" == *"frontier_region_debug_node.py"* ]]; then
  SCRIPT_NAME_MATCH=true
fi
if [[ "$CWD" == "$PROJECT_DIR" ]]; then
  CWD_MATCH=true
fi

source_ros_environment
if ros2 node list 2>/dev/null | grep -qx '/frontier_region_debug'; then
  ROS_NODE_MATCH=true
fi

echo "[PROCESS_IDENTITY]"
echo "pid=$PID"
echo "cmdline=$CMDLINE"
echo "cwd=$CWD"
echo "script_name_match=$SCRIPT_NAME_MATCH"
echo "cwd_match=$CWD_MATCH"
echo "ros_node_match=$ROS_NODE_MATCH"

if [[ "$SCRIPT_NAME_MATCH" != true ]] || [[ "$CWD_MATCH" != true ]] || [[ "$ROS_NODE_MATCH" != true ]]; then
  echo "[STOP][FAIL] code=PID_IDENTITY_MISMATCH decision=DENY_EXACT_STOP"
  exit 2
fi

echo "decision=ALLOW_EXACT_STOP"
echo "[STOP] sending INT to pid=$PID"
kill -INT "$PID" 2>/dev/null || true

for _ in $(seq 1 20); do
  if ! kill -0 "$PID" 2>/dev/null; then
    break
  fi
  sleep 0.25
done

if kill -0 "$PID" 2>/dev/null; then
  echo "[STOP] sending TERM to pid=$PID"
  kill -TERM "$PID" 2>/dev/null || true
  sleep 1
fi

if kill -0 "$PID" 2>/dev/null; then
  echo "[STOP][FAIL] node still running after INT/TERM pid=$PID"
  exit 3
fi

rm -f "$PID_FILE" "$META_FILE"
echo "[STOP][PASS] target_pid=$PID target_identity_verified=true debug_node_stopped=true other_processes_targeted=0 broad_kill_used=false"
