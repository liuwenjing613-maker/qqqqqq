#!/usr/bin/env bash
# Start Qwen region selector dry-run node (no motion).
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_DIR"

CONFIG_FILE="$PROJECT_DIR/configs/qwen_region_selector.yaml"
RUNTIME_DIR="$PROJECT_DIR/runtime/qwen_region_selector"
LOG_ROOT="$PROJECT_DIR/logs/qwen_region_selection"
SCRIPT_PATH="$(readlink -f "$PROJECT_DIR/src/vlm/qwen_region_selector_node.py")"

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
  if [[ -f "$HOME/ydlidar_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/ydlidar_ws/install/setup.bash"
  fi
  if [[ "$had_nounset" -eq 1 ]]; then set -u; fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    *) CONFIG_FILE="$1"; shift ;;
  esac
done

CONFIG_FILE="$(readlink -f "$CONFIG_FILE")"
source_ros_environment

for envfile in "$PROJECT_DIR/.env" "$PROJECT_DIR/voice_interaction/.env"; do
  if [[ -f "$envfile" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$envfile"
    set +a
    break
  fi
done

if ros2 node list 2>/dev/null | grep -qx '/qwen_region_selector'; then
  echo "[FATAL] code=NODE_ALREADY_RUNNING node=/qwen_region_selector"
  exit 2
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$(readlink -f "$LOG_ROOT/$RUN_ID")"
mkdir -p "$RUN_DIR/calls" "$RUNTIME_DIR"

ros2 node list > "$RUN_DIR/ros_nodes_before.txt" 2>&1 || true
ros2 topic info /cmd_vel -v > "$RUN_DIR/cmd_vel_before.txt" 2>&1 || true

python3 -u "$SCRIPT_PATH" \
  --config "$CONFIG_FILE" \
  --run-dir "$RUN_DIR" \
  >> "$RUN_DIR/console.log" 2>&1 &

NODE_PID=$!
sleep 1
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "[FATAL] node exited early; see $RUN_DIR/console.log"
  tail -20 "$RUN_DIR/console.log" || true
  exit 4
fi

START_TIME="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
META_FILE="$RUNTIME_DIR/process_meta.json"

python3 - <<PY
import json
from pathlib import Path
meta = {
    "pid": int("$NODE_PID"),
    "project_dir": "$PROJECT_DIR",
    "script_path": "$SCRIPT_PATH",
    "run_dir": "$RUN_DIR",
    "start_time": "$START_TIME",
    "node_name": "qwen_region_selector",
}
Path("$META_FILE").write_text(json.dumps(meta, indent=2), encoding="utf-8")
PY

echo "$NODE_PID" > "$RUNTIME_DIR/qwen_region_selector.pid"
echo "$RUN_DIR" > "$RUNTIME_DIR/qwen_region_selector.run_dir"
echo "[START] node_pid=$NODE_PID run_dir=$RUN_DIR"
