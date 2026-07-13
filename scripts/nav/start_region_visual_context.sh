#!/usr/bin/env bash
# Start region visual context node (passive capture only; no motion control).
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_DIR"

CHECK_ONLY=0
CONFIG_FILE="$PROJECT_DIR/configs/qwen_region_visual_context.yaml"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check-only)
      CHECK_ONLY=1
      shift
      ;;
    *)
      CONFIG_FILE="$1"
      shift
      ;;
  esac
done

CONFIG_FILE="$(readlink -f "$CONFIG_FILE")"
RUNTIME_DIR="$PROJECT_DIR/runtime/qwen_visual_context"
LOG_ROOT="$PROJECT_DIR/logs/qwen_visual_context"
SCRIPT_PATH="$(readlink -f "$PROJECT_DIR/src/vlm/region_visual_context_node.py")"

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
  else
    echo "[FATAL] code=ROS_SETUP_NOT_FOUND"
    return 1
  fi
  if [[ "$had_nounset" -eq 1 ]]; then
    set -u
  fi
}

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "[FATAL] config not found: $CONFIG_FILE"
  exit 1
fi

source_ros_environment

if ros2 node list 2>/dev/null | grep -qx '/region_visual_context'; then
  echo "[FATAL] code=NODE_ALREADY_RUNNING node=/region_visual_context"
  exit 2
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$(readlink -f "$LOG_ROOT/$RUN_ID")"
mkdir -p "$RUN_DIR" "$RUNTIME_DIR"

echo "============================================================"
echo " QWEN REGION VISUAL CONTEXT — PASSIVE CAPTURE ONLY"
echo "============================================================"
echo "project_dir=$PROJECT_DIR"
echo "config=$CONFIG_FILE"
echo "run_id=$RUN_ID"
echo "run_dir=$RUN_DIR"
echo "allow_motion=false"
echo "allow_cmd_vel=false"

if [[ "$CHECK_ONLY" -eq 1 ]]; then
  echo "[CHECK-ONLY] node_not_started=true"
  exit 0
fi

python3 -u "$SCRIPT_PATH" \
  --config "$CONFIG_FILE" \
  --run-dir "$RUN_DIR" \
  >> "$RUN_DIR/console.log" 2>&1 &

NODE_PID=$!
sleep 1
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "[FATAL] visual context node exited early; see $RUN_DIR/console.log"
  exit 4
fi

CMDLINE="$(tr '\0' ' ' < "/proc/$NODE_PID/cmdline" 2>/dev/null || true)"
if [[ "$CMDLINE" != *"region_visual_context_node.py"* ]]; then
  echo "[FATAL] unexpected process pid=$NODE_PID cmdline=$CMDLINE"
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
    "node_name": "region_visual_context",
}
Path("$META_FILE").write_text(json.dumps(meta, indent=2), encoding="utf-8")
PY

echo "$NODE_PID" > "$RUNTIME_DIR/region_visual_context.pid"
echo "$RUN_DIR" > "$RUNTIME_DIR/region_visual_context.run_dir"

echo "[START] node_pid=$NODE_PID"
echo "[START] region_visual_context running (pid=$NODE_PID)"
echo "[START] logs=$RUN_DIR"
echo "[START] process_meta=$META_FILE"
