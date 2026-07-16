#!/usr/bin/env bash
# Start frontier region debug node (observation-only; does not stop other stacks).
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_DIR"

CHECK_ONLY=0
CONFIG_FILE="$PROJECT_DIR/configs/qwen_region_explore_debug.yaml"

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
RUNTIME_DIR="$PROJECT_DIR/runtime/qwen_region_debug"
LOG_ROOT="$PROJECT_DIR/logs/qwen_region_explore"
SCRIPT_PATH="$(readlink -f "$PROJECT_DIR/src/planning/frontier_region_debug_node.py")"

ROS_SETUP_USED=""
YDLIDAR_SETUP_USED=""

source_ros_environment() {
  local had_nounset=0

  case "$-" in
    *u*) had_nounset=1 ;;
  esac

  set +u

  if [[ -f /opt/tros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/tros/humble/setup.bash
    ROS_SETUP_USED=/opt/tros/humble/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    ROS_SETUP_USED=/opt/ros/humble/setup.bash
  else
    echo "[FATAL] code=ROS_SETUP_NOT_FOUND"
    return 1
  fi

  if [[ -f "$HOME/ydlidar_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/ydlidar_ws/install/setup.bash"
    YDLIDAR_SETUP_USED="$HOME/ydlidar_ws/install/setup.bash"
  else
    YDLIDAR_SETUP_USED=""
  fi

  # shellcheck source=scripts/lib/ros_dds_env.sh
  source "${PROJECT_DIR}/scripts/lib/ros_dds_env.sh"
  attach_ros_dds_env

  if [[ "$had_nounset" -eq 1 ]]; then
    set -u
  fi
}

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "[FATAL] config not found: $CONFIG_FILE"
  exit 1
fi

source_ros_environment

echo "[ENV]"
echo "ros_setup=${ROS_SETUP_USED:-unset}"
echo "ydlidar_setup=${YDLIDAR_SETUP_USED:-unset}"
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-default}"
echo "nounset_restored=true"

if ros2 node list 2>/dev/null | grep -qx '/frontier_region_debug'; then
  echo "[FATAL] code=NODE_ALREADY_RUNNING node=/frontier_region_debug"
  exit 2
fi

if ! ros2 topic list 2>/dev/null | grep -qx '/map'; then
  echo "[FATAL]"
  echo "code=MAP_TOPIC_MISSING"
  echo "topic=/map"
  echo "existing_processes_modified=false"
  echo "other_modules_started=false"
  echo "diagnostic_node_started=false"
  exit 3
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$(readlink -f "$LOG_ROOT/$RUN_ID")"
mkdir -p "$RUN_DIR" "$RUN_DIR/snapshots" "$RUNTIME_DIR"

ros2 node list > "$RUN_DIR/ros_nodes_before.txt" 2>&1 || true
ros2 topic list -t > "$RUN_DIR/ros_topics_before.txt" 2>&1 || true
ros2 topic info /cmd_vel -v > "$RUN_DIR/cmd_vel_before.txt" 2>&1 || true

GIT_BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
GIT_COMMIT="$(git rev-parse HEAD 2>/dev/null || echo unknown)"

echo "============================================================"
echo " QWEN FRONTIER REGION DEBUG — OBSERVATION ONLY"
echo "============================================================"
echo "project_dir=$PROJECT_DIR"
echo "config=$CONFIG_FILE"
echo "run_id=$RUN_ID"
echo "run_dir=$RUN_DIR"
echo "git_branch=$GIT_BRANCH"
echo "git_commit=$GIT_COMMIT"
echo "motion_enabled=false"
echo "qwen_enabled=false"
echo "nav2_enabled=false"
echo "existing_processes_will_not_be_stopped=true"
echo ""
echo "[CHECK] config: PASS"
echo "[CHECK] /map topic: PASS"
echo "[CHECK] duplicate debug node: PASS"
echo "[CHECK] cmd_vel baseline captured: PASS"
echo "[CHECK] log directory: PASS"

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
  echo "[FATAL] diagnostic node exited early; see $RUN_DIR/console.log"
  tail -20 "$RUN_DIR/console.log" || true
  exit 4
fi

# Verify it is the python debug node (not a shell wrapper)
CMDLINE="$(tr '\0' ' ' < "/proc/$NODE_PID/cmdline" 2>/dev/null || true)"
if [[ "$CMDLINE" != *"frontier_region_debug_node.py"* ]]; then
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
    "node_name": "frontier_region_debug",
}
Path("$META_FILE").write_text(json.dumps(meta, indent=2), encoding="utf-8")
PY

echo "$NODE_PID" > "$RUNTIME_DIR/frontier_region_debug.pid"
echo "$RUN_DIR" > "$RUNTIME_DIR/frontier_region_debug.run_dir"

ros2 topic info /cmd_vel -v > "$RUN_DIR/cmd_vel_after.txt" 2>&1 || true

echo "[START] node_pid=$NODE_PID"
echo "[START] frontier_region_debug running (pid=$NODE_PID)"
echo "[START] logs=$RUN_DIR"
echo "[START] process_meta=$META_FILE"
