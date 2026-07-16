#!/usr/bin/env bash
# Start Foxglove viz node for Qwen session (robot arrow + Qwen goal markers).
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_DIR"

GOAL_JSON="${1:-$PROJECT_DIR/runtime/qwen_session/navigation_goal_proposal.json}"
RUNTIME_DIR="$PROJECT_DIR/runtime/qwen_session"
LOG_FILE="$RUNTIME_DIR/foxglove_viz.log"
PID_FILE="$RUNTIME_DIR/foxglove_viz.pid"

source_ros_environment() {
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
  # shellcheck source=scripts/lib/ros_dds_env.sh
  source "${PROJECT_DIR}/scripts/lib/ros_dds_env.sh"
  attach_ros_dds_env
  set -u
}

mkdir -p "$RUNTIME_DIR"
GOAL_JSON="$(readlink -f "$GOAL_JSON" 2>/dev/null || echo "$GOAL_JSON")"
touch "$GOAL_JSON" 2>/dev/null || true

if [[ -f "$PID_FILE" ]]; then
  old_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
    cmdline="$(tr '\0' ' ' < "/proc/$old_pid/cmdline" 2>/dev/null || true)"
    if [[ "$cmdline" == *"qwen_session_foxglove_viz_node.py"* ]]; then
      echo "[START][SKIP] qwen_session_foxglove_viz already running pid=$old_pid"
      exit 0
    fi
  fi
fi

source_ros_environment

python3 -u "$PROJECT_DIR/scripts/debug/qwen_session_foxglove_viz_node.py" \
  --ros-args \
  -p goal_json_path:="$GOAL_JSON" \
  >> "$LOG_FILE" 2>&1 &
pid=$!
sleep 1
if ! kill -0 "$pid" 2>/dev/null; then
  echo "[START][FAIL] viz node exited early; see $LOG_FILE"
  tail -20 "$LOG_FILE" || true
  exit 1
fi

echo "$pid" > "$PID_FILE"
echo "[START] qwen_session_foxglove_viz pid=$pid"
echo "[START] goal_json=$GOAL_JSON"
echo "[START] log=$LOG_FILE"
echo "[START] topics:"
echo "  /qwen_session/robot_pose_markers"
echo "  /qwen_session/qwen_goal_markers"
echo "  /qwen_session/candidate_markers"
echo "  /qwen_session/qwen_goal_pose"
