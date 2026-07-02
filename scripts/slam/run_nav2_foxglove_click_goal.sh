#!/usr/bin/env bash
# Start saved-map Nav2, Foxglove bridge, and click-goal bridge.
# This script does NOT modify the existing mapping scripts.

set -Eeo pipefail

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
MAP_YAML="${MAP_YAML:-$PROJECT_DIR/maps/joy_calibrated_corridor_map.yaml}"
GOAL_TOPIC="${GOAL_TOPIC:-/foxglove_goal_pose}"
START_NAV2="${START_NAV2:-1}"
STATE_DIR="$PROJECT_DIR/state"
POSE_STATE_FILE="${POSE_STATE_FILE:-$STATE_DIR/last_pose_map.json}"
LOG_DIR="${PROJECT_DIR}/logs/nav2_foxglove_click_$(date +%Y%m%d_%H%M%S)"

PIDS=()

log() { echo "[CLICK_NAV2] $*"; }

source_ros() {
  set +u
  [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
  [ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
  [ -f "$HOME/ydlidar_ws/install/setup.bash" ] && source "$HOME/ydlidar_ws/install/setup.bash"
  set -u
}

start_bg() {
  local name="$1"
  shift
  log "start ${name}: $*"
  "$@" > "${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

zero_cmd() {
  timeout 1.5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
}

cleanup() {
  log "cleanup: stop robot and processes started by this wrapper"
  zero_cmd
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" >/dev/null 2>&1 || true
  done
}
trap cleanup INT TERM EXIT

wait_topic_exists() {
  local topic="$1"
  local timeout_sec="${2:-60}"
  local start
  start="$(date +%s)"
  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      log "topic OK: $topic"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_sec" ]; then
      log "ERROR: topic not found: $topic"
      return 1
    fi
    sleep 1
  done
}

wait_action_exists() {
  local action_name="$1"
  local timeout_sec="${2:-90}"
  local start
  start="$(date +%s)"
  while true; do
    if ros2 action list 2>/dev/null | grep -qx "$action_name"; then
      log "action OK: $action_name"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_sec" ]; then
      log "ERROR: action not found: $action_name"
      return 1
    fi
    sleep 1
  done
}

main() {
  source_ros
  cd "$PROJECT_DIR"
  mkdir -p "$LOG_DIR" "$STATE_DIR"

  log "PROJECT_DIR=$PROJECT_DIR"
  log "MAP_YAML=$MAP_YAML"
  log "GOAL_TOPIC=$GOAL_TOPIC"
  log "POSE_STATE_FILE=$POSE_STATE_FILE"
  log "LOG_DIR=$LOG_DIR"

  if [ ! -f "$MAP_YAML" ] && [ -f "$PROJECT_DIR/maps/joy_corridor_map.yaml" ]; then
    log "WARN: MAP_YAML not found, fallback to joy_corridor_map.yaml"
    MAP_YAML="$PROJECT_DIR/maps/joy_corridor_map.yaml"
  fi

  if [ ! -f "$MAP_YAML" ]; then
    log "ERROR: map yaml not found: $MAP_YAML"
    log "先运行 scripts/slam/run_joy_mapping_calibrated.sh，Ctrl+C 保存地图，再运行本脚本。"
    exit 1
  fi

  if [ ! -f "$PROJECT_DIR/scripts/slam/foxglove_click_goal_bridge.py" ]; then
    log "ERROR: missing $PROJECT_DIR/scripts/slam/foxglove_click_goal_bridge.py"
    exit 1
  fi

  chmod +x "$PROJECT_DIR/scripts/slam/foxglove_click_goal_bridge.py" || true

  if [ "$START_NAV2" = "1" ]; then
    if [ ! -x "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh" ]; then
      log "WARN: run_nav2_saved_map.sh is not executable; trying chmod +x"
      chmod +x "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh" || true
    fi
    start_bg nav2_saved_map bash "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh"
  else
    log "START_NAV2=0: assume Nav2 is already running."
    start_bg pose_memory python3 "$PROJECT_DIR/scripts/slam/pose_memory_node.py" \
      --state-file "$POSE_STATE_FILE" \
      --map-frame map \
      --base-frame base_link \
      --fallback-base-frame base_footprint \
      --save-period 1.0 \
      --publish-initial \
      --initial-repeat 5 \
      --initial-interval 0.3
  fi

  wait_topic_exists /map 120 || exit 1
  wait_topic_exists /odom 120 || exit 1
  wait_topic_exists /tf 60 || exit 1
  wait_action_exists /navigate_to_pose 160 || exit 1

  # /compute_path_to_pose is used only to pre-draw the path; navigation can still work if it is delayed.
  wait_action_exists /compute_path_to_pose 30 || log "WARN: /compute_path_to_pose not ready yet; bridge will keep trying when goals arrive."

  start_bg click_goal_bridge python3 "$PROJECT_DIR/scripts/slam/foxglove_click_goal_bridge.py" \
    --goal-topic "$GOAL_TOPIC" \
    --goal-frame map

  BOARD_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo
  echo "========== Foxglove click navigation ready =========="
  echo "Connect: ws://${BOARD_IP}:8765"
  echo "3D fixed frame: map"
  echo "Optional layout: ${PROJECT_DIR}/configs/foxglove_click_goal_nav.layout.json"
  echo "Initial pose tool topic: /initialpose"
  echo "Goal 2D pose tool topic: ${GOAL_TOPIC} (geometry_msgs/PoseStamped)"
  echo "Path topic to display: /foxglove_click_planned_path"
  echo "Marker topic to display: /foxglove_click_path_marker"
  echo "Pose memory file: ${POSE_STATE_FILE}"
  echo "Logs: ${LOG_DIR}"
  echo "Press Ctrl+C here to stop wrapper-started processes."
  echo "===================================================="

  while true; do sleep 3600; done
}

main "$@"
