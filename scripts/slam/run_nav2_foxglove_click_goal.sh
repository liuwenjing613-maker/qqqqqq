#!/usr/bin/env bash
# Start saved-map Nav2, Foxglove bridge, and click-goal bridge.
# This script does NOT modify the existing mapping scripts.

set -Eeo pipefail

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
source "${PROJECT_DIR}/scripts/lib/nav2_localization_bootstrap.sh"
source "${PROJECT_DIR}/scripts/lib/cleanup_lidar_slam_nav.sh"
MAP_YAML="${MAP_YAML:-$PROJECT_DIR/maps/joy_calibrated_corridor_map.yaml}"
GOAL_POINT_TOPIC="${GOAL_POINT_TOPIC:-/foxglove_goal_point}"
GOAL_POSE_TOPIC="${GOAL_POSE_TOPIC:-/foxglove_goal_pose}"
GOAL_TOPIC="${GOAL_TOPIC:-$GOAL_POSE_TOPIC}"
ROBOT_BASE_FRAME="${ROBOT_BASE_FRAME:-base_link}"
START_NAV2="${START_NAV2:-1}"
STATE_DIR="$PROJECT_DIR/state"
POSE_STATE_FILE="${POSE_STATE_FILE:-$STATE_DIR/last_pose_map.json}"
LOG_DIR="${PROJECT_DIR}/logs/nav2_foxglove_click_$(date +%Y%m%d_%H%M%S)"

PIDS=()
CLEANUP_DONE=0

log() { echo "[CLICK_NAV2] $*"; }

source_ros() {
  set +u
  [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
  [ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
  [ -f "$HOME/ydlidar_ws/install/setup.bash" ] && source "$HOME/ydlidar_ws/install/setup.bash"
  set -u
  export_ros_dds_env
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
  if [ "$CLEANUP_DONE" = "1" ]; then
    return 0
  fi
  CLEANUP_DONE=1
  trap - INT TERM EXIT

  log "cleanup: stopping click-nav stack (Ctrl+C / exit)..."

  # Stop direct children first (nav2_saved_map, click_goal_bridge, pose_memory).
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 1
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done

  # Kill any leftover Nav2 / lidar / chassis / bridge nodes and clear DDS shm.
  cleanup_click_nav_stack_processes "CLICK_NAV2" log

  log "cleanup done — safe to restart run_nav2_foxglove_click_goal.sh"
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

wait_map_base_link_tf_ready() {
  local timeout_sec="${1:-240}"
  python3 - "$timeout_sec" <<'PY'
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

timeout = float(sys.argv[1])
rclpy.init()
node = Node("click_nav_wait_map_base")
buf = Buffer(cache_time=Duration(seconds=30.0))
TransformListener(buf, node, spin_thread=False)
start = time.time()
while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        buf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.3))
        print("[CLICK_NAV2] TF map -> base_link OK")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    except Exception:
        pass
node.destroy_node()
rclpy.shutdown()
print("[CLICK_NAV2] ERROR: map -> base_link TF not available (align /initialpose in Foxglove)")
raise SystemExit(1)
PY
}

wait_nav2_saved_map_ready() {
  local ready_timeout="${1:-300}"
  local min_epoch="${2:-0}"
  local start last_heartbeat
  start="$(date +%s)"
  last_heartbeat="$start"
  log "waiting for Nav2 full stack (ready file, up to ${ready_timeout}s)..."
  log "  backend steps: lidar -> chassis -> map_server -> AMCL -> planner -> bt_navigator"
  log "  tail progress: tail -f ${LOG_DIR}/nav2_saved_map.log"
  while true; do
    if [ -n "${NAV2_SAVED_MAP_PID:-}" ] && ! kill -0 "$NAV2_SAVED_MAP_PID" 2>/dev/null; then
      log "ERROR: nav2_saved_map backend exited before ready file appeared"
      log "HINT: tail -30 ${LOG_DIR}/nav2_saved_map.log"
      tail -n 8 "${LOG_DIR}/nav2_saved_map.log" 2>/dev/null | sed 's/^/[CLICK_NAV2]   /' || true
      return 1
    fi
    local ready_file
    ready_file="$(python3 - "$PROJECT_DIR" "$min_epoch" <<'PY'
import glob
import os
import sys

project_dir = sys.argv[1]
min_epoch = float(sys.argv[2])
candidates = []
for path in glob.glob(os.path.join(project_dir, "logs", "nav2_*", "ready")):
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        continue
    if mtime + 1.0 >= min_epoch:
        candidates.append((mtime, path))
if not candidates:
    raise SystemExit(0)
candidates.sort(reverse=True)
print(candidates[0][1])
PY
)"
    if [ -n "$ready_file" ] && [ -f "$ready_file" ]; then
      log "nav2_saved_map ready: $ready_file (elapsed=$(( $(date +%s) - start ))s)"
      return 0
    fi
    local now elapsed
    now="$(date +%s)"
    elapsed=$(( now - start ))
    if [ $(( now - last_heartbeat )) -ge 15 ]; then
      local boot_hint
      boot_hint="$(tail -n 1 "${LOG_DIR}/nav2_saved_map.log" 2>/dev/null | sed 's/^[[:space:]]*//')"
      if [ -n "$boot_hint" ]; then
        log "still waiting for Nav2 ready... ${elapsed}s elapsed | backend: ${boot_hint}"
      else
        log "still waiting for Nav2 ready... ${elapsed}s elapsed (normal: 90-210s on first boot)"
      fi
      last_heartbeat="$now"
    fi
    if [ "$elapsed" -ge "$ready_timeout" ]; then
      log "ERROR: nav2_saved_map ready file not found after ${ready_timeout}s"
      log "HINT: stale ready files are ignored; waiting for the Nav2 stack started by this script."
      log "HINT: check ${LOG_DIR}/nav2_saved_map.log for lifecycle errors."
      return 1
    fi
    sleep 2
  done
}

print_click_ready_banner() {
  local board_ip
  board_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo
  echo "╔══════════════════════════════════════════════════════════════╗"
  echo "║  可以开始点击鼠标导航了 / READY FOR MOUSE CLICK NAVIGATION   ║"
  echo "╚══════════════════════════════════════════════════════════════╝"
  echo
  echo "前提检查（Foxglove 3D 面板）："
  echo "  1. Fixed frame = map"
  echo "  2. 显示 /map 与 /scan_filtered（或 /scan）"
  echo "  3. 激光 scan 应与地图白色墙壁对齐；若明显错位，请先用"
  echo "     Publish -> 2D 位姿估计 -> /initialpose 手动校正朝向"
  echo
  if [ -f "$POSE_STATE_FILE" ]; then
    print_pose_state_summary "$POSE_STATE_FILE" || true
  fi
  echo
  echo "连接: ws://${board_ip}:8765"
  echo "布局: ${PROJECT_DIR}/configs/foxglove_click_goal_nav.layout.json"
  echo "工具: Publish -> 2D point -> ${GOAL_POINT_TOPIC}"
  echo "操作: 在地图白色区域单击一次（导航进行中请勿重复点击）"
  echo "路径显示: /foxglove_click_planned_path"
  echo "终点标注: /foxglove_click_goal_marker + /foxglove_click_goal_label"
  echo "日志目录:"
  echo "  点击导航: ${LOG_DIR}/click_goal_bridge.log"
  echo "  Nav2栈:   ${LOG_DIR}/nav2_saved_map.log  (及 logs/nav2_*/nav2.log)"
  echo "实时: tail -f ${LOG_DIR}/click_goal_bridge.log"
  echo "按 Ctrl+C 停止本脚本（将自动清理全部 Nav2 / 雷达 / 底盘 / Foxglove 进程）"
  echo "=============================================================="
}

main() {
  source_ros
  cd "$PROJECT_DIR"
  mkdir -p "$LOG_DIR" "$STATE_DIR"

  log "PROJECT_DIR=$PROJECT_DIR"
  log "MAP_YAML=$MAP_YAML"
  log "GOAL_POINT_TOPIC=$GOAL_POINT_TOPIC"
  log "GOAL_POSE_TOPIC=$GOAL_POSE_TOPIC"
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

  local nav2_stack_start_epoch
  nav2_stack_start_epoch="$(date +%s)"

  if [ "$START_NAV2" = "1" ]; then
    if [ ! -x "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh" ]; then
      log "WARN: run_nav2_saved_map.sh is not executable; trying chmod +x"
      chmod +x "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh" || true
    fi
    export NAV2_STOP_CONFLICTS="${NAV2_STOP_CONFLICTS:-1}"
    export NAV2_REUSE_EXISTING="${NAV2_REUSE_EXISTING:-0}"
    # Click-nav only: uniform motor trims (does not change mapping / other scripts).
    export CLICK_NAV_CHASSIS_MOTOR_TRIMS="${CLICK_NAV_CHASSIS_MOTOR_TRIMS:-1.0,1.0,1.0,1.0}"
    log "CLICK_NAV motor trims=${CLICK_NAV_CHASSIS_MOTOR_TRIMS} (override for this script only)"
    start_bg nav2_saved_map bash "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh"
    NAV2_SAVED_MAP_PID="${PIDS[-1]}"
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

  wait_topic_exists /map 180 || exit 1
  wait_topic_exists /odom 180 || exit 1
  wait_topic_exists /tf 120 || exit 1
  wait_nav2_saved_map_ready 300 "$nav2_stack_start_epoch" || exit 1
  wait_map_base_link_tf_ready 120 || exit 1
  if [ "$START_NAV2" = "0" ]; then
    verify_nav2_navigation_ready 90 || exit 1
  fi
  wait_action_exists /navigate_to_pose 90 || exit 1

  # /compute_path_to_pose is used only to pre-draw the path; navigation can still work if it is delayed.
  wait_action_exists /compute_path_to_pose 30 || log "WARN: /compute_path_to_pose not ready yet; bridge will keep trying when goals arrive."

  start_bg click_goal_bridge python3 "$PROJECT_DIR/scripts/slam/foxglove_click_goal_bridge.py" \
    --goal-point-topic "$GOAL_POINT_TOPIC" \
    --goal-pose-topic "$GOAL_POSE_TOPIC" \
    --map-frame map \
    --base-frame "$ROBOT_BASE_FRAME" \
    --fallback-base-frame base_footprint \
    --prefer-current-yaw \
    --default-goal-yaw 0.0 \
    --goal-frame map

  sleep 2
  print_click_ready_banner

  if [ "${AUTO_TEST_CLICK:-0}" = "1" ]; then
    sleep 2
    log "AUTO_TEST_CLICK: publish test point (0.5, 0.0) -> ${GOAL_POINT_TOPIC}"
    (
      source_ros
      bash "$PROJECT_DIR/scripts/slam/test_publish_foxglove_point.sh" 0.5 0.0
    ) || true
    sleep 5
    if source_ros && timeout 3 ros2 topic echo /foxglove_click_planned_path --once 2>/dev/null | grep -q "poses:"; then
      log "AUTO_TEST_CLICK: planned path received OK"
    elif grep -q "Planned path published" "${LOG_DIR}/click_goal_bridge.log" 2>/dev/null; then
      log "AUTO_TEST_CLICK: bridge reported planned path OK"
    else
      log "WARN: AUTO_TEST_CLICK: no /foxglove_click_planned_path yet (check ${LOG_DIR}/click_goal_bridge.log)"
    fi
  fi

  while true; do sleep 3600; done
}

main "$@"
