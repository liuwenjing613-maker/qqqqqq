#!/usr/bin/env bash
# 启动 saved-map Nav2（冷启动，对齐 run_nav2_foxglove_click_goal.sh）并发送 Qwen 导航目标。
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_DIR"

MAP_YAML="${1:-}"
GOAL_JSON="${2:-}"
POSE_STATE_FILE="${POSE_STATE_FILE:-$PROJECT_DIR/state/last_pose_map.json}"
LOG_DIR="${LOG_DIR:-$PROJECT_DIR/logs/qwen_session_nav2_$(date +%Y%m%d_%H%M%S)}"
NAV2_PID=""
STARTED_NAV2=0
NAV2_START_EPOCH=0
BOOT_LOG_TAIL_PID=""
NAV2_TOTAL_STEPS=4

# shellcheck source=scripts/lib/nav2_localization_bootstrap.sh
source "${PROJECT_DIR}/scripts/lib/nav2_localization_bootstrap.sh"
# shellcheck source=scripts/lib/nav2_stack_reuse.sh
source "${PROJECT_DIR}/scripts/lib/nav2_stack_reuse.sh"
# shellcheck source=scripts/lib/ros_dds_env.sh
source "${PROJECT_DIR}/scripts/lib/ros_dds_env.sh"

if [[ -z "$MAP_YAML" ]] || [[ -z "$GOAL_JSON" ]]; then
  echo "Usage: $0 <map.yaml> <navigation_goal_proposal.json>"
  echo "  env: POSE_STATE_FILE, LOG_DIR, NAV2_REUSE_EXISTING=0"
  exit 1
fi

MAP_YAML="$(readlink -f "$MAP_YAML")"
GOAL_JSON="$(readlink -f "$GOAL_JSON")"
POSE_STATE_FILE="$(readlink -f "$POSE_STATE_FILE")"
mkdir -p "$LOG_DIR"

source_ros() {
  set +u
  [[ -f /opt/tros/humble/setup.bash ]] && source /opt/tros/humble/setup.bash
  [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
  [[ -f "$HOME/ydlidar_ws/install/setup.bash" ]] && source "$HOME/ydlidar_ws/install/setup.bash"
  set -u
  prepare_ros_dds_env
}

log() { echo "[QWEN_NAV2] $*"; }

log_step() {
  local step="$1"
  shift
  echo ""
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  log "步骤 ${step}/${NAV2_TOTAL_STEPS}: $*"
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

wait_topic_exists_cli() {
  local topic="$1"
  local timeout_sec="${2:-60}"
  local start
  start="$(date +%s)"
  log "等待 topic ${topic} (最多 ${timeout_sec}s) ..."
  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      log "topic OK: ${topic}"
      return 0
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      log "ERROR: topic not found: ${topic}"
      return 1
    fi
    sleep 1
  done
}

wait_action_exists_cli() {
  local action_name="$1"
  local timeout_sec="${2:-90}"
  local start
  start="$(date +%s)"
  log "等待 action ${action_name} (最多 ${timeout_sec}s) ..."
  while true; do
    if ros2 action list 2>/dev/null | grep -qx "$action_name"; then
      log "action OK: ${action_name}"
      return 0
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      log "ERROR: action not found: ${action_name}"
      return 1
    fi
    sleep 1
  done
}

find_nav2_ready_file_since() {
  local min_epoch="${1:-0}"
  python3 - "$PROJECT_DIR" "$min_epoch" "$LOG_DIR" <<'PY'
import glob
import os
import sys

project_dir = sys.argv[1]
min_epoch = float(sys.argv[2])
log_dir = sys.argv[3]
candidates = []

def consider(path: str) -> None:
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return
    if mtime + 1.0 >= min_epoch:
        candidates.append((mtime, path))

if log_dir:
    consider(os.path.join(log_dir, "ready"))
for path in glob.glob(os.path.join(project_dir, "logs", "nav2_*", "ready")):
    consider(path)
if not candidates:
    raise SystemExit(0)
candidates.sort(reverse=True)
print(candidates[0][1])
PY
}

wait_nav2_saved_map_ready() {
  local ready_timeout="${1:-300}"
  local min_epoch="${2:-0}"
  local start last_heartbeat ready_file elapsed
  start="$(date +%s)"
  last_heartbeat="$start"
  log "等待 Nav2 ready 文件 (最多 ${ready_timeout}s，正常约 90–210s) ..."
  log "boot 日志: $LOG_DIR/nav2_saved_map.log"
  while true; do
    if [[ -n "${NAV2_PID:-}" ]] && ! kill -0 "$NAV2_PID" 2>/dev/null; then
      log "FAIL: run_nav2_saved_map 在 ready 之前退出"
      tail -40 "$LOG_DIR/nav2_saved_map.log" 2>/dev/null || true
      return 1
    fi
    ready_file="$(find_nav2_ready_file_since "$min_epoch")"
    if [[ -n "$ready_file" && -f "$ready_file" ]]; then
      log "Nav2 ready: $ready_file (耗时 $(( $(date +%s) - start ))s)"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( $(date +%s) - last_heartbeat >= 15 )); then
      local boot_hint
      boot_hint="$(tail -n 1 "$LOG_DIR/nav2_saved_map.log" 2>/dev/null | sed 's/^[[:space:]]*//')"
      if [[ -n "$boot_hint" ]]; then
        log "仍在等待 ready... ${elapsed}s | ${boot_hint:0:120}"
      else
        log "仍在等待 ready... ${elapsed}s"
      fi
      last_heartbeat="$(date +%s)"
    fi
    if (( elapsed >= ready_timeout )); then
      log "FAIL: ${ready_timeout}s 内未出现 ready 文件"
      tail -40 "$LOG_DIR/nav2_saved_map.log" 2>/dev/null || true
      return 1
    fi
    sleep 2
  done
}

wait_nav2_stack_like_click_nav() {
  local min_epoch="${1:-0}"
  log "按 Foxglove 点击导航顺序验证 Nav2 栈 ..."
  wait_topic_exists_cli /map 180 || return 1
  wait_topic_exists_cli /odom 180 || return 1
  wait_topic_exists_cli /tf 120 || return 1
  wait_nav2_saved_map_ready 300 "$min_epoch" || return 1
  log "rclpy 等待 map -> base_link TF (120s) ..."
  wait_map_base_link_tf_rclpy 120 "QWEN_NAV2" || return 1
  log "等待 Nav2 navigation actions ..."
  wait_nav_actions_ready 90 || return 1
  wait_action_exists_cli /navigate_to_pose 30 || return 1
  wait_action_exists_cli /compute_path_to_pose 30 \
    || log "WARN: /compute_path_to_pose 未就绪（发目标仍可进行）"
  wait_amcl_localization_settle 45 \
    || log "WARN: AMCL 尚未完全稳定；若导航失败请在 Foxglove 用 /initialpose 校正"
  return 0
}

start_boot_log_follower() {
  local logfile="$LOG_DIR/nav2_saved_map.log"
  touch "$logfile"
  (
    tail -F -n 0 "$logfile" 2>/dev/null | while IFS= read -r line; do
      if echo "$line" | grep -qE '\[NAV2\]|\[NAV2_BOOT\]|OK:|ERROR:|FAIL:|WARN:|ready|AMCL|TF|topic|launch|planner|navigator'; then
        echo "[NAV2_BOOT] $line"
      fi
    done
  ) &
  BOOT_LOG_TAIL_PID=$!
}

stop_boot_log_follower() {
  if [[ -n "${BOOT_LOG_TAIL_PID:-}" ]] && kill -0 "$BOOT_LOG_TAIL_PID" 2>/dev/null; then
    kill "$BOOT_LOG_TAIL_PID" 2>/dev/null || true
    wait "$BOOT_LOG_TAIL_PID" 2>/dev/null || true
  fi
  BOOT_LOG_TAIL_PID=""
}

print_nav_goal_from_json() {
  python3 - "$GOAL_JSON" <<'PY'
import json
import sys
from pathlib import Path

p = Path(sys.argv[1])
if not p.is_file():
    raise SystemExit(0)
data = json.loads(p.read_text(encoding="utf-8"))
goal = data.get("goal_pose_map") or {}
if goal:
    print(
        f"[QWEN_NAV2] 目标: x={float(goal['x']):.3f} y={float(goal['y']):.3f} "
        f"yaw_deg={float(goal.get('yaw_deg', 0)):.1f} status={data.get('selection_status')}"
    )
PY
}

cleanup() {
  stop_boot_log_follower
  if [[ "$STARTED_NAV2" -eq 1 ]] && [[ -n "$NAV2_PID" ]] && kill -0 "$NAV2_PID" 2>/dev/null; then
    log "Nav2 后台继续运行 (pid=$NAV2_PID)"
    log "停止: source scripts/lib/cleanup_lidar_slam_nav.sh && cleanup_click_nav_stack_processes STOP echo"
  fi
}
trap cleanup EXIT

source_ros
export MAP_YAML
export POSE_STATE_FILE
export LOG_DIR
export NAV2_STOP_CONFLICTS="${NAV2_STOP_CONFLICTS:-1}"
export NAV2_REUSE_EXISTING="${NAV2_REUSE_EXISTING:-0}"

log "===== Qwen 会话 Nav2 导航（方案A 冷启动）====="
log "MAP_YAML=$MAP_YAML"
log "GOAL_JSON=$GOAL_JSON"
log "POSE_STATE_FILE=$POSE_STATE_FILE"
log "LOG_DIR=$LOG_DIR"
log "NAV2_REUSE_EXISTING=$NAV2_REUSE_EXISTING"
print_nav_goal_from_json

log_step 1 "冷启动 run_nav2_saved_map.sh"
NAV2_START_EPOCH="$(date +%s)"
log "后台启动 run_nav2_saved_map.sh ..."
log "  步骤: 雷达 → 底盘 → map_server → AMCL → planner → bt_navigator"
start_boot_log_follower
bash "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh" >> "$LOG_DIR/nav2_saved_map.log" 2>&1 &
NAV2_PID=$!
STARTED_NAV2=1
log "run_nav2_saved_map pid=$NAV2_PID"

log_step 2 "等待 Nav2 栈就绪（与 Foxglove 点击导航相同顺序）"
if ! wait_nav2_stack_like_click_nav "$NAV2_START_EPOCH"; then
  stop_boot_log_follower
  exit 1
fi
stop_boot_log_follower
print_pose_state_summary "$POSE_STATE_FILE" || true

log_step 3 "发送 Qwen 目标到 /navigate_to_pose"
log "Nav2 栈与 map->base_link TF 已就绪，发送目标 ..."
python3 -u "$PROJECT_DIR/scripts/nav/send_navigation_goal_proposal.py" \
  --goal-json "$GOAL_JSON" \
  --pose-state-file "$POSE_STATE_FILE" \
  --wait-tf-s 60 \
  --timeout-s 180 \
  2>&1 | tee "$LOG_DIR/send_goal.log"
rc=${PIPESTATUS[0]}

log_step 4 "导航结果"
if [[ "$rc" -eq 0 ]]; then
  log "导航成功完成"
else
  log "导航未成功 (exit=$rc)，详见 $LOG_DIR/send_goal.log"
fi
exit "$rc"
