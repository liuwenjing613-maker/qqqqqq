#!/usr/bin/env bash
# 启动 saved-map Nav2（若未运行）并发送 Qwen 导航目标。
# 必须等 AMCL 定位完成（ready + map->base_link + actions）后再发目标。
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
NAV2_TOTAL_STEPS=5

# shellcheck source=scripts/lib/nav2_localization_bootstrap.sh
source "${PROJECT_DIR}/scripts/lib/nav2_localization_bootstrap.sh"
# shellcheck source=scripts/lib/nav2_stack_reuse.sh
source "${PROJECT_DIR}/scripts/lib/nav2_stack_reuse.sh"
# shellcheck source=scripts/lib/ros_dds_env.sh
source "${PROJECT_DIR}/scripts/lib/ros_dds_env.sh"

if [[ -z "$MAP_YAML" ]] || [[ -z "$GOAL_JSON" ]]; then
  echo "Usage: $0 <map.yaml> <navigation_goal_proposal.json>"
  echo "  env: POSE_STATE_FILE, LOG_DIR"
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

print_nav_snapshot() {
  local label="${1:-当前状态}"
  log "--- ${label} ---"
  if nav2_action_running; then
    log "  /navigate_to_pose : 已发现"
  else
    log "  /navigate_to_pose : 未就绪"
  fi
  if map_base_link_tf_ready 2>/dev/null; then
    log "  TF map->base_link : 可用"
  else
    log "  TF map->base_link : CLI 未可见 (boot 子进程可能已 OK，见 nav2_saved_map.log)"
  fi
  if ros2 topic list 2>/dev/null | grep -qx /map; then
    log "  /map topic        : 存在"
  else
    log "  /map topic        : 缺失"
  fi
  if [[ -n "${NAV2_PID:-}" ]] && kill -0 "$NAV2_PID" 2>/dev/null; then
    log "  run_nav2_saved_map: 运行中 (pid=$NAV2_PID)"
  elif pgrep -f "run_nav2_saved_map.sh" >/dev/null 2>&1; then
    log "  run_nav2_saved_map: 运行中 (外部进程)"
  else
    log "  run_nav2_saved_map: 未运行"
  fi
  local boot_log="$LOG_DIR/nav2_saved_map.log"
  if [[ -f "$boot_log" ]]; then
    local last
    last="$(tail -1 "$boot_log" 2>/dev/null | sed 's/^[[:space:]]*//')"
    if [[ -n "$last" ]]; then
      log "  boot 最新日志    : ${last:0:140}"
    fi
  fi
  log "---"
}

start_boot_log_follower() {
  local logfile="$LOG_DIR/nav2_saved_map.log"
  touch "$logfile"
  (
    tail -F -n 0 "$logfile" 2>/dev/null | while IFS= read -r line; do
      if echo "$line" | grep -qE '\[NAV2\]|OK:|ERROR:|FAIL:|WARN:|ready|AMCL|TF|topic|launch|planner|navigator'; then
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

nav2_action_running() {
  ros2 action list 2>/dev/null | grep -qx /navigate_to_pose
}

boot_log_indicates_nav2_bootstrapped() {
  local boot_log="${1:-$LOG_DIR/nav2_saved_map.log}"
  [[ -f "$boot_log" ]] || return 1
  grep -qE 'TF OK: map -> base_link|AMCL bootstrap OK:|Nav2 navigation stack active|READY file:' "$boot_log" 2>/dev/null
}

find_nav2_ready_file_since() {
  local min_epoch="${1:-0}"
  python3 - "$PROJECT_DIR" "$min_epoch" <<'PY'
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
}

wait_nav2_ready_file() {
  local timeout_sec="${1:-360}"
  local min_epoch="${2:-0}"
  local start now ready
  start="$(date +%s)"
  log "等待 Nav2 完整启动 (ready 文件 / boot 定位完成 / map->base_link TF，最多 ${timeout_sec}s，正常约 90–210s) ..."
  log "详细 boot 日志: $LOG_DIR/nav2_saved_map.log"
  while true; do
    ready="$(find_nav2_ready_file_since "$min_epoch")"
    if [[ -n "$ready" && -f "$ready" ]]; then
      log "Nav2 ready 文件已出现: $ready (耗时 $(( $(date +%s) - start ))s)"
      return 0
    fi
    if boot_log_indicates_nav2_bootstrapped "$LOG_DIR/nav2_saved_map.log"; then
      log "Nav2 boot 日志已报告 AMCL/栈就绪 (耗时 $(( $(date +%s) - start ))s)"
      return 0
    fi
    if map_base_link_tf_ready; then
      log "TF map->base_link 已可用 (耗时 $(( $(date +%s) - start ))s)"
      return 0
    fi
    if [[ -n "${NAV2_PID:-}" ]] && ! kill -0 "$NAV2_PID" 2>/dev/null; then
      log "FAIL: run_nav2_saved_map 进程已退出"
      tail -40 "$LOG_DIR/nav2_saved_map.log" 2>/dev/null || true
      return 1
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_sec )); then
      log "FAIL: ${timeout_sec}s 内 Nav2 未完成启动/定位"
      tail -40 "$LOG_DIR/nav2_saved_map.log" 2>/dev/null || true
      return 1
    fi
    if (( (now - start) % 15 == 0 && now > start )); then
      print_nav_snapshot "等待 Nav2 启动 ($((now - start))s)"
    fi
    sleep 2
  done
}

wait_for_background_localization() {
  local timeout_sec="${1:-180}"
  local boot_log="$LOG_DIR/nav2_saved_map.log"
  local start now
  start="$(date +%s)"
  log "等待 run_nav2_saved_map 完成 AMCL 定位 (最多 ${timeout_sec}s) ..."
  while true; do
    if map_base_link_tf_ready; then
      log "TF map->base_link 已可用 (耗时 $(( $(date +%s) - start ))s)"
      return 0
    fi
    if boot_log_indicates_nav2_bootstrapped "$boot_log"; then
      log "boot 日志已报告 AMCL/Nav2 就绪，确认 TF ..."
      if wait_map_base_link_tf 30; then
        return 0
      fi
    fi
    if [[ -n "${NAV2_PID:-}" ]] && ! kill -0 "$NAV2_PID" 2>/dev/null; then
      log "run_nav2_saved_map 进程已退出 (耗时 $(( $(date +%s) - start ))s)"
      if boot_log_indicates_nav2_bootstrapped "$boot_log" && wait_map_base_link_tf 15; then
        return 0
      fi
      return 1
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_sec )); then
      return 1
    fi
    if (( (now - start) % 15 == 0 && now > start )); then
      print_nav_snapshot "等待后台 AMCL 定位 ($((now - start))s)"
    fi
    sleep 2
  done
}

wait_map_base_link_tf_python() {
  local timeout_sec="${1:-120}"
  local label="${2:-等待 TF map->base_link}"
  log "${label} (最多 ${timeout_sec}s) ..."
  python3 - "$timeout_sec" <<'PY'
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

timeout = float(sys.argv[1])
rclpy.init()
node = Node("qwen_nav_wait_map_base")
buf = Buffer(cache_time=Duration(seconds=30.0))
TransformListener(buf, node, spin_thread=False)
start = time.time()
last_report = start
while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.1)
    now = time.time()
    if now - last_report >= 10.0:
        print(f"[QWEN_NAV2]   ... 仍在等待 TF ({int(now - start)}s)", flush=True)
        last_report = now
    try:
        tf = buf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.3))
        t = tf.transform.translation
        print(f"[QWEN_NAV2] TF map -> base_link OK  x={t.x:.3f} y={t.y:.3f}", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    except Exception:
        pass
node.destroy_node()
rclpy.shutdown()
print("[QWEN_NAV2] ERROR: map -> base_link TF not available")
raise SystemExit(1)
PY
}

ensure_map_localized() {
  if map_base_link_tf_ready; then
    log "TF map->base_link 已可用，跳过 bootstrap"
    wait_amcl_localization_settle 30 || log "WARN: AMCL 尚未完全稳定，继续尝试发目标"
    return 0
  fi

  local bg_wait="${QWEN_NAV2_BG_LOCALIZE_WAIT_SEC:-180}"
  if wait_for_background_localization "$bg_wait"; then
    log "后台 AMCL 定位完成"
    wait_amcl_localization_settle 60 || log "WARN: AMCL 尚未完全稳定，继续尝试发目标"
    return 0
  fi

  if [[ -n "${NAV2_PID:-}" ]] && kill -0 "$NAV2_PID" 2>/dev/null; then
    log "WARN: ${bg_wait}s 内后台仍未定位，但 run_nav2_saved_map 仍在运行"
    log "      继续等待 TF (最多 90s)，避免与后台 bootstrap 冲突 ..."
    if wait_map_base_link_tf 90; then
      wait_amcl_localization_settle 60 || log "WARN: AMCL 尚未完全稳定，继续尝试发目标"
      return 0
    fi
    log "WARN: 后台仍在运行但未定位；等待其结束后再补救 ..."
    local extra=0
    while kill -0 "$NAV2_PID" 2>/dev/null && (( extra < 120 )); do
      if map_base_link_tf_ready; then
        wait_amcl_localization_settle 60 || true
        return 0
      fi
      sleep 2
      extra=$((extra + 2))
    done
  fi

  log "后台定位未完成，使用 pose 文件补救 bootstrap AMCL ..."
  if [[ ! -f "$POSE_STATE_FILE" ]]; then
    log "FAIL: POSE_STATE_FILE 不存在: $POSE_STATE_FILE"
    return 1
  fi
  print_pose_state_summary "$POSE_STATE_FILE" || true
  if ! bootstrap_amcl_from_state_file "$POSE_STATE_FILE" 120; then
    log "FAIL: AMCL bootstrap 失败"
    return 1
  fi
  if ! wait_map_base_link_tf_python 60 "补救 bootstrap 后等待 TF"; then
    log "FAIL: bootstrap 后仍无 map -> base_link TF"
    return 1
  fi
  log "等待 AMCL 定位稳定 ..."
  wait_amcl_localization_settle 60 || log "WARN: AMCL 尚未完全稳定，继续尝试发目标"
  return 0
}

ensure_nav2_navigation_ready() {
  log "检查 Nav2 navigation actions ..."
  if ! wait_nav_actions_ready 60; then
    log "WARN: actions 未就绪，尝试 retry_navigation_bringup ..."
    retry_navigation_bringup 90 || return 1
  fi
  verify_nav2_navigation_ready 60 || return 1
  log "Nav2 navigation actions 已就绪"
  return 0
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
    log "停止 Nav2: source scripts/lib/cleanup_lidar_slam_nav.sh && cleanup_click_nav_stack_processes STOP echo"
  fi
}
trap cleanup EXIT

source_ros
export MAP_YAML
export POSE_STATE_FILE
export NAV2_STOP_CONFLICTS="${NAV2_STOP_CONFLICTS:-0}"
export NAV2_REUSE_EXISTING="${NAV2_REUSE_EXISTING:-1}"

log "===== Qwen 会话 Nav2 导航 ====="
log "MAP_YAML=$MAP_YAML"
log "GOAL_JSON=$GOAL_JSON"
log "POSE_STATE_FILE=$POSE_STATE_FILE"
log "LOG_DIR=$LOG_DIR"
print_nav_goal_from_json

log_step 1 "检查 / 启动 saved-map Nav2 栈"
if nav2_action_running; then
  log "检测到 /navigate_to_pose 已在运行，复用现有 Nav2 栈"
  print_nav_snapshot "复用 Nav2"
else
  NAV2_START_EPOCH="$(date +%s)"
  log "后台启动 run_nav2_saved_map.sh ..."
  log "  预计步骤: 雷达 → 底盘 → map_server → AMCL → planner → bt_navigator"
  start_boot_log_follower
  bash "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh" >> "$LOG_DIR/nav2_saved_map.log" 2>&1 &
  NAV2_PID=$!
  STARTED_NAV2=1
  log "run_nav2_saved_map pid=$NAV2_PID"

  if ! wait_nav2_ready_file 360 "$NAV2_START_EPOCH"; then
    stop_boot_log_follower
    exit 1
  fi
  stop_boot_log_follower
fi

log_step 2 "AMCL 定位 (map -> base_link TF)"
if ! ensure_map_localized; then
  exit 1
fi
print_nav_snapshot "定位完成"

log_step 3 "确认 Nav2 navigation actions 就绪"
if ! ensure_nav2_navigation_ready; then
  log "FAIL: Nav2 navigation 未就绪"
  exit 1
fi

log_step 4 "发送 Qwen 目标到 /navigate_to_pose"
log "Nav2 定位与 action 均已就绪，开始发送目标 ..."
python3 -u "$PROJECT_DIR/scripts/nav/send_navigation_goal_proposal.py" \
  --goal-json "$GOAL_JSON" \
  --pose-state-file "$POSE_STATE_FILE" \
  --wait-tf-s 30 \
  --timeout-s 180 \
  2>&1 | tee "$LOG_DIR/send_goal.log"
rc=${PIPESTATUS[0]}

log_step 5 "导航结果"
if [[ "$rc" -eq 0 ]]; then
  log "导航成功完成"
else
  log "导航未成功 (exit=$rc)，详见 $LOG_DIR/send_goal.log"
fi
exit "$rc"
