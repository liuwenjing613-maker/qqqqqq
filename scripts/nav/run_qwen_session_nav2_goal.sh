#!/usr/bin/env bash
# 启动 saved-map Nav2 并发送 Qwen 导航目标（快速/冷启动由入口脚本决定）。
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
NAV2_START_ONLY="${NAV2_START_ONLY:-0}"
NAV2_REQUIRE_AMCL_SETTLED="${NAV2_REQUIRE_AMCL_SETTLED:-1}"
NAV2_STAGE_TIMING_JSON="$LOG_DIR/nav2_stage_timing.json"

# shellcheck source=scripts/lib/nav2_localization_bootstrap.sh
source "${PROJECT_DIR}/scripts/lib/nav2_localization_bootstrap.sh"
# shellcheck source=scripts/lib/nav2_stack_reuse.sh
source "${PROJECT_DIR}/scripts/lib/nav2_stack_reuse.sh"
# shellcheck source=scripts/lib/ros_dds_env.sh
source "${PROJECT_DIR}/scripts/lib/ros_dds_env.sh"

if [[ -z "$MAP_YAML" ]] || [[ -z "$GOAL_JSON" ]]; then
  echo "Usage: $0 <map.yaml> <navigation_goal_proposal.json>"
  echo "  env: POSE_STATE_FILE, LOG_DIR, NAV2_REUSE_EXISTING, NAV2_STOP_CONFLICTS, NAV2_START_ONLY, NAV2_REQUIRE_AMCL_SETTLED"
  exit 1
fi

MAP_YAML="$(readlink -f "$MAP_YAML")"
GOAL_JSON="$(readlink -f "$GOAL_JSON")"
POSE_STATE_FILE="$(readlink -f "$POSE_STATE_FILE")"
mkdir -p "$LOG_DIR"
update_nav2_stage_timing_json "$NAV2_STAGE_TIMING_JSON" handoff_start "$(python3 -c 'import time; print(time.time())')"

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

wait_action_exists_cli() {
  local action_name="$1"
  local timeout_sec="${2:-30}"
  local start
  start="$(date +%s)"
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

validate_ready_json() {
  local ready_json="$1"
  python3 - "$ready_json" "$NAV2_REQUIRE_AMCL_SETTLED" "$NAV2_START_ONLY" <<'PY'
import json
import sys

path = sys.argv[1]
require_settled = sys.argv[2] == "1"
start_only = sys.argv[3] == "1"

with open(path, encoding="utf-8") as f:
    data = json.load(f)

schema = int(data.get("schema_version", 0))
status = str(data.get("status", ""))
ready_for_goal = bool(data.get("ready_for_goal", False))
amcl_settled = bool(data.get("amcl_settled", False))

if schema < 2:
    print(f"FAIL: ready.json schema_version={schema}, expected >= 2")
    raise SystemExit(1)

if start_only and status == "LOCALIZATION_UNSETTLED":
    print(
        "WARN: ready.json status=LOCALIZATION_UNSETTLED "
        "(NAV2_START_ONLY=1 allows continue; auto-nav would reject)"
    )
    raise SystemExit(0)

errors = []
if status != "READY_FOR_GOAL" and not ready_for_goal:
    errors.append(f"status={status!r}, ready_for_goal={ready_for_goal}")
if require_settled and not amcl_settled:
    errors.append("amcl_settled=false")
for key in ("compute_path_to_pose_ready", "navigate_to_pose_ready", "map_to_base_link_fresh"):
    if not bool(data.get(key, False)):
        errors.append(f"{key}=false")

if errors:
    for err in errors:
        print(f"FAIL: {err}")
    metrics = data.get("amcl_metrics") or {}
    print(
        "FAIL: amcl_metrics "
        f"samples={data.get('amcl_sample_count', 0)} "
        f"x_spread={metrics.get('x_spread_m', 0)} "
        f"y_spread={metrics.get('y_spread_m', 0)} "
        f"yaw_spread={metrics.get('yaw_spread_deg', 0)} "
        f"x_cov={metrics.get('x_cov', 0)} "
        f"y_cov={metrics.get('y_cov', 0)} "
        f"yaw_cov={metrics.get('yaw_cov', 0)}"
    )
    raise SystemExit(1)

print(f"ready.json validation OK (status={status}, amcl_settled={amcl_settled})")
raise SystemExit(0)
PY
}

record_goal_stage_timing_from_log() {
  local goal_log="$1"
  local timing_file="$2"
  local compute_path_start="${3:-}"
  python3 - "$goal_log" "$timing_file" "$compute_path_start" <<'PY'
import json
import os
import sys
import time
from pathlib import Path

goal_log = Path(sys.argv[1])
timing_file = Path(sys.argv[2])
compute_path_start = float(sys.argv[3]) if sys.argv[3] else None
text = goal_log.read_text(encoding="utf-8", errors="replace") if goal_log.is_file() else ""
now = time.time()

updates: dict[str, float] = {"navigation_finished": now}
if compute_path_start is not None:
    updates["compute_path_start"] = compute_path_start
if "ComputePathToPose OK" in text:
    updates["compute_path_end"] = now
if "发送 Nav2 目标" in text:
    updates["goal_sent"] = now
if "Nav2 已接受目标" in text:
    updates["goal_accepted"] = now

data: dict = {}
if timing_file.is_file():
    data = json.loads(timing_file.read_text(encoding="utf-8"))
data.update(updates)
timing_file.parent.mkdir(parents=True, exist_ok=True)
tmp = timing_file.with_suffix(timing_file.suffix + ".tmp")
tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
os.replace(tmp, timing_file)
PY
}

wait_ready_json() {
  local ready_timeout="${1:-300}"
  local min_epoch="${2:-0}"
  local expected_map="${3:-}"
  local start last_heartbeat elapsed
  start="$(date +%s)"
  last_heartbeat="$start"
  log "等待 ready.json (最多 ${ready_timeout}s) ..."
  while true; do
    if [[ -n "${NAV2_PID:-}" ]] && ! kill -0 "$NAV2_PID" 2>/dev/null; then
      log "FAIL: run_nav2_saved_map 在 ready 之前退出"
      tail -40 "$LOG_DIR/nav2_saved_map.log" 2>/dev/null || true
      return 1
    fi
    local ready_json="$LOG_DIR/ready.json"
    if [[ -f "$ready_json" ]]; then
      local mtime map_in_json
      mtime="$(stat -c %Y "$ready_json" 2>/dev/null || echo 0)"
      if (( mtime + 1 >= min_epoch )); then
        map_in_json="$(python3 - "$ready_json" <<'PY'
import json, sys
print(json.load(open(sys.argv[1], encoding="utf-8")).get("map_yaml", ""))
PY
)"
        if [[ -z "$expected_map" ]] || [[ "$(readlink -f "$map_in_json")" == "$(readlink -f "$expected_map")" ]]; then
          log "Nav2 ready.json OK (耗时 $(( $(date +%s) - start ))s)"
          return 0
        fi
        log "WARN: ready.json map_yaml 不匹配，继续等待 ..."
      fi
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( $(date +%s) - last_heartbeat >= 15 )); then
      log "仍在等待 ready.json... ${elapsed}s"
      last_heartbeat="$(date +%s)"
    fi
    if (( elapsed >= ready_timeout )); then
      log "FAIL: ${ready_timeout}s 内未出现有效 ready.json"
      return 1
    fi
    sleep 2
  done
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
import json, sys
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
  fi
}
trap cleanup EXIT

source_ros
export MAP_YAML
export POSE_STATE_FILE
export LOG_DIR
export NAV2_STOP_CONFLICTS="${NAV2_STOP_CONFLICTS:-0}"
export NAV2_REUSE_EXISTING="${NAV2_REUSE_EXISTING:-1}"
export NAV2_SKIP_DAEMON_REFRESH="${NAV2_SKIP_DAEMON_REFRESH:-1}"

export NAV2_SKIP_DAEMON_REFRESH="${NAV2_SKIP_DAEMON_REFRESH:-1}"
export NAV2_REQUIRE_AMCL_SETTLED="$NAV2_REQUIRE_AMCL_SETTLED"

log "===== Qwen 会话 Nav2 ======"
log "MAP_YAML=$MAP_YAML"
log "GOAL_JSON=$GOAL_JSON"
log "NAV2_REUSE_EXISTING=$NAV2_REUSE_EXISTING NAV2_STOP_CONFLICTS=$NAV2_STOP_CONFLICTS"
log "NAV2_REQUIRE_AMCL_SETTLED=$NAV2_REQUIRE_AMCL_SETTLED NAV2_START_ONLY=$NAV2_START_ONLY"
print_nav_goal_from_json

log_step 1 "启动 run_nav2_saved_map.sh"
NAV2_START_EPOCH="$(date +%s)"
start_boot_log_follower
bash "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh" >> "$LOG_DIR/nav2_saved_map.log" 2>&1 &
NAV2_PID=$!
STARTED_NAV2=1
log "run_nav2_saved_map pid=$NAV2_PID"

log_step 2 "等待并校验 ready.json（schema v2 + AMCL settle 硬门禁）"
if ! wait_ready_json 300 "$NAV2_START_EPOCH" "$MAP_YAML"; then
  stop_boot_log_follower
  exit 1
fi
if ! validate_ready_json "$LOG_DIR/ready.json"; then
  stop_boot_log_follower
  exit 1
fi
stop_boot_log_follower
wait_action_exists_cli /navigate_to_pose 20 || exit 1
wait_action_exists_cli /compute_path_to_pose 20 || exit 1

if [[ "$NAV2_START_ONLY" == "1" ]]; then
  log_step 3 "NAV2_START_ONLY：不发送导航目标"
  log "Nav2 已启动；stage timing: $NAV2_STAGE_TIMING_JSON"
  exit 0
fi

log_step 3 "发送 Qwen 目标（ComputePathToPose 硬门禁）"
COMPUTE_PATH_START="$(python3 -c 'import time; print(time.time())')"
update_nav2_stage_timing_json "$NAV2_STAGE_TIMING_JSON" compute_path_start "$COMPUTE_PATH_START"
python3 -u "$PROJECT_DIR/scripts/nav/send_navigation_goal_proposal.py" \
  --goal-json "$GOAL_JSON" \
  --pose-state-file "$POSE_STATE_FILE" \
  --wait-tf-s 60 \
  --timeout-s 180 \
  2>&1 | tee "$LOG_DIR/send_goal.log"
rc=${PIPESTATUS[0]}
record_goal_stage_timing_from_log "$LOG_DIR/send_goal.log" "$NAV2_STAGE_TIMING_JSON" "$COMPUTE_PATH_START"

log_step 4 "导航结果"
if [[ "$rc" -eq 0 ]]; then
  log "导航成功完成"
else
  log "导航未成功 (exit=$rc)，详见 $LOG_DIR/send_goal.log"
fi
exit "$rc"
