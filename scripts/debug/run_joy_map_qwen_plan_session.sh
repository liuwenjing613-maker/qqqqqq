#!/usr/bin/env bash
# 手柄建图 + 路径/位姿标注 + OK 保存 + Qwen 全局区域规划 — 仅编排已有脚本，不修改其它进程代码。
#
# 流程：
#   1) 调用 run_joy_mapping_calibrated.sh（或 --attach-only 附着已运行的建图栈）
#   2) 调用 start_frontier_region_debug.sh 实时标注已走路径（若尚未运行）
#   3) 终端输入 OK → 保存地图与标注 → 调用 export_session_map_annotations.py（大号朝向箭头）
#   4) 调用 qwen_live_session_planner.py（v6 候选 + Qwen 第二阶段）
#   5) Qwen 选点完成后自动 Nav2 导航（默认开启，可用 --no-auto-nav 关闭）
#
# 启动：默认不预清理、不重复 ros2 健康检查（建图脚本内部已验证）；冲突时加 --preflight-cleanup
#
# 安全：本脚本不直接发布 /cmd_vel；仅停止本脚本自己启动的子进程。
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_DIR"

ATTACH_ONLY=0
SKIP_QWEN=0
DRY_RUN_QWEN=0
KEEP_MAPPING=1
STOP_AFTER_SAVE=0
AUTO_NAV=1
PREFLIGHT_CLEANUP=0
MAP_NAME="${MAP_NAME:-joy_calibrated_corridor_map}"
TASK="${QWEN_TASK:-优先探索尚未覆盖、最可能扩展地图的区域。}"
QWEN_EXTRA_ARGS=()

# shellcheck source=scripts/lib/ros_dds_env.sh
source "${PROJECT_DIR}/scripts/lib/ros_dds_env.sh"
# shellcheck source=scripts/lib/ros_stack_health.sh
source "${PROJECT_DIR}/scripts/lib/ros_stack_health.sh"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --attach-only)
      ATTACH_ONLY=1
      shift
      ;;
    --skip-qwen)
      SKIP_QWEN=1
      shift
      ;;
    --dry-run)
      DRY_RUN_QWEN=1
      shift
      ;;
    --stop-after-save)
      STOP_AFTER_SAVE=1
      KEEP_MAPPING=0
      shift
      ;;
    --keep-mapping)
      KEEP_MAPPING=1
      shift
      ;;
    --auto-nav)
      AUTO_NAV=1
      shift
      ;;
    --no-auto-nav)
      AUTO_NAV=0
      shift
      ;;
    --preflight-cleanup)
      PREFLIGHT_CLEANUP=1
      shift
      ;;
    --skip-preflight-cleanup)
      PREFLIGHT_CLEANUP=0
      shift
      ;;
    --map-name)
      MAP_NAME="$2"
      shift 2
      ;;
    --task)
      TASK="$2"
      shift 2
      ;;
    --mock-response)
      QWEN_EXTRA_ARGS+=(--mock-response "$2")
      shift 2
      ;;
    -h|--help)
      sed -n '1,40p' "$0" | tail -n +2
      exit 0
      ;;
    *)
      echo "[FATAL] unknown arg: $1 (use --help)"
      exit 1
      ;;
  esac
done

SESSION_ID="JQS_$(date -u +%Y%m%dT%H%M%SZ)"
SESSION_DIR="$PROJECT_DIR/logs/joy_qwen_session/$SESSION_ID"
MAP_DIR="$PROJECT_DIR/maps"
STATE_DIR="$PROJECT_DIR/state"
POSE_STATE_FILE="$STATE_DIR/last_pose_map.json"
TRAJ_FILE="$PROJECT_DIR/runtime/qwen_region_debug/trajectory_session.json"
RUNTIME_DEBUG_DIR="$PROJECT_DIR/runtime/qwen_region_debug"

mkdir -p "$SESSION_DIR" "$MAP_DIR" "$STATE_DIR"

STARTED_JOY=0
STARTED_DEBUG=0
STARTED_FOXGLOVE_VIZ=0
CLEANUP_DONE=0
JOY_PID=""
JOY_PGID=""
NAV_GOAL_JSON="$PROJECT_DIR/runtime/qwen_session/navigation_goal_proposal.json"

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
  attach_ros_dds_env
  if [[ "$had_nounset" -eq 1 ]]; then
    set -u
  fi
}

log() {
  echo "[$(date +%H:%M:%S)] $*" | tee -a "$SESSION_DIR/session.log"
}

refresh_ros2_daemon() {
  source_ros_environment
  timeout 5 ros2 daemon stop >> "$SESSION_DIR/session.log" 2>&1 || true
  timeout 10 ros2 daemon start >> "$SESSION_DIR/session.log" 2>&1 || true
}

map_topic_has_publisher() {
  local pub_count
  pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
  [[ -n "${pub_count:-}" && "${pub_count:-0}" -gt 0 ]]
}

wait_topic_exists() {
  local topic="$1"
  local timeout_sec="${2:-90}"
  local i
  source_ros_environment
  for ((i = 0; i < timeout_sec; i++)); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_joy_mapping_boot() {
  local timeout_sec="${1:-150}"
  local logfile="$SESSION_DIR/joy_mapping.log"
  local i

  log "等待 run_joy_mapping_calibrated 内部就绪 (最多 ${timeout_sec}s) ..."
  for ((i = 0; i < timeout_sec; i++)); do
    if [[ -f "$logfile" ]] && grep -q "System is running" "$logfile" 2>/dev/null; then
      log "  建图栈就绪 (${i}s)"
      return 0
    fi
    if [[ -n "${JOY_PID:-}" ]] && ! kill -0 "$JOY_PID" 2>/dev/null; then
      log "FAIL: joy_mapping 进程已退出"
      tail -40 "$logfile" 2>/dev/null || true
      return 1
    fi
    if (( i > 0 && i % 30 == 0 )); then
      log "  ... 仍在等待建图栈 (${i}s)"
    fi
    sleep 1
  done

  log "WARN: ${timeout_sec}s 内未见「System is running」，继续（请检查 joy_mapping.log）"
  return 0
}

wait_map_for_save() {
  local timeout_sec="${1:-30}"
  local label="${2:-保存前确认 /map}"
  local i pub_count

  source_ros_environment
  log "${label} (最多 ${timeout_sec}s) ..."
  for ((i = 0; i < timeout_sec; i++)); do
    if ros2 topic list 2>/dev/null | grep -qx "/map"; then
      if map_topic_has_publisher; then
        pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
        log "  /map 就绪 (publishers=${pub_count}, 耗时 ${i}s)"
        return 0
      fi
    fi
    sleep 1
  done
  log "ERROR: ${timeout_sec}s 内 /map 不可用"
  return 1
}

verify_joy_cmd_vel_from_log() {
  local logfile="$SESSION_DIR/joy_mapping.log"
  if [[ ! -f "$logfile" ]]; then
    log "WARN: 无 joy_mapping.log，跳过 cmd_vel 链检查"
    return 0
  fi
  if ! grep -q "System is running" "$logfile" 2>/dev/null; then
    log "WARN: 建图栈未报告 System is running"
    return 0
  fi
  if grep -A20 "/cmd_vel info" "$logfile" 2>/dev/null | grep -q "Publisher count: 1"; then
    log "手柄链 OK: teleop 已发布 /cmd_vel，底盘桥已订阅（见 joy_mapping.log）"
    return 0
  fi
  log "WARN: joy_mapping.log 未确认 /cmd_vel 发布者；若手柄无效请重启会话"
  return 0
}

wait_map_topic_ready() {
  local timeout_sec="${1:-60}"
  local label="${2:-等待 /map 就绪}"
  local i pub_count

  source_ros_environment
  log "${label} (最多 ${timeout_sec}s) ..."
  refresh_ros2_daemon

  for ((i = 0; i < timeout_sec; i++)); do
    if ros2 topic list 2>/dev/null | grep -qx "/map"; then
      if map_topic_has_publisher; then
        pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
        log "  /map 就绪 (publishers=${pub_count}, 耗时 ${i}s)"
        return 0
      fi
      if (( i % 10 == 0 )); then
        log "  WARN: /map 在列表中但无发布者 (${i}s)"
      fi
    elif (( i > 0 && i % 15 == 0 )); then
      log "  ... 仍在等待 /map (${i}s)"
      if (( i % 30 == 0 )); then
        refresh_ros2_daemon
      fi
    fi
    sleep 1
  done

  log "ERROR: ${timeout_sec}s 内 /map 未就绪或无发布者"
  log "HINT: 确认 SLAM 仍在运行；勿用 stop_nav.sh 代替完整 cleanup"
  ros2 topic list 2>/dev/null | grep -E '^/(map|scan|scan_filtered|joy)$' || true
  return 1
}

preflight_cleanup_conflicting_stacks() {
  source_ros_environment
  log "[0/4] 预清理旧 Nav2 / 冲突建图栈（避免 /map 双发布与 ros2 CLI 不可见）..."
  cleanup_click_nav_stack_processes "JQS_PREFLIGHT" log
  pkill -9 -f "run_joy_mapping_calibrated.sh" 2>/dev/null || true
  pkill -9 -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
  pkill -9 -f "run_slam_calibrated.sh" 2>/dev/null || true
  refresh_ros2_daemon
  sleep 2
  log "[0/4] 预清理完成"
}

warn_if_nav2_still_running() {
  if pgrep -f "run_nav2_saved_map.sh|nav2_click_nav_bringup_launch.py" >/dev/null 2>&1; then
    log "FAIL: 检测到 Nav2 saved-map 栈仍在运行，会与 SLAM /map / Foxglove 冲突"
    log "      请先: source scripts/lib/cleanup_lidar_slam_nav.sh && cleanup_click_nav_stack_processes STOP echo"
    return 1
  fi
  return 0
}

save_map_to_session() {
  local map_out="$SESSION_DIR/map/${MAP_NAME}"
  local map_tmp="${map_out}.tmp_$(date +%Y%m%d_%H%M%S)"
  local save_start_epoch saver_rc pub_count file_epoch

  mkdir -p "$SESSION_DIR/map"
  save_start_epoch="$(date +%s)"

  if ! wait_map_for_save 30 "保存前确认 /map"; then
    log "ERROR: 无法保存地图（/map 不可用）"
    return 1
  fi

  if ! wait_tf_frames_python map base_link 15; then
    log "WARN: 保存前无 map->base_link TF，仍尝试保存（请确认已用手柄移动过）"
  fi

  pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
  log "保存地图到 ${map_out} ... (publishers=${pub_count})"
  set +e
  timeout 30 ros2 run nav2_map_server map_saver_cli \
    -t /map \
    -f "$map_tmp" \
    --ros-args \
    -p save_map_timeout:=20.0 \
    >> "$SESSION_DIR/map_saver.log" 2>&1
  saver_rc=$?
  set -e

  if [[ "$saver_rc" -ne 0 ]] || [[ ! -f "${map_tmp}.pgm" ]] || [[ ! -f "${map_tmp}.yaml" ]]; then
    log "ERROR: map_saver_cli 失败 (rc=$saver_rc)"
    tail -20 "$SESSION_DIR/map_saver.log" 2>/dev/null || true
    return 1
  fi

  file_epoch="$(stat -c %Y "${map_tmp}.pgm" 2>/dev/null || echo 0)"
  if [[ "$file_epoch" -lt "$save_start_epoch" ]]; then
    log "ERROR: 地图文件时间戳异常"
    return 1
  fi

  mv "${map_tmp}.pgm" "${map_out}.pgm"
  mv "${map_tmp}.yaml" "${map_out}.yaml"

  python3 - "${map_out}.yaml" "${MAP_NAME}.pgm" <<'PY'
import sys
from pathlib import Path
import yaml

path = Path(sys.argv[1])
image_name = sys.argv[2]
data = yaml.safe_load(path.read_text(encoding="utf-8"))
data["image"] = image_name
path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
PY

  cp -f "${map_out}.yaml" "$MAP_DIR/${MAP_NAME}.yaml"
  cp -f "${map_out}.pgm" "$MAP_DIR/${MAP_NAME}.pgm"

  python3 - "${map_out}.pgm" "$SESSION_DIR/map/${MAP_NAME}.png" <<'PY' || true
import sys
from pathlib import Path
pgm, png = Path(sys.argv[1]), Path(sys.argv[2])
try:
    from PIL import Image
    Image.open(pgm).save(png)
    print(f"[OK] preview {png}")
except Exception as exc:
    print(f"[WARN] PNG preview skipped: {exc}", file=sys.stderr)
PY

  log "地图已保存: ${map_out}.yaml / .pgm"
  return 0
}

reset_nav_goal_json_pending() {
  mkdir -p "$(dirname "$NAV_GOAL_JSON")"
  python3 - "$NAV_GOAL_JSON" "$SESSION_ID" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
session_id = sys.argv[2]
payload = {
    "schema_version": "qwen_live_session_nav_goal_v1",
    "session_id": session_id,
    "selection_status": "PENDING",
    "note": "Session started; Qwen goal will appear after OK save + planner.",
}
path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
PY
  log "已重置 Qwen 目标文件为 PENDING: $NAV_GOAL_JSON"
}

copy_live_debug_artifacts() {
  local run_dir live_png
  if [[ -f "$RUNTIME_DEBUG_DIR/frontier_region_debug.run_dir" ]]; then
    run_dir="$(cat "$RUNTIME_DEBUG_DIR/frontier_region_debug.run_dir")"
    live_png="$run_dir/latest_annotated_map.png"
    if [[ -f "$live_png" ]]; then
      cp -f "$live_png" "$SESSION_DIR/live_annotated_map.png"
      log "已复制实时标注图: $live_png"
    fi
    if [[ -f "$run_dir/console.log" ]]; then
      tail -80 "$run_dir/console.log" > "$SESSION_DIR/frontier_debug_tail.log" || true
    fi
  fi
  if [[ -f "$TRAJ_FILE" ]]; then
    cp -f "$TRAJ_FILE" "$SESSION_DIR/trajectory_session.json"
  fi
  if [[ -f "$POSE_STATE_FILE" ]]; then
    cp -f "$POSE_STATE_FILE" "$SESSION_DIR/last_pose_map.json"
  fi
}

print_nav_goal_summary() {
  local goal_json="$1"
  python3 - "$goal_json" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
if not path.is_file():
    print("[GOAL] none")
    raise SystemExit(0)
goal = json.loads(path.read_text(encoding="utf-8")).get("goal_pose_map")
if not goal:
    print("[GOAL] none")
else:
    print(
        f"[GOAL] x={float(goal['x']):.3f} "
        f"y={float(goal['y']):.3f} "
        f"yaw_deg={float(goal['yaw_deg']):.1f}"
    )
PY
}

goal_ready_for_nav() {
  local goal_json="$1"
  python3 - "$goal_json" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
if not path.is_file():
    raise SystemExit(1)
data = json.loads(path.read_text(encoding="utf-8"))
if data.get("selection_status") != "REGION_PROPOSED":
    raise SystemExit(1)
goal = data.get("goal_pose_map")
if not goal or "x" not in goal or "y" not in goal:
    raise SystemExit(1)
raise SystemExit(0)
PY
}

log_phase() {
  echo ""
  echo "════════════════════════════════════════════════════════════"
  log "$*"
  echo "════════════════════════════════════════════════════════════"
}

stop_slam_stack_for_nav() {
  log "  [1/4] 停止 SLAM/手柄栈（Nav2 切换；不再触发 joy 内置存图）..."
  source_ros_environment
  timeout 1.2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 5 \
    >/dev/null 2>&1 || true

  if [[ "$STARTED_JOY" -eq 1 ]] && [[ -n "$JOY_PID" ]]; then
    log "        → 结束 joy_mapping pid=$JOY_PID（SIGKILL，避免 INT 二次 save_map）"
    kill -9 "$JOY_PID" 2>/dev/null || true
    if [[ -n "$JOY_PGID" ]]; then
      kill -9 "-$JOY_PGID" 2>/dev/null || true
    fi
  fi

  log "  [2/4] 停止 frontier_region_debug（若由本会话启动）..."
  if [[ "$STARTED_DEBUG" -eq 1 ]]; then
    bash "$PROJECT_DIR/scripts/nav/stop_frontier_region_debug.sh" >> "$SESSION_DIR/session.log" 2>&1 || true
  fi

  log "  [3/4] 清理 SLAM / 手柄 / 底盘桥残留 ..."
  pkill -9 -f "run_joy_mapping_calibrated.sh" 2>/dev/null || true
  pkill -9 -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
  pkill -9 -f "run_slam_calibrated.sh" 2>/dev/null || true
  pkill -9 -f "joy_node|teleop_twist_joy" 2>/dev/null || true
  pkill -9 -f "async_slam_toolbox_node|sync_slam_toolbox_node" 2>/dev/null || true
  pkill -9 -f "m1_pwm_cmd_vel_bridge.py|simple_scan_filter.py" 2>/dev/null || true
  pkill -9 -f "ydlidar_ros2_driver_node|start_lidar_only.sh" 2>/dev/null || true
  pkill -9 -f "foxglove_bridge" 2>/dev/null || true
  sleep 2

  log "  [4/4] 刷新 DDS 环境，准备 Nav2 冷启动 ..."
  prepare_ros_dds_env
  sleep 2
}

run_qwen_nav2_phase() {
  if [[ ! -f "$NAV_GOAL_JSON" ]]; then
    log "FAIL: 缺少 navigation_goal_proposal.json"
    exit 1
  fi
  if ! goal_ready_for_nav "$NAV_GOAL_JSON"; then
    log "FAIL: Qwen 目标无效 (需要 selection_status=REGION_PROPOSED)"
    exit 1
  fi

  log_phase "[5/5] 自动 Nav2 导航到 Qwen 目标"
  print_nav_goal_summary "$NAV_GOAL_JSON"
  log "Nav2 冷启动通常需 90–210s；下方会逐步打印 [QWEN_NAV2] 进度"
  log "完整日志目录: $SESSION_DIR/nav2_logs/"

  stop_slam_stack_for_nav

  SESSION_POSE="$SESSION_DIR/last_pose_map.json"
  if [[ -f "$SESSION_POSE" ]]; then
    export POSE_STATE_FILE="$SESSION_POSE"
  else
    export POSE_STATE_FILE="$POSE_STATE_FILE"
  fi
  export LOG_DIR="$SESSION_DIR/nav2_logs"
  export NAV2_STOP_CONFLICTS=1
  export NAV2_REUSE_EXISTING=0
  mkdir -p "$LOG_DIR"

  log "启动 run_qwen_session_nav2_goal.sh ..."
  log "  map=$MAP_YAML"
  log "  pose=$POSE_STATE_FILE"
  if bash "$PROJECT_DIR/scripts/nav/run_qwen_session_nav2_goal.sh" \
    "$MAP_YAML" "$NAV_GOAL_JSON" \
    2>&1 | tee "$SESSION_DIR/nav2_run.log"; then
    log "✓ Nav2 导航成功完成"
  else
    log "✗ Nav2 导航失败，详见 $SESSION_DIR/nav2_run.log"
    tail -40 "$SESSION_DIR/nav2_run.log" 2>/dev/null || true
    exit 1
  fi
}

stop_started_processes() {
  local mode="${1:-light}"
  local i

  if [[ "$STARTED_FOXGLOVE_VIZ" -eq 1 ]]; then
    log "停止 qwen_session_foxglove_viz ..."
    timeout 5 bash "$PROJECT_DIR/scripts/debug/stop_qwen_session_foxglove_viz.sh" \
      >> "$SESSION_DIR/session.log" 2>&1 || true
    STARTED_FOXGLOVE_VIZ=0
  fi

  if [[ "$STARTED_DEBUG" -eq 1 ]] && {
    [[ "$STOP_AFTER_SAVE" -eq 1 ]] || [[ "$mode" == "full" ]]
  }; then
    log "停止 frontier_region_debug ..."
    timeout 10 bash "$PROJECT_DIR/scripts/nav/stop_frontier_region_debug.sh" \
      >> "$SESSION_DIR/session.log" 2>&1 || true
    STARTED_DEBUG=0
  fi

  if [[ "$STARTED_JOY" -eq 1 ]] && [[ -n "$JOY_PID" ]] && {
    [[ "$STOP_AFTER_SAVE" -eq 1 ]] || [[ "$mode" == "full" ]]
  }; then
    log "停止手柄建图栈 (pid=$JOY_PID) ..."
    kill -INT "$JOY_PID" 2>/dev/null || true
    if [[ -n "$JOY_PGID" ]]; then
      kill -INT "-$JOY_PGID" 2>/dev/null || true
    fi
    for i in $(seq 1 8); do
      if ! kill -0 "$JOY_PID" 2>/dev/null; then
        break
      fi
      sleep 0.5
    done
    if kill -0 "$JOY_PID" 2>/dev/null; then
      log "  → 强制结束 joy_mapping / SLAM 包装脚本"
      kill -9 "$JOY_PID" 2>/dev/null || true
      pkill -9 -f "run_joy_mapping_calibrated.sh" 2>/dev/null || true
      pkill -9 -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
      pkill -9 -f "run_slam_calibrated.sh" 2>/dev/null || true
    fi
    STARTED_JOY=0
    log "手柄建图栈已停止"
  fi
}

session_cleanup_and_exit() {
  local rc="${1:-0}"
  if [[ "$CLEANUP_DONE" -eq 1 ]]; then
    exit "$rc"
  fi
  CLEANUP_DONE=1
  trap - EXIT INT TERM HUP

  if [[ "${SESSION_SAVE_DONE:-0}" != "1" ]]; then
    log "会话未保存即退出 (exit=$rc)，正在清理 ..."
    stop_started_processes full
  fi
  exit "$rc"
}

cleanup_on_exit() {
  session_cleanup_and_exit "${1:-$?}"
}

load_qwen_env() {
  for envfile in "$PROJECT_DIR/.env" "$PROJECT_DIR/voice_interaction/.env"; do
    if [[ -f "$envfile" ]]; then
      set -a
      # shellcheck disable=SC1090
      source "$envfile"
      set +a
      break
    fi
  done
}

write_session_meta() {
  export JQS_META_SESSION_ID="$SESSION_ID"
  export JQS_META_SESSION_DIR="$SESSION_DIR"
  export JQS_META_MAP_NAME="$MAP_NAME"
  export JQS_META_ATTACH_ONLY="$ATTACH_ONLY"
  export JQS_META_STARTED_JOY="$STARTED_JOY"
  export JQS_META_STARTED_DEBUG="$STARTED_DEBUG"
  export JQS_META_SKIP_QWEN="$SKIP_QWEN"
  export JQS_META_KEEP_MAPPING="$KEEP_MAPPING"
  export JQS_META_AUTO_NAV="$AUTO_NAV"
  export JQS_META_TASK="$TASK"
  python3 - <<'PY'
import json
import os
from pathlib import Path

meta = {
    "session_id": os.environ["JQS_META_SESSION_ID"],
    "session_dir": os.environ["JQS_META_SESSION_DIR"],
    "map_name": os.environ["JQS_META_MAP_NAME"],
    "attach_only": os.environ.get("JQS_META_ATTACH_ONLY") == "1",
    "started_joy_mapping": os.environ.get("JQS_META_STARTED_JOY") == "1",
    "started_frontier_debug": os.environ.get("JQS_META_STARTED_DEBUG") == "1",
    "skip_qwen": os.environ.get("JQS_META_SKIP_QWEN") == "1",
    "dry_run_qwen": os.environ.get("JQS_META_DRY_RUN_QWEN") == "1",
    "keep_mapping": os.environ.get("JQS_META_KEEP_MAPPING") == "1",
    "stop_after_save": os.environ.get("JQS_META_STOP_AFTER_SAVE") == "1",
    "auto_nav": os.environ.get("JQS_META_AUTO_NAV") == "1",
    "task": os.environ.get("JQS_META_TASK", ""),
}
Path(os.environ["JQS_META_SESSION_DIR"] + "/session_meta.json").write_text(
    json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
)
PY
}

trap 'session_cleanup_and_exit 130' INT
trap 'session_cleanup_and_exit 143' TERM
trap 'session_cleanup_and_exit $?' EXIT

log "===== 手柄建图 + Qwen 规划会话 ====="
log "session_dir=$SESSION_DIR"
log "map_name=$MAP_NAME"
if [[ "$AUTO_NAV" -eq 1 ]]; then
  log "模式: Qwen 选点后自动 Nav2 导航（--no-auto-nav 可关闭）"
else
  log "模式: Qwen 选点后不自动导航（需手动或 --auto-nav）"
fi
write_session_meta

# ---------------------------------------------------------------------------
# Phase 0: 可选预清理（默认关闭，与 9 点跑通版一致；冲突时可 --preflight-cleanup）
# ---------------------------------------------------------------------------
if [[ "$PREFLIGHT_CLEANUP" -eq 1 ]]; then
  if [[ "$ATTACH_ONLY" -eq 1 ]] && ! warn_if_nav2_still_running; then
    exit 1
  fi
  if [[ "$ATTACH_ONLY" -eq 0 ]]; then
    preflight_cleanup_conflicting_stacks
  fi
fi

# ---------------------------------------------------------------------------
# Phase 1: 建图栈
# ---------------------------------------------------------------------------
if [[ "$ATTACH_ONLY" -eq 1 ]]; then
  log "[1/4] --attach-only：不启动新手柄建图，等待已有 /map ..."
  if ! wait_topic_exists /map 120; then
    log "FAIL: 120s 内 /map 未就绪。请先运行 scripts/slam/run_joy_mapping_calibrated.sh"
    exit 1
  fi
  log "  /map 已存在"
else
  log "[1/4] 启动 run_joy_mapping_calibrated.sh（后台）..."
  prepare_ros_dds_env
  setsid env FASTRTPS_DEFAULT_PROFILES="${FASTRTPS_DEFAULT_PROFILES:-}" \
    bash "$PROJECT_DIR/scripts/slam/run_joy_mapping_calibrated.sh" \
    >> "$SESSION_DIR/joy_mapping.log" 2>&1 &
  JOY_PID=$!
  JOY_PGID="$(ps -o pgid= -p "$JOY_PID" 2>/dev/null | tr -d ' ' || true)"
  STARTED_JOY=1
  log "joy_mapping pid=$JOY_PID pgid=${JOY_PGID:-unknown}"

  if ! wait_joy_mapping_boot 150; then
    exit 1
  fi
  wait_topic_exists /joy 30 || log "WARN: /joy 未就绪，请检查手柄设备"
fi

source_ros_environment

# ---------------------------------------------------------------------------
# Phase 2: 路径标注 debug 节点
# ---------------------------------------------------------------------------
if ros2 node list 2>/dev/null | grep -qx '/frontier_region_debug'; then
  log "[2/4] frontier_region_debug 已在运行，复用现有节点"
else
  log "[2/4] 启动 start_frontier_region_debug.sh ..."
  if bash "$PROJECT_DIR/scripts/nav/start_frontier_region_debug.sh" >> "$SESSION_DIR/frontier_debug_start.log" 2>&1; then
    STARTED_DEBUG=1
    log "frontier_region_debug 已启动"
  else
    log "WARN: frontier_region_debug 启动失败（见 $SESSION_DIR/frontier_debug_start.log）"
    log "      将继续会话，但实时路径标注可能不可用"
    tail -20 "$SESSION_DIR/frontier_debug_start.log" || true
  fi
fi

# ---------------------------------------------------------------------------
# Phase 2b: Foxglove 机器人/目标点可视化
# ---------------------------------------------------------------------------
mkdir -p "$PROJECT_DIR/runtime/qwen_session"
# 停掉上次会话可能残留的 viz，避免 Foxglove 继续显示旧 QWEN GOAL
bash "$PROJECT_DIR/scripts/debug/stop_qwen_session_foxglove_viz.sh" \
  >> "$SESSION_DIR/session.log" 2>&1 || true
reset_nav_goal_json_pending
log "[2b/4] 启动 qwen_session Foxglove 可视化节点 ..."
if bash "$PROJECT_DIR/scripts/debug/start_qwen_session_foxglove_viz.sh" "$NAV_GOAL_JSON" \
  >> "$SESSION_DIR/foxglove_viz_start.log" 2>&1; then
  STARTED_FOXGLOVE_VIZ=1
  log "Foxglove 可视化节点已启动（机器人箭头；Qwen 目标点需 OK 后才会出现）"
  log "  布局: configs/foxglove_slam_mapping.layout.json"
  log "  ws://<RDK-IP>:8765  3D 面板可见 /map /scan_filtered /qwen_session/*"
else
  log "WARN: Foxglove 可视化节点启动失败，见 $SESSION_DIR/foxglove_viz_start.log"
fi

verify_joy_cmd_vel_from_log

# ---------------------------------------------------------------------------
# Phase 3: 等待用户 OK
# ---------------------------------------------------------------------------
log "[3/4] 请用手柄驾驶建图。"
log "      Foxglove 3D 固定参考系=map；若画面空白请轻推摇杆 1–2m"
log "      实时标注（小箭头）见 logs/qwen_region_explore/<run_id>/latest_annotated_map.png"
log "      输入 OK 保存地图与标注并进入 Qwen；输入 status 查看状态；输入 quit 退出"

while true; do
  if ! read -r -p "> " user_line; then
    log "输入结束，退出会话"
    session_cleanup_and_exit 0
  fi
  case "${user_line^^}" in
    OK)
      break
      ;;
    STATUS|status)
      source_ros_environment
      echo "--- ROS topics (map/joy/cmd_vel) ---"
      ros2 topic list 2>/dev/null | egrep '^/(map|joy|cmd_vel|scan|scan_filtered|odom|tf)$' || true
      echo "--- TF ---"
      wait_tf_frames_python odom base_link 5 || echo "odom->base_link: MISSING"
      wait_tf_frames_python map base_link 5 || echo "map->base_link: MISSING"
      if [[ -f "$RUNTIME_DEBUG_DIR/frontier_region_debug.run_dir" ]]; then
        echo "debug_run_dir=$(cat "$RUNTIME_DEBUG_DIR/frontier_region_debug.run_dir")"
        ls -lh "$(cat "$RUNTIME_DEBUG_DIR/frontier_region_debug.run_dir")/latest_annotated_map.png" 2>/dev/null || true
      fi
      ls -lh "$POSE_STATE_FILE" "$TRAJ_FILE" 2>/dev/null || true
      ;;
    QUIT|quit|EXIT|exit)
      log "用户取消，不保存"
      session_cleanup_and_exit 0
      ;;
    *)
      echo "未知命令。请输入 OK / status / quit"
      ;;
  esac
done

# ---------------------------------------------------------------------------
# Phase 4: 保存 + 导出标注 + Qwen
# ---------------------------------------------------------------------------
log "[4/4] 收到 OK，开始保存 ..."

copy_live_debug_artifacts

if ! save_map_to_session; then
  log "FAIL: 地图保存失败"
  exit 1
fi

MAP_YAML="$SESSION_DIR/map/${MAP_NAME}.yaml"
ANNOTATED_PNG="$SESSION_DIR/annotated_map_for_qwen.png"
POSE_UV_JSON="$SESSION_DIR/robot_pose_uv.json"
QWEN_OUT="$SESSION_DIR/qwen_live"
LIVE_REF=""
if [[ -f "$SESSION_DIR/live_annotated_map.png" ]]; then
  LIVE_REF="$SESSION_DIR/live_annotated_map.png"
fi

if [[ ! -f "$POSE_STATE_FILE" ]]; then
  log "FAIL: 位姿文件不存在: $POSE_STATE_FILE"
  exit 1
fi

TRAJ_FOR_QWEN="$SESSION_DIR/trajectory_session.json"
if [[ ! -f "$TRAJ_FOR_QWEN" ]] && [[ -f "$TRAJ_FILE" ]]; then
  TRAJ_FOR_QWEN="$TRAJ_FILE"
fi

QWEN_MAP_YAML="$SESSION_DIR/map/${MAP_NAME}_qwen.yaml"
if [[ -f "$TRAJ_FOR_QWEN" ]]; then
  python3 "$PROJECT_DIR/scripts/debug/export_qwen_visited_map.py" \
    --map-yaml "$MAP_YAML" \
    --trajectory-json "$TRAJ_FOR_QWEN" \
    --output-dir "$SESSION_DIR/map" \
    | tee "$SESSION_DIR/export_qwen_map.stdout.json"
  log "Qwen 地图: $QWEN_MAP_YAML"
else
  log "WARN: 无 trajectory，跳过 Qwen 专用 PGM 导出"
fi

python3 "$PROJECT_DIR/scripts/debug/export_session_map_annotations.py" \
  --map-yaml "$MAP_YAML" \
  --pose-json "$POSE_STATE_FILE" \
  --trajectory-json "$TRAJ_FOR_QWEN" \
  --output-png "$ANNOTATED_PNG" \
  --output-pose-json "$POSE_UV_JSON" \
  ${LIVE_REF:+--copy-live-annotated "$LIVE_REF"} \
  | tee "$SESSION_DIR/robot_pose_uv.stdout.json"

ROBOT_U="$(python3 -c "import json;print(json.load(open('$POSE_UV_JSON'))['robot_image_pose']['u'])")"
ROBOT_V="$(python3 -c "import json;print(json.load(open('$POSE_UV_JSON'))['robot_image_pose']['v'])")"
ROBOT_YAW="$(python3 -c "import json;print(json.load(open('$POSE_UV_JSON'))['robot_image_pose']['yaw_deg'])")"

SESSION_SAVE_DONE=1
log "标注地图: $ANNOTATED_PNG"
log "机器人归一化位姿: u=$ROBOT_U v=$ROBOT_V yaw_deg=$ROBOT_YAW"

if [[ "$STOP_AFTER_SAVE" -eq 1 ]]; then
  stop_started_processes
  trap - EXIT INT TERM
else
  log "保持建图栈与 Foxglove 可视化运行（默认 --keep-mapping）"
  log "Foxglove: 实时 /map + /qwen_session/robot_pose_markers"
  trap - EXIT INT TERM
fi

if [[ "$SKIP_QWEN" -eq 1 ]]; then
  log "--skip-qwen：跳过 Qwen 调用"
  log "完成。输出目录: $SESSION_DIR"
  exit 0
fi

load_qwen_env
mkdir -p "$(dirname "$NAV_GOAL_JSON")" "$QWEN_OUT"
QWEN_CMD=(
  python3 -u "$PROJECT_DIR/scripts/debug/qwen_live_session_planner.py"
  --map-yaml "$MAP_YAML"
  --pose-json "$POSE_STATE_FILE"
  --trajectory-json "$TRAJ_FOR_QWEN"
  --output-dir "$QWEN_OUT"
  --nav-goal-json "$NAV_GOAL_JSON"
  --session-id "$SESSION_ID"
)
if [[ -f "$QWEN_MAP_YAML" ]]; then
  QWEN_CMD+=(--qwen-map-yaml "$QWEN_MAP_YAML")
fi
if [[ "$DRY_RUN_QWEN" -eq 1 ]]; then
  QWEN_CMD+=(--dry-run)
fi
if [[ ${#QWEN_EXTRA_ARGS[@]} -gt 0 ]]; then
  QWEN_CMD+=("${QWEN_EXTRA_ARGS[@]}")
fi

log "调用 qwen_live_session_planner.py（v6 程序候选 + Qwen 第二阶段）..."
if ! "${QWEN_CMD[@]}" 2>&1 | tee "$SESSION_DIR/qwen_run.log"; then
  log "FAIL: Qwen live session 规划失败"
  exit 1
fi

if [[ -f "$NAV_GOAL_JSON" ]]; then
  cp -f "$NAV_GOAL_JSON" "$SESSION_DIR/navigation_goal_proposal.json"
  log "Qwen 导航提案: $NAV_GOAL_JSON"
  print_nav_goal_summary "$NAV_GOAL_JSON"
fi

# ---------------------------------------------------------------------------
# Phase 5: Nav2 导航（Qwen 选点后默认自动执行）
# ---------------------------------------------------------------------------
if [[ "$AUTO_NAV" -eq 1 ]]; then
  run_qwen_nav2_phase
else
  log "[5/5] 已跳过自动导航 (--no-auto-nav)。手动执行:"
  log "  export POSE_STATE_FILE=$SESSION_DIR/last_pose_map.json"
  log "  bash scripts/nav/run_qwen_session_nav2_goal.sh $MAP_YAML $NAV_GOAL_JSON"
fi

QWEN_RUN_DIR="$QWEN_OUT"
log "===== 会话完成 ====="
log "session_dir=$SESSION_DIR"
log "saved_map=$MAP_YAML"
log "annotated_map=$ANNOTATED_PNG"
log "robot_pose_uv=$POSE_UV_JSON"
if [[ -n "$QWEN_RUN_DIR" ]]; then
  log "qwen_output=$QWEN_RUN_DIR"
  ls -lh "$QWEN_RUN_DIR" 2>/dev/null || true
fi

cat > "$SESSION_DIR/README.txt" <<EOF
会话 ID: $SESSION_ID

文件说明:
  map/${MAP_NAME}.yaml / .pgm     — SLAM 保存的地图
  annotated_map_for_qwen.png      — 已走路径 + 大号机器人朝向箭头
  robot_pose_uv.json              — Qwen 用归一化 u/v/yaw_deg
  live_annotated_map.png          — frontier debug 实时标注（小箭头，若可用）
  trajectory_session.json         — 轨迹顶点
  last_pose_map.json              — 最后位姿快照
  qwen_live/                      — v6 候选图、Qwen 结果、live_report.json
  navigation_goal_proposal.json   — map 坐标 Nav2/Foxglove 目标

Foxglove（ws://<RDK-IP>:8765）:
  导入布局 configs/foxglove_slam_mapping.layout.json
  /map, /scan_filtered, /qwen_explore_debug/trajectory_path
  /qwen_session/robot_pose_markers, /qwen_session/qwen_goal_markers

Nav2:
  默认 Qwen 选点后自动导航；跳过请加 --no-auto-nav
  bash scripts/nav/run_qwen_session_nav2_goal.sh $MAP_YAML $NAV_GOAL_JSON
  日志: $SESSION_DIR/nav2_run.log  $SESSION_DIR/nav2_logs/

重新跑 Qwen live planner:
  python3 scripts/debug/qwen_live_session_planner.py \\
    --map-yaml $MAP_YAML --pose-json $POSE_STATE_FILE \\
    --trajectory-json $SESSION_DIR/trajectory_session.json \\
    --output-dir $QWEN_OUT --nav-goal-json $NAV_GOAL_JSON \\
    --session-id $SESSION_ID
EOF

exit 0
