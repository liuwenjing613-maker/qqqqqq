#!/usr/bin/env bash
# 手柄建图 + 路径/位姿标注 + OK 保存 + Qwen 全局区域规划 — 编排层脚本。
#
# 流程：
#   1) run_joy_mapping_calibrated.sh（或 --attach-only）
#   2) start_frontier_region_debug.sh 实时标注
#   3) OK → 保存地图 → export 标注 → Qwen 选点
#   4) 快速 SLAM→Nav2 交接（默认 --fast-nav）或冷启动回退（--cold-nav）
#
# 安全：本脚本在 Nav2 交接时会 pkill 建图/teleop/slam_toolbox，快速模式保留雷达/底盘；
#       冷启动模式会调用 cleanup_click_nav_stack_processes 全量清理。预清理见 --preflight-cleanup。
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
FAST_NAV=1
ALLOW_NAV_FALLBACK=0
NAV2_START_ONLY=0
MAP_NAME="${MAP_NAME:-joy_calibrated_corridor_map}"
TASK="${QWEN_TASK:-优先探索尚未覆盖、最可能扩展地图的区域。}"
QWEN_EXTRA_ARGS=()

# 阶段耗时（秒，浮点）
TIMING_STOP_ROBOT_S=0
TIMING_COPY_ARTIFACTS_S=0
TIMING_SAVE_MAP_S=0
TIMING_EXPORT_QWEN_MAP_S=0
TIMING_EXPORT_ANNOTATION_S=0
TIMING_CANDIDATE_GENERATION_S=0
TIMING_QWEN_API_S=0
TIMING_MAPPING_TO_NAV_HANDOFF_S=0
TIMING_NAV2_BOOT_S=0
TIMING_COMPUTE_PATH_S=0
TIMING_NAVIGATION_S=0
TIMING_TOTAL_AFTER_OK_S=0
OK_EPOCH=0

# shellcheck source=scripts/lib/ros_dds_env.sh
source "${PROJECT_DIR}/scripts/lib/ros_dds_env.sh"
# shellcheck source=scripts/lib/cleanup_lidar_slam_nav.sh
source "${PROJECT_DIR}/scripts/lib/cleanup_lidar_slam_nav.sh"
# shellcheck source=scripts/lib/nav2_stack_reuse.sh
source "${PROJECT_DIR}/scripts/lib/nav2_stack_reuse.sh"
# shellcheck source=scripts/lib/ros_stack_health.sh
source "${PROJECT_DIR}/scripts/lib/ros_stack_health.sh"

NAV2_REUSE_SCAN=0

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
      AUTO_NAV=0
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
    --fast-nav)
      FAST_NAV=1
      shift
      ;;
    --cold-nav)
      FAST_NAV=0
      shift
      ;;
    --allow-nav-with-fallback)
      ALLOW_NAV_FALLBACK=1
      shift
      ;;
    --nav2-start-only)
      NAV2_START_ONLY=1
      AUTO_NAV=1
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

if [[ "$DRY_RUN_QWEN" -eq 1 && "$AUTO_NAV" -eq 1 ]]; then
  echo "[FATAL] --dry-run 禁止真实运动，不能与 --auto-nav 同时使用"
  exit 1
fi

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

timing_log() {
  log "[TIMING] $*"
}

write_pipeline_timing_json() {
  local mode="${1:-fast_nav}"
  python3 - "$SESSION_DIR/pipeline_timing.json" "$mode" <<'PY'
import json, os, sys
from pathlib import Path
path, mode = sys.argv[1], sys.argv[2]
payload = {
    "mode": mode,
    "stop_robot_s": float(os.environ.get("JQS_TIMING_STOP_ROBOT_S", 0)),
    "copy_artifacts_s": float(os.environ.get("JQS_TIMING_COPY_ARTIFACTS_S", 0)),
    "save_map_s": float(os.environ.get("JQS_TIMING_SAVE_MAP_S", 0)),
    "export_qwen_map_s": float(os.environ.get("JQS_TIMING_EXPORT_QWEN_MAP_S", 0)),
    "export_annotation_s": float(os.environ.get("JQS_TIMING_EXPORT_ANNOTATION_S", 0)),
    "candidate_generation_s": float(os.environ.get("JQS_TIMING_CANDIDATE_GENERATION_S", 0)),
    "qwen_api_s": float(os.environ.get("JQS_TIMING_QWEN_API_S", 0)),
    "mapping_to_nav_handoff_s": float(os.environ.get("JQS_TIMING_MAPPING_TO_NAV_HANDOFF_S", 0)),
    "nav2_boot_s": float(os.environ.get("JQS_TIMING_NAV2_BOOT_S", 0)),
    "compute_path_s": float(os.environ.get("JQS_TIMING_COMPUTE_PATH_S", 0)),
    "navigation_s": float(os.environ.get("JQS_TIMING_NAVIGATION_S", 0)),
    "total_after_ok_s": float(os.environ.get("JQS_TIMING_TOTAL_AFTER_OK_S", 0)),
}
Path(path).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
PY
}

export_timing_env() {
  export JQS_TIMING_STOP_ROBOT_S="$TIMING_STOP_ROBOT_S"
  export JQS_TIMING_COPY_ARTIFACTS_S="$TIMING_COPY_ARTIFACTS_S"
  export JQS_TIMING_SAVE_MAP_S="$TIMING_SAVE_MAP_S"
  export JQS_TIMING_EXPORT_QWEN_MAP_S="$TIMING_EXPORT_QWEN_MAP_S"
  export JQS_TIMING_EXPORT_ANNOTATION_S="$TIMING_EXPORT_ANNOTATION_S"
  export JQS_TIMING_CANDIDATE_GENERATION_S="$TIMING_CANDIDATE_GENERATION_S"
  export JQS_TIMING_QWEN_API_S="$TIMING_QWEN_API_S"
  export JQS_TIMING_MAPPING_TO_NAV_HANDOFF_S="$TIMING_MAPPING_TO_NAV_HANDOFF_S"
  export JQS_TIMING_NAV2_BOOT_S="$TIMING_NAV2_BOOT_S"
  export JQS_TIMING_COMPUTE_PATH_S="$TIMING_COMPUTE_PATH_S"
  export JQS_TIMING_NAVIGATION_S="$TIMING_NAVIGATION_S"
  export JQS_TIMING_TOTAL_AFTER_OK_S="$TIMING_TOTAL_AFTER_OK_S"
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

# Prefer real OccupancyGrid samples over ros2 CLI discovery (FastDDS/daemon lag).
map_topic_has_data() {
  local timeout_sec="${1:-12}"
  topic_is_publishing /map 1 "$timeout_sec"
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
    # /map: CLI list often lags; accept live samples as existence proof.
    if [[ "$topic" == "/map" ]] && map_topic_has_data 3; then
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
  local timeout_sec="${1:-45}"
  local label="${2:-保存前确认 /map}"
  local i pub_count
  local refreshed=0

  source_ros_environment
  log "${label} (最多 ${timeout_sec}s，优先 rclpy 收 OccupancyGrid) ..."

  # Fast path: actually receive /map (TRANSIENT_LOCAL). This survives CLI discovery lag.
  if map_topic_has_data 15; then
    pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
    log "  /map 数据就绪 (rclpy samples OK, cli_publishers=${pub_count:-unknown})"
    return 0
  fi

  for ((i = 0; i < timeout_sec; i++)); do
    if map_topic_has_data 4; then
      pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
      log "  /map 就绪 (rclpy samples OK, cli_publishers=${pub_count:-unknown}, 耗时 ${i}s)"
      return 0
    fi
    # Mid-wait: one daemon refresh can heal stale ros2 CLI graph (used later by map_saver_cli).
    if [[ "$refreshed" -eq 0 && "$i" -ge 8 ]]; then
      log "  WARN: /map 样本未到，刷新 ros2 daemon 后重试 ..."
      refresh_ros2_daemon
      refreshed=1
    fi
    # CLI fallback (weaker): topic list + publisher count
    if ros2 topic list 2>/dev/null | grep -qx "/map" && map_topic_has_publisher; then
      pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
      log "  /map 就绪 (CLI publishers=${pub_count}, 耗时 ${i}s；样本探测未确认)"
      return 0
    fi
    sleep 1
  done
  log "ERROR: ${timeout_sec}s 内 /map 不可用（rclpy 未收到 OccupancyGrid，且 CLI 无发布者）"
  log "HINT: 检查 slam_toolbox 是否仍在；可手动: python3 scripts/lib/ros_topic_probe.py has-samples /map 1 10"
  pgrep -af "slam_toolbox|async_slam_toolbox" 2>/dev/null | head -5 || true
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

start_frontier_region_debug_session() {
  if ros2 node list 2>/dev/null | grep -qx '/frontier_region_debug'; then
    log "[2/4] frontier_region_debug 已在运行，复用现有节点"
    return 0
  fi

  local attempt
  for attempt in 1 2; do
    if [[ "$attempt" -gt 1 ]]; then
      log "[2/4] frontier_region_debug 重试 ($attempt/2)：等待 /map 后再启动 ..."
      wait_map_topic_ready 45 "frontier debug 重试前 /map" || true
      sleep 2
    else
      log "[2/4] 启动 start_frontier_region_debug.sh ..."
    fi
    if bash "$PROJECT_DIR/scripts/nav/start_frontier_region_debug.sh" \
      >> "$SESSION_DIR/frontier_debug_start.log" 2>&1; then
      STARTED_DEBUG=1
      log "frontier_region_debug 已启动"
      return 0
    fi
    tail -20 "$SESSION_DIR/frontier_debug_start.log" || true
  done

  log "WARN: frontier_region_debug 启动失败（见 $SESSION_DIR/frontier_debug_start.log）"
  log "      将继续会话，但实时路径标注可能不可用"
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

  if ! wait_map_for_save 45 "保存前确认 /map"; then
    log "ERROR: 无法保存地图（/map 不可用）"
    return 1
  fi

  if ! wait_tf_frames_python map base_link 15; then
    log "WARN: 保存前无 map->base_link TF，仍尝试保存（请确认已用手柄移动过）"
  fi

  # map_saver_cli is a separate ROS process; refresh daemon so it can discover /map.
  if ! map_topic_has_publisher; then
    log "  CLI 仍看不到 /map 发布者，刷新 ros2 daemon 后再调 map_saver_cli ..."
    refresh_ros2_daemon
    sleep 2
  fi

  pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
  log "保存地图到 ${map_out} ... (cli_publishers=${pub_count:-unknown})"
  set +e
  timeout 45 ros2 run nav2_map_server map_saver_cli \
    -t /map \
    -f "$map_tmp" \
    --ros-args \
    -p save_map_timeout:=30.0 \
    >> "$SESSION_DIR/map_saver.log" 2>&1
  saver_rc=$?
  set -e

  if [[ "$saver_rc" -ne 0 ]] || [[ ! -f "${map_tmp}.pgm" ]] || [[ ! -f "${map_tmp}.yaml" ]]; then
    log "WARN: map_saver_cli 首次失败 (rc=$saver_rc)，刷新 daemon 后重试一次 ..."
    tail -20 "$SESSION_DIR/map_saver.log" 2>/dev/null || true
    refresh_ros2_daemon
    sleep 2
    set +e
    timeout 45 ros2 run nav2_map_server map_saver_cli \
      -t /map \
      -f "$map_tmp" \
      --ros-args \
      -p save_map_timeout:=30.0 \
      >> "$SESSION_DIR/map_saver.log" 2>&1
    saver_rc=$?
    set -e
  fi

  if [[ "$saver_rc" -ne 0 ]] || [[ ! -f "${map_tmp}.pgm" ]] || [[ ! -f "${map_tmp}.yaml" ]]; then
    log "ERROR: map_saver_cli 失败 (rc=$saver_rc)"
    tail -30 "$SESSION_DIR/map_saver.log" 2>/dev/null || true
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
  rm -f "$PROJECT_DIR/runtime/qwen_session/live_candidates_foxglove.json"
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

goal_ready_for_nav_validation() {
  local goal_json="$1"
  local map_yaml="$2"
  local allow_fallback="${3:-0}"
  local bundle_json="${4:-}"
  python3 - "$goal_json" "$map_yaml" "$allow_fallback" "$PROJECT_DIR/scripts/debug" "$bundle_json" <<'PY'
import json
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[4])
from qwen_map_goal_utils import validate_goal_for_nav_validation, validate_proposal_against_bundle

goal_path = Path(sys.argv[1])
ok, msg = validate_goal_for_nav_validation(
    goal_path,
    expected_map_yaml=Path(sys.argv[2]),
    allow_fallback=sys.argv[3] == "1",
)
if not ok:
    print(msg, file=sys.stderr)
    raise SystemExit(1)

bundle_path = Path(sys.argv[5]) if sys.argv[5] else None
if bundle_path and bundle_path.is_file():
    proposal = json.loads(goal_path.read_text(encoding="utf-8"))
    bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
    ok2, msg2 = validate_proposal_against_bundle(
        proposal,
        bundle,
        expected_session_id=str(proposal.get("session_id", "")),
        allow_fallback=sys.argv[3] == "1",
    )
    if not ok2:
        print(msg2, file=sys.stderr)
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

refresh_session_pose_before_nav() {
  local out="$SESSION_DIR/last_pose_map.json"
  source_ros_environment
  log "  刷新位姿快照 → $out (map->base_link TF) ..."
  if ! wait_tf_frames_python map base_link 8; then
    log "        WARN: 切换前无 map->base_link TF，沿用 OK 保存时的位姿"
    return 1
  fi
  if python3 - "$out" <<'PY'
import json
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

out = sys.argv[1]
rclpy.init()
node = Node("qwen_session_pose_snap")
buf = Buffer(cache_time=Duration(seconds=10.0))
TransformListener(buf, node, spin_thread=False)
for _ in range(80):
    rclpy.spin_once(node, timeout_sec=0.05)
    try:
        tf = buf.lookup_transform(
            "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.3)
        )
        t = tf.transform.translation
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        payload = {
            "frame_id": "map",
            "child_frame_id": "base_link",
            "x": float(t.x),
            "y": float(t.y),
            "z": float(t.z),
            "qx": float(q.x),
            "qy": float(q.y),
            "qz": float(q.z),
            "qw": float(q.w),
            "yaw": float(yaw),
            "stamp": time.time(),
            "source": "tf_pre_nav",
        }
        with open(out, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
            f.write("\n")
        print(f"pose snap OK x={t.x:.3f} y={t.y:.3f} yaw_deg={math.degrees(yaw):.1f}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    except Exception:
        pass
node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
PY
  then
    return 0
  fi
  log "        WARN: rclpy 位姿快照失败，沿用 OK 保存时的位姿"
  return 1
}

stop_mapping_control_for_fast_nav() {
  local t0 i
  local handoff_file="$PROJECT_DIR/runtime/request_nav_handoff"
  t0="$(date +%s)"
  log "  [快速交接] 停止 teleop + slam_toolbox，保留雷达/底盘/scan_filter ..."
  source_ros_environment
  timeout 1.2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 5 \
    >/dev/null 2>&1 || true

  if [[ "$STARTED_DEBUG" -eq 1 ]]; then
    bash "$PROJECT_DIR/scripts/nav/stop_frontier_region_debug.sh" >> "$SESSION_DIR/session.log" 2>&1 || true
  fi

  # Prefer precise PID / controlled handoff — NEVER pkill -9 the corridor wrapper
  # (that would either run cleanup killing sensors, or leave orphans).
  mkdir -p "$PROJECT_DIR/runtime"
  : > "$handoff_file"
  if [[ -n "${JOY_PID:-}" ]] && kill -0 "$JOY_PID" 2>/dev/null; then
    kill -USR1 "$JOY_PID" 2>/dev/null || true
  fi
  local corridor_pids
  corridor_pids="$(pgrep -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true)"
  for pid in $corridor_pids; do
    kill -USR1 "$pid" 2>/dev/null || true
  done

  local handoff_json="$PROJECT_DIR/runtime/sensor_base_stack.json"
  local handoff_wait
  for handoff_wait in $(seq 1 30); do
    if [[ -f "$handoff_json" ]]; then
      log "        受控交接完成: $handoff_json"
      break
    fi
    if [[ -z "$corridor_pids" ]] || ! pgrep -f "run_corridor_mapping_live_foxglove.sh" >/dev/null 2>&1; then
      break
    fi
    sleep 0.5
  done

  pkill -TERM -f "joy_node|teleop_twist_joy" 2>/dev/null || true
  pkill -TERM -f "async_slam_toolbox_node|sync_slam_toolbox_node" 2>/dev/null || true

  for i in $(seq 1 20); do
    if ! slam_toolbox_running && ! map_topic_has_publisher; then
      break
    fi
    sleep 0.5
  done
  if slam_toolbox_running; then
    log "        WARN: slam_toolbox 仍在运行，尝试 SIGKILL（仅 slam 节点）"
    pkill -9 -f "async_slam_toolbox_node|sync_slam_toolbox_node" 2>/dev/null || true
    sleep 1
  fi

  # Hard checks: both must be clear
  if map_topic_has_publisher; then
    log "[FAST_NAV] FAIL: SLAM 仍在发布 /map"
    TIMING_STOP_ROBOT_S=$(( $(date +%s) - t0 ))
    timing_log "stop_robot=${TIMING_STOP_ROBOT_S}s"
    return 1
  fi
  if slam_toolbox_running; then
    log "[FAST_NAV] FAIL: slam_toolbox 尚未退出"
    TIMING_STOP_ROBOT_S=$(( $(date +%s) - t0 ))
    timing_log "stop_robot=${TIMING_STOP_ROBOT_S}s"
    return 1
  fi

  # 快速交接不得 TERM joy_mapping 父进程：其 cleanup/stop_live_stack 会杀掉雷达/底盘/static_tf。
  log "        保留传感器进程；不停止 joy_mapping 父进程 (pid=${JOY_PID:-none})"

  TIMING_STOP_ROBOT_S=$(( $(date +%s) - t0 ))
  timing_log "stop_robot=${TIMING_STOP_ROBOT_S}s"
  return 0
}

stop_all_stacks_for_cold_nav() {
  local t0
  t0="$(date +%s)"
  log "  [冷启动交接] 全量停止并清理 ..."
  source_ros_environment
  timeout 1.2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 5 \
    >/dev/null 2>&1 || true

  if [[ "$STARTED_DEBUG" -eq 1 ]]; then
    bash "$PROJECT_DIR/scripts/nav/stop_frontier_region_debug.sh" >> "$SESSION_DIR/session.log" 2>&1 || true
  fi
  if [[ "$STARTED_JOY" -eq 1 ]] && [[ -n "$JOY_PID" ]]; then
    kill -TERM "$JOY_PID" 2>/dev/null || true
    [[ -n "$JOY_PGID" ]] && kill -TERM "-$JOY_PGID" 2>/dev/null || true
    sleep 2
    kill -9 "$JOY_PID" 2>/dev/null || true
    [[ -n "$JOY_PGID" ]] && kill -9 "-$JOY_PGID" 2>/dev/null || true
  fi
  pkill -9 -f "run_joy_mapping_calibrated.sh|run_corridor_mapping_live_foxglove.sh|run_slam_calibrated.sh" 2>/dev/null || true
  pkill -9 -f "joy_node|teleop_twist_joy|async_slam_toolbox_node|sync_slam_toolbox_node" 2>/dev/null || true
  sleep 2
  cleanup_click_nav_stack_processes "QWEN_NAV2" log
  timeout 5 ros2 daemon stop >> "$SESSION_DIR/session.log" 2>&1 || true
  sleep 2
  timeout 10 ros2 daemon start >> "$SESSION_DIR/session.log" 2>&1 || true
  sleep 2
  prepare_ros_dds_env
  sleep 2
  TIMING_STOP_ROBOT_S=$(( $(date +%s) - t0 ))
  timing_log "stop_robot(cold)=${TIMING_STOP_ROBOT_S}s"
}

perform_mapping_to_nav_handoff() {
  local t0 mode
  t0="$(date +%s)"
  # 禁止在交接前静默刷新会话位姿；使用 OK 时冻结的 SESSION_POSE

  if [[ "$FAST_NAV" -eq 1 ]]; then
    if ! stop_mapping_control_for_fast_nav; then
      log "  WARN: 快速交接硬检查失败，回退冷启动"
      stop_all_stacks_for_cold_nav
      mode="cold_nav_fallback"
      export NAV2_STOP_CONFLICTS=1
      export NAV2_REUSE_EXISTING=0
      unset NAV2_SKIP_DAEMON_REFRESH
    elif prepare_fast_nav_sensor_stack "FAST_NAV"; then
      mode="fast_nav"
      export NAV2_STOP_CONFLICTS=0
      export NAV2_REUSE_EXISTING=1
      export NAV2_SKIP_DAEMON_REFRESH=1
      log "  快速模式：复用传感器，仅启动 map_server/AMCL/Nav2"
    else
      log "  WARN: 快速模式健康检查失败，回退冷启动"
      stop_all_stacks_for_cold_nav
      mode="cold_nav_fallback"
      export NAV2_STOP_CONFLICTS=1
      export NAV2_REUSE_EXISTING=0
      unset NAV2_SKIP_DAEMON_REFRESH
    fi
  else
    stop_all_stacks_for_cold_nav
    mode="cold_nav"
    export NAV2_STOP_CONFLICTS=1
    export NAV2_REUSE_EXISTING=0
    unset NAV2_SKIP_DAEMON_REFRESH
  fi

  TIMING_MAPPING_TO_NAV_HANDOFF_S=$(( $(date +%s) - t0 ))
  timing_log "mapping_to_nav_handoff=${TIMING_MAPPING_TO_NAV_HANDOFF_S}s mode=$mode"
  export JQS_NAV_MODE="$mode"
}

stop_slam_stack_for_nav() {
  perform_mapping_to_nav_handoff
}

run_qwen_nav2_phase() {
  local bundle_json="$SESSION_DIR/qwen_live/candidate_bundle.json"
  local session_goal="$SESSION_DIR/navigation_goal_proposal.json"
  local nav_args=()

  if [[ "$NAV2_START_ONLY" -ne 1 ]]; then
    if [[ ! -f "$NAV_GOAL_JSON" ]]; then
      log "FAIL: 缺少 navigation_goal_proposal.json"
      exit 1
    fi
    if ! goal_ready_for_nav_validation "$NAV_GOAL_JSON" "$MAP_YAML" "$ALLOW_NAV_FALLBACK" "$bundle_json"; then
      log "FAIL: 目标未通过 Nav2 验证门禁（geometry/bundle/allow-fallback）"
      exit 1
    fi
    cp -f "$NAV_GOAL_JSON" "$session_goal"
  fi

  log_phase "[5/5] 自动 Nav2 导航到 Qwen 目标（reuse pipeline）"
  if [[ "$NAV2_START_ONLY" -eq 1 ]]; then
    log "NAV2_START_ONLY：仅启动 Nav2，不发送目标"
    nav_args+=(--start-only)
    if [[ ! -f "$session_goal" ]]; then
      python3 - "$session_goal" "$SESSION_ID" "$MAP_YAML" <<'PY'
import json, sys
from pathlib import Path
path, sid, my = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
path.write_text(json.dumps({
    "schema_version": "qwen_live_session_nav_goal_v1",
    "session_id": sid,
    "map_yaml": my,
    "selection_status": "REGION_PROPOSED",
    "goal_pose_map": {"x": 0.0, "y": 0.0, "yaw_rad": 0.0, "yaw_deg": 0.0},
    "selected_candidate_id": 1,
    "bundle_fingerprint": "",
    "safety": {"candidate_geometry_validated": True},
}, indent=2) + "\n", encoding="utf-8")
PY
    fi
  else
    print_nav_goal_summary "$NAV_GOAL_JSON"
  fi
  log "Nav2 runtime 目录: $SESSION_DIR/nav2_runtime/"

  local nav_t0 nav_boot_t0
  nav_t0="$(date +%s)"
  nav_boot_t0="$(date +%s)"

  if bash "$PROJECT_DIR/scripts/nav/run_qwen_target_nav2_reuse.sh" \
    --session-id "$SESSION_ID" \
    --session-dir "$SESSION_DIR" \
    --map-yaml "$MAP_YAML" \
    --pose-json "$SESSION_DIR/last_pose_map.json" \
    --goal-json "$session_goal" \
    --candidate-bundle "$bundle_json" \
    --mapping-pid "${JOY_PID:-}" \
    "${nav_args[@]}" \
    2>&1 | tee "$SESSION_DIR/nav2_run.log"; then
    log "✓ Nav2 reuse 阶段完成"
    # Detach JOY_PID from session EXIT cleanup list; ownership.json gates sensor retention.
    JOY_PID=""
    STARTED_JOY=0
  else
    log "✗ Nav2 reuse 阶段失败，详见 $SESSION_DIR/nav2_run.log"
    if [[ -f "$SESSION_DIR/nav2_runtime/nav2_state.json" ]]; then
      log "nav2_state.json:"
      cat "$SESSION_DIR/nav2_runtime/nav2_state.json" || true
    fi
    timeout 1.2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
      >/dev/null 2>&1 || true
    TIMING_NAV2_BOOT_S=$(( $(date +%s) - nav_boot_t0 ))
    TIMING_TOTAL_AFTER_OK_S=$(( $(date +%s) - OK_EPOCH ))
    export_timing_env
    write_pipeline_timing_json "qwen_nav2_reuse"
    exit 1
  fi

  if [[ -f "$SESSION_DIR/nav2_runtime/nav2_timing.json" ]]; then
    eval "$(python3 - "$SESSION_DIR/nav2_runtime/nav2_timing.json" <<'PY'
import json, sys
from pathlib import Path
d = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
if "nav2_boot_s" in d:
    print(f'TIMING_NAV2_BOOT_S={d["nav2_boot_s"]}')
if "total_s" in d:
    print(f'TIMING_NAVIGATION_S={d["total_s"]}')
PY
)"
  else
    TIMING_NAV2_BOOT_S=$(( $(date +%s) - nav_boot_t0 ))
    TIMING_NAVIGATION_S=$(( $(date +%s) - nav_t0 ))
  fi
  TIMING_TOTAL_AFTER_OK_S=$(( $(date +%s) - OK_EPOCH ))
  export_timing_env
  write_pipeline_timing_json "qwen_nav2_reuse"
  timing_log "nav2_boot=${TIMING_NAV2_BOOT_S}s navigation=${TIMING_NAVIGATION_S}s total_after_ok=${TIMING_TOTAL_AFTER_OK_S}s"
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

  local own_json="${SESSION_DIR}/nav2_runtime/ownership.json"
  if [[ -f "$own_json" ]] && python3 - "$own_json" "${PROJECT_DIR}/scripts/nav" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[2])
from qwen_nav2_common import ownership_protects_sensors
raise SystemExit(0 if ownership_protects_sensors(Path(sys.argv[1])) else 1)
PY
  then
    log "ownership protects sensors — skip JOY/sensor full cleanup (exit=$rc)"
    # Still cancel Nav2 best-effort; never kill lidar/filter/chassis/static_tf/foxglove
    timeout 0.8 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
      >/dev/null 2>&1 || true
    exit "$rc"
  fi

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
  export JQS_META_DRY_RUN_QWEN="$DRY_RUN_QWEN"
  export JQS_META_FAST_NAV="$FAST_NAV"
  export JQS_META_ALLOW_NAV_FALLBACK="$ALLOW_NAV_FALLBACK"
  export JQS_META_NAV2_START_ONLY="$NAV2_START_ONLY"
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
    "fast_nav": os.environ.get("JQS_META_FAST_NAV", "1") == "1",
    "allow_nav_fallback": os.environ.get("JQS_META_ALLOW_NAV_FALLBACK") == "1",
    "nav2_start_only": os.environ.get("JQS_META_NAV2_START_ONLY") == "1",
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
start_frontier_region_debug_session || true

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
  log "  ws://<RDK-IP>:8765  浅绿走廊: /qwen_explore_debug/map_with_visited"
  log "  勿启用 visited_area_grid（灰度层）；候选: /qwen_session/candidate_markers"
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
OK_EPOCH="$(date +%s)"
log "停车并冻结会话输入：零速 → 停 teleop → 等 odom 接近零 → 保存地图/轨迹/位姿 ..."
source_ros_environment
# 1) 连续发布零速度
timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
  >/dev/null 2>&1 || true
# 2) 立即停止/挂起 teleop，保证 OK 后机器人不再移动
pkill -TERM -f "teleop_twist_joy|joy_node" 2>/dev/null || true
sleep 0.5
# 3) 等待 /odom 速度接近零（尽力而为）
python3 - <<'PY' || true
import time
try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    rclpy.init()
    node = Node("jqs_wait_odom_zero")
    state = {"ok": False}
    def cb(msg):
        v = abs(msg.twist.twist.linear.x) + abs(msg.twist.twist.angular.z)
        if v < 0.01:
            state["ok"] = True
    node.create_subscription(Odometry, "/odom", cb, 10)
    t0 = time.time()
    while time.time() - t0 < 3.0 and not state["ok"]:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
except Exception:
    pass
PY

t_copy="$(date +%s)"
# 先保存位姿快照到会话（冻结），再复制轨迹；后续禁止再用全局可变文件
refresh_session_pose_before_nav || true
if [[ -f "$POSE_STATE_FILE" ]]; then
  cp -f "$POSE_STATE_FILE" "$SESSION_DIR/last_pose_map.json"
fi
if [[ -f "$TRAJ_FILE" ]]; then
  cp -f "$TRAJ_FILE" "$SESSION_DIR/trajectory_session.json"
fi
copy_live_debug_artifacts
TIMING_COPY_ARTIFACTS_S=$(( $(date +%s) - t_copy ))

t_save="$(date +%s)"
if ! save_map_to_session; then
  log "FAIL: 地图保存失败"
  exit 1
fi
TIMING_SAVE_MAP_S=$(( $(date +%s) - t_save ))
timing_log "save_map=${TIMING_SAVE_MAP_S}s"

MAP_YAML="$SESSION_DIR/map/${MAP_NAME}.yaml"
SESSION_MAP_YAML="$MAP_YAML"
SESSION_MAP_PGM="$SESSION_DIR/map/${MAP_NAME}.pgm"
SESSION_TRAJECTORY="$SESSION_DIR/trajectory_session.json"
SESSION_POSE="$SESSION_DIR/last_pose_map.json"
ANNOTATED_PNG="$SESSION_DIR/annotated_map_for_qwen.png"
POSE_UV_JSON="$SESSION_DIR/robot_pose_uv.json"
QWEN_OUT="$SESSION_DIR/qwen_live"
LIVE_REF=""
if [[ -f "$SESSION_DIR/live_annotated_map.png" ]]; then
  LIVE_REF="$SESSION_DIR/live_annotated_map.png"
fi

if [[ ! -f "$SESSION_POSE" ]]; then
  log "FAIL: 会话位姿快照不存在: $SESSION_POSE"
  exit 1
fi

# OK handoff snapshot: frozen map + odom poses (used by Nav2 odom hard gate)
python3 - "$SESSION_DIR/handoff_snapshot.json" "$SESSION_ID" "$SESSION_POSE" "$MAP_YAML" <<'PY'
import json, math, os, sys, time
from pathlib import Path
out, sid, pose_path, map_yaml = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3]), sys.argv[4]
pose = json.loads(pose_path.read_text(encoding="utf-8"))
map_pose = {
    "x": float(pose["x"]),
    "y": float(pose["y"]),
    "yaw": float(pose.get("yaw", 0.0)),
}
odom_pose = None
pose_source = str(pose.get("source", "last_pose_map"))
try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = Node("jqs_handoff_odom_snap")
    box = {"m": None}
    node.create_subscription(Odometry, "/odom", lambda m: box.__setitem__("m", m), qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < 2.0 and box["m"] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    if box["m"] is not None:
        p = box["m"].pose.pose.position
        q = box["m"].pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        odom_pose = {"x": float(p.x), "y": float(p.y), "yaw": float(yaw)}
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
except Exception as exc:
    pose_source = pose_source + f"|odom_snap_err:{exc}"
payload = {
    "session_id": sid,
    "snapshot_epoch": time.time(),
    "map_pose": map_pose,
    "odom_pose": odom_pose,
    "map_yaml": map_yaml,
    "pose_source": pose_source,
}
tmp = out.with_suffix(".tmp")
with open(tmp, "w", encoding="utf-8") as fh:
    fh.write(json.dumps(payload, indent=2) + "\n")
    fh.flush(); os.fsync(fh.fileno())
os.replace(tmp, out)
if odom_pose is None:
    print("WARN: handoff_snapshot missing odom_pose — continue without odom gate", file=sys.stderr)
print("handoff_snapshot written")
PY

mkdir -p "$SESSION_DIR/nav2_runtime"
python3 - "$SESSION_DIR/nav2_runtime/ownership.json" "$SESSION_ID" "${PROJECT_DIR}/scripts/nav" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[3])
from qwen_nav2_common import write_ownership
write_ownership(Path(sys.argv[1]), sys.argv[2], "MAPPING_OWNS_SENSOR")
PY

if [[ ! -f "$SESSION_TRAJECTORY" ]] && [[ -f "$TRAJ_FILE" ]]; then
  cp -f "$TRAJ_FILE" "$SESSION_TRAJECTORY"
fi

# 校验快照时间差
python3 - "$SESSION_MAP_YAML" "$SESSION_POSE" "$SESSION_TRAJECTORY" <<'PY'
import os, sys, time
from pathlib import Path
paths = [Path(p) for p in sys.argv[1:] if p]
times = [p.stat().st_mtime for p in paths if p.is_file()]
if len(times) >= 2:
    delta = max(times) - min(times)
    limit = float(os.environ.get("SNAPSHOT_MAX_TIME_DELTA_S", "30"))
    print(f"[SNAPSHOT] time_delta_s={delta:.2f} limit={limit:.1f}")
    if delta > limit:
        raise SystemExit(f"FAIL: snapshot_time_delta_s={delta:.1f} > {limit}")
PY

TRAJ_FOR_QWEN="$SESSION_TRAJECTORY"
POSE_FOR_QWEN="$SESSION_POSE"

VISITED_CORRIDOR_RADIUS_M="${VISITED_CORRIDOR_RADIUS_M:-$(
  python3 -c "import sys; sys.path.insert(0,'$PROJECT_DIR/scripts/debug'); from qwen_map_goal_utils import load_visited_corridor_radius_m; print(load_visited_corridor_radius_m())"
)}"

QWEN_MAP_YAML="$SESSION_DIR/map/${MAP_NAME}_qwen.yaml"
t_eqwen="$(date +%s)"
if [[ -f "$TRAJ_FOR_QWEN" ]]; then
  python3 "$PROJECT_DIR/scripts/debug/export_qwen_visited_map.py" \
    --map-yaml "$MAP_YAML" \
    --trajectory-json "$TRAJ_FOR_QWEN" \
    --output-dir "$SESSION_DIR/map" \
    --corridor-radius-m "$VISITED_CORRIDOR_RADIUS_M" \
    | tee "$SESSION_DIR/export_qwen_map.stdout.json"
  log "Qwen 地图: $QWEN_MAP_YAML (corridor_radius_m=$VISITED_CORRIDOR_RADIUS_M)"
else
  log "WARN: 无 trajectory，跳过 Qwen 专用 PGM 导出"
fi
TIMING_EXPORT_QWEN_MAP_S=$(( $(date +%s) - t_eqwen ))

t_annot="$(date +%s)"
python3 "$PROJECT_DIR/scripts/debug/export_session_map_annotations.py" \
  --map-yaml "$MAP_YAML" \
  --pose-json "$POSE_FOR_QWEN" \
  --trajectory-json "$TRAJ_FOR_QWEN" \
  --output-png "$ANNOTATED_PNG" \
  --output-pose-json "$POSE_UV_JSON" \
  --corridor-radius-m "$VISITED_CORRIDOR_RADIUS_M" \
  ${LIVE_REF:+--copy-live-annotated "$LIVE_REF"} \
  | tee "$SESSION_DIR/robot_pose_uv.stdout.json"
TIMING_EXPORT_ANNOTATION_S=$(( $(date +%s) - t_annot ))
timing_log "export_qwen_map=${TIMING_EXPORT_QWEN_MAP_S}s export_annotation=${TIMING_EXPORT_ANNOTATION_S}s"

read -r ROBOT_U ROBOT_V ROBOT_YAW < <(
python3 - "$POSE_UV_JSON" <<'PY'
import json, sys
with open(sys.argv[1], encoding="utf-8") as f:
    data = json.load(f)
pose = data["robot_image_pose"]
print(pose["u"], pose["v"], pose["yaw_deg"])
PY
)

SESSION_SAVE_DONE=1
log "标注地图: $ANNOTATED_PNG"
log "机器人归一化位姿: u=$ROBOT_U v=$ROBOT_V yaw_deg=$ROBOT_YAW"
log "会话冻结输入: MAP=$SESSION_MAP_YAML POSE=$SESSION_POSE TRAJ=$SESSION_TRAJECTORY"

if [[ "$STOP_AFTER_SAVE" -eq 1 ]]; then
  stop_started_processes
  trap - EXIT INT TERM
else
  log "保持建图栈与 Foxglove 可视化运行（默认 --keep-mapping）"
  log "Foxglove: 实时 /map + /qwen_session/robot_pose_markers"
  log "请打开布局: configs/foxglove_slam_mapping.layout.json（建图）"
  trap - EXIT INT TERM
fi

# --skip-qwen --nav2-start-only：跳过 Qwen，直接做快速 Nav2 启动
if [[ "$SKIP_QWEN" -eq 1 && "$NAV2_START_ONLY" -eq 1 ]]; then
  log "--skip-qwen --nav2-start-only：跳过候选/Qwen，进入 Nav2 启动"
  AUTO_NAV=1
  run_qwen_nav2_phase
  exit 0
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
  --map-yaml "$SESSION_MAP_YAML"
  --pose-json "$POSE_FOR_QWEN"
  --trajectory-json "$TRAJ_FOR_QWEN"
  --output-dir "$QWEN_OUT"
  --nav-goal-json "$NAV_GOAL_JSON"
  --session-id "$SESSION_ID"
  --task "$TASK"
  --candidate-bundle "$QWEN_OUT/candidate_bundle.json"
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

t_cand="$(date +%s)"
log "预计算 v6 候选（candidates-only，不调用 Qwen API）..."
QWEN_PREVIEW=( "${QWEN_CMD[@]}" --candidates-only )
if ! "${QWEN_PREVIEW[@]}" 2>&1 | tee "$SESSION_DIR/qwen_candidates_preview.log"; then
  log "FAIL: candidates-only 失败"
  exit 1
fi
TIMING_CANDIDATE_GENERATION_S=$(( $(date +%s) - t_cand ))
timing_log "candidates=${TIMING_CANDIDATE_GENERATION_S}s"
log "Foxglove 候选点: runtime/qwen_session/live_candidates_foxglove.json"

if [[ "$DRY_RUN_QWEN" -eq 1 ]]; then
  t_qwen="$(date +%s)"
  log "dry-run：执行完整 planner（复用 candidate_bundle，Python 选点，不调用 Qwen API）..."
  if ! "${QWEN_CMD[@]}" 2>&1 | tee "$SESSION_DIR/qwen_run.log"; then
    log "FAIL: Qwen dry-run 失败"
    exit 1
  fi
  TIMING_QWEN_API_S=$(( $(date +%s) - t_qwen ))
else
  t_qwen="$(date +%s)"
  log "调用 qwen_live_session_planner.py（复用 candidate_bundle + Qwen 第二阶段）..."
  if ! "${QWEN_CMD[@]}" 2>&1 | tee "$SESSION_DIR/qwen_run.log"; then
    log "FAIL: Qwen live session 规划失败"
    exit 1
  fi
  TIMING_QWEN_API_S=$(( $(date +%s) - t_qwen ))
fi
timing_log "qwen=${TIMING_QWEN_API_S}s"

if [[ -f "$NAV_GOAL_JSON" ]]; then
  cp -f "$NAV_GOAL_JSON" "$SESSION_DIR/navigation_goal_proposal.json"
  log "Qwen 导航提案: $NAV_GOAL_JSON"
  print_nav_goal_summary "$NAV_GOAL_JSON"
fi

# ---------------------------------------------------------------------------
# Phase 5: Nav2 导航
# ---------------------------------------------------------------------------
if [[ "$AUTO_NAV" -eq 1 ]]; then
  if [[ "$NAV2_START_ONLY" -eq 1 ]]; then
    run_qwen_nav2_phase
  elif grep -q '"selected_by": "python_fallback_after_qwen_error"' "$NAV_GOAL_JSON" 2>/dev/null \
    && [[ "$ALLOW_NAV_FALLBACK" -ne 1 ]]; then
    log "[5/5] Qwen fallback 目标已生成，但未开启 --allow-nav-with-fallback，跳过自动导航"
  else
    run_qwen_nav2_phase
  fi
else
  log "[5/5] 已跳过自动导航 (--no-auto-nav)"
  log "导航布局请打开: configs/foxglove_nav2_saved_map.layout.json（若存在）或启用 /map + /qwen_session/planned_path"
  export_timing_env
  write_pipeline_timing_json "no_auto_nav"
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
  trajectory_session.json         — 轨迹顶点（OK 时冻结）
  last_pose_map.json              — 位姿快照（OK 时冻结）
  qwen_live/                      — v6 候选图、Qwen 结果、live_report.json
  navigation_goal_proposal.json   — map 坐标 Nav2/Foxglove 目标

Foxglove（ws://<RDK-IP>:8765）:
  建图布局: configs/foxglove_slam_mapping.layout.json
  浅绿走廊: /qwen_explore_debug/map_with_visited（勿开 visited_area_grid）
  导航布局: /map + /qwen_session/planned_path
  /scan_filtered, /qwen_session/robot_pose_markers, /qwen_session/candidate_markers

Nav2:
  默认 Qwen 选点后自动导航；跳过请加 --no-auto-nav
  bash scripts/nav/run_qwen_session_nav2_goal.sh $MAP_YAML $NAV_GOAL_JSON
  日志: $SESSION_DIR/nav2_run.log  $SESSION_DIR/nav2_logs/

重新跑 Qwen live planner（必须用会话冻结文件）:
  python3 scripts/debug/qwen_live_session_planner.py \\
    --map-yaml $SESSION_MAP_YAML --pose-json $SESSION_POSE \\
    --trajectory-json $SESSION_TRAJECTORY \\
    --output-dir $QWEN_OUT --nav-goal-json $NAV_GOAL_JSON \\
    --session-id $SESSION_ID --candidate-bundle $QWEN_OUT/candidate_bundle.json
EOF

exit 0
