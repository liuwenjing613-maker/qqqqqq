#!/usr/bin/env bash
# 手柄建图 + 路径标注 + OK 保存 + Qwen 规划 — 编排脚本（只调用已有脚本，不修改它们）。
#
# 流程：
#   1. 启动 SLAM / 轨迹 debug / pose memory / 手柄 teleop（若已在运行则复用）
#   2. 用手柄建图，终端输入 OK 保存地图与标注
#   3. 调用 test_qwen_map_global_region.py 做下一步探索规划
#
# 用法：
#   bash scripts/slam/run_joy_mapping_qwen_plan_session.sh
#
# 终端命令：
#   OK      — 保存地图、标注、并调用 Qwen
#   STATUS  — 查看话题/位姿/会话目录
#   QUIT    — 退出（仅停止本脚本启动的进程）
#
# 环境变量（可选）：
#   QWEN_TASK              — 给 Qwen 的任务说明
#   QWEN_DRY_RUN=1         — 只生成 Qwen 输入，不调 API
#   STOP_STACK_ON_QUIT=1   — QUIT 时停止本脚本拉起的 SLAM/joy
#   JOY_DEV=/dev/input/js0
#   MAP_NAME=...           — 默认随会话 ID 自动生成

set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_DIR"

# shellcheck source=scripts/lib/slam_calibrated_env.sh
source "${PROJECT_DIR}/scripts/lib/slam_calibrated_env.sh"
export SLAM_USE_CALIBRATION=1

SESSION_ID="MQS_$(date -u +%Y%m%dT%H%M%SZ)"
SESSION_DIR="${PROJECT_DIR}/logs/mapping_qwen_sessions/${SESSION_ID}"
MAP_DIR="${SESSION_DIR}/map"
ANNOT_DIR="${SESSION_DIR}/annotations"
QWEN_OUT_DIR="${SESSION_DIR}/qwen"
LOG_DIR="${SESSION_DIR}/logs"
STATE_DIR="${SESSION_DIR}/state"
MAP_NAME="${MAP_NAME:-map_${SESSION_ID}}"
POSE_STATE_FILE="${STATE_DIR}/last_pose_map.json"
TRAJ_RUNTIME="${PROJECT_DIR}/runtime/qwen_region_debug/trajectory_session.json"
DEBUG_RUN_DIR_FILE="${PROJECT_DIR}/runtime/qwen_region_debug/frontier_region_debug.run_dir"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
QWEN_TASK="${QWEN_TASK:-优先探索尚未覆盖、最可能扩展地图的区域。}"
QWEN_DRY_RUN="${QWEN_DRY_RUN:-0}"
STOP_STACK_ON_QUIT="${STOP_STACK_ON_QUIT:-0}"

PIDS=()
STARTED_SLAM=0
STARTED_DEBUG=0
STARTED_POSE=0
STARTED_JOY=0
STARTED_TELEOP=0
REUSED_DEBUG=0
ROS_ENV_READY=0
SAVED_COUNT=0

mkdir -p "$MAP_DIR" "$ANNOT_DIR" "$QWEN_OUT_DIR" "$LOG_DIR" "$STATE_DIR"

log() {
  echo "[$(date +%H:%M:%S)] $*"
}

source_ros() {
  if [ "$ROS_ENV_READY" = "1" ]; then
    return 0
  fi
  set +u
  if [ -f /opt/tros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/tros/humble/setup.bash
  elif [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [ -f "$HOME/ydlidar_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$HOME/ydlidar_ws/install/setup.bash"
  fi
  set -u
  ROS_ENV_READY=1
}

start_bg() {
  local name="$1"
  shift
  log "Starting ${name} ..."
  "$@" >>"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
  echo "$!" >> "${SESSION_DIR}/started_pids.txt"
  sleep 1
}

wait_topic_exists() {
  local topic="$1"
  local timeout_sec="${2:-90}"
  log "Waiting for ${topic} ..."
  for _ in $(seq 1 "$timeout_sec"); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      log "OK: ${topic}"
      return 0
    fi
    sleep 1
  done
  log "WARN: timeout waiting for ${topic}"
  return 1
}

map_is_live() {
  source_ros
  if ! ros2 topic list 2>/dev/null | grep -qx "/map"; then
    return 1
  fi
  local pub_count
  pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
  [ "${pub_count:-0}" != "0" ]
}

maybe_convert_png() {
  local pgm="$1"
  local png="$2"
  if [ ! -f "$pgm" ]; then
    return 1
  fi
  python3 - "$pgm" "$png" <<'PY'
import sys
from pathlib import Path
pgm, png = Path(sys.argv[1]), Path(sys.argv[2])
try:
    from PIL import Image
except ImportError:
    sys.exit(1)
Image.open(pgm).save(png)
print(f"[OK] {png}")
PY
}

save_map_to_session() {
  local MAP_OUT="${MAP_DIR}/${MAP_NAME}"
  local MAP_TMP="${MAP_OUT}.tmp_$(date +%Y%m%d_%H%M%S)"
  local save_start_epoch saver_rc pub_count file_epoch

  source_ros
  save_start_epoch="$(date +%s)"

  log "Saving map to ${MAP_OUT} ..."

  if ! ros2 topic list 2>/dev/null | grep -qx "/map"; then
    log "ERROR: /map does not exist"
    return 1
  fi

  pub_count="$(ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}')"
  if [ "${pub_count:-0}" = "0" ]; then
    log "ERROR: /map has no publisher"
    return 1
  fi

  set +e
  timeout 30 ros2 run nav2_map_server map_saver_cli \
    -t /map \
    -f "$MAP_TMP" \
    --ros-args \
    -p save_map_timeout:=20.0 \
    >> "${LOG_DIR}/map_saver.log" 2>&1
  saver_rc=$?
  set -u

  if [ "$saver_rc" -ne 0 ]; then
    log "ERROR: map_saver_cli failed (rc=${saver_rc})"
    tail -20 "${LOG_DIR}/map_saver.log" 2>/dev/null || true
    rm -f "${MAP_TMP}.pgm" "${MAP_TMP}.yaml" 2>/dev/null || true
    return 1
  fi

  if [ ! -f "${MAP_TMP}.pgm" ] || [ ! -f "${MAP_TMP}.yaml" ]; then
    log "ERROR: map files missing after save"
    return 1
  fi

  mv "${MAP_TMP}.pgm" "${MAP_OUT}.pgm"
  mv "${MAP_TMP}.yaml" "${MAP_OUT}.yaml"

  python3 - "${MAP_OUT}.yaml" "${MAP_NAME}.pgm" <<'PY'
import sys
from pathlib import Path
import yaml
path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding="utf-8"))
data["image"] = sys.argv[2]
path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
PY

  maybe_convert_png "${MAP_OUT}.pgm" "${MAP_OUT}.png" || true
  log "Map saved: ${MAP_OUT}.yaml / .pgm"
  return 0
}

copy_debug_artifacts() {
  local debug_run_dir=""
  if [ -f "$DEBUG_RUN_DIR_FILE" ]; then
    debug_run_dir="$(tr -d '\n' < "$DEBUG_RUN_DIR_FILE")"
  fi
  if [ -n "$debug_run_dir" ] && [ -d "$debug_run_dir" ]; then
    cp -f "$debug_run_dir/latest_annotated_map.png" "${ANNOT_DIR}/debug_latest_annotated_map.png" 2>/dev/null || true
    cp -f "$debug_run_dir/latest_map_health.json" "${ANNOT_DIR}/debug_latest_map_health.json" 2>/dev/null || true
    echo "$debug_run_dir" > "${ANNOT_DIR}/debug_run_dir.txt"
  fi
  if [ -f "$TRAJ_RUNTIME" ]; then
    cp -f "$TRAJ_RUNTIME" "${ANNOT_DIR}/trajectory_session.json"
  fi
  if [ -f "$POSE_STATE_FILE" ]; then
    cp -f "$POSE_STATE_FILE" "${ANNOT_DIR}/last_pose_map.json"
  fi
}

compose_annotated_map() {
  local map_png="${MAP_DIR}/${MAP_NAME}.png"
  local map_pgm="${MAP_DIR}/${MAP_NAME}.pgm"
  local map_yaml="${MAP_DIR}/${MAP_NAME}.yaml"
  local map_image="$map_png"
  local compose_meta="${ANNOT_DIR}/compose_meta.json"
  local pose_uv="${ANNOT_DIR}/robot_pose_uv.json"
  local annotated="${ANNOT_DIR}/session_annotated_map.png"

  if [ ! -f "$map_image" ]; then
    map_image="$map_pgm"
  fi

  python3 "${PROJECT_DIR}/scripts/slam/compose_mapping_session_map.py" \
    --map-image "$map_image" \
    --map-yaml "$map_yaml" \
    --trajectory-json "${ANNOT_DIR}/trajectory_session.json" \
    --pose-json "${ANNOT_DIR}/last_pose_map.json" \
    --output "$annotated" \
    --pose-uv-json "$pose_uv" \
    --arrow-scale 2.5 \
    | tee "$compose_meta"
}

run_qwen_plan() {
  local pose_uv="${ANNOT_DIR}/robot_pose_uv.json"
  local annotated="${ANNOT_DIR}/session_annotated_map.png"
  local qwen_cmd=(
    python3 "${PROJECT_DIR}/scripts/debug/test_qwen_map_global_region.py"
    --map "$annotated"
    --output-dir "$QWEN_OUT_DIR"
    --task "$QWEN_TASK"
  )

  if [ ! -f "$pose_uv" ] || [ ! -f "$annotated" ]; then
    log "ERROR: missing annotated map or pose_uv for Qwen"
    return 1
  fi

  local robot_u robot_v robot_yaw
  robot_u="$(python3 - <<PY
import json
from pathlib import Path
d=json.loads(Path("$pose_uv").read_text())
print(d["u"])
PY
)"
  robot_v="$(python3 - <<PY
import json
from pathlib import Path
d=json.loads(Path("$pose_uv").read_text())
print(d["v"])
PY
)"
  robot_yaw="$(python3 - <<PY
import json
from pathlib import Path
d=json.loads(Path("$pose_uv").read_text())
print(d["yaw_deg"])
PY
)"

  qwen_cmd+=(--robot-u "$robot_u" --robot-v "$robot_v" --robot-yaw-deg "$robot_yaw")

  if [ "$QWEN_DRY_RUN" = "1" ]; then
    qwen_cmd+=(--dry-run)
  fi

  log "Calling Qwen planner ..."
  "${qwen_cmd[@]}" | tee "${LOG_DIR}/qwen_run.log"
}

write_session_manifest() {
  local manifest="${SESSION_DIR}/session_manifest.json"
  python3 - <<PY
import json
from pathlib import Path
from datetime import datetime, timezone

session = Path("$SESSION_DIR")
manifest = {
    "session_id": "$SESSION_ID",
    "updated_at": datetime.now(timezone.utc).isoformat(),
    "saved_count": int("$SAVED_COUNT"),
    "map_dir": str(session / "map"),
    "annotations_dir": str(session / "annotations"),
    "qwen_dir": str(session / "qwen"),
    "map_name": "$MAP_NAME",
    "started": {
        "slam": bool(int("$STARTED_SLAM")),
        "frontier_debug": bool(int("$STARTED_DEBUG")),
        "reused_debug": bool(int("$REUSED_DEBUG")),
        "pose_memory": bool(int("$STARTED_POSE")),
        "joy": bool(int("$STARTED_JOY")),
        "teleop": bool(int("$STARTED_TELEOP")),
    },
    "artifacts": {
        "map_yaml": str(session / "map" / "${MAP_NAME}.yaml"),
        "map_pgm": str(session / "map" / "${MAP_NAME}.pgm"),
        "session_annotated_map": str(session / "annotations" / "session_annotated_map.png"),
        "robot_pose_uv": str(session / "annotations" / "robot_pose_uv.json"),
        "trajectory_session": str(session / "annotations" / "trajectory_session.json"),
    },
}
qwen_root = session / "qwen"
if qwen_root.is_dir():
    runs = sorted([p for p in qwen_root.iterdir() if p.is_dir()], key=lambda p: p.name)
    if runs:
        latest = runs[-1]
        manifest["qwen_latest_run"] = str(latest)
        for name in (
            "qwen_selected_region.png",
            "qwen_parsed_response.json",
            "map_input_for_qwen.png",
            "prompt.txt",
        ):
            p = latest / name
            if p.is_file():
                manifest.setdefault("qwen_artifacts", {})[name] = str(p)

Path("$manifest").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(f"[OK] manifest -> {manifest}")
PY
}

handle_ok() {
  SAVED_COUNT=$((SAVED_COUNT + 1))
  local save_tag
  save_tag="$(date -u +%Y%m%dT%H%M%SZ)"
  log "===== OK #${SAVED_COUNT} (${save_tag}) — save + annotate + Qwen ====="

  if ! save_map_to_session; then
    log "FAIL: map save aborted"
    return 1
  fi

  copy_debug_artifacts

  if ! compose_annotated_map; then
    log "FAIL: annotated map composition failed"
    return 1
  fi

  if ! run_qwen_plan; then
    log "FAIL: Qwen planning failed"
    return 1
  fi

  write_session_manifest

  log "===== Session save complete ====="
  log "Session dir: ${SESSION_DIR}"
  log "Annotated map: ${ANNOT_DIR}/session_annotated_map.png"
  log "Robot UV: ${ANNOT_DIR}/robot_pose_uv.json"
  log "Qwen output: ${QWEN_OUT_DIR}/"
  log "Type OK again to save another round, or QUIT to exit."
}

show_status() {
  source_ros
  echo
  echo "========== Mapping + Qwen Session =========="
  echo "session_id=${SESSION_ID}"
  echo "session_dir=${SESSION_DIR}"
  echo "saved_count=${SAVED_COUNT}"
  echo "pose_state=${POSE_STATE_FILE}"
  echo "started_slam=${STARTED_SLAM} started_debug=${STARTED_DEBUG} reused_debug=${REUSED_DEBUG}"
  echo
  echo "========== ROS topics (subset) =========="
  ros2 topic list 2>/dev/null | sort | egrep "joy|cmd_vel|scan|odom|map|tf" || true
  echo
  if [ -f "$POSE_STATE_FILE" ]; then
    echo "========== Last pose =========="
    cat "$POSE_STATE_FILE"
    echo
  fi
  if [ -f "$TRAJ_RUNTIME" ]; then
    echo "========== Trajectory summary =========="
    python3 - <<PY
import json
from pathlib import Path
p = Path("$TRAJ_RUNTIME")
if p.is_file():
    d = json.loads(p.read_text())
    print("trajectory_session_id:", d.get("trajectory_session_id"))
    print("vertex_count:", len(d.get("vertices") or []))
    print("raw_sample_count:", len(d.get("raw_samples") or []))
PY
  fi
  echo
}

stop_pid_if_alive() {
  local pid="$1"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
  fi
}

stop_tracked_pids() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    stop_pid_if_alive "$pid"
  done
  sleep 1
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
}

stop_session_stack() {
  log "Stopping session-owned processes only (tracked PIDs: ${#PIDS[@]}) ..."

  stop_tracked_pids

  if [ "$STARTED_DEBUG" = "1" ] && [ "$REUSED_DEBUG" = "0" ]; then
    if bash "${PROJECT_DIR}/scripts/nav/stop_frontier_region_debug.sh" >>"${LOG_DIR}/stop_debug.log" 2>&1; then
      log "OK: stopped frontier_region_debug (started by this session)"
    else
      log "WARN: stop_frontier_region_debug failed; see ${LOG_DIR}/stop_debug.log"
    fi
  fi

  if [ "$STARTED_SLAM" = "1" ] && [ "$STOP_STACK_ON_QUIT" = "1" ]; then
    local slam_pid=""
    if [ -f "${SESSION_DIR}/started_pids.txt" ]; then
      slam_pid="$(sed -n '1p' "${SESSION_DIR}/started_pids.txt" 2>/dev/null || true)"
    fi
    stop_pid_if_alive "$slam_pid"
    sleep 1
    stop_pid_if_alive "$slam_pid"
  fi

  source_ros 2>/dev/null || true
  timeout 1.2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 5 \
    >/dev/null 2>&1 || true
}

cleanup_on_exit() {
  stop_session_stack
}

trap cleanup_on_exit EXIT INT TERM

start_stack_components() {
  source_ros

  log "===== Joy Mapping + Qwen Plan Session ====="
  log "Session: ${SESSION_DIR}"

  cat > "${SESSION_DIR}/session_meta.json" <<EOF
{
  "session_id": "${SESSION_ID}",
  "project_dir": "${PROJECT_DIR}",
  "pose_state_file": "${POSE_STATE_FILE}",
  "qwen_task": "${QWEN_TASK}"
}
EOF

  if map_is_live; then
    log "Reuse existing /map publisher (will NOT start new SLAM stack)"
    STARTED_SLAM=0
  else
    log "[1/5] Start SLAM via scripts/slam/run_slam_calibrated.sh"
    start_bg slam_stack setsid bash "${PROJECT_DIR}/scripts/slam/run_slam_calibrated.sh"
    STARTED_SLAM=1

    wait_topic_exists /scan 120 || exit 1
    wait_topic_exists /scan_filtered 120 || exit 1
    wait_topic_exists /odom 120 || exit 1
    wait_topic_exists /map 120 || exit 1
  fi

  if ros2 node list 2>/dev/null | grep -qx '/frontier_region_debug'; then
    log "[2/5] Reuse existing /frontier_region_debug (trajectory overlay)"
    REUSED_DEBUG=1
    STARTED_DEBUG=0
  else
    log "[2/5] Start trajectory/path overlay via scripts/nav/start_frontier_region_debug.sh"
    if bash "${PROJECT_DIR}/scripts/nav/start_frontier_region_debug.sh" >>"${LOG_DIR}/start_debug.log" 2>&1; then
      STARTED_DEBUG=1
    else
      log "WARN: frontier_region_debug failed to start; path overlay may be empty"
      log "WARN: see ${LOG_DIR}/start_debug.log"
    fi
  fi

  log "[3/5] Start pose memory (session-local state file)"
  start_bg pose_memory python3 "${PROJECT_DIR}/scripts/slam/pose_memory_node.py" \
    --state-file "$POSE_STATE_FILE" \
    --map-frame map \
    --base-frame base_link \
    --fallback-base-frame base_footprint \
    --save-period 1.0
  STARTED_POSE=1

  if [ ! -e "$JOY_DEV" ]; then
    log "WARN: joystick ${JOY_DEV} not found"
    ls -l /dev/input/js* 2>/dev/null || true
  fi

  log "[4/5] Start joy_node + teleop -> /cmd_vel"
  start_bg joy_node ros2 run joy joy_node --ros-args \
    -p "dev:=${JOY_DEV}" \
    -p "deadzone:=${JOY_DEADZONE}" \
    -p autorepeat_rate:=20.0
  STARTED_JOY=1

  wait_topic_exists /joy 30 || log "WARN: /joy not ready yet"

  start_bg teleop ros2 run teleop_twist_joy teleop_node --ros-args \
    -p require_enable_button:=false \
    -p "axis_linear.x:=${JOY_AXIS_LINEAR}" \
    -p "scale_linear.x:=${JOY_SCALE_LINEAR_X}" \
    -p "axis_angular.yaw:=${JOY_AXIS_ANGULAR}" \
    -p "scale_angular.yaw:=${JOY_SCALE_ANGULAR_YAW}"
  STARTED_TELEOP=1

  sleep 2
  log "[5/5] Stack ready"
  show_status

  echo
  echo "============================================================"
  echo " 手柄建图会话已启动"
  echo "============================================================"
  echo " 1. 用手柄缓慢驱动小车建图"
  echo " 2. 终端输入 OK  + Enter  → 保存地图、路径标注、调用 Qwen"
  echo " 3. 输入 STATUS          → 查看状态"
  echo " 4. 输入 QUIT  + Enter   → 退出（默认保留 SLAM；设 STOP_STACK_ON_QUIT=1 可一并停 SLAM）"
  echo ""
  echo " Session: ${SESSION_DIR}"
  echo "============================================================"
}

main() {
  start_stack_components

  while true; do
    printf "> "
    if ! read -r line; then
      break
    fi
    case "$(echo "$line" | tr '[:lower:]' '[:upper:]')" in
      OK)
        handle_ok || true
        ;;
      STATUS)
        show_status
        ;;
      QUIT|EXIT|Q)
        log "QUIT requested"
        break
        ;;
      "")
        ;;
      *)
        echo "未知命令。可用: OK | STATUS | QUIT"
        ;;
    esac
  done
}

main "$@"
