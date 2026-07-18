#!/usr/bin/env bash
# One-command sprint flow:
# voice -> calibrated live SLAM -> Nav2 -> live map/Qwen backend -> bridge
# -> simplified evidence-based handoff -> proven first-person voice servo.
set -Eeuo pipefail

V1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPO_ROOT="$(cd "$V1_ROOT/.." && pwd)"

SIMPLE_CFG="${SIMPLE_HANDOFF_CONFIG:-$V1_ROOT/configs/simple_handoff_v2.yaml}"
BASE_SERVO_CFG="${SIMPLE_HANDOFF_BASE_SERVO_CONFIG:-$V1_ROOT/configs/qwen3_vln_servo.yaml}"
FUSION_CFG="${SIMPLE_HANDOFF_FUSION_CONFIG:-$V1_ROOT/configs/online_map_plan_fusion_fullflow_v2.yaml}"
BACKEND_CFG="${SIMPLE_HANDOFF_BACKEND_CONFIG:-$V1_ROOT/configs/online_map_plan_fullflow_v2.yaml}"
RUNTIME_DIR="${SIMPLE_HANDOFF_RUNTIME_DIR:-/tmp/rdk_x5_simple_handoff_v2_${USER:-robot}}"
LOG_BASE="$V1_ROOT/logs/simple_handoff_v2"

TASK="${QWEN_TASK:-}"
MOTION_ENABLED="${MOTION_ENABLED:-0}"
START_SLAM="${SIMPLE_HANDOFF_START_SLAM:-1}"
REUSE_SLAM="${SIMPLE_HANDOFF_REUSE_SLAM:-1}"
KEEP_RAW_LOGS="${SIMPLE_HANDOFF_KEEP_RAW_LOGS:-0}"
TF_WAIT_SEC="${SIMPLE_HANDOFF_TF_WAIT_SEC:-60}"
TOPIC_WAIT_SEC="${SIMPLE_HANDOFF_TOPIC_WAIT_SEC:-60}"
NAV2_WAIT_SEC="${SIMPLE_HANDOFF_NAV2_WAIT_SEC:-90}"
START_FOXGLOVE="${START_FOXGLOVE:-1}"
START_VISITED_CORRIDOR_DEBUG="${START_VISITED_CORRIDOR_DEBUG:-1}"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/fusion/start_v1_map_qwen_simple_voice_v2.sh [options]

Options:
  --task 'find the bottle'  Bypass wake-word recording and use this instruction.
  --motion                  Enable real chassis motion.
  --dry-run                 Disable chassis motion and request backend dry-run.
  --no-slam                 Require an already-running /map,/odom,/scan_filtered stack.
  --no-reuse-slam           Refuse an already-running SLAM stack.
  --no-foxglove             Do not start Foxglove from the first-person script.
  --no-visited-corridor     Skip persistent visited-corridor map overlay node.
  --keep-raw-logs           Keep transient compatibility component logs after exit.
  --config PATH             Override simple_handoff_v2.yaml.
  -h, --help                Show this help.

Main logs (exactly two per run):
  logs/simple_handoff_v2/latest/first_person.log
  logs/simple_handoff_v2/latest/third_person.log
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --task) TASK="${2:?missing --task value}"; shift 2 ;;
    --motion) MOTION_ENABLED=1; shift ;;
    --dry-run) MOTION_ENABLED=0; export MAP_QWEN_DRY_RUN=1; shift ;;
    --no-slam) START_SLAM=0; shift ;;
    --no-reuse-slam) REUSE_SLAM=0; shift ;;
    --no-foxglove) START_FOXGLOVE=0; shift ;;
    --no-visited-corridor) START_VISITED_CORRIDOR_DEBUG=0; shift ;;
    --keep-raw-logs) KEEP_RAW_LOGS=1; shift ;;
    --config) SIMPLE_CFG="${2:?missing --config value}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

mkdir -p "$RUNTIME_DIR" "$LOG_BASE"
RUN_ID="$(date '+%Y%m%d_%H%M%S')"
RUN_DIR="$LOG_BASE/$RUN_ID"
mkdir -p "$RUN_DIR"
rm -rf "$LOG_BASE/latest"
ln -s "$RUN_DIR" "$LOG_BASE/latest"
FIRST_LOG="$RUN_DIR/first_person.log"
THIRD_LOG="$RUN_DIR/third_person.log"
: >"$FIRST_LOG"
: >"$THIRD_LOG"

STACK_T0="$(date +%s.%N)"
now_s() { date +%s.%N; }
elapsed_s() { awk -v s="$STACK_T0" -v e="$(now_s)" 'BEGIN {printf "%.1f", e-s}'; }
log() {
  local line="[$(date '+%H:%M:%S')][+$(elapsed_s)s] $*"
  echo "$line"
  echo "$line" >>"$THIRD_LOG"
}
fatal() {
  local line="[$(date '+%H:%M:%S')][+$(elapsed_s)s][FATAL] $*"
  echo "$line" >&2
  echo "$line" >>"$THIRD_LOG"
}

source_ros() {
  set +u
  if [[ -f /opt/tros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/tros/humble/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  else
    fatal "ROS2 Humble/TROS setup not found"
    exit 2
  fi
  [[ -f "$HOME/ydlidar_ws/install/setup.bash" ]] && source "$HOME/ydlidar_ws/install/setup.bash"
  if [[ -f "$REPO_ROOT/scripts/lib/ros_dds_env.sh" ]]; then
    # shellcheck disable=SC1090
    source "$REPO_ROOT/scripts/lib/ros_dds_env.sh"
    if declare -F prepare_ros_dds_env >/dev/null 2>&1; then
      prepare_ros_dds_env
    elif declare -F attach_ros_dds_env >/dev/null 2>&1; then
      attach_ros_dds_env
    fi
  fi
  set -u
}

for required in \
  "$SIMPLE_CFG" \
  "$BASE_SERVO_CFG" \
  "$FUSION_CFG" \
  "$BACKEND_CFG" \
  "$V1_ROOT/src/apps/simple_handoff_supervisor_v2.py" \
  "$V1_ROOT/src/apps/online_map_plan_bridge_node.py" \
  "$V1_ROOT/src/apps/online_map_qwen_nav_backend_v2.py" \
  "$V1_ROOT/src/apps/cmd_vel_intervention_mux.py" \
  "$V1_ROOT/scripts/fusion/make_simple_handoff_runtime_config_v2.py" \
  "$V1_ROOT/scripts/fusion/make_nav2_online_params_v2.py" \
  "$V1_ROOT/scripts/fusion/make_simple_handoff_visited_debug_config_v2.py" \
  "$REPO_ROOT/configs/qwen_region_explore_debug.yaml" \
  "$REPO_ROOT/src/planning/frontier_region_debug_node.py" \
  "$V1_ROOT/scripts/qwen_servo/start_live_servo_voice.sh" \
  "$REPO_ROOT/scripts/slam/run_slam_calibrated.sh" \
  "$REPO_ROOT/configs/nav2_params.yaml" \
  "$REPO_ROOT/configs/nav2_online_slam_fusion_navigation_launch_v2.py"; do
  [[ -f "$required" ]] || { fatal "missing required file: $required"; exit 2; }
done

# Load repository API defaults, then voice .env as the highest-priority source.
if [[ -f "$REPO_ROOT/.env" ]]; then
  set +u; set -a
  # shellcheck disable=SC1090
  source "$REPO_ROOT/.env"
  set +a; set -u
fi
VOICE_ROOT="${VOICE_ROOT:-$REPO_ROOT/voice_interaction}"
VOICE_ENV_FILE="${VOICE_ENV_FILE:-$VOICE_ROOT/.env}"
VOICE_RUNNER="${VOICE_RUNNER:-$VOICE_ROOT/scripts/run_voice_instruction_once_voice.sh}"
if [[ -f "$VOICE_ENV_FILE" ]]; then
  set +u; set -a
  # shellcheck disable=SC1090
  source "$VOICE_ENV_FILE"
  set +a; set -u
fi

if [[ -z "$TASK" ]]; then
  [[ -x "$VOICE_RUNNER" ]] || { fatal "voice runner missing: $VOICE_RUNNER"; exit 2; }
  VOICE_OUT="$RUNTIME_DIR/voice_instruction.txt"
  rm -f "$VOICE_OUT"
  echo "[$(date '+%H:%M:%S')][VOICE] 等待唤醒 -> 录音 -> ASR -> 英译" | tee -a "$FIRST_LOG"
  set +e
  VOICE_ENV_FILE="$VOICE_ENV_FILE" bash "$VOICE_RUNNER" --output-file "$VOICE_OUT" \
    > >(stdbuf -oL sed -u 's/^/[VOICE] /' | tee -a "$FIRST_LOG") 2>&1
  voice_rc=$?
  set -e
  [[ $voice_rc -eq 0 && -s "$VOICE_OUT" ]] || { fatal "voice pipeline failed"; exit 2; }
  TASK="$(tr '\r\n' ' ' <"$VOICE_OUT" | xargs)"
fi
[[ -n "$TASK" ]] || { fatal "empty navigation task"; exit 2; }

if [[ "$MOTION_ENABLED" == "1" && "${MAP_QWEN_DRY_RUN:-0}" != "1" ]]; then
  [[ -n "${DASHSCOPE_API_KEY:-${QWEN_API_KEY:-}}" ]] || {
    fatal "DASHSCOPE_API_KEY/QWEN_API_KEY is not configured"
    exit 2
  }
fi

source_ros
python3 - <<'PY'
import cv2, numpy, yaml
print('[preflight] Python deps: cv2/numpy/yaml OK')
PY
ros2 pkg prefix visualization_msgs >/dev/null 2>&1 || {
  fatal "ROS package visualization_msgs is missing"
  exit 2
}

# Do not let an abandoned previous fusion stack compete for the same topics.
for pattern in \
  simple_handoff_supervisor_v2.py \
  online_map_plan_bridge_node.py \
  online_map_qwen_nav_backend_v2.py \
  cmd_vel_intervention_mux.py \
  qwen_visual_servo_node.py \
  frontier_region_debug_node.py; do
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    fatal "检测到旧进程 $pattern；请先停止旧流程，避免双重发布控制命令"
    exit 3
  fi
done

log "任务：$TASK"
log "运动：$MOTION_ENABLED | dry_run=${MAP_QWEN_DRY_RUN:-0}"
log "配置：$SIMPLE_CFG"
log "主日志：$FIRST_LOG | $THIRD_LOG"

publisher_count() {
  timeout 4 ros2 topic info "$1" 2>/dev/null | awk -F': ' '/Publisher count:/ {print $2+0}' | tail -n1
}
wait_topic() {
  local topic="$1" timeout_sec="$2" start elapsed count
  start="$(date +%s)"
  while true; do
    count="$(publisher_count "$topic" || true)"
    elapsed=$(( $(date +%s) - start ))
    if [[ "${count:-0}" -gt 0 ]]; then
      log "READY topic $topic publishers=$count (${elapsed}s)"
      return 0
    fi
    (( elapsed >= timeout_sec )) && { fatal "timeout waiting $topic"; return 1; }
    sleep 1
  done
}
wait_tf() {
  local start elapsed out
  start="$(date +%s)"
  while true; do
    out="$(timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
    if grep -Eq 'Translation:|At time' <<<"$out"; then
      log "READY TF map -> base_link"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    (( elapsed >= TF_WAIT_SEC )) && { fatal "timeout waiting TF map->base_link"; return 1; }
    sleep 1
  done
}
wait_action() {
  # On RDK, ros2 action info often fails DDS discovery even after Nav2 is active.
  # Mirror fullflow_v2: also accept lifecycle "Managed nodes are active" in third_person.log.
  local action="$1" timeout_sec="$2" start elapsed info
  start="$(date +%s)"
  log "等待 action $action (timeout ${timeout_sec}s)"
  while true; do
    info="$(timeout 4 ros2 action info "$action" 2>/dev/null || true)"
    elapsed=$(( $(date +%s) - start ))
    if grep -Eq 'Action servers: [1-9]' <<<"$info"; then
      log "READY action $action (${elapsed}s)"
      return 0
    fi
    if [[ -f "$THIRD_LOG" ]] && grep -q 'Managed nodes are active' "$THIRD_LOG"; then
      log "READY action $action by nav2 lifecycle log (${elapsed}s)"
      return 0
    fi
    if (( elapsed >= timeout_sec )); then
      fatal "timeout waiting action $action after ${elapsed}s"
      [[ -n "$info" ]] && echo "$info" | tail -n 20 >&2 || true
      return 1
    fi
    (( elapsed % 10 == 0 )) && log "... action $action not ready (${elapsed}/${timeout_sec}s)"
    sleep 1
  done
}

SLAM_PID=""; NAV2_PID=""; BACKEND_PID=""; BRIDGE_PID=""; MUX_PID=""
SUPERVISOR_PID=""; EVENT_PID=""; FANIN_PID=""; VOICE_STACK_PID=""
VISITED_DEBUG_PID=""
STARTED_SLAM=0; CLEANED=0

start_logged() {
  local __var="$1" tag="$2"; shift 2
  setsid stdbuf -oL -eL "$@" \
    > >(stdbuf -oL sed -u "s/^/[$tag] /" >>"$THIRD_LOG") 2>&1 &
  local pid=$!
  printf -v "$__var" '%s' "$pid"
  log "START $tag pid=$pid"
}

kill_group() {
  local pid="${1:-}"
  [[ -z "$pid" ]] && return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

RAW_LOGS=(
  "$V1_ROOT/logs/camera.log"
  "$V1_ROOT/logs/image_raw_bridge.log"
  "$V1_ROOT/logs/qwen_servo_foxglove.log"
  "$V1_ROOT/logs/qwen_servo_cmd_vel_mux.log"
  "$V1_ROOT/logs/qwen_live_servo_qwen.log"
  "$V1_ROOT/logs/qwen_visual_servo.log"
)

cleanup() {
  [[ "$CLEANED" == "1" ]] && return 0
  CLEANED=1
  log "停止双视角全流程"
  timeout 2 ros2 topic pub --once /third_view/intervention/control_mode std_msgs/msg/String \
    "{data: '{\"mode\":\"HOLD\",\"reason\":\"launcher_shutdown\"}'}" >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub --once /cmd_vel_autonomy geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  kill_group "$VOICE_STACK_PID"
  kill_group "$EVENT_PID"
  kill_group "$SUPERVISOR_PID"
  kill_group "$MUX_PID"
  kill_group "$BRIDGE_PID"
  kill_group "$BACKEND_PID"
  kill_group "$VISITED_DEBUG_PID"
  kill_group "$NAV2_PID"
  [[ "$STARTED_SLAM" == "1" ]] && kill_group "$SLAM_PID"
  kill_group "$FANIN_PID"
  # First-person stack owns the camera (same as start_live_servo_voice.sh).
  if declare -F stop_camera_tree >/dev/null 2>&1; then
    stop_camera_tree "" || true
  else
    pkill -f '[p]ython3? -u .*/opencv_compressed_cam.py' 2>/dev/null || true
    pkill -x hobot_usb_cam 2>/dev/null || true
  fi
  if [[ "$KEEP_RAW_LOGS" != "1" ]]; then
    for f in "${RAW_LOGS[@]}"; do rm -f "$f"; done
  fi
  rm -rf "$RUNTIME_DIR"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

if ros2 action info /navigate_to_pose 2>/dev/null | grep -Eq 'Action servers: [1-9]'; then
  fatal "/navigate_to_pose already has a server; stop the old Nav2 stack first"
  exit 3
fi

# Camera is started by the stable first-person path
# (scripts/qwen_servo/start_live_servo_voice.sh -> ensure_compressed_camera).
# Match that proven profile; do not open /dev/video0 here (early open raced
# with post-voice USB settle and aborted the whole stack before SLAM).
export ROBOT_PROJECT_DIR="${ROBOT_PROJECT_DIR:-$REPO_ROOT}"
export CAMERA_BACKEND="${CAMERA_BACKEND:-opencv}"
export CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
export CAMERA_FPS="${CAMERA_FPS:-15}"
export CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
# shellcheck source=/dev/null
source "$V1_ROOT/scripts/lib/camera_stack.sh"
mkdir -p "$V1_ROOT/logs"
# Clear stale holders so the first-person stack can open cleanly later.
stop_camera_tree "" || true
log "相机交由第一视角启动 backend=$CAMERA_BACKEND ${CAMERA_WIDTH}x${CAMERA_HEIGHT}@${CAMERA_FPS} dev=$CAMERA_DEV"

map_count="$(publisher_count /map || true)"
odom_count="$(publisher_count /odom || true)"
scan_count="$(publisher_count /scan_filtered || true)"
if [[ "${map_count:-0}" -gt 0 && "${odom_count:-0}" -gt 0 && "${scan_count:-0}" -gt 0 ]]; then
  [[ "$REUSE_SLAM" == "1" ]] || { fatal "SLAM exists but reuse disabled"; exit 3; }
  log "复用现有 SLAM/传感器栈"
elif [[ "$START_SLAM" == "1" ]]; then
  start_logged SLAM_PID SLAM bash "$REPO_ROOT/scripts/slam/run_slam_calibrated.sh"
  STARTED_SLAM=1
else
  fatal "SLAM topics unavailable and --no-slam was used"
  exit 3
fi
wait_topic /map "$TOPIC_WAIT_SEC"
wait_topic /odom "$TOPIC_WAIT_SEC"
wait_topic /scan_filtered "$TOPIC_WAIT_SEC"
wait_tf

VISITED_DEBUG_CFG="$RUNTIME_DIR/qwen_region_explore_simple_handoff_v2.yaml"
VISITED_DEBUG_LOG="$RUN_DIR/visited_corridor_debug"
if [[ "$START_VISITED_CORRIDOR_DEBUG" == "1" ]]; then
  mkdir -p "$VISITED_DEBUG_LOG"
  python3 "$V1_ROOT/scripts/fusion/make_simple_handoff_visited_debug_config_v2.py" \
    --base "$REPO_ROOT/configs/qwen_region_explore_debug.yaml" \
    --simple-config "$SIMPLE_CFG" \
    --output "$VISITED_DEBUG_CFG" \
    --trajectory-file "$RUNTIME_DIR/trajectory_session.json" \
    --log-root "$VISITED_DEBUG_LOG" >>"$THIRD_LOG" 2>&1
  start_logged VISITED_DEBUG_PID VISITED \
    python3 -u "$REPO_ROOT/src/planning/frontier_region_debug_node.py" \
    --config "$VISITED_DEBUG_CFG" \
    --run-dir "$VISITED_DEBUG_LOG"
  sleep 2
  kill -0 "$VISITED_DEBUG_PID" 2>/dev/null || { fatal "visited corridor debug node exited"; exit 5; }
  wait_topic /qwen_explore_debug/map_with_visited 20
  log "已扫走廊图层：/qwen_explore_debug/map_with_visited（固定在 map 坐标系，随轨迹累积）"
else
  log "跳过 visited corridor debug（--no-visited-corridor）"
fi

NAV2_RUNTIME="$RUNTIME_DIR/nav2_params_online_v2.yaml"
SERVO_RUNTIME="$RUNTIME_DIR/qwen3_vln_servo_simple_handoff_v2.yaml"
BACKEND_RUNTIME="$RUNTIME_DIR/online_map_plan_fullflow_simple_handoff_v2.yaml"
python3 "$V1_ROOT/scripts/fusion/make_nav2_online_params_v2.py" \
  --input "$REPO_ROOT/configs/nav2_params.yaml" --output "$NAV2_RUNTIME" >>"$THIRD_LOG" 2>&1
python3 "$V1_ROOT/scripts/fusion/make_simple_handoff_runtime_config_v2.py" \
  --servo-config "$BASE_SERVO_CFG" \
  --fusion-config "$FUSION_CFG" \
  --simple-config "$SIMPLE_CFG" \
  --backend-config "$BACKEND_CFG" \
  --servo-output "$SERVO_RUNTIME" \
  --backend-output "$BACKEND_RUNTIME" \
  --backend-debug-dir "$RUNTIME_DIR/backend_debug" >>"$THIRD_LOG" 2>&1
log "运行时配置已生成，不改写稳定配置"

start_logged NAV2_PID NAV2 ros2 launch \
  "$REPO_ROOT/configs/nav2_online_slam_fusion_navigation_launch_v2.py" \
  params_file:="$NAV2_RUNTIME" use_sim_time:=false autostart:=true use_composition:=False
wait_action /navigate_to_pose "$NAV2_WAIT_SEC"

start_logged BACKEND_PID BACKEND python3 -u \
  "$V1_ROOT/src/apps/online_map_qwen_nav_backend_v2.py" \
  --config "$BACKEND_RUNTIME" --task "$TASK"
sleep 3
kill -0 "$BACKEND_PID" 2>/dev/null || { fatal "online backend exited"; exit 5; }

export THIRD_VIEW_FLOW_LOG=/dev/null
start_logged BRIDGE_PID BRIDGE python3 -u \
  "$V1_ROOT/src/apps/online_map_plan_bridge_node.py" \
  --config "$SERVO_RUNTIME" --task "$TASK"
start_logged MUX_PID HANDOFF_MUX python3 -u \
  "$V1_ROOT/src/apps/cmd_vel_intervention_mux.py" --config "$SERVO_RUNTIME"
start_logged SUPERVISOR_PID HANDOFF python3 -u \
  "$V1_ROOT/src/apps/simple_handoff_supervisor_v2.py" \
  --config "$SIMPLE_CFG" --task "$TASK"

for pid_name in BACKEND_PID BRIDGE_PID MUX_PID SUPERVISOR_PID; do
  pid="${!pid_name}"
  sleep 1
  kill -0 "$pid" 2>/dev/null || { fatal "$pid_name exited during startup"; exit 6; }
done
wait_topic /third_view/simple_handoff/status 15

setsid python3 -u "$V1_ROOT/scripts/fusion/simple_handoff_event_console_v2.py" \
  --topic /third_view/simple_handoff/event &
EVENT_PID=$!

# The stable first-person script writes fixed compatibility logs. Aggregate them
# into one main file, then remove the transient files on exit by default.
mkdir -p "$V1_ROOT/logs"
for f in "${RAW_LOGS[@]}"; do : >"$f"; done
FANIN_ARGS=(--output "$FIRST_LOG")
FANIN_ARGS+=(--source "CAMERA=$V1_ROOT/logs/camera.log")
FANIN_ARGS+=(--source "RAW_BRIDGE=$V1_ROOT/logs/image_raw_bridge.log")
FANIN_ARGS+=(--source "FOXGLOVE=$V1_ROOT/logs/qwen_servo_foxglove.log")
FANIN_ARGS+=(--source "JOY_MUX=$V1_ROOT/logs/qwen_servo_cmd_vel_mux.log")
FANIN_ARGS+=(--source "QWEN=$V1_ROOT/logs/qwen_live_servo_qwen.log")
FANIN_ARGS+=(--source "SERVO=$V1_ROOT/logs/qwen_visual_servo.log")
setsid python3 -u "$V1_ROOT/scripts/fusion/log_fan_in_v2.py" "${FANIN_ARGS[@]}" &
FANIN_PID=$!

export FOXGLOVE_TOPIC_WHITELIST="${FOXGLOVE_TOPIC_WHITELIST:-['^/image$','^/camera_info$','^/qwen_vln/annotated_image/compressed$','^/qwen_vln/servo/.*','^/qwen_vln/(command|state|result_json|latency_ms|pixel_point|prompt_text)$','^/third_view/.*','^/map_qwen_plan/(backend_debug|bridge_status|status|candidate_summary|candidate_markers|selected_goal_markers)$','^/qwen_explore_debug/map_with_visited$','^/tf$','^/tf_static$','^/scan_filtered$','^/odom$','^/map$','^/map_metadata$','^/plan$']}"

log "所有第三视角组件就绪，启动稳定第一视角"
# Same camera env as a direct start_live_servo_voice.sh run.
setsid env \
  MOTION_ENABLED="$MOTION_ENABLED" \
  SERVO_CONFIG="$SERVO_RUNTIME" \
  VOICE_INSTRUCTION_OVERRIDE="$TASK" \
  START_FOXGLOVE="$START_FOXGLOVE" \
  FOXGLOVE_TOPIC_WHITELIST="$FOXGLOVE_TOPIC_WHITELIST" \
  ROBOT_PROJECT_DIR="${ROBOT_PROJECT_DIR:-$REPO_ROOT}" \
  CAMERA_BACKEND="${CAMERA_BACKEND:-opencv}" \
  CAMERA_WIDTH="${CAMERA_WIDTH:-1280}" \
  CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}" \
  CAMERA_FPS="${CAMERA_FPS:-15}" \
  CAMERA_DEV="${CAMERA_DEV:-/dev/video0}" \
  bash "$V1_ROOT/scripts/qwen_servo/start_live_servo_voice.sh" \
  > >(stdbuf -oL sed -u 's/^/[FIRST_PERSON] /' | tee -a "$FIRST_LOG" | \
      stdbuf -oL grep -E --line-buffered '\[VOICE->NAV\]|\[step\]|\[nav\]|\[wait\]|ERROR|FATAL|stack ready|最终导航指令' || true) \
  2>&1 &
VOICE_STACK_PID=$!
log "第一视角 pid=$VOICE_STACK_PID；中转站只显示关键转换事件"
log "Foxglove 3D：Fixed frame=map；已扫走廊 /qwen_explore_debug/map_with_visited"
log "Foxglove MAP：/map_qwen_plan/candidate_markers（黄点）+ /map_qwen_plan/selected_goal_markers（红目标）"
log "Foxglove：/third_view/simple_handoff/{status,event,markers,recent_path,history_path}"

health_tick=0
while true; do
  if ! kill -0 "$VOICE_STACK_PID" 2>/dev/null; then
    wait "$VOICE_STACK_PID" || rc=$?
    rc="${rc:-0}"
    log "第一视角流程结束 rc=$rc"
    exit "$rc"
  fi
  for item in \
    "NAV2_PID:$NAV2_PID" "BACKEND_PID:$BACKEND_PID" "BRIDGE_PID:$BRIDGE_PID" \
    "MUX_PID:$MUX_PID" "SUPERVISOR_PID:$SUPERVISOR_PID" \
    "VISITED_DEBUG_PID:$VISITED_DEBUG_PID"; do
    name="${item%%:*}"; pid="${item#*:}"
    [[ -z "$pid" ]] && continue
    if ! kill -0 "$pid" 2>/dev/null; then
      fatal "$name exited unexpectedly (pid=$pid)"
      exit 7
    fi
  done
  if [[ "$STARTED_SLAM" == "1" ]] && ! kill -0 "$SLAM_PID" 2>/dev/null; then
    fatal "SLAM exited unexpectedly"
    exit 7
  fi
  health_tick=$((health_tick + 1))
  if (( health_tick % 120 == 0 )); then
    log "HEALTH all stacks alive"
  fi
  sleep 0.5
done
