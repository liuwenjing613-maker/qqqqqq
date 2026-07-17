#!/usr/bin/env bash
# One-command real full flow:
# live calibrated SLAM -> navigation-only Nav2 -> real online map/Qwen backend
# -> existing bridge/intervention/mux -> proven V1 first-person navigation.
set -Eeuo pipefail

V1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPO_ROOT="$(cd "$V1_ROOT/.." && pwd)"
FULL_CFG="${FULLFLOW_CONFIG:-$V1_ROOT/configs/online_map_plan_fullflow_v2.yaml}"
FUSION_CFG="${FULLFLOW_FUSION_CONFIG:-$V1_ROOT/configs/online_map_plan_fusion_fullflow_v2.yaml}"
BASE_SERVO_CFG="${FULLFLOW_BASE_SERVO_CONFIG:-$V1_ROOT/configs/qwen3_vln_servo.yaml}"
RUNTIME_DIR="${FULLFLOW_RUNTIME_DIR:-/tmp/rdk_x5_vln_fullflow_v2_${USER:-robot}}"
LOG_DIR="$V1_ROOT/logs/fullflow_v2"
PID_FILE="$RUNTIME_DIR/pids.env"
TASK="${QWEN_TASK:-find the bottle}"
MOTION_ENABLED="${MOTION_ENABLED:-0}"
START_SLAM="${FULLFLOW_START_SLAM:-1}"
REUSE_SLAM="${FULLFLOW_REUSE_SLAM:-1}"
TF_WAIT_SEC="${FULLFLOW_TF_WAIT_SEC:-60}"
TOPIC_WAIT_SEC="${FULLFLOW_TOPIC_WAIT_SEC:-60}"
NAV2_WAIT_SEC="${FULLFLOW_NAV2_WAIT_SEC:-90}"
# Fullflow saturates the X5: bridge debug/status + compressed images, skip raw image/map blobs.
export FOXGLOVE_TOPIC_WHITELIST="${FOXGLOVE_TOPIC_WHITELIST:-['^/image$','^/camera_info$','^/qwen_vln/annotated_image/compressed$','^/qwen_vln/servo/.*','^/qwen_vln/(command|state|result_json|latency_ms|pixel_point|prompt_text)$','^/third_view/.*','^/map_qwen_plan/(backend_debug|bridge_status|status|candidate_summary)$','^/tf$','^/tf_static$','^/scan_filtered$','^/odom$','^/map$','^/map_metadata$']}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --task) TASK="${2:?missing --task value}"; shift 2 ;;
    --motion) MOTION_ENABLED=1; shift ;;
    --dry-run) MOTION_ENABLED=0; export MAP_QWEN_DRY_RUN=1; shift ;;
    --no-slam) START_SLAM=0; shift ;;
    --no-reuse-slam) REUSE_SLAM=0; shift ;;
    -h|--help)
      cat <<EOF
Usage:
  MOTION_ENABLED=1 bash ${BASH_SOURCE[0]} --task 'find the bottle'
  bash ${BASH_SOURCE[0]} --dry-run --task 'find the bottle'

Environment:
  DASHSCOPE_API_KEY       required for real map-Qwen selection
  MAP_QWEN_DRY_RUN=1      use deterministic geometric fallback
  FULLFLOW_START_SLAM=0   attach to an already-running SLAM stack
  FULLFLOW_TF_WAIT_SEC    TF map->base_link wait (default 60)
  FULLFLOW_TOPIC_WAIT_SEC /map /odom /scan_filtered wait (default 60)
  FULLFLOW_NAV2_WAIT_SEC  /navigate_to_pose wait (default 90)
EOF
      exit 0 ;;
    *)
      if [[ "$TASK" == "find the bottle" ]]; then TASK="$1"; shift; else echo "Unknown argument: $1" >&2; exit 2; fi ;;
  esac
done

mkdir -p "$RUNTIME_DIR" "$LOG_DIR"

STACK_T0="$(date +%s.%N)"
STEP_T0="$STACK_T0"
declare -A PROC_T0=()

now_s() { date +%s.%N; }
elapsed_s() {
  local start="${1:-$STACK_T0}"
  awk -v s="$start" -v e="$(now_s)" 'BEGIN { printf "%.1f", e - s }'
}
log_ts() { echo "[$(date '+%H:%M:%S')][+$(elapsed_s "$STACK_T0")s] $*"; }
log_fatal() { echo "[$(date '+%H:%M:%S')][+$(elapsed_s "$STACK_T0")s][FATAL] $*" >&2; }
mark_step() {
  local name="$1" took
  took="$(elapsed_s "$STEP_T0")"
  log_ts "[step] DONE: $name (took ${took}s since previous step)"
  STEP_T0="$(now_s)"
}
mark_proc() {
  local name="$1" pid="${2:-}"
  PROC_T0["$name"]="$(now_s)"
  if [[ -n "$pid" ]]; then log_ts "[proc] start $name pid=$pid"; else log_ts "[proc] start $name (reuse/external)"; fi
}
proc_status_line() {
  local name="$1" pid="${2:-}" age="-"
  [[ -n "${PROC_T0[$name]:-}" ]] && age="$(elapsed_s "${PROC_T0[$name]}")s"
  if [[ -z "$pid" ]]; then printf '%s=n/a(%s)' "$name" "$age"
  elif kill -0 "$pid" 2>/dev/null; then printf '%s=alive(pid=%s,age=%s)' "$name" "$pid" "$age"
  else printf '%s=DEAD(pid=%s,age=%s)' "$name" "$pid" "$age"; fi
}
log_proc_snapshot() {
  log_ts "[procs] $(proc_status_line slam "${SLAM_PID:-}") $(proc_status_line nav2 "${NAV2_PID:-}") $(proc_status_line backend "${BACKEND_PID:-}") $(proc_status_line fusion "${FUSION_PID:-}")"
}

source_ros() {
  log_ts "[env] sourcing ROS2 / TROS ..."
  set +u
  if [[ -f /opt/tros/humble/setup.bash ]]; then source /opt/tros/humble/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash
  else log_fatal "ROS2 Humble/TROS setup not found"; exit 2; fi
  [[ -f "$HOME/ydlidar_ws/install/setup.bash" ]] && source "$HOME/ydlidar_ws/install/setup.bash"
  [[ -f "$REPO_ROOT/scripts/lib/ros_dds_env.sh" ]] && source "$REPO_ROOT/scripts/lib/ros_dds_env.sh"
  if declare -F prepare_ros_dds_env >/dev/null 2>&1; then prepare_ros_dds_env; fi
  set -u
  mark_step "ROS env ready"
}
source_ros

log_ts "===== FULLFLOW_V2 begin ====="
log_ts "task='$TASK' motion=$MOTION_ENABLED dry_run=${MAP_QWEN_DRY_RUN:-0}"
log_ts "start_slam=$START_SLAM reuse_slam=$REUSE_SLAM"
log_ts "tf_wait=${TF_WAIT_SEC}s topic_wait=${TOPIC_WAIT_SEC}s nav2_wait=${NAV2_WAIT_SEC}s"

for f in \
  "$FULL_CFG" "$FUSION_CFG" "$BASE_SERVO_CFG" \
  "$REPO_ROOT/scripts/slam/run_slam_calibrated.sh" \
  "$REPO_ROOT/configs/nav2_params.yaml" \
  "$REPO_ROOT/configs/nav2_online_slam_fusion_navigation_launch_v2.py" \
  "$V1_ROOT/src/apps/online_map_qwen_nav_backend_v2.py" \
  "$V1_ROOT/scripts/fusion/start_v1_online_map_plan_fusion.sh" \
  "$V1_ROOT/scripts/fusion/make_online_fusion_runtime_config.py"; do
  [[ -f "$f" ]] || { log_fatal "missing $f"; exit 2; }
done
mark_step "file checks"

# Repo .env owns the three API exports with highest priority (overrides shell).
if [[ -f "$REPO_ROOT/.env" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$REPO_ROOT/.env"
  set +a
fi
if [[ "$MOTION_ENABLED" == "1" && -z "${DASHSCOPE_API_KEY:-${QWEN_API_KEY:-}}" && "${MAP_QWEN_DRY_RUN:-0}" != "1" ]]; then
  log_fatal "set DASHSCOPE_API_KEY in $REPO_ROOT/.env (or shell before launch)"
  exit 2
fi
: "${QWEN_BASE_URL:=${DASHSCOPE_BASE_URL:-https://dashscope.aliyuncs.com/compatible-mode/v1}}"
: "${QWEN_MODEL:=qwen3-vl-flash}"
export QWEN_BASE_URL QWEN_MODEL
log_ts "[api] model=$QWEN_MODEL base_url=$QWEN_BASE_URL key_set=$([ -n "${DASHSCOPE_API_KEY:-}" ] && echo 1 || echo 0)"
mark_step "API key check"

publisher_count() {
  timeout 4 ros2 topic info "$1" 2>/dev/null | awk -F': ' '/Publisher count:/ {print $2+0}' | tail -n1
}
wait_topic() {
  local topic="$1" timeout_sec="$2" start elapsed count
  start="$(date +%s)"
  log_ts "[wait] begin topic $topic (timeout ${timeout_sec}s)"
  while true; do
    count="$(publisher_count "$topic" || true)"
    elapsed=$(( $(date +%s) - start ))
    if [[ "${count:-0}" -gt 0 ]]; then
      log_ts "[wait] READY topic $topic publishers=$count (elapsed ${elapsed}s)"
      return 0
    fi
    if (( elapsed >= timeout_sec )); then
      log_fatal "timeout waiting $topic after ${elapsed}s"
      return 1
    fi
    (( elapsed % 5 == 0 )) && log_ts "[wait] ... $topic not ready (${elapsed}/${timeout_sec}s)"
    sleep 1
  done
}
wait_action() {
  local action="$1" timeout_sec="$2" nav2_log="${3:-}" start elapsed info
  start="$(date +%s)"
  log_ts "[wait] begin action $action (timeout ${timeout_sec}s)"
  while true; do
    info="$(timeout 4 ros2 action info "$action" 2>/dev/null || true)"
    elapsed=$(( $(date +%s) - start ))
    if grep -Eq 'Action servers: [1-9]' <<<"$info"; then
      log_ts "[wait] READY action $action (elapsed ${elapsed}s)"
      return 0
    fi
    if [[ -n "$nav2_log" && -f "$nav2_log" ]] && grep -q 'Managed nodes are active' "$nav2_log"; then
      log_ts "[wait] READY action $action by nav2 lifecycle log (elapsed ${elapsed}s)"
      return 0
    fi
    if (( elapsed >= timeout_sec )); then
      log_fatal "timeout waiting action $action after ${elapsed}s"
      echo "$info" | tail -n 20 >&2 || true
      return 1
    fi
    (( elapsed % 5 == 0 )) && log_ts "[wait] ... action $action not ready (${elapsed}/${timeout_sec}s)"
    sleep 1
  done
}
wait_tf_map_base() {
  local timeout_sec="$1" start elapsed out
  start="$(date +%s)"
  log_ts "[wait] begin TF map -> base_link (timeout ${timeout_sec}s)"
  while true; do
    elapsed=$(( $(date +%s) - start ))
    out="$(timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
    if grep -Eq 'Translation:|At time' <<<"$out"; then
      log_ts "[wait] READY TF map -> base_link (elapsed ${elapsed}s)"
      return 0
    fi
    if (( elapsed >= timeout_sec )); then
      log_fatal "TF map -> base_link unavailable after ${elapsed}s"
      echo "$out" | tail -n 20 >&2
      return 1
    fi
    (( elapsed % 5 == 0 )) && log_ts "[wait] ... TF not ready (${elapsed}/${timeout_sec}s)"
    sleep 1
  done
}

SLAM_PID=""; NAV2_PID=""; BACKEND_PID=""; FUSION_PID=""; STARTED_SLAM=0; CLEANED=0
RESTARTED_SLAM_AFTER_TF_FAIL=0

start_owned_slam_stack() {
  log_ts "[slam] starting calibrated live SLAM -> $LOG_DIR/slam.log"
  setsid bash "$REPO_ROOT/scripts/slam/run_slam_calibrated.sh" >"$LOG_DIR/slam.log" 2>&1 &
  SLAM_PID=$!; STARTED_SLAM=1
  mark_proc slam "$SLAM_PID"
}

kill_group() {
  local pid="${1:-}"; [[ -z "$pid" ]] && return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
cleanup() {
  [[ "$CLEANED" == 1 ]] && return 0
  CLEANED=1
  log_ts "[cleanup] stopping full flow (runtime $(elapsed_s "$STACK_T0")s)"
  log_proc_snapshot
  timeout 2 ros2 topic pub --once /third_view/intervention/control_mode std_msgs/msg/String \
    "{data: '{\"mode\":\"HOLD\",\"reason\":\"fullflow_shutdown\"}'}" >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub --once /cmd_vel_autonomy geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  kill_group "$FUSION_PID"; kill_group "$BACKEND_PID"; kill_group "$NAV2_PID"
  if [[ "$STARTED_SLAM" == 1 ]]; then kill_group "$SLAM_PID"; fi
  rm -f "$PID_FILE"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

if ros2 action info /navigate_to_pose 2>/dev/null | grep -Eq 'Action servers: [1-9]'; then
  log_fatal "/navigate_to_pose already has a server"; exit 3
fi
mark_step "Nav2 conflict check"

MAP_COUNT="$(publisher_count /map || true)"
ODOM_COUNT="$(publisher_count /odom || true)"
SCAN_COUNT="$(publisher_count /scan_filtered || true)"
log_ts "[slam] publishers: /map=$MAP_COUNT /odom=$ODOM_COUNT /scan_filtered=$SCAN_COUNT"

if [[ "${MAP_COUNT:-0}" -gt 0 && "${ODOM_COUNT:-0}" -gt 0 && "${SCAN_COUNT:-0}" -gt 0 ]]; then
  if [[ "$REUSE_SLAM" == 1 ]]; then
    log_ts "[slam] reusing existing live SLAM/sensor stack"
    mark_proc slam ""
  else
    log_fatal "live map stack exists and reuse is disabled"; exit 3
  fi
elif [[ "$START_SLAM" == 1 ]]; then
  start_owned_slam_stack
else
  log_fatal "/map,/odom,/scan_filtered are not ready and --no-slam was used"; exit 3
fi
mark_step "SLAM launch decision"

wait_topic /map "$TOPIC_WAIT_SEC"
wait_topic /odom "$TOPIC_WAIT_SEC"
wait_topic /scan_filtered "$TOPIC_WAIT_SEC"
mark_step "sensor topics ready"

if ! wait_tf_map_base "$TF_WAIT_SEC"; then
  if [[ "$STARTED_SLAM" == "0" && "$START_SLAM" == "1" && "$RESTARTED_SLAM_AFTER_TF_FAIL" == "0" ]]; then
    RESTARTED_SLAM_AFTER_TF_FAIL=1
    log_ts "[recovery] reused SLAM has broken TF; starting owned clean SLAM once"
    start_owned_slam_stack
    wait_topic /map "$TOPIC_WAIT_SEC"
    wait_topic /odom "$TOPIC_WAIT_SEC"
    wait_topic /scan_filtered "$TOPIC_WAIT_SEC"
    wait_tf_map_base "$TF_WAIT_SEC" || exit 4
  else
    exit 4
  fi
fi
mark_step "TF map->base_link ready"

NAV2_RUNTIME="$RUNTIME_DIR/nav2_params_online_v2.yaml"
SERVO_RUNTIME="$RUNTIME_DIR/qwen3_vln_servo_fullflow_v2.yaml"
python3 "$V1_ROOT/scripts/fusion/make_nav2_online_params_v2.py" \
  --input "$REPO_ROOT/configs/nav2_params.yaml" --output "$NAV2_RUNTIME" >/dev/null
python3 "$V1_ROOT/scripts/fusion/make_fullflow_servo_config_v2.py" \
  --input "$BASE_SERVO_CFG" --output "$SERVO_RUNTIME" >/dev/null
mark_step "runtime configs generated"

log_ts "[nav2] starting navigation-only Nav2 -> $LOG_DIR/nav2.log"
setsid ros2 launch "$REPO_ROOT/configs/nav2_online_slam_fusion_navigation_launch_v2.py" \
  params_file:="$NAV2_RUNTIME" use_sim_time:=false autostart:=true use_composition:=False \
  >"$LOG_DIR/nav2.log" 2>&1 &
NAV2_PID=$!; mark_proc nav2 "$NAV2_PID"
wait_action /navigate_to_pose "$NAV2_WAIT_SEC" "$LOG_DIR/nav2.log" || { tail -n 80 "$LOG_DIR/nav2.log" >&2; exit 5; }
mark_step "Nav2 ready"

log_ts "[backend] starting -> $LOG_DIR/backend.log"
setsid python3 -u "$V1_ROOT/src/apps/online_map_qwen_nav_backend_v2.py" \
  --config "$FULL_CFG" --task "$TASK" >"$LOG_DIR/backend.log" 2>&1 &
BACKEND_PID=$!; mark_proc backend "$BACKEND_PID"
for _ in $(seq 1 5); do
  sleep 1
  if ! kill -0 "$BACKEND_PID" 2>/dev/null; then
    log_fatal "backend exited during startup"
    tail -n 200 "$LOG_DIR/backend.log" >&2 || true
    exit 5
  fi
done
if ! grep -q 'fullflow V2 backend ready' "$LOG_DIR/backend.log" 2>/dev/null; then
  log_ts "[backend] WARN: ready line not seen yet; continuing with process alive check"
fi
mark_step "backend started"

cat > "$PID_FILE" <<EOF
SLAM_PID=$SLAM_PID
STARTED_SLAM=$STARTED_SLAM
NAV2_PID=$NAV2_PID
BACKEND_PID=$BACKEND_PID
FUSION_PID=
EOF

log_ts "[fusion] starting V1 fusion stack -> $LOG_DIR/fusion_v1.log"
FLOW_LOG="${THIRD_VIEW_FLOW_LOG:-$V1_ROOT/logs/third_view_flow.log}"
export THIRD_VIEW_FLOW_LOG="$FLOW_LOG"
mkdir -p "$(dirname "$FLOW_LOG")"
: >"$FLOW_LOG"
log_ts "[fusion] 第三视角流程日志 -> $FLOW_LOG (tail -f 查看)"
setsid env MOTION_ENABLED="$MOTION_ENABLED" SERVO_CONFIG="$SERVO_RUNTIME" \
  FUSION_CONFIG="$FUSION_CFG" FUSION_RUNTIME_CONFIG="$RUNTIME_DIR/fusion_runtime_v2.yaml" \
  THIRD_VIEW_FLOW_LOG="$FLOW_LOG" \
  bash "$V1_ROOT/scripts/fusion/start_v1_online_map_plan_fusion.sh" "$TASK" \
  >"$LOG_DIR/fusion_v1.log" 2>&1 &
FUSION_PID=$!; mark_proc fusion "$FUSION_PID"
sed -i "s/^FUSION_PID=.*/FUSION_PID=$FUSION_PID/" "$PID_FILE"
for _ in $(seq 1 8); do
  sleep 1
  if ! kill -0 "$FUSION_PID" 2>/dev/null; then
    log_fatal "fusion stack exited during startup"
    tail -n 240 "$LOG_DIR/fusion_v1.log" >&2 || true
    exit 6
  fi
done
mark_step "fusion stack started"

log_ts "===== FULLFLOW_V2 ALL STACKS STARTED in $(elapsed_s "$STACK_T0")s ====="
cat <<EOF

[FULLFLOW_V2] ALL STACKS STARTED
  task   : $TASK
  motion : $MOTION_ENABLED
  logs   : $LOG_DIR
  第三视角流程日志: $FLOW_LOG
  实时查看: tail -f $FLOW_LOG

Keep this terminal open. Ctrl+C stops every process started by this launcher.
EOF

HEALTH_TICK=0
while true; do
  for pair in "NAV2:$NAV2_PID:$LOG_DIR/nav2.log" "BACKEND:$BACKEND_PID:$LOG_DIR/backend.log" "FUSION:$FUSION_PID:$LOG_DIR/fusion_v1.log"; do
    IFS=: read -r name pid log <<< "$pair"
    if ! kill -0 "$pid" 2>/dev/null; then
      log_fatal "$name exited (pid=$pid)"
      tail -n 240 "$log" >&2 || true
      exit 7
    fi
  done
  if [[ "$STARTED_SLAM" == 1 ]] && ! kill -0 "$SLAM_PID" 2>/dev/null; then
    log_fatal "SLAM exited (pid=$SLAM_PID)"
    tail -n 240 "$LOG_DIR/slam.log" >&2 || true
    exit 7
  fi
  HEALTH_TICK=$((HEALTH_TICK + 1))
  if (( HEALTH_TICK % 60 == 0 )); then
    log_ts "[health] all stacks alive (uptime $(elapsed_s "$STACK_T0")s)"
    log_proc_snapshot
  fi
  sleep 0.5
done
