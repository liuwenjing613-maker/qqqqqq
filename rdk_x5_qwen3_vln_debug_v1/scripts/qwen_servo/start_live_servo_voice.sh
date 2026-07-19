#!/usr/bin/env bash
# Voice-triggered version of start_live_servo.sh.
# Flow: wait for wake word -> record 10s -> Qwen ASR -> English translation ->
# start the existing Qwen visual-servo navigation stack with that instruction.
#
# As with the original script, lidar/chassis/SLAM are not started here. Start the
# proven /root/rdk_x5_vln_robot/scripts/slam/run_slam_calibrated.sh beforehand.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_DIR="$(cd "$ROOT/.." && pwd)"
VOICE_ROOT="${VOICE_ROOT:-$PROJECT_DIR/voice_interaction}"
VOICE_RUNNER="${VOICE_RUNNER:-$VOICE_ROOT/scripts/run_voice_instruction_once_voice.sh}"
VOICE_ENV_FILE="${VOICE_ENV_FILE:-$VOICE_ROOT/.env}"

source "$ROOT/scripts/lib/ros_env.sh"
source "$ROOT/scripts/lib/camera_stack.sh"
source "$ROOT/scripts/lib/qwen_ready.sh"
source "$ROOT/scripts/lib/nav_api_env.sh"

if [[ -f "$PROJECT_DIR/scripts/lib/ros_dds_env.sh" ]]; then
  # shellcheck disable=SC1091
  source "$PROJECT_DIR/scripts/lib/ros_dds_env.sh"
  attach_ros_dds_env
fi

mkdir -p "$ROOT/logs"

if [[ ! -d "$VOICE_ROOT" ]]; then
  echo "ERROR: voice module directory missing: $VOICE_ROOT" >&2
  exit 1
fi
if [[ ! -x "$VOICE_RUNNER" ]]; then
  echo "ERROR: voice runner missing or not executable: $VOICE_RUNNER" >&2
  exit 1
fi
if [[ ! -f "$VOICE_ENV_FILE" ]]; then
  echo "ERROR: voice .env missing: $VOICE_ENV_FILE" >&2
  exit 1
fi

# Load voice .env first (ASR/mic), then nav API loader.
# Voice .env is passed as the highest-priority override for the three API exports:
# DASHSCOPE_API_KEY / QWEN_BASE_URL / QWEN_MODEL (beats shell and repo .env).
set -a
# shellcheck disable=SC1090
source "$VOICE_ENV_FILE"
set +a
load_nav_api_env "$PROJECT_DIR" "$ROOT" "$VOICE_ENV_FILE"
: "${DASHSCOPE_API_KEY:?Please configure DASHSCOPE_API_KEY in $VOICE_ENV_FILE}"
: "${QWEN_BASE_URL:?Please configure QWEN_BASE_URL in $VOICE_ENV_FILE}"
: "${QWEN_MODEL:?Please configure QWEN_MODEL in $VOICE_ENV_FILE}"

# The voice command itself is the final user confirmation, so the voice version
# enables real motion by default. Use MOTION_ENABLED=0 for a dry run.
export MOTION_ENABLED="${MOTION_ENABLED:-1}"
export VOICE_RECORD_SECONDS="${VOICE_RECORD_SECONDS:-10}"

VOICE_INSTRUCTION_FILE="$ROOT/logs/voice_instruction_current.txt"
rm -f "$VOICE_INSTRUCTION_FILE"

cat <<EOF

===== Voice-triggered Qwen3-VL navigation =====
Voice root: $VOICE_ROOT
Wake flow : 小车你好 -> 提示音 -> ${VOICE_RECORD_SECONDS}s 录音 -> ASR -> 英译
Motion    : $MOTION_ENABLED (0=dry run, 1=real chassis)
Status    : 正在等待语音指令；成功后自动启动导航

EOF

if [[ -n "${VOICE_INSTRUCTION_OVERRIDE:-}" ]]; then
  # Hardware-free integration test path. It is never used unless explicitly set.
  printf '%s\n' "$VOICE_INSTRUCTION_OVERRIDE" > "$VOICE_INSTRUCTION_FILE"
  echo "[VOICE->NAV][DEBUG] 使用 VOICE_INSTRUCTION_OVERRIDE"
else
  VOICE_ENV_FILE="$VOICE_ENV_FILE" \
    bash "$VOICE_RUNNER" --output-file "$VOICE_INSTRUCTION_FILE"
fi

if [[ ! -s "$VOICE_INSTRUCTION_FILE" ]]; then
  echo "ERROR: voice pipeline exited without a valid instruction" >&2
  exit 1
fi

INSTRUCTION="$(tr '\r\n' '  ' < "$VOICE_INSTRUCTION_FILE" | xargs)"
if [[ -z "$INSTRUCTION" ]]; then
  echo "ERROR: normalized voice instruction is empty" >&2
  exit 1
fi

echo
echo "============================================================"
echo "[VOICE->NAV] 最终导航指令：$INSTRUCTION"
echo "[VOICE->NAV] 语音阶段完成，开始启动相机、Qwen 与视觉伺服。"
echo "============================================================"
echo

BASE_CONFIG="${QWEN_BASE_CONFIG:-$ROOT/configs/qwen3_vln_debug.yaml}"
FAST_CONFIG="${QWEN_FAST_CONFIG:-$ROOT/configs/qwen3_vln_debug_servo_fast.yaml}"
python3 "$ROOT/scripts/qwen_servo/make_fast_qwen_config.py" \
  --input "$BASE_CONFIG" --output "$FAST_CONFIG" \
  --observe "${QWEN_OBSERVE_INTERVAL:-0.9}" \
  --track "${QWEN_TRACK_INTERVAL:-0.85}" \
  --search "${QWEN_SEARCH_INTERVAL:-0.90}"

CAMERA_PID=""
BRIDGE_PID=""
QWEN_PID=""
SERVO_PID=""
MUX_PID=""
FOXGLOVE_PID=""
CLEANUP_DONE=0
REUSED_QWEN=0
REUSED_CAMERA=0
REUSE_QWEN_PREWARM="${REUSE_QWEN_PREWARM:-0}"
if [[ "$REUSE_QWEN_PREWARM" == "1" ]]; then
  # Hub owns camera/bridge for the whole online session when prewarm is requested.
  REUSED_CAMERA=1
fi

# Startup timing helpers: wall clock + per-process age while waiting.
STACK_T0="$(date +%s.%N)"
STEP_T0="$STACK_T0"
declare -A PROC_T0=()

now_s() { date +%s.%N; }

elapsed_s() {
  local start="${1:-$STACK_T0}"
  awk -v s="$start" -v e="$(now_s)" 'BEGIN { printf "%.1f", e - s }'
}

log_ts() {
  echo "[$(date '+%H:%M:%S')][+$(elapsed_s "$STACK_T0")s] $*"
}

mark_step() {
  local name="$1"
  local took
  took="$(elapsed_s "$STEP_T0")"
  log_ts "[step] $name (took ${took}s since previous step)"
  STEP_T0="$(now_s)"
}

mark_proc() {
  local name="$1"
  local pid="${2:-}"
  PROC_T0["$name"]="$(now_s)"
  if [[ -n "$pid" ]]; then
    log_ts "[proc] start $name pid=$pid"
  else
    log_ts "[proc] start $name (reuse/external)"
  fi
}

proc_status_line() {
  local name="$1"
  local pid="${2:-}"
  local age="-"
  if [[ -n "${PROC_T0[$name]:-}" ]]; then
    age="$(elapsed_s "${PROC_T0[$name]}")s"
  fi
  if [[ -z "$pid" ]]; then
    printf '%s=n/a(%s)' "$name" "$age"
  elif kill -0 "$pid" 2>/dev/null; then
    printf '%s=alive(pid=%s,age=%s)' "$name" "$pid" "$age"
  else
    printf '%s=DEAD(pid=%s,age=%s)' "$name" "$pid" "$age"
  fi
}

log_proc_snapshot() {
  local parts=()
  parts+=("$(proc_status_line camera "${CAMERA_PID:-}")")
  parts+=("$(proc_status_line bridge "${BRIDGE_PID:-}")")
  parts+=("$(proc_status_line foxglove "${FOXGLOVE_PID:-}")")
  parts+=("$(proc_status_line mux "${MUX_PID:-}")")
  parts+=("$(proc_status_line qwen "${QWEN_PID:-}")")
  parts+=("$(proc_status_line servo "${SERVO_PID:-}")")
  log_ts "[procs] ${parts[*]}"
}

kill_pid_tree() {
  local pid="$1"
  [[ -z "$pid" ]] && return 0
  if ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi
  local child
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    kill_pid_tree "$child"
  done
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 10); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  if [[ "$CLEANUP_DONE" == "1" ]]; then
    return 0
  fi
  CLEANUP_DONE=1
  echo "[cleanup] publish zero and stop processes started here"
  timeout 3 ros2 topic pub --once /qwen_vln/servo/command std_msgs/msg/String \
    "{data: 'disable'}" >/dev/null 2>&1 || true
  # Leave hub-owned prewarm camera/qwen alive for the next online command.
  if [[ "$REUSED_QWEN" == "1" ]]; then
    timeout 3 ros2 topic pub --once /qwen_vln/command std_msgs/msg/String \
      "{data: 'pause'}" >/dev/null 2>&1 || true
  fi
  for pid in "$SERVO_PID" "$FOXGLOVE_PID" "$MUX_PID"; do
    kill_pid_tree "$pid"
  done
  if [[ "$REUSED_QWEN" != "1" ]]; then
    kill_pid_tree "$QWEN_PID"
  fi
  if [[ "$REUSED_CAMERA" != "1" ]]; then
    kill_pid_tree "$BRIDGE_PID"
    stop_camera_tree "${CAMERA_PID:-}" || true
  fi
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

# Reuse the tested camera -> bgr8 bridge stack.
STACK_T0="$(now_s)"
STEP_T0="$STACK_T0"
log_ts "[nav] begin camera / bridge / mux / qwen / servo startup"

# Prefer process/log evidence over ros2 CLI: under Nav2 bringup, topic info
# often flakes to 0 publishers even while prewarm opencv is healthy.
prewarm_camera_alive=0
if [[ "$REUSE_QWEN_PREWARM" == "1" ]] && opencv_camera_process_alive; then
  if camera_topic_has_publisher /image || camera_log_shows_live_frames "$ROOT/logs/camera.log" 15; then
    if camera_topic_has_publisher /image_raw || pgrep -f '[p]ython3? -u .*/compressed_to_raw_image\.py' >/dev/null 2>&1; then
      prewarm_camera_alive=1
    fi
  fi
fi

if [[ "$prewarm_camera_alive" == "1" ]]; then
  log_ts "[camera] REUSE_QWEN_PREWARM=1: keep hub prewarm camera/bridge"
  REUSED_CAMERA=1
  mark_proc camera ""
  mark_step "camera ready (reused)"
  mark_proc bridge ""
  mark_step "image_raw bridge ready (reused)"
else
  if [[ "$REUSE_QWEN_PREWARM" == "1" ]]; then
    log_ts "[camera] WARN prewarm camera missing; cold-starting camera path"
    REUSED_CAMERA=0
  fi
  ensure_compressed_camera "$ROOT" "$ROOT/logs/camera.log"
  mark_proc camera "${CAMERA_PID:-}"
  mark_step "camera ready"
  start_raw_bridge "$ROOT" "$ROOT/logs/image_raw_bridge.log"
  mark_proc bridge "${BRIDGE_PID:-}"
  mark_step "image_raw bridge ready"
fi

if [[ "${START_FOXGLOVE:-1}" == "1" ]]; then
  port="${FOXGLOVE_PORT:-8765}"
  whitelist="${FOXGLOVE_TOPIC_WHITELIST:-['^/image$','^/camera_info$','^/qwen_vln/annotated_image/compressed$','^/qwen_vln/servo/.*','^/qwen_vln/(command|state|result_json|latency_ms|pixel_point|prompt_text)$','^/third_view/.*','^/map_qwen_plan/(backend_debug|bridge_status|status|candidate_summary)$','^/tf$','^/tf_static$','^/scan_filtered$','^/odom$','^/map$','^/map_metadata$']}"
  if ss -tln 2>/dev/null | grep -q ":${port} "; then
    mark_proc foxglove ""
    echo "[foxglove] reuse port $port"
  elif ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    FOXGLOVE_PORT="$port" FOXGLOVE_TOPIC_WHITELIST="$whitelist" \
      bash "$PROJECT_DIR/scripts/lidar/start_foxglove.sh" \
      >"$ROOT/logs/qwen_servo_foxglove.log" 2>&1 &
    FOXGLOVE_PID=$!
    mark_proc foxglove "$FOXGLOVE_PID"
    echo "[foxglove] starting on ws://:$port whitelist=$whitelist"
  else
    echo "[foxglove] WARN: foxglove_bridge package not installed"
  fi
  mark_step "foxglove checked"
fi

# Reuse the joy > autonomy priority mux used by the current V1 stack.
if [[ "${START_CMD_VEL_MUX:-1}" == "1" ]]; then
  if pgrep -f "cmd_vel_priority_mux.py" >/dev/null 2>&1; then
    mark_proc mux ""
    echo "[mux] reuse existing cmd_vel_priority_mux.py"
  elif [[ -f "$PROJECT_DIR/scripts/control/cmd_vel_priority_mux.py" ]]; then
    python3 -u "$PROJECT_DIR/scripts/control/cmd_vel_priority_mux.py" \
      --autonomy-topic /cmd_vel_autonomy \
      --joy-cmd-topic /cmd_vel_joy \
      --output-topic /cmd_vel \
      --joy-topic /joy \
      --axis-linear "${JOY_AXIS_LINEAR:-1}" \
      --axis-angular "${JOY_AXIS_ANGULAR:-0}" \
      --joy-deadzone "${JOY_DEADZONE:-0.08}" \
      >"$ROOT/logs/qwen_servo_cmd_vel_mux.log" 2>&1 &
    MUX_PID=$!
    mark_proc mux "$MUX_PID"
    echo "[mux] /cmd_vel_autonomy -> /cmd_vel"
  else
    echo "ERROR: cmd_vel mux missing: $PROJECT_DIR/scripts/control/cmd_vel_priority_mux.py" >&2
    exit 1
  fi
  mark_step "mux ready"
fi

# The English instruction produced by the voice stage is passed exactly through
# the same positional interface used by the original start_live_servo.sh.
qwen_prewarm_marker() {
  local runtime="${QWEN_PREWARM_RUNTIME_DIR:-/tmp/rdk_x5_voice_demo_qwen_prewarm_${USER:-robot}}"
  echo "$runtime/ready.marker"
}

qwen_debug_node_running() {
  pgrep -f 'qwen_vln_debug_node\.py' >/dev/null 2>&1
}

qwen_prewarm_ready() {
  # Hub marker is authoritative: process may be PAUSED (no fresh result_json traffic).
  local marker
  marker="$(qwen_prewarm_marker)"
  if [[ -f "$marker" ]] && grep -q 'ready' "$marker" 2>/dev/null && qwen_debug_node_running; then
    return 0
  fi
  if qwen_debug_node_running; then
    if timeout 5 ros2 topic info /qwen_vln/result_json 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
      return 0
    fi
  fi
  [[ -f "$ROOT/logs/qwen_live_servo_qwen.log" ]] && grep -q 'started model=' "$ROOT/logs/qwen_live_servo_qwen.log" && qwen_debug_node_running
}

if [[ "$REUSE_QWEN_PREWARM" == "1" ]] && qwen_prewarm_ready; then
  log_ts "[qwen] REUSE_QWEN_PREWARM=1: reuse qwen3_vln_debug_node"
  REUSED_QWEN=1
  timeout 3 ros2 topic pub --once /qwen_vln/instruction std_msgs/msg/String \
    "{data: \"${INSTRUCTION//\"/\\\"}\"}" >/dev/null 2>&1 || true
  timeout 3 ros2 topic pub --once /qwen_vln/command std_msgs/msg/String \
    "{data: 'resume'}" >/dev/null 2>&1 || true
  mark_proc qwen ""
  mark_step "qwen /qwen_vln/result_json ready (reused)"
elif [[ "$REUSE_QWEN_PREWARM" == "1" ]] && qwen_debug_node_running; then
  # Marker/topic flaky, but node is already up — never start a second Qwen.
  log_ts "[qwen] REUSE_QWEN_PREWARM=1: qwen process present; resume without cold-start"
  REUSED_QWEN=1
  timeout 3 ros2 topic pub --once /qwen_vln/instruction std_msgs/msg/String \
    "{data: \"${INSTRUCTION//\"/\\\"}\"}" >/dev/null 2>&1 || true
  timeout 3 ros2 topic pub --once /qwen_vln/command std_msgs/msg/String \
    "{data: 'resume'}" >/dev/null 2>&1 || true
  mark_proc qwen ""
  mark_step "qwen /qwen_vln/result_json ready (reused process)"
else
  if [[ "$REUSE_QWEN_PREWARM" == "1" ]]; then
    log_ts "[qwen] WARN prewarm requested but not ready; cold-starting"
    REUSED_CAMERA=0
  fi
  log_ts "[qwen] launching start_debug_node.sh (log: $ROOT/logs/qwen_live_servo_qwen.log)"
  QWEN_CONFIG="$FAST_CONFIG" \
    bash "$ROOT/scripts/start_debug_node.sh" "$INSTRUCTION" \
    >"$ROOT/logs/qwen_live_servo_qwen.log" 2>&1 &
  QWEN_PID=$!
  mark_proc qwen "$QWEN_PID"

  QWEN_LOG="$ROOT/logs/qwen_live_servo_qwen.log"
  QWEN_WAIT_MAX="${QWEN_WAIT_MAX:-60}"

  _qwen_wait_tick() {
    local attempt="$1"
    local max_attempts="$2"
    log_ts "[wait] try ${attempt}/${max_attempts} qwen not ready yet (qwen age $(elapsed_s "${PROC_T0[qwen]}")s)"
    log_proc_snapshot
  }

  if ! wait_qwen_debug_ready "$QWEN_PID" "$QWEN_LOG" "$QWEN_WAIT_MAX" \
      /qwen_vln/result_json _qwen_wait_tick; then
    log_ts "[wait] FAILED after ${QWEN_WAIT_MAX}s / total $(elapsed_s "$STACK_T0")s"
    log_proc_snapshot
    echo "ERROR: Qwen node did not become ready within ${QWEN_WAIT_MAX}s" >&2
    tail -n 100 "$QWEN_LOG" || true
    exit 1
  fi
  log_ts "[wait] qwen ready (qwen age $(elapsed_s "${PROC_T0[qwen]}")s)"
  log_proc_snapshot
  mark_step "qwen /qwen_vln/result_json ready"
fi

SERVO_ARGS=(--config "${SERVO_CONFIG:-$ROOT/configs/qwen3_vln_servo.yaml}")
if [[ "$MOTION_ENABLED" == "1" ]]; then
  SERVO_ARGS+=(--enable-motion)
fi

log_ts "[servo] launching qwen_visual_servo_node.py"
python3 -u "$ROOT/src/apps/qwen_visual_servo_node.py" "${SERVO_ARGS[@]}" \
  >"$ROOT/logs/qwen_visual_servo.log" 2>&1 &
SERVO_PID=$!
mark_proc servo "$SERVO_PID"

sleep 1
if ! kill -0 "$SERVO_PID" 2>/dev/null; then
  log_ts "[servo] FAILED during startup"
  log_proc_snapshot
  echo "ERROR: visual servo node exited during startup" >&2
  tail -n 100 "$ROOT/logs/qwen_visual_servo.log" || true
  exit 1
fi
mark_step "servo process alive"
log_proc_snapshot
log_ts "[nav] stack ready in $(elapsed_s "$STACK_T0")s total"

cat <<EOF

===== Qwen3-VL visual servo: VOICE AUTO START =====
Task: $INSTRUCTION
Motion: $MOTION_ENABLED (0=dry run, 1=real chassis)
Expected base topics: /scan_filtered and a subscriber on /cmd_vel
Qwen image: /qwen_vln/annotated_image/compressed
Servo status: /qwen_vln/servo/status
Raw command: /qwen_vln/servo/cmd_raw
Limited command: /qwen_vln/servo/cmd_limited
Real clipped command: /cmd_vel_sent
Front distance: /qwen_vln/servo/front_distance
Voice transcript file: $VOICE_INSTRUCTION_FILE
Enable/disable while running:
  ros2 topic pub --once /qwen_vln/servo/command std_msgs/msg/String "{data: 'enable'}"
  ros2 topic pub --once /qwen_vln/servo/command std_msgs/msg/String "{data: 'disable'}"
EOF

echo "Press Ctrl+C to stop this voice-triggered visual servo stack."
if [[ -n "${QWEN_PID:-}" ]]; then
  wait "$QWEN_PID" || true
else
  # Reused hub prewarm: keep first-person alive with the servo process.
  wait "$SERVO_PID" || true
fi
