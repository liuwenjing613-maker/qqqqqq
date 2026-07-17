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

# Load the same key for ASR/translation and visual navigation. Unlike the old
# script, this file contains no hard-coded cloud credential.
set -a
# shellcheck disable=SC1090
source "$VOICE_ENV_FILE"
set +a
: "${DASHSCOPE_API_KEY:?Please configure DASHSCOPE_API_KEY in $VOICE_ENV_FILE}"

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
  for pid in "$SERVO_PID" "$QWEN_PID" "$BRIDGE_PID" "$FOXGLOVE_PID" "$MUX_PID"; do
    kill_pid_tree "$pid"
  done
  stop_camera_tree "${CAMERA_PID:-}" || true
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

# Reuse the tested camera -> bgr8 bridge stack.
ensure_compressed_camera "$ROOT" "$ROOT/logs/camera.log"
start_raw_bridge "$ROOT" "$ROOT/logs/image_raw_bridge.log"

if [[ "${START_FOXGLOVE:-1}" == "1" ]]; then
  port="${FOXGLOVE_PORT:-8765}"
  if ss -tln 2>/dev/null | grep -q ":${port} "; then
    echo "[foxglove] reuse port $port"
  elif ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="$port" \
      >"$ROOT/logs/qwen_servo_foxglove.log" 2>&1 &
    FOXGLOVE_PID=$!
    echo "[foxglove] starting on ws://:$port"
  else
    echo "[foxglove] WARN: foxglove_bridge package not installed"
  fi
fi

# Reuse the joy > autonomy priority mux used by the current V1 stack.
if [[ "${START_CMD_VEL_MUX:-1}" == "1" ]]; then
  if pgrep -f "cmd_vel_priority_mux.py" >/dev/null 2>&1; then
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
    echo "[mux] /cmd_vel_autonomy -> /cmd_vel"
  else
    echo "ERROR: cmd_vel mux missing: $PROJECT_DIR/scripts/control/cmd_vel_priority_mux.py" >&2
    exit 1
  fi
fi

# The English instruction produced by the voice stage is passed exactly through
# the same positional interface used by the original start_live_servo.sh.
QWEN_CONFIG="$FAST_CONFIG" \
  bash "$ROOT/scripts/start_debug_node.sh" "$INSTRUCTION" \
  >"$ROOT/logs/qwen_live_servo_qwen.log" 2>&1 &
QWEN_PID=$!

for _ in $(seq 1 30); do
  if ros2 topic info /qwen_vln/result_json 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
    break
  fi
  if ! kill -0 "$QWEN_PID" 2>/dev/null; then
    echo "ERROR: Qwen node exited" >&2
    tail -n 100 "$ROOT/logs/qwen_live_servo_qwen.log" || true
    exit 1
  fi
  sleep 1
done

if ! ros2 topic info /qwen_vln/result_json 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
  echo "ERROR: /qwen_vln/result_json did not become ready within 30 seconds" >&2
  tail -n 100 "$ROOT/logs/qwen_live_servo_qwen.log" || true
  exit 1
fi

SERVO_ARGS=(--config "${SERVO_CONFIG:-$ROOT/configs/qwen3_vln_servo.yaml}")
if [[ "$MOTION_ENABLED" == "1" ]]; then
  SERVO_ARGS+=(--enable-motion)
fi

python3 -u "$ROOT/src/apps/qwen_visual_servo_node.py" "${SERVO_ARGS[@]}" \
  >"$ROOT/logs/qwen_visual_servo.log" 2>&1 &
SERVO_PID=$!

sleep 1
if ! kill -0 "$SERVO_PID" 2>/dev/null; then
  echo "ERROR: visual servo node exited during startup" >&2
  tail -n 100 "$ROOT/logs/qwen_visual_servo.log" || true
  exit 1
fi

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
wait "$QWEN_PID" || true
