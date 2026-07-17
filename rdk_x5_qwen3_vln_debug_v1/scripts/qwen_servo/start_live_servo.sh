#!/usr/bin/env bash
# Start the already-tested camera/Qwen stack plus the additive any-point servo node.
# This script intentionally does not start lidar/chassis/SLAM. Run the proven
# scripts/slam/run_slam_calibrated.sh in a separate terminal first.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_DIR="$(cd "$ROOT/.." && pwd)"
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
INSTRUCTION="${1:-find the bottle}"
load_nav_api_env "$PROJECT_DIR" "$ROOT"
: "${DASHSCOPE_API_KEY:?Please export DASHSCOPE_API_KEY first}"

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

# Reuse the same camera -> bgr8 bridge proven by the current V1.1 stack.
ensure_compressed_camera "$ROOT" "$ROOT/logs/camera.log"
start_raw_bridge "$ROOT" "$ROOT/logs/image_raw_bridge.log"

if [[ "${START_FOXGLOVE:-1}" == "1" ]]; then
  port="${FOXGLOVE_PORT:-8765}"
  whitelist="${FOXGLOVE_TOPIC_WHITELIST:-['^/image$','^/camera_info$','^/qwen_vln/annotated_image/compressed$','^/qwen_vln/servo/.*','^/qwen_vln/(command|state|result_json|latency_ms|pixel_point|prompt_text)$','^/third_view/.*','^/map_qwen_plan/(backend_debug|bridge_status|status|candidate_summary)$','^/tf$','^/tf_static$','^/scan_filtered$','^/odom$','^/map$','^/map_metadata$']}"
  if ss -tln 2>/dev/null | grep -q ":${port} "; then
    echo "[foxglove] reuse port $port"
  elif ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    FOXGLOVE_PORT="$port" FOXGLOVE_TOPIC_WHITELIST="$whitelist" \
      bash "$PROJECT_DIR/scripts/lidar/start_foxglove.sh" \
      >"$ROOT/logs/qwen_servo_foxglove.log" 2>&1 &
    FOXGLOVE_PID=$!
    echo "[foxglove] starting on ws://:$port whitelist=$whitelist"
  else
    echo "[foxglove] WARN: foxglove_bridge package not installed"
  fi
fi

# Reuse the same joy > autonomy priority mux used by EXP2. It is harmless when
# no joystick is connected and preserves the option for manual override.
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

# start_debug_node.sh honors QWEN_CONFIG. The tuned prompts/client/FSM remain
# untouched; the generated config changes only request intervals.
QWEN_CONFIG="$FAST_CONFIG" \
  bash "$ROOT/scripts/start_debug_node.sh" "$INSTRUCTION" \
  >"$ROOT/logs/qwen_live_servo_qwen.log" 2>&1 &
QWEN_PID=$!

QWEN_LOG="$ROOT/logs/qwen_live_servo_qwen.log"
QWEN_WAIT_MAX="${QWEN_WAIT_MAX:-60}"
if ! wait_qwen_debug_ready "$QWEN_PID" "$QWEN_LOG" "$QWEN_WAIT_MAX"; then
  echo "ERROR: Qwen node did not become ready within ${QWEN_WAIT_MAX}s" >&2
  tail -n 100 "$QWEN_LOG" || true
  exit 1
fi

SERVO_ARGS=(--config "${SERVO_CONFIG:-$ROOT/configs/qwen3_vln_servo.yaml}")
if [[ "${MOTION_ENABLED:-0}" == "1" ]]; then
  SERVO_ARGS+=(--enable-motion)
fi
python3 -u "$ROOT/src/apps/qwen_visual_servo_node.py" "${SERVO_ARGS[@]}" \
  >"$ROOT/logs/qwen_visual_servo.log" 2>&1 &
SERVO_PID=$!

sleep 1
cat <<EOF
===== Qwen3-VL visual servo V2: ANY VALID PIXEL =====
Task: $INSTRUCTION
Motion: ${MOTION_ENABLED:-0}  (0=dry run, 1=real chassis)
Expected base topics: /scan_filtered and a subscriber on /cmd_vel
Qwen image: /qwen_vln/annotated_image/compressed
Servo status: /qwen_vln/servo/status
Point policy: TARGET_VISIBLE / SEARCH_HINT / VERIFY_SUCCESS all drive when point exists
Raw command: /qwen_vln/servo/cmd_raw
Limited command: /qwen_vln/servo/cmd_limited
Real clipped command (existing bridge): /cmd_vel_sent
Front distance: /qwen_vln/servo/front_distance

Enable/disable while running:
  ros2 topic pub --once /qwen_vln/servo/command std_msgs/msg/String "{data: 'enable'}"
  ros2 topic pub --once /qwen_vln/servo/command std_msgs/msg/String "{data: 'disable'}"
EOF

echo "Press Ctrl+C to stop this visual servo stack."
wait "$QWEN_PID" || true
