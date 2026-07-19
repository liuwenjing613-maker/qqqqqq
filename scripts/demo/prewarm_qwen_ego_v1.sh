#!/usr/bin/env bash
# Boot-time ego Qwen prewarm for voice_demo_hub_v1.
# Starts camera -> image_raw bridge -> qwen3_vln_debug_node, waits until ready,
# then PAUSE so idle hub does not keep spending API tokens. Online mode reuses
# this stack via REUSE_QWEN_PREWARM=1.
set -Eeuo pipefail

REPO_ROOT="${ROBOT_PROJECT_DIR:-/root/rdk_x5_vln_robot}"
V1_ROOT="${V1_ROOT:-$REPO_ROOT/rdk_x5_qwen3_vln_debug_v1}"
VOICE_ENV_FILE="${VOICE_ENV_FILE:-$REPO_ROOT/voice_interaction/.env}"
INSTRUCTION="${QWEN_PREWARM_INSTRUCTION:-find the bottle}"
RUNTIME_DIR="${QWEN_PREWARM_RUNTIME_DIR:-/tmp/rdk_x5_voice_demo_qwen_prewarm_${USER:-robot}}"
WAIT_SEC="${QWEN_PREWARM_WAIT_SEC:-90}"

mkdir -p "$RUNTIME_DIR" "$V1_ROOT/logs"
MARKER="$RUNTIME_DIR/ready.marker"
PID_FILE="$RUNTIME_DIR/pids.env"
# Never leave an empty marker — hub treats "ready" in the file as success.
rm -f "$MARKER"
: >"$PID_FILE"

source "$V1_ROOT/scripts/lib/ros_env.sh"
source "$V1_ROOT/scripts/lib/camera_stack.sh"
source "$V1_ROOT/scripts/lib/qwen_ready.sh"
source "$V1_ROOT/scripts/lib/nav_api_env.sh"

if [[ -f "$REPO_ROOT/scripts/lib/ros_dds_env.sh" ]]; then
  # shellcheck disable=SC1091
  source "$REPO_ROOT/scripts/lib/ros_dds_env.sh"
  attach_ros_dds_env
fi

if [[ -f "$VOICE_ENV_FILE" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$VOICE_ENV_FILE"
  set +a
fi
load_nav_api_env "$REPO_ROOT" "$V1_ROOT" "$VOICE_ENV_FILE"
: "${DASHSCOPE_API_KEY:?Please configure DASHSCOPE_API_KEY in $VOICE_ENV_FILE}"

export ROBOT_PROJECT_DIR="${ROBOT_PROJECT_DIR:-$REPO_ROOT}"
export CAMERA_BACKEND="${CAMERA_BACKEND:-opencv}"
export CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
export CAMERA_FPS="${CAMERA_FPS:-15}"
export CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"

CAMERA_PID=""
BRIDGE_PID=""
QWEN_PID=""
CLEANED=0

log() { echo "[$(date '+%H:%M:%S')] [qwen-prewarm] $*"; }

cleanup() {
  [[ "$CLEANED" == "1" ]] && return 0
  CLEANED=1
  log "stopping prewarm stack"
  rm -f "$MARKER"
  for pid in "$QWEN_PID" "$BRIDGE_PID"; do
    [[ -n "${pid:-}" ]] || continue
    kill -TERM "$pid" 2>/dev/null || true
  done
  stop_camera_tree "${CAMERA_PID:-}" || true
  for pid in "$QWEN_PID" "$BRIDGE_PID"; do
    [[ -n "${pid:-}" ]] || continue
    wait "$pid" 2>/dev/null || true
  done
  rm -f "$PID_FILE"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

BASE_CONFIG="${QWEN_BASE_CONFIG:-$V1_ROOT/configs/qwen3_vln_debug.yaml}"
FAST_CONFIG="${QWEN_FAST_CONFIG:-$V1_ROOT/configs/qwen3_vln_debug_servo_fast.yaml}"
python3 "$V1_ROOT/scripts/qwen_servo/make_fast_qwen_config.py" \
  --input "$BASE_CONFIG" --output "$FAST_CONFIG" \
  --observe "${QWEN_OBSERVE_INTERVAL:-0.9}" \
  --track "${QWEN_TRACK_INTERVAL:-0.85}" \
  --search "${QWEN_SEARCH_INTERVAL:-0.90}"

CAMERA_LOG="$V1_ROOT/logs/camera.log"
BRIDGE_LOG="$V1_ROOT/logs/image_raw_bridge.log"
QWEN_LOG="$V1_ROOT/logs/qwen_live_servo_qwen.log"

log "instruction=$INSTRUCTION"
log "camera backend=$CAMERA_BACKEND ${CAMERA_WIDTH}x${CAMERA_HEIGHT}@${CAMERA_FPS} dev=$CAMERA_DEV"
ensure_compressed_camera "$V1_ROOT" "$CAMERA_LOG"
start_raw_bridge "$V1_ROOT" "$BRIDGE_LOG"

log "starting qwen3_vln_debug_node"
: >"$QWEN_LOG"
QWEN_CONFIG="$FAST_CONFIG" \
  bash "$V1_ROOT/scripts/start_debug_node.sh" "$INSTRUCTION" \
  >"$QWEN_LOG" 2>&1 &
QWEN_PID=$!

if ! wait_qwen_debug_ready "$QWEN_PID" "$QWEN_LOG" "$WAIT_SEC" /qwen_vln/result_json; then
  log "ERROR: qwen failed to become ready within ${WAIT_SEC}s"
  tail -n 80 "$QWEN_LOG" || true
  exit 1
fi

# Freeze inference while hub is idle so prewarm does not burn API quota.
timeout 3 ros2 topic pub --once /qwen_vln/command std_msgs/msg/String \
  "{data: 'pause'}" >/dev/null 2>&1 || true

cat >"$PID_FILE" <<EOF
CAMERA_PID=${CAMERA_PID:-}
BRIDGE_PID=${BRIDGE_PID:-}
QWEN_PID=${QWEN_PID:-}
INSTRUCTION=$(printf '%q' "$INSTRUCTION")
EOF
printf 'ready ts=%s instruction=%s\n' "$(date -Iseconds)" "$INSTRUCTION" >"$MARKER"
log "READY camera+qwen paused (reuse with REUSE_QWEN_PREWARM=1)"

# Stay alive so the hub can manage this process group.
while true; do
  if ! kill -0 "$QWEN_PID" 2>/dev/null; then
    log "ERROR: qwen process exited"
    exit 1
  fi
  sleep 2
done
