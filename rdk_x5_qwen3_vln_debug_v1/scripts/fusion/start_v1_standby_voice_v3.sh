#!/usr/bin/env bash
# Scheme 2: warm heavy components first; start task-bound light nodes after voice.
# This is a new launcher. It does not modify or invoke the old all-in-one launcher.
set -Eeuo pipefail

V1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPO_ROOT="$(cd "$V1_ROOT/.." && pwd)"

QWEN_BASE_CFG="${QWEN_BASE_CONFIG:-$V1_ROOT/configs/qwen3_vln_debug.yaml}"
SIMPLE_BASE_CFG="${SIMPLE_HANDOFF_CONFIG:-$V1_ROOT/configs/simple_handoff_v2.yaml}"
BASE_SERVO_CFG="${SIMPLE_HANDOFF_BASE_SERVO_CONFIG:-$V1_ROOT/configs/qwen3_vln_servo.yaml}"
FUSION_CFG="${SIMPLE_HANDOFF_FUSION_CONFIG:-$V1_ROOT/configs/online_map_plan_fusion_fullflow_v2.yaml}"
BACKEND_CFG="${SIMPLE_HANDOFF_BACKEND_CONFIG:-$V1_ROOT/configs/online_map_plan_fullflow_v2.yaml}"

RUNTIME_BASE="${V1_STANDBY_RUNTIME_BASE:-/tmp/rdk_x5_v1_standby_v3_${USER:-robot}}"
RUN_ID="$(date '+%Y%m%d_%H%M%S')"
RUNTIME_DIR="$RUNTIME_BASE/$RUN_ID"
LOG_DIR="$V1_ROOT/logs/v1_standby_v3/$RUN_ID"
LOCK_FILE="${V1_STANDBY_LOCK_FILE:-/tmp/rdk_x5_v1_standby_v3.lock}"

TASK="${QWEN_TASK:-}"
MOTION_ENABLED=0
START_SLAM=1
REUSE_SLAM=1
REUSE_NAV2=0
START_FOXGLOVE="${START_FOXGLOVE:-1}"
TOPIC_WAIT_SEC="${V1_STANDBY_TOPIC_WAIT_SEC:-60}"
TF_WAIT_SEC="${V1_STANDBY_TF_WAIT_SEC:-60}"
NAV2_WAIT_SEC="${V1_STANDBY_NAV2_WAIT_SEC:-90}"
QWEN_WAIT_SEC="${QWEN_WAIT_MAX:-90}"
VOICE_READY_SEC="${V1_STANDBY_VOICE_READY_SEC:-90}"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/fusion/start_v1_standby_voice_v3.sh [options]

Options:
  --motion             Enable real chassis motion after all mission gates pass.
  --dry-run            Keep servo disabled; backend uses dry-run. This is default.
  --task TEXT          Bypass wake word/ASR for staged testing.
  --no-slam            Require existing /map, /odom and /scan_filtered.
  --no-reuse-slam      Refuse an already-running complete SLAM/sensor stack.
  --reuse-nav2         Explicitly reuse an existing /navigate_to_pose server.
                       Default is to refuse it, preventing accidental old goals.
  --no-foxglove        Do not start Foxglove.
  -h, --help           Show this help.

The script never pkill's an existing stack. Conflicting processes cause a fail-fast.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --motion) MOTION_ENABLED=1; shift ;;
    --dry-run) MOTION_ENABLED=0; shift ;;
    --task) TASK="${2:?missing --task value}"; shift 2 ;;
    --no-slam) START_SLAM=0; shift ;;
    --no-reuse-slam) REUSE_SLAM=0; shift ;;
    --reuse-nav2) REUSE_NAV2=1; shift ;;
    --no-foxglove) START_FOXGLOVE=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

mkdir -p "$RUNTIME_DIR" "$LOG_DIR"
rm -rf "$RUNTIME_BASE/latest"
ln -s "$RUNTIME_DIR" "$RUNTIME_BASE/latest"
mkdir -p "$V1_ROOT/logs/v1_standby_v3"
rm -rf "$V1_ROOT/logs/v1_standby_v3/latest"
ln -s "$LOG_DIR" "$V1_ROOT/logs/v1_standby_v3/latest"

MAIN_LOG="$LOG_DIR/main.log"
VOICE_LOG="$LOG_DIR/voice.log"
SLAM_LOG="$LOG_DIR/slam.log"
NAV2_LOG="$LOG_DIR/nav2.log"
CAMERA_LOG="$LOG_DIR/camera.log"
RAW_BRIDGE_LOG="$LOG_DIR/image_raw_bridge.log"
FOXGLOVE_LOG="$LOG_DIR/foxglove.log"
PRIORITY_MUX_LOG="$LOG_DIR/cmd_vel_priority_mux.log"
QWEN_LOG="$LOG_DIR/qwen.log"
INTERVENTION_MUX_LOG="$LOG_DIR/intervention_mux.log"
SERVO_LOG="$LOG_DIR/servo.log"
BACKEND_LOG="$LOG_DIR/backend.log"
BRIDGE_LOG="$LOG_DIR/map_bridge.log"
SUPERVISOR_LOG="$LOG_DIR/supervisor.log"
GATE_LOG="$LOG_DIR/mission_gate.log"
: >"$MAIN_LOG"

log() {
  local line="[$(date '+%H:%M:%S')] $*"
  echo "$line"
  echo "$line" >>"$MAIN_LOG"
}

fatal() {
  local line="[$(date '+%H:%M:%S')][FATAL] $*"
  echo "$line" >&2
  echo "$line" >>"$MAIN_LOG"
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

source_ros
# shellcheck source=/dev/null
source "$V1_ROOT/scripts/lib/camera_stack.sh"
# shellcheck source=/dev/null
source "$V1_ROOT/scripts/lib/qwen_ready.sh"
# shellcheck source=/dev/null
source "$V1_ROOT/scripts/lib/nav_api_env.sh"

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
load_nav_api_env "$REPO_ROOT" "$V1_ROOT" "$VOICE_ENV_FILE"
: "${DASHSCOPE_API_KEY:?DASHSCOPE_API_KEY is required for Qwen warmup}"
: "${QWEN_BASE_URL:?QWEN_BASE_URL is required}"
: "${QWEN_MODEL:?QWEN_MODEL is required}"

export ROBOT_PROJECT_DIR="${ROBOT_PROJECT_DIR:-$REPO_ROOT}"
export CAMERA_BACKEND="${CAMERA_BACKEND:-opencv}"
export CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-480}"
export CAMERA_FPS="${CAMERA_FPS:-15}"
export CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
export MAP_QWEN_DRY_RUN="$(( MOTION_ENABLED == 1 ? 0 : 1 ))"

required_files=(
  "$QWEN_BASE_CFG"
  "$SIMPLE_BASE_CFG"
  "$BASE_SERVO_CFG"
  "$FUSION_CFG"
  "$BACKEND_CFG"
  "$V1_ROOT/scripts/qwen_servo/make_fast_qwen_config.py"
  "$V1_ROOT/scripts/fusion/make_v1_standby_runtime_v3.py"
  "$V1_ROOT/scripts/fusion/activate_standby_mission_v3.py"
  "$V1_ROOT/scripts/fusion/make_nav2_online_params_v2.py"
  "$V1_ROOT/scripts/fusion/make_simple_handoff_runtime_config_v2.py"
  "$V1_ROOT/scripts/start_debug_node.sh"
  "$V1_ROOT/src/apps/qwen_visual_servo_node.py"
  "$V1_ROOT/src/apps/cmd_vel_intervention_mux.py"
  "$V1_ROOT/src/apps/online_map_qwen_nav_backend_v2.py"
  "$V1_ROOT/src/apps/online_map_plan_bridge_node.py"
  "$V1_ROOT/src/apps/simple_handoff_supervisor_v2.py"
  "$REPO_ROOT/scripts/slam/run_slam_calibrated.sh"
  "$REPO_ROOT/configs/nav2_params.yaml"
  "$REPO_ROOT/configs/nav2_online_slam_fusion_navigation_launch_v2.py"
  "$REPO_ROOT/scripts/control/cmd_vel_priority_mux.py"
)
for path in "${required_files[@]}"; do
  [[ -f "$path" ]] || { fatal "missing required file: $path"; exit 2; }
done
if [[ -z "$TASK" ]]; then
  [[ -x "$VOICE_RUNNER" ]] || { fatal "voice runner missing: $VOICE_RUNNER"; exit 2; }
  [[ -f "$VOICE_ENV_FILE" ]] || { fatal "voice env missing: $VOICE_ENV_FILE"; exit 2; }
fi

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  fatal "another standby-v3 launcher holds $LOCK_FILE"
  exit 3
fi

conflict_patterns=(
  '[s]tart_v1_map_qwen_simple_voice_v2.sh'
  '[s]tart_live_servo_voice.sh'
  '[q]wen_vln_debug_node.py'
  '[q]wen_visual_servo_node.py'
  '[c]md_vel_intervention_mux.py'
  '[o]nline_map_qwen_nav_backend_v2.py'
  '[o]nline_map_plan_bridge_node.py'
  '[s]imple_handoff_supervisor_v2.py'
  '[r]un_voice_instruction_once_voice.sh'
  '[k]ws_instruction_once_voice.py'
)
for pattern in "${conflict_patterns[@]}"; do
  matches="$(pgrep -af "$pattern" 2>/dev/null | awk -v self="$$" '$1 != self' || true)"
  if [[ -n "$matches" ]]; then
    fatal "conflicting process detected; nothing was killed: $matches"
    exit 3
  fi
done

# Fail closed on unknown velocity publishers. The joystick path may publish
# /cmd_vel_joy; autonomous publishers must not already exist.
for topic in /cmd_vel_autonomy /cmd_vel_ego /cmd_vel_map; do
  existing="$(timeout 4 ros2 topic info "$topic" 2>/dev/null | awk -F': ' '/Publisher count:/ {print $2+0}' | tail -n1 || true)"
  if [[ "${existing:-0}" -gt 0 ]]; then
    fatal "existing publisher on $topic; refusing to add a second controller"
    exit 3
  fi
done

publisher_count() {
  timeout 4 ros2 topic info "$1" 2>/dev/null |
    awk -F': ' '/Publisher count:/ {print $2+0}' | tail -n1
}

wait_topic() {
  local topic="$1" timeout_sec="$2" start count
  start="$(date +%s)"
  while true; do
    count="$(publisher_count "$topic" || true)"
    if [[ "${count:-0}" -gt 0 ]]; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      fatal "timeout waiting for publisher: $topic"
      return 1
    fi
    sleep 1
  done
}

wait_tf() {
  local start out
  start="$(date +%s)"
  while true; do
    out="$(timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
    if grep -Eq 'Translation:|At time' <<<"$out"; then
      return 0
    fi
    if (( $(date +%s) - start >= TF_WAIT_SEC )); then
      fatal "timeout waiting TF map -> base_link"
      return 1
    fi
    sleep 1
  done
}

nav2_present() {
  local nodes processes
  if timeout 4 ros2 action info /navigate_to_pose 2>/dev/null | grep -Eq 'Action servers: [1-9]'; then
    return 0
  fi
  nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
  if grep -Eq '^/(bt_navigator|controller_server|planner_server|behavior_server|waypoint_follower|velocity_smoother)$' <<<"$nodes"; then
    return 0
  fi
  processes="$(pgrep -af 'nav2_online_slam_fusion_navigation_launch_v2.py|[b]t_navigator|[c]ontroller_server|[p]lanner_server|[n]av2_container' 2>/dev/null || true)"
  [[ -n "$processes" ]]
}

wait_action() {
  local action="$1" timeout_sec="$2" log_file="${3:-}" start info
  start="$(date +%s)"
  while true; do
    info="$(timeout 4 ros2 action info "$action" 2>/dev/null || true)"
    if grep -Eq 'Action servers: [1-9]' <<<"$info"; then
      return 0
    fi
    if [[ -n "$log_file" && -f "$log_file" ]] && grep -q 'Managed nodes are active' "$log_file"; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      fatal "timeout waiting action: $action"
      return 1
    fi
    sleep 1
  done
}

wait_log_or_exit() {
  local pid="$1" file="$2" regex="$3" timeout_sec="$4" start
  start="$(date +%s)"
  while true; do
    if [[ -f "$file" ]] && grep -Eq "$regex" "$file"; then
      return 0
    fi
    if ! kill -0 "$pid" 2>/dev/null; then
      fatal "process pid=$pid exited before readiness; tail $file"
      tail -n 80 "$file" >&2 2>/dev/null || true
      return 1
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      fatal "timeout waiting log readiness: $regex"
      tail -n 80 "$file" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

start_group() {
  local __var="$1" tag="$2" logfile="$3"
  shift 3
  : >"$logfile"
  setsid stdbuf -oL -eL "$@" >"$logfile" 2>&1 &
  local pid=$!
  printf -v "$__var" '%s' "$pid"
  echo "[$tag] pid=$pid log=$logfile" >>"$MAIN_LOG"
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

kill_tree() {
  local pid="${1:-}" child
  [[ -z "$pid" ]] && return 0
  kill -0 "$pid" 2>/dev/null || return 0
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    kill_tree "$child"
  done
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

publish_safe() {
  timeout 3 ros2 topic pub --once /qwen_vln/servo/command std_msgs/msg/String \
    "{data: 'disable'}" >/dev/null 2>&1 || true
  timeout 3 ros2 topic pub --once /third_view/intervention/control_mode std_msgs/msg/String \
    "{data: '{\"mode\":\"HOLD\",\"reason\":\"standby_launcher_safe\"}'}" \
    >/dev/null 2>&1 || true
  timeout 3 ros2 topic pub --once /cmd_vel_autonomy geometry_msgs/msg/Twist '{}' \
    >/dev/null 2>&1 || true
}

VOICE_PID=""
SLAM_PID=""
NAV2_PID=""
CAMERA_PID=""
RAW_BRIDGE_PID=""
FOXGLOVE_PID=""
PRIORITY_MUX_PID=""
QWEN_PID=""
INTERVENTION_MUX_PID=""
SERVO_PID=""
BACKEND_PID=""
BRIDGE_PID=""
SUPERVISOR_PID=""
OWN_CAMERA=0
OWN_RAW_BRIDGE=0
CLEANED=0

cleanup() {
  [[ "$CLEANED" == "1" ]] && return 0
  CLEANED=1
  publish_safe
  kill_group "$SUPERVISOR_PID"
  kill_group "$BRIDGE_PID"
  kill_group "$BACKEND_PID"
  kill_group "$SERVO_PID"
  kill_group "$INTERVENTION_MUX_PID"
  kill_group "$QWEN_PID"
  kill_group "$PRIORITY_MUX_PID"
  kill_group "$FOXGLOVE_PID"
  [[ "$OWN_RAW_BRIDGE" == "1" ]] && kill_tree "$RAW_BRIDGE_PID"
  [[ "$OWN_CAMERA" == "1" ]] && kill_tree "$CAMERA_PID"
  kill_group "$NAV2_PID"
  kill_group "$SLAM_PID"
  kill_group "$VOICE_PID"
  log "STOP complete; reused external processes were not touched"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

# Start the one-shot KWS process before opening the camera. This ensures USB mic
# setup has completed first. The operator must still wait for final READY.
VOICE_OUT="$RUNTIME_DIR/voice_instruction.txt"
if [[ -z "$TASK" ]]; then
  rm -f "$VOICE_OUT"
  start_group VOICE_PID VOICE "$VOICE_LOG" env \
    VOICE_ENV_FILE="$VOICE_ENV_FILE" \
    bash "$VOICE_RUNNER" --output-file "$VOICE_OUT"
  wait_log_or_exit "$VOICE_PID" "$VOICE_LOG" \
    '单次语音导航入口已启动' "$VOICE_READY_SEC"
  sleep "${V1_STANDBY_MIC_SETTLE_SEC:-2}"
  log "[READY] 语音监听已初始化；请等总 READY 后再说“小车你好”"
else
  log "[READY] 使用 --task，跳过语音硬件"
fi

# Reuse only a complete base stack; partial discovery fails closed.
map_count="$(publisher_count /map || true)"
odom_count="$(publisher_count /odom || true)"
scan_count="$(publisher_count /scan_filtered || true)"
if [[ "${map_count:-0}" -gt 0 && "${odom_count:-0}" -gt 0 && "${scan_count:-0}" -gt 0 ]]; then
  [[ "$REUSE_SLAM" == "1" ]] || { fatal "complete SLAM stack exists but reuse is disabled"; exit 3; }
  log "复用现有 SLAM/传感器栈，不接管其进程"
elif [[ "${map_count:-0}" -eq 0 && "${odom_count:-0}" -eq 0 && "${scan_count:-0}" -eq 0 ]]; then
  [[ "$START_SLAM" == "1" ]] || { fatal "base topics absent and --no-slam was used"; exit 3; }
  start_group SLAM_PID SLAM "$SLAM_LOG" bash "$REPO_ROOT/scripts/slam/run_slam_calibrated.sh"
else
  fatal "partial base stack detected: /map=$map_count /odom=$odom_count /scan_filtered=$scan_count; refusing duplicate startup"
  exit 3
fi
wait_topic /map "$TOPIC_WAIT_SEC"
wait_topic /odom "$TOPIC_WAIT_SEC"
wait_topic /scan_filtered "$TOPIC_WAIT_SEC"
wait_tf
log "[READY] 雷达 / 里程计 / SLAM / TF"

# Runtime-only configuration copies.
QWEN_FAST_BASE="$RUNTIME_DIR/qwen_fast_base.yaml"
QWEN_STANDBY="$RUNTIME_DIR/qwen_standby.yaml"
SIMPLE_STANDBY="$RUNTIME_DIR/simple_handoff_standby.yaml"
NAV2_RUNTIME="$RUNTIME_DIR/nav2_params_online.yaml"
SERVO_RUNTIME="$RUNTIME_DIR/qwen3_vln_servo_standby.yaml"
BACKEND_RUNTIME="$RUNTIME_DIR/online_map_backend_standby.yaml"

python3 "$V1_ROOT/scripts/qwen_servo/make_fast_qwen_config.py" \
  --input "$QWEN_BASE_CFG" --output "$QWEN_FAST_BASE" \
  --observe "${QWEN_OBSERVE_INTERVAL:-0.9}" \
  --track "${QWEN_TRACK_INTERVAL:-0.85}" \
  --search "${QWEN_SEARCH_INTERVAL:-0.90}" >>"$MAIN_LOG" 2>&1
python3 "$V1_ROOT/scripts/fusion/make_v1_standby_runtime_v3.py" \
  --qwen-input "$QWEN_FAST_BASE" --qwen-output "$QWEN_STANDBY" \
  --simple-input "$SIMPLE_BASE_CFG" --simple-output "$SIMPLE_STANDBY" \
  >>"$MAIN_LOG" 2>&1
python3 "$V1_ROOT/scripts/fusion/make_nav2_online_params_v2.py" \
  --input "$REPO_ROOT/configs/nav2_params.yaml" --output "$NAV2_RUNTIME" \
  >>"$MAIN_LOG" 2>&1
python3 "$V1_ROOT/scripts/fusion/make_simple_handoff_runtime_config_v2.py" \
  --servo-config "$BASE_SERVO_CFG" \
  --fusion-config "$FUSION_CFG" \
  --simple-config "$SIMPLE_STANDBY" \
  --backend-config "$BACKEND_CFG" \
  --servo-output "$SERVO_RUNTIME" \
  --backend-output "$BACKEND_RUNTIME" \
  --backend-debug-dir "$RUNTIME_DIR/backend_debug" >>"$MAIN_LOG" 2>&1
python3 - "$QWEN_STANDBY" "$SIMPLE_STANDBY" "$SERVO_RUNTIME" >>"$MAIN_LOG" 2>&1 <<'PY'
import sys, yaml
qwen = yaml.safe_load(open(sys.argv[1], encoding='utf-8')) or {}
simple_file = yaml.safe_load(open(sys.argv[2], encoding='utf-8')) or {}
servo = yaml.safe_load(open(sys.argv[3], encoding='utf-8')) or {}
sm = qwen.get('state_machine', {})
simple = simple_file.get('simple_handoff_v2', simple_file)
assert sm.get('auto_enter_search') is False
assert sm.get('initial_instruction') == 'standby'
assert simple.get('cmd_mux', {}).get('default_mode') == 'HOLD'
assert servo.get('topics', {}).get('cmd_output') == simple.get('topics', {}).get('ego_cmd', '/cmd_vel_ego')
print('[config-check] standby invariants OK')
PY

# Nav2: refuse an old server unless the operator explicitly chose reuse.
if nav2_present; then
  [[ "$REUSE_NAV2" == "1" ]] || { fatal "existing Nav2 detected; use --reuse-nav2 only after confirming it has no active goal"; exit 3; }
  log "复用现有 Nav2，不接管其进程"
  wait_action /navigate_to_pose 10 ""
else
  start_group NAV2_PID NAV2 "$NAV2_LOG" ros2 launch \
    "$REPO_ROOT/configs/nav2_online_slam_fusion_navigation_launch_v2.py" \
    params_file:="$NAV2_RUNTIME" use_sim_time:=false autostart:=true use_composition:=False
  wait_action /navigate_to_pose "$NAV2_WAIT_SEC" "$NAV2_LOG"
fi
log "[READY] Nav2"

# Non-destructive camera policy: reuse a real frame; never clear a busy unknown owner.
CAMERA_PID=""
if camera_ready "${CAMERA_COMPRESSED_TOPIC:-/image}"; then
  log "复用现有 /image，相机进程不归本脚本管理"
else
  if video_device_busy "$CAMERA_DEV" || pgrep -x hobot_usb_cam >/dev/null 2>&1 || \
     pgrep -f '[o]pencv_compressed_cam.py' >/dev/null 2>&1; then
    fatal "$CAMERA_DEV busy but /image has no real frame; refusing stop_camera_tree to protect existing processes"
    exit 4
  fi
  ensure_compressed_camera "$V1_ROOT" "$CAMERA_LOG" >>"$MAIN_LOG" 2>&1
  [[ -n "${CAMERA_PID:-}" ]] || { fatal "camera helper returned no owned PID"; exit 4; }
  OWN_CAMERA=1
fi

BRIDGE_PID=""
existing_raw_count="$(publisher_count /image_raw || true)"
start_raw_bridge "$V1_ROOT" "$RAW_BRIDGE_LOG" >>"$MAIN_LOG" 2>&1
if [[ "${existing_raw_count:-0}" -eq 0 ]]; then
  RAW_BRIDGE_PID="${BRIDGE_PID:-}"
  BRIDGE_PID=""
  [[ -n "$RAW_BRIDGE_PID" ]] || { fatal "raw bridge was expected to start but has no PID"; exit 4; }
  OWN_RAW_BRIDGE=1
else
  RAW_BRIDGE_PID=""
fi
camera_topic_has_frame /image_raw || { fatal "/image_raw publisher exists but no real frame arrived"; exit 4; }
log "[READY] 相机 / 图像桥接"

if [[ "$START_FOXGLOVE" == "1" ]]; then
  port="${FOXGLOVE_PORT:-8765}"
  if ss -tln 2>/dev/null | grep -q ":${port} "; then
    log "复用 Foxglove port=$port"
  elif ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    whitelist="${FOXGLOVE_TOPIC_WHITELIST:-['^/image$','^/camera_info$','^/qwen_vln/.*','^/third_view/.*','^/map_qwen_plan/.*','^/tf$','^/tf_static$','^/scan_filtered$','^/odom$','^/map$','^/map_metadata$']}"
    start_group FOXGLOVE_PID FOXGLOVE "$FOXGLOVE_LOG" env \
      FOXGLOVE_PORT="$port" FOXGLOVE_TOPIC_WHITELIST="$whitelist" \
      bash "$REPO_ROOT/scripts/lidar/start_foxglove.sh"
  else
    log "[WARN] foxglove_bridge package not installed"
  fi
fi

if pgrep -f '[c]md_vel_priority_mux.py' >/dev/null 2>&1; then
  log "复用现有 cmd_vel_priority_mux.py"
else
  start_group PRIORITY_MUX_PID PRIORITY_MUX "$PRIORITY_MUX_LOG" python3 -u \
    "$REPO_ROOT/scripts/control/cmd_vel_priority_mux.py" \
    --autonomy-topic /cmd_vel_autonomy \
    --joy-cmd-topic /cmd_vel_joy \
    --output-topic /cmd_vel \
    --joy-topic /joy \
    --axis-linear "${JOY_AXIS_LINEAR:-1}" \
    --axis-angular "${JOY_AXIS_ANGULAR:-0}" \
    --joy-deadzone "${JOY_DEADZONE:-0.08}"
  sleep 1
  kill -0 "$PRIORITY_MUX_PID" 2>/dev/null || { fatal "priority mux exited"; exit 5; }
fi

start_group QWEN_PID QWEN "$QWEN_LOG" env QWEN_CONFIG="$QWEN_STANDBY" \
  bash "$V1_ROOT/scripts/start_debug_node.sh" standby
wait_qwen_debug_ready "$QWEN_PID" "$QWEN_LOG" "$QWEN_WAIT_SEC" /qwen_vln/result_json >>"$MAIN_LOG" 2>&1
log "[READY] Qwen warmup（待机，不持续推理）"

start_group INTERVENTION_MUX_PID INTERVENTION_MUX "$INTERVENTION_MUX_LOG" python3 -u \
  "$V1_ROOT/src/apps/cmd_vel_intervention_mux.py" --config "$SERVO_RUNTIME"
sleep 1
kill -0 "$INTERVENTION_MUX_PID" 2>/dev/null || { fatal "intervention mux exited"; exit 5; }
wait_topic /third_view/intervention/cmd_mux_status 15

# Deliberately omit --enable-motion. Runtime command is /cmd_vel_ego.
start_group SERVO_PID SERVO "$SERVO_LOG" python3 -u \
  "$V1_ROOT/src/apps/qwen_visual_servo_node.py" --config "$SERVO_RUNTIME"
sleep 1
kill -0 "$SERVO_PID" 2>/dev/null || { fatal "visual servo exited"; tail -n 80 "$SERVO_LOG" >&2; exit 5; }
wait_topic /qwen_vln/servo/status 15
publish_safe
log "[READY] 视觉伺服 disabled"
log "[READY] 控制模式 HOLD"

cat <<EOF

============================================================
[READY] 语音模块
[READY] 相机
[READY] 雷达 / 里程计
[READY] SLAM / TF
[READY] Nav2
[READY] Qwen warmup
[READY] 视觉伺服 disabled
[READY] 控制模式 HOLD
[READY] 现在可以说“小车你好”
日志目录：$LOG_DIR
============================================================
EOF

# Wait for the already-running voice process only after heavy readiness.
if [[ -z "$TASK" ]]; then
  set +e
  wait "$VOICE_PID"
  voice_rc=$?
  set -e
  VOICE_PID=""
  if [[ $voice_rc -ne 0 || ! -s "$VOICE_OUT" ]]; then
    fatal "voice pipeline failed rc=$voice_rc"
    tail -n 100 "$VOICE_LOG" >&2 2>/dev/null || true
    exit 6
  fi
  TASK="$(tr '\r\n' ' ' <"$VOICE_OUT" | xargs)"
fi
TASK="$(printf '%s' "$TASK" | tr '\r\n' ' ' | xargs)"
[[ -n "$TASK" ]] || { fatal "empty mission task"; exit 6; }
log "语音任务确认：$TASK"

# Light, task-bound nodes start only now.
start_group BACKEND_PID BACKEND "$BACKEND_LOG" python3 -u \
  "$V1_ROOT/src/apps/online_map_qwen_nav_backend_v2.py" \
  --config "$BACKEND_RUNTIME" --task "$TASK"
sleep 3
kill -0 "$BACKEND_PID" 2>/dev/null || { fatal "online backend exited"; tail -n 100 "$BACKEND_LOG" >&2; exit 7; }

export THIRD_VIEW_FLOW_LOG=/dev/null
start_group BRIDGE_PID MAP_BRIDGE "$BRIDGE_LOG" python3 -u \
  "$V1_ROOT/src/apps/online_map_plan_bridge_node.py" \
  --config "$SERVO_RUNTIME" --task "$TASK"
sleep 1
kill -0 "$BRIDGE_PID" 2>/dev/null || { fatal "map bridge exited"; tail -n 100 "$BRIDGE_LOG" >&2; exit 7; }

start_group SUPERVISOR_PID SUPERVISOR "$SUPERVISOR_LOG" python3 -u \
  "$V1_ROOT/src/apps/simple_handoff_supervisor_v2.py" \
  --config "$SIMPLE_STANDBY" --task "$TASK"
sleep 1
kill -0 "$SUPERVISOR_PID" 2>/dev/null || { fatal "supervisor exited"; tail -n 100 "$SUPERVISOR_LOG" >&2; exit 7; }
wait_topic /third_view/simple_handoff/status 15

# Supervisor publishes EGO on startup, but servo is still disabled. The gate is
# the only place that can inject the new task and enable real movement.
gate_args=(--task "$TASK" --timeout "${V1_STANDBY_GATE_TIMEOUT:-25}")
if [[ "$MOTION_ENABLED" == "1" ]]; then
  gate_args+=(--motion)
fi
set +e
python3 -u "$V1_ROOT/scripts/fusion/activate_standby_mission_v3.py" "${gate_args[@]}" \
  > >(tee "$GATE_LOG") 2>&1
gate_rc=$?
set -e
if [[ $gate_rc -ne 0 ]]; then
  fatal "mission gate rejected activation rc=$gate_rc; robot remains stopped"
  publish_safe
  exit 8
fi

if [[ "$MOTION_ENABLED" == "1" ]]; then
  log "[ACTIVE] $TASK | real motion enabled after ACK + fresh sensors"
else
  log "[ACTIVE-DRY-RUN] $TASK | servo remains disabled"
fi

# Monitor only processes owned by this launcher. External reused stacks are not killed.
health_tick=0
while true; do
  for item in \
    "QWEN:$QWEN_PID" \
    "INTERVENTION_MUX:$INTERVENTION_MUX_PID" \
    "SERVO:$SERVO_PID" \
    "BACKEND:$BACKEND_PID" \
    "MAP_BRIDGE:$BRIDGE_PID" \
    "SUPERVISOR:$SUPERVISOR_PID"; do
    name="${item%%:*}"
    pid="${item#*:}"
    if [[ -n "$pid" ]] && ! kill -0 "$pid" 2>/dev/null; then
      fatal "$name exited unexpectedly; entering HOLD"
      publish_safe
      exit 9
    fi
  done
  if [[ -n "$NAV2_PID" ]] && ! kill -0 "$NAV2_PID" 2>/dev/null; then
    fatal "owned Nav2 exited unexpectedly; entering HOLD"
    publish_safe
    exit 9
  fi
  if [[ -n "$SLAM_PID" ]] && ! kill -0 "$SLAM_PID" 2>/dev/null; then
    fatal "owned SLAM exited unexpectedly; entering HOLD"
    publish_safe
    exit 9
  fi
  health_tick=$((health_tick + 1))
  if (( health_tick % 120 == 0 )); then
    log "HEALTH owned nodes alive"
  fi
  sleep 0.5
done
