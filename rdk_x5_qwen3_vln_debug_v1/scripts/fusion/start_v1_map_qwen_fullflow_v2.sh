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
EOF
      exit 0 ;;
    *)
      if [[ "$TASK" == "find the bottle" ]]; then TASK="$1"; shift; else echo "Unknown argument: $1" >&2; exit 2; fi ;;
  esac
done

mkdir -p "$RUNTIME_DIR" "$LOG_DIR"

source_ros() {
  set +u
  if [[ -f /opt/tros/humble/setup.bash ]]; then
    source /opt/tros/humble/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
  else
    echo "[FULLFLOW_V2][FATAL] ROS2 Humble/TROS setup not found" >&2
    exit 2
  fi
  [[ -f "$HOME/ydlidar_ws/install/setup.bash" ]] && source "$HOME/ydlidar_ws/install/setup.bash"
  [[ -f "$REPO_ROOT/scripts/lib/ros_dds_env.sh" ]] && source "$REPO_ROOT/scripts/lib/ros_dds_env.sh"
  if declare -F prepare_ros_dds_env >/dev/null 2>&1; then prepare_ros_dds_env; fi
  set -u
}
source_ros

for f in \
  "$FULL_CFG" \
  "$FUSION_CFG" \
  "$BASE_SERVO_CFG" \
  "$REPO_ROOT/scripts/slam/run_slam_calibrated.sh" \
  "$REPO_ROOT/configs/nav2_params.yaml" \
  "$REPO_ROOT/configs/nav2_online_slam_fusion_navigation_launch_v2.py" \
  "$V1_ROOT/src/apps/online_map_qwen_nav_backend_v2.py" \
  "$V1_ROOT/scripts/fusion/start_v1_online_map_plan_fusion.sh"; do
  [[ -f "$f" ]] || { echo "[FULLFLOW_V2][FATAL] missing $f" >&2; exit 2; }
done

if [[ "$MOTION_ENABLED" == "1" && -z "${DASHSCOPE_API_KEY:-${QWEN_API_KEY:-}}" && "${MAP_QWEN_DRY_RUN:-0}" != "1" ]]; then
  if [[ -f "$REPO_ROOT/.env" ]]; then
    set -a; source "$REPO_ROOT/.env"; set +a
  fi
fi
if [[ "$MOTION_ENABLED" == "1" && -z "${DASHSCOPE_API_KEY:-${QWEN_API_KEY:-}}" && "${MAP_QWEN_DRY_RUN:-0}" != "1" ]]; then
  echo "[FULLFLOW_V2][FATAL] set DASHSCOPE_API_KEY in the shell or $REPO_ROOT/.env" >&2
  echo "[FULLFLOW_V2] For geometric-only connectivity testing: MAP_QWEN_DRY_RUN=1" >&2
  exit 2
fi

publisher_count() {
  ros2 topic info "$1" 2>/dev/null | awk -F': ' '/Publisher count:/ {print $2+0}' | tail -n1
}
wait_topic() {
  local topic="$1" timeout_sec="$2" start now count
  start="$(date +%s)"
  while true; do
    count="$(publisher_count "$topic" || true)"
    if [[ "${count:-0}" -gt 0 ]]; then echo "[FULLFLOW_V2] ready topic $topic"; return 0; fi
    now="$(date +%s)"
    if (( now - start >= timeout_sec )); then echo "[FULLFLOW_V2][FATAL] timeout waiting $topic" >&2; return 1; fi
    sleep 1
  done
}
wait_action() {
  local action="$1" timeout_sec="$2" start now
  start="$(date +%s)"
  while true; do
    if ros2 action info "$action" 2>/dev/null | grep -Eq 'Action servers: [1-9]'; then
      echo "[FULLFLOW_V2] ready action $action"; return 0
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_sec )); then echo "[FULLFLOW_V2][FATAL] timeout waiting action $action" >&2; return 1; fi
    sleep 1
  done
}

SLAM_PID=""; NAV2_PID=""; BACKEND_PID=""; FUSION_PID=""; STARTED_SLAM=0; CLEANED=0
kill_group() {
  local pid="${1:-}"
  [[ -z "$pid" ]] && return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
cleanup() {
  [[ "$CLEANED" == 1 ]] && return 0
  CLEANED=1
  echo "[FULLFLOW_V2] stopping full flow"
  timeout 2 ros2 topic pub --once /third_view/intervention/control_mode std_msgs/msg/String \
    "{data: '{\"mode\":\"HOLD\",\"reason\":\"fullflow_shutdown\"}'}" >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub --once /cmd_vel_autonomy geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  kill_group "$FUSION_PID"
  kill_group "$BACKEND_PID"
  kill_group "$NAV2_PID"
  if [[ "$STARTED_SLAM" == 1 ]]; then kill_group "$SLAM_PID"; fi
  rm -f "$PID_FILE"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

# Refuse to stack a second Nav2 on top of an unknown command route.
if ros2 action info /navigate_to_pose 2>/dev/null | grep -Eq 'Action servers: [1-9]'; then
  echo "[FULLFLOW_V2][FATAL] /navigate_to_pose already has a server." >&2
  echo "Stop the old Nav2 first. Reusing an unknown Nav2 may bypass the fusion mux." >&2
  exit 3
fi

MAP_COUNT="$(publisher_count /map || true)"
ODOM_COUNT="$(publisher_count /odom || true)"
SCAN_COUNT="$(publisher_count /scan_filtered || true)"
if [[ "${MAP_COUNT:-0}" -gt 0 && "${ODOM_COUNT:-0}" -gt 0 && "${SCAN_COUNT:-0}" -gt 0 ]]; then
  if [[ "$REUSE_SLAM" == 1 ]]; then
    echo "[FULLFLOW_V2] reusing existing live SLAM/sensor stack"
  else
    echo "[FULLFLOW_V2][FATAL] live map stack exists and reuse is disabled" >&2; exit 3
  fi
elif [[ "$START_SLAM" == 1 ]]; then
  echo "[FULLFLOW_V2] starting calibrated live SLAM"
  setsid bash "$REPO_ROOT/scripts/slam/run_slam_calibrated.sh" >"$LOG_DIR/slam.log" 2>&1 &
  SLAM_PID=$!; STARTED_SLAM=1
else
  echo "[FULLFLOW_V2][FATAL] /map,/odom,/scan_filtered are not ready and --no-slam was used" >&2; exit 3
fi

wait_topic /map 60
wait_topic /odom 60
wait_topic /scan_filtered 60
# Verify the actual global pose chain, not merely topic existence.
TF_OUT="$(timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
if ! grep -Eq 'Translation:|At time' <<<"$TF_OUT"; then
  echo "[FULLFLOW_V2][FATAL] TF map -> base_link unavailable" >&2
  echo "$TF_OUT" | tail -n 30 >&2
  exit 4
fi

NAV2_RUNTIME="$RUNTIME_DIR/nav2_params_online_v2.yaml"
SERVO_RUNTIME="$RUNTIME_DIR/qwen3_vln_servo_fullflow_v2.yaml"
python3 "$V1_ROOT/scripts/fusion/make_nav2_online_params_v2.py" \
  --input "$REPO_ROOT/configs/nav2_params.yaml" --output "$NAV2_RUNTIME" >/dev/null
python3 "$V1_ROOT/scripts/fusion/make_fullflow_servo_config_v2.py" \
  --input "$BASE_SERVO_CFG" --output "$SERVO_RUNTIME" >/dev/null

# Navigation-only: live slam_toolbox already supplies /map and map->odom.
echo "[FULLFLOW_V2] starting navigation-only Nav2 with A*"
setsid ros2 launch "$REPO_ROOT/configs/nav2_online_slam_fusion_navigation_launch_v2.py" \
  params_file:="$NAV2_RUNTIME" use_sim_time:=false autostart:=true use_composition:=False \
  >"$LOG_DIR/nav2.log" 2>&1 &
NAV2_PID=$!
wait_action /navigate_to_pose 45

# The real teammate-side online backend.
echo "[FULLFLOW_V2] starting live map -> candidates -> Qwen -> Nav2 backend"
setsid python3 -u "$V1_ROOT/src/apps/online_map_qwen_nav_backend_v2.py" \
  --config "$FULL_CFG" --task "$TASK" >"$LOG_DIR/backend.log" 2>&1 &
BACKEND_PID=$!
sleep 1
kill -0 "$BACKEND_PID" 2>/dev/null || { tail -n 200 "$LOG_DIR/backend.log" >&2 || true; exit 5; }

cat > "$PID_FILE" <<EOF
SLAM_PID=$SLAM_PID
STARTED_SLAM=$STARTED_SLAM
NAV2_PID=$NAV2_PID
BACKEND_PID=$BACKEND_PID
FUSION_PID=
EOF

# Existing tested bridge + intervention manager + EGO/MAP/HOLD mux + V1.
echo "[FULLFLOW_V2] starting V1 fusion stack"
echo "[FULLFLOW_V2] task=$TASK motion=$MOTION_ENABLED qwen_dry_run=${MAP_QWEN_DRY_RUN:-0}"
setsid env \
  MOTION_ENABLED="$MOTION_ENABLED" \
  SERVO_CONFIG="$SERVO_RUNTIME" \
  FUSION_CONFIG="$FUSION_CFG" \
  FUSION_RUNTIME_CONFIG="$RUNTIME_DIR/fusion_runtime_v2.yaml" \
  bash "$V1_ROOT/scripts/fusion/start_v1_online_map_plan_fusion.sh" "$TASK" \
  >"$LOG_DIR/fusion_v1.log" 2>&1 &
FUSION_PID=$!
sed -i "s/^FUSION_PID=.*/FUSION_PID=$FUSION_PID/" "$PID_FILE"
sleep 2
kill -0 "$FUSION_PID" 2>/dev/null || { tail -n 240 "$LOG_DIR/fusion_v1.log" >&2 || true; exit 6; }

cat <<EOF

[FULLFLOW_V2] ALL STACKS STARTED
  live map      : /map
  first-person  : /cmd_vel_ego
  map Nav2 raw  : /map_qwen_plan/cmd_vel_raw
  backend safe  : /map_qwen_plan/cmd_vel
  map mux input : /cmd_vel_map
  autonomy      : /cmd_vel_autonomy
  chassis       : /cmd_vel
  status        : /map_qwen_plan/backend_debug
  logs          : $LOG_DIR

Keep this terminal open. Ctrl+C stops every process started by this launcher.
EOF

while true; do
  for pair in "NAV2:$NAV2_PID:$LOG_DIR/nav2.log" "BACKEND:$BACKEND_PID:$LOG_DIR/backend.log" "FUSION:$FUSION_PID:$LOG_DIR/fusion_v1.log"; do
    IFS=: read -r name pid log <<< "$pair"
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "[FULLFLOW_V2][FATAL] $name exited" >&2
      tail -n 240 "$log" >&2 || true
      exit 7
    fi
  done
  if [[ "$STARTED_SLAM" == 1 ]] && ! kill -0 "$SLAM_PID" 2>/dev/null; then
    echo "[FULLFLOW_V2][FATAL] SLAM exited" >&2
    tail -n 240 "$LOG_DIR/slam.log" >&2 || true
    exit 7
  fi
  sleep 0.5
done
