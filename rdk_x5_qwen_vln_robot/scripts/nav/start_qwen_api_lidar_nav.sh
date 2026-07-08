#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

# shellcheck source=scripts/lib/stop_qwen_stack.sh
source "$PROJECT_DIR/scripts/lib/stop_qwen_stack.sh"
trap stop_qwen_stack EXIT INT TERM

CONFIG="$PROJECT_DIR/configs/qwen_api_lidar_nav.yaml"
INSTRUCTION="${1:-find bottle}"
CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
# Set RUN_CHASSIS=0 to dry-run without moving the robot.
RUN_CHASSIS="${RUN_CHASSIS:-1}"
# Set RUN_FOXGLOVE_VIZ=0 to skip Foxglove viz (bridge + annotated topics).
RUN_FOXGLOVE_VIZ="${RUN_FOXGLOVE_VIZ:-1}"

# Chassis port/PWM from qwen yaml (aligned with nav_yolo_lidar_semantic_explore.yaml).
# shellcheck source=scripts/lib/load_chassis_from_config.sh
source "$PROJECT_DIR/scripts/lib/load_chassis_from_config.sh"
load_chassis_from_config "$CONFIG"

CMD_TOPIC="$(python3 - "$CONFIG" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1])) or {}
print(cfg.get("cmd_topic", "/cmd_vel"))
PY
)"

: "${DASHSCOPE_API_KEY:?Missing DASHSCOPE_API_KEY}"
: "${QWEN_BASE_URL:?Missing QWEN_BASE_URL}"
: "${QWEN_MODEL:?Missing QWEN_MODEL}"

source_ros_env() {
  set +u
  if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
  elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
  fi
  if [ -f "$HOME/ydlidar_ws/install/setup.bash" ]; then
    source "$HOME/ydlidar_ws/install/setup.bash"
  fi
  set -u
}

source_ros_env
mkdir -p logs data/images/qwen_api_lidar_debug

echo "===== Qwen cloud API + LiDAR (rdk_x5_qwen_vln_robot) ====="
echo "PROJECT_DIR=$PROJECT_DIR"
echo "INSTRUCTION=$INSTRUCTION"
echo "cmd_topic=$CMD_TOPIC RUN_CHASSIS=$RUN_CHASSIS CHASSIS_PORT=${CHASSIS_PORT:-unset} RUN_FOXGLOVE_VIZ=$RUN_FOXGLOVE_VIZ"
if [ "$CMD_TOPIC" != "/cmd_vel" ]; then
  echo "NOTE: cmd_topic=$CMD_TOPIC (dry-run). Chassis bridge listens to /cmd_vel only — robot will NOT move."
  echo "      Set cmd_topic: /cmd_vel in configs/qwen_api_lidar_nav.yaml when ready for real driving."
fi
if [ ! -e "${CHASSIS_PORT:-/dev/ttyUSB2}" ]; then
  echo "WARN: chassis port ${CHASSIS_PORT:-/dev/ttyUSB2} not found. Run: ls -la /dev/ttyUSB* /dev/rosmaster"
  echo "      Update chassis.port in $CONFIG before real driving."
fi

echo "[1/7] stop competing nav processes..."
pkill -f "$PROJECT_DIR/src/apps/run_qwen_api_lidar_nav.py" || true
pkill -f "run_shared_nav" || true
pkill -f "run_yolo_lidar" || true
pkill -f "hobot_yolo_world" || true
pkill -f "hobot_usb_cam" || true
pkill -f "compressed_to_raw_image.py" || true
sleep 2

echo "[2/7] publish zero cmd_vel once..."
timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 >/dev/null 2>&1 || true

echo "[3/7] start camera + image bridge (use original repo camera_stack, 1280x720)..."
_saved_project_dir="$PROJECT_DIR"
PROJECT_DIR="$RDK_ORIGINAL_ROOT"
# shellcheck source=/root/rdk_x5_vln_robot/scripts/lib/camera_stack.sh
source "$RDK_ORIGINAL_ROOT/scripts/lib/camera_stack.sh"
export CAMERA_DEV
ensure_camera_image_stream "$_saved_project_dir/logs/qwen_api_lidar_camera.log" || {
  echo "ERROR: camera failed; see $_saved_project_dir/logs/qwen_api_lidar_camera.log"
  exit 1
}
ensure_image_raw_stream "$_saved_project_dir/logs/qwen_api_lidar_image_raw.log" || {
  echo "ERROR: image bridge failed; see $_saved_project_dir/logs/qwen_api_lidar_image_raw.log"
  exit 1
}
PROJECT_DIR="$_saved_project_dir"

LIDAR_PID=""
NAV_PID=""
FOXGLOVE_VIZ_PID=""

echo "[4/7] start lidar (read-only call to original repo launch)..."
ros2 launch "$RDK_ORIGINAL_ROOT/lidar/launch/tmini_plus.launch.py" \
  > "$PROJECT_DIR/logs/qwen_api_lidar_scan.log" 2>&1 &
LIDAR_PID=$!
sleep 3

echo "[5/7] wait for /image_raw and /scan..."
for topic in /image_raw /scan; do
  ok=0
  for i in $(seq 1 15); do
    if timeout 3 ros2 topic echo "$topic" --once >/dev/null 2>&1; then ok=1; break; fi
    sleep 2
  done
  [ "$ok" -eq 1 ] || { echo "ERROR: $topic not available"; exit 1; }
  echo " $topic OK"
done

# Foxglove bridge is slow (ros2 launch); start early while chassis/nav still booting.
if [ "$RUN_FOXGLOVE_VIZ" = "1" ]; then
  # shellcheck source=scripts/lib/ensure_foxglove_bridge.sh
  source "$PROJECT_DIR/scripts/lib/ensure_foxglove_bridge.sh"
  echo "[5.5/7] pre-start Foxglove bridge (parallel with chassis/nav boot)..."
  ensure_foxglove_bridge "$PROJECT_DIR/logs/qwen_api_lidar_foxglove_bridge.log" || {
    echo "WARN: Foxglove bridge not ready; viz may lag. See logs/qwen_api_lidar_foxglove_bridge.log"
  }
  echo "[5.6/7] pre-start Foxglove viz node (annotated image/markers)..."
  start_qwen_api_foxglove_viz_node "$CONFIG" "$PROJECT_DIR/logs/qwen_api_lidar_foxglove_viz.log" || {
    echo "WARN: Foxglove viz node failed; see logs/qwen_api_lidar_foxglove_viz.log"
    FOXGLOVE_VIZ_PID=""
  }
  FOXGLOVE_VIZ_PID="${FOXGLOVE_VIZ_PID:-}"
fi

if [ "$RUN_CHASSIS" = "1" ]; then
  echo "[6/7] start chassis bridge (from qwen yaml chassis block)..."
  python3 "$RDK_ORIGINAL_ROOT/debug_tools/m1_runtime_sanitize.py" --port "$CHASSIS_PORT" \
    > "$PROJECT_DIR/logs/qwen_api_lidar_sanitize.log" 2>&1 || {
    echo "WARN: M1 sanitize failed; see $PROJECT_DIR/logs/qwen_api_lidar_sanitize.log"
  }
  _saved_project_dir="$PROJECT_DIR"
  PROJECT_DIR="$RDK_ORIGINAL_ROOT"
  load_chassis_from_config "$CONFIG"
  source "$RDK_ORIGINAL_ROOT/scripts/lib/run_chassis_bridge.sh"
  run_chassis_bridge "$_saved_project_dir/logs/qwen_api_lidar_chassis.log"
  PROJECT_DIR="$_saved_project_dir"
  sleep 1
else
  echo "[6/7] RUN_CHASSIS=0, skip chassis bridge (robot will not move)."
fi

echo "[7/7] start Qwen API LiDAR nav node..."
python3 - "$CONFIG" <<'PY' || true
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1])) or {}
s = cfg.get("servo") or {}
band = float(s.get("center_deadband", 0.10))
hyst = float(s.get("straight_hysteresis", 0.02))
w = int(cfg.get("image_width", 1280))
px = int(band * w)
a = cfg.get("angle_servo") or {}
angle_on = bool(s.get("angle_servo_enabled", a.get("enabled", False)))
if angle_on:
    cap = a.get("absolute_cap_deg", 1)
    wait = a.get("wait_turn_complete", True)
    print(
        f"[servo] angle_servo one-shot: θ<=min({a.get('max_turn_deg',30)},{cap})° "
        f"turn_wz={a.get('turn_wz',0.04)} wait_turn_complete={wait}"
    )
print(f"[servo] straight_band: |u-center|<={px}px (deadband={band}, hysteresis={hyst}) -> forward only, no turn")
PY
python3 "$PROJECT_DIR/src/apps/run_qwen_api_lidar_nav.py" \
  --config "$CONFIG" --instruction "$INSTRUCTION" \
  > "$PROJECT_DIR/logs/qwen_api_lidar_nav.log" 2>&1 &
NAV_PID=$!

if [ "$RUN_FOXGLOVE_VIZ" = "1" ]; then
  if [ -n "${FOXGLOVE_VIZ_PID:-}" ] && kill -0 "$FOXGLOVE_VIZ_PID" 2>/dev/null; then
    echo "[viz] Foxglove already up (bridge + viz pid=$FOXGLOVE_VIZ_PID)"
  else
    echo "[viz] WARN: viz not running; start manually: bash scripts/nav/start_qwen_api_lidar_foxglove_viz.sh"
  fi
  echo "  Foxglove WebSocket: ws://<robot-ip>:${FOXGLOVE_PORT:-8765}"
  echo "  Image (red cross): /qwen_api_viz/image/compressed"
else
  echo "[viz] RUN_FOXGLOVE_VIZ=0, skip Foxglove visualization."
fi

echo "Started nav pid=$NAV_PID (lidar pid=${LIDAR_PID:-n/a}, foxglove pid=${FOXGLOVE_VIZ_PID:-n/a})."
echo "  tail -f $PROJECT_DIR/logs/qwen_api_lidar_nav.log"
echo "  ros2 topic echo /qwen_api_json"
echo "  ros2 topic echo /qwen_api_state"
echo "  ros2 topic echo $CMD_TOPIC"
echo "  ros2 topic echo /cmd_vel_sent"
echo "Waiting for nav to exit; Ctrl+C or nav crash -> stop all (camera/lidar/chassis/viz)."

wait "$NAV_PID" 2>/dev/null || true
NAV_EXIT=$?
if [ "$NAV_EXIT" -ne 0 ]; then
  echo "Nav exited with code $NAV_EXIT"
fi
exit "$NAV_EXIT"
