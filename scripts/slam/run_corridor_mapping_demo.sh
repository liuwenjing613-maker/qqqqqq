#!/usr/bin/env bash
# Corridor SLAM mapping one-shot demo (LiDAR + odom + slam_toolbox + slow motion + map save).
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
source "${PROJECT_DIR}/scripts/lib/cleanup_lidar_slam_nav.sh"
source "${PROJECT_DIR}/scripts/lib/lidar_frame_config.sh"

LOG_DIR="${PROJECT_DIR}/logs/slam_demo"
MAP_DIR="${PROJECT_DIR}/maps"
SLAM_CONFIG="${PROJECT_DIR}/configs/slam_toolbox.yaml"
LIDAR_DEV="/dev/ydlidar"
CHASSIS_DEV="/dev/rosmaster"

# Corridor demo speeds (m/s, rad/s).
# M1 实测：vx>=0.05、wz>=0.24 才易克服静摩擦；默认取偏保守但可明显观察的值。
DEMO_VX="${DEMO_VX:-0.06}"
DEMO_WZ="${DEMO_WZ:-0.22}"
DEMO_PUB_HZ="${DEMO_PUB_HZ:-20}"

PIDS=()
DEMO_STARTED=0
KEEP_RUNNING="${KEEP_RUNNING:-0}"
SKIP_CLEANUP=0

set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "${HOME}/ydlidar_ws/install/setup.bash" ]; then
  source "${HOME}/ydlidar_ws/install/setup.bash"
fi
set -u

publish_zero_cmd() {
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
  timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
    >/dev/null 2>&1 || true
}

kill_demo_processes() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  pkill -f "async_slam_toolbox_node" 2>/dev/null || true
  pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
  pkill -f "static_transform_publisher.*base_link.*laser" 2>/dev/null || true
  pkill -f "ydlidar_ros2_driver_node" 2>/dev/null || true
}

cleanup() {
  echo ""
  echo "[cleanup] publishing zero /cmd_vel..."
  publish_zero_cmd
  if [ "${SKIP_CLEANUP:-0}" = "1" ]; then
    echo "[cleanup] KEEP_RUNNING=1: leaving background SLAM demo processes alive"
    return
  fi
  echo "[cleanup] stopping demo processes..."
  kill_demo_processes
  if [ -f "${PROJECT_DIR}/logs/lidar_driver.log" ]; then
    cp -f "${PROJECT_DIR}/logs/lidar_driver.log" "${LOG_DIR}/lidar_driver.log" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

log() {
  echo "[$(date +%H:%M:%S)] $*"
}

ensure_slam_config() {
  if [ -f "$SLAM_CONFIG" ]; then
    log "SLAM config OK: $SLAM_CONFIG"
    return 0
  fi
  log "Creating default SLAM config: $SLAM_CONFIG"
  mkdir -p "$(dirname "$SLAM_CONFIG")"
  cat >"$SLAM_CONFIG" <<'EOF'
slam_toolbox:
  ros__parameters:
    use_sim_time: false
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan_filtered
    mode: mapping
    resolution: 0.05
    max_laser_range: 6.0
    minimum_time_interval: 0.1
    transform_timeout: 1.0
    tf_buffer_duration: 60.0
    map_update_interval: 1.0
    throttle_scans: 1
    transform_publish_period: 0.02
    debug_logging: false
    enable_interactive_mode: true
    stack_size_to_use: 40000000
EOF
}

preflight_checks() {
  log "Preflight checks..."
  if [ ! -e "$LIDAR_DEV" ]; then
    echo "FAIL: LiDAR device not found: $LIDAR_DEV" >&2
    exit 1
  fi
  if [ ! -e "$CHASSIS_DEV" ]; then
    echo "FAIL: chassis device not found: $CHASSIS_DEV" >&2
    exit 1
  fi
  ensure_slam_config
  mkdir -p "$LOG_DIR" "$MAP_DIR"
  ls -l "$LIDAR_DEV" "$CHASSIS_DEV"
}

start_background() {
  local name="$1"
  shift
  local log_file="${LOG_DIR}/${name}.log"
  log "Starting ${name} -> ${log_file}"
  "$@" >"$log_file" 2>&1 &
  PIDS+=("$!")
  sleep 0.3
}

wait_for_topic_msgs() {
  local topic="$1"
  local timeout_sec="${2:-15}"
  python3 "${PROJECT_DIR}/scripts/lib/wait_ros_topic.py" \
    --topic "$topic" \
    --timeout "$timeout_sec" \
    --min-msgs 1
}

check_topic_hz() {
  local topic="$1"
  local label="$2"
  local out
  out="$(timeout 8 ros2 topic hz "$topic" 2>&1 | head -5 || true)"
  if echo "$out" | grep -q "average rate"; then
    log "OK ${label}: $(echo "$out" | grep 'average rate' | head -1)"
    return 0
  fi
  echo "FAIL: ${label} (${topic}) has no publish rate within 8s" >&2
  echo "$out" >&2
  return 1
}

check_tf_chain() {
  log "Checking TF topic /tf ..."
  if ! ros2 topic list | grep -qx '/tf'; then
    echo "FAIL: /tf topic not found" >&2
    return 1
  fi
  log "OK /tf topic exists"

  log "Checking transform odom -> laser ..."
  local tf_out
  tf_out="$(timeout 8 ros2 run tf2_ros tf2_echo odom laser 2>&1 | head -20 || true)"
  if echo "$tf_out" | grep -q "Translation"; then
    log "OK TF odom -> laser is available"
    echo "$tf_out" | grep -E "Translation|Rotation" | head -4
    return 0
  fi
  echo "FAIL: could not lookup transform odom -> laser" >&2
  echo "$tf_out" >&2
  return 1
}

check_map_metadata() {
  log "Checking /map_metadata ..."
  local meta
  meta="$(timeout 15 ros2 topic echo /map_metadata --once 2>&1 || true)"
  if echo "$meta" | grep -q "resolution:"; then
    log "OK /map_metadata received"
    echo "$meta" | head -12
    return 0
  fi
  echo "FAIL: /map_metadata not received within 15s" >&2
  echo "$meta" >&2
  return 1
}

post_start_checks() {
  log "Post-start health checks..."
  wait_for_topic_msgs /scan 20
  wait_for_topic_msgs /odom 20
  check_topic_hz /scan "LiDAR /scan"
  check_topic_hz /odom "Odometry /odom"
  check_tf_chain
  check_map_metadata
}

slow_rotate() {
  local wz="$1"
  local duration="$2"
  log "Motion: rotate in place wz=${wz} for ${duration}s (pub ${DEMO_PUB_HZ}Hz)"
  timeout "$duration" ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${wz}}}" -r "${DEMO_PUB_HZ}" \
    >/dev/null 2>&1 || true
  publish_zero_cmd
}

slow_forward() {
  local vx="$1"
  local duration="$2"
  log "Motion: forward vx=${vx} for ${duration}s (pub ${DEMO_PUB_HZ}Hz)"
  timeout "$duration" ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: ${vx}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r "${DEMO_PUB_HZ}" \
    >/dev/null 2>&1 || true
  publish_zero_cmd
}

run_motion_demo() {
  log "Starting slow corridor motion sequence..."
  DEMO_STARTED=1
  slow_rotate "$DEMO_WZ" 4
  sleep 1
  slow_forward "$DEMO_VX" 2
  sleep 1
  slow_rotate "-${DEMO_WZ}" 4
  publish_zero_cmd
  log "Motion sequence complete."
}

confirm_motion_safety() {
  echo ""
  echo "============================================================"
  echo " 脚本即将让小车低速运动，请确认走廊清空。"
  echo " 输入 YES 继续，其他任何输入将退出。"
  echo "============================================================"
  read -r -p "> " reply
  if [ "$reply" != "YES" ]; then
    log "用户未确认，退出。"
    publish_zero_cmd
    exit 1
  fi
  log "用户已确认，开始低速运动。"
  log "Motion params: DEMO_VX=${DEMO_VX} DEMO_WZ=${DEMO_WZ} DEMO_PUB_HZ=${DEMO_PUB_HZ}"
  log "Tip: 若仍不明显，试 DEMO_VX=0.07 DEMO_WZ=0.24"
}

save_map() {
  log "Saving map to ${MAP_DIR}/corridor_map ..."
  publish_zero_cmd
  sleep 1
  ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/corridor_map" \
    >"${LOG_DIR}/map_saver.log" 2>&1

  if [ ! -f "${MAP_DIR}/corridor_map.yaml" ] || [ ! -f "${MAP_DIR}/corridor_map.pgm" ]; then
    echo "FAIL: map files not created under ${MAP_DIR}" >&2
    cat "${LOG_DIR}/map_saver.log" >&2 || true
    exit 1
  fi
  log "Saved: ${MAP_DIR}/corridor_map.yaml ${MAP_DIR}/corridor_map.pgm"
}

maybe_convert_png() {
  python3 - "$MAP_DIR" <<'PY' || true
import sys
from pathlib import Path

map_dir = Path(sys.argv[1])
pgm = map_dir / "corridor_map.pgm"
png = map_dir / "corridor_map.png"
if not pgm.is_file():
    sys.exit(0)
try:
    from PIL import Image
except ImportError:
    print("[info] Pillow not installed; skip corridor_map.png conversion")
    sys.exit(0)

img = Image.open(pgm)
img.save(png)
print(f"[info] Wrote {png}")
PY
}

main() {
  log "===== Corridor SLAM Mapping Demo ====="
  log "PROJECT_DIR=$PROJECT_DIR"

  preflight_checks

  log "Stopping previous demo-related processes (YOLO/Qwen untouched)..."
  cleanup_lidar_slam_nav_processes
  pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
  sleep 0.5

  log "[1/5] LiDAR driver via scripts/lidar/start_lidar_only.sh"
  start_background lidar bash "${PROJECT_DIR}/scripts/lidar/start_lidar_only.sh"
  sleep 3
  cp -f "${PROJECT_DIR}/logs/lidar_driver.log" "${LOG_DIR}/lidar_driver.log" 2>/dev/null || true

  log "[2/5] Scan filter /scan -> /scan_filtered"
  start_background scan_filter python3 "${PROJECT_DIR}/ros2_bridge/simple_scan_filter.py" \
    --in-topic /scan \
    --out-topic /scan_filtered \
    --min-range 0.18 \
    --max-range "${SCAN_FILTER_MAX_RANGE:-4.0}" \
    --isolated-window "${SCAN_FILTER_ISOLATED_WINDOW:-2}" \
    --isolated-delta "${SCAN_FILTER_ISOLATED_DELTA:-0.25}" \
    --min-support-neighbors "${SCAN_FILTER_MIN_SUPPORT:-1}" \
    --stats-every 50
  sleep 2

  log "[3/5] Chassis PWM bridge + /odom"
  source "${PROJECT_DIR}/scripts/lib/load_mvp_tune.sh"
  source "${PROJECT_DIR}/scripts/lib/run_chassis_bridge.sh"
  export CHASSIS_PORT="${CHASSIS_DEV}"
  run_chassis_bridge "${LOG_DIR}/chassis_bridge.log"
  sleep 2

  log "[4/5] Static TF base_link -> ${LASER_FRAME}"
  start_background static_tf \
    ros2 run tf2_ros static_transform_publisher \
    --x "${LASER_X}" \
    --y "${LASER_Y}" \
    --z "${LASER_Z}" \
    --roll "${LASER_ROLL}" \
    --pitch "${LASER_PITCH}" \
    --yaw "${LASER_YAW}" \
    --frame-id base_link \
    --child-frame-id "${LASER_FRAME}"
  sleep 1

  log "[5/5] slam_toolbox online_async"
  start_background slam_toolbox \
    ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:="${SLAM_CONFIG}"
  sleep 4

  post_start_checks
  confirm_motion_safety
  run_motion_demo
  save_map
  maybe_convert_png

  log "===== Demo finished successfully ====="
  log "Map: ${MAP_DIR}/corridor_map.yaml ${MAP_DIR}/corridor_map.pgm"
  if [ -f "${MAP_DIR}/corridor_map.png" ]; then
    log "Preview: ${MAP_DIR}/corridor_map.png"
  fi
  log "Logs: ${LOG_DIR}/"

  if [ "${KEEP_RUNNING}" = "1" ]; then
    SKIP_CLEANUP=1
    log "KEEP_RUNNING=1: SLAM stack still running."
    log "Open RViz2 or Foxglove to view live /map, /scan, /tf."
    log "Saved map: ${MAP_DIR}/corridor_map.yaml"
    log "Press Ctrl+C to exit this script (background nodes stay up)."
    while true; do sleep 3600; done
  fi
}

main "$@"
