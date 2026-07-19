#!/usr/bin/env bash
# Live corridor SLAM stack:
# LiDAR + odom + static TF + slam_toolbox + optional Foxglove.
# Does NOT auto-drive the robot. Manual /cmd_vel only.

set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
source "${PROJECT_DIR}/scripts/lib/cleanup_lidar_slam_nav.sh"
source "${PROJECT_DIR}/scripts/lib/lidar_frame_config.sh"

LOG_DIR="${PROJECT_DIR}/logs/slam_live"
SLAM_CONFIG="${PROJECT_DIR}/configs/slam_toolbox.yaml"
LIDAR_DEV="/dev/ydlidar"
CHASSIS_DEV="/dev/rosmaster"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

PIDS=()
declare -A NAMED_PIDS=()
FOXGLOVE_STARTED=0
HANDOFF_DONE=0
CONTROLLED_NAV_HANDOFF=0
HANDOFF_REQUEST_FILE="${PROJECT_DIR}/runtime/request_nav_handoff"
SENSOR_BASE_STACK_JSON="${PROJECT_DIR}/runtime/sensor_base_stack.json"
PID_DIR=""

# Optional: --handoff-to-nav enables watching for handoff signal from the start.
for _arg in "$@"; do
  case "$_arg" in
    --handoff-to-nav) export CORRIDOR_HANDOFF_WATCH=1 ;;
  esac
done

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

# shellcheck source=scripts/lib/ros_dds_env.sh
source "${PROJECT_DIR}/scripts/lib/ros_dds_env.sh"
# Hub may restart this stack while camera/Qwen prewarm is still attached.
# Never wipe /dev/shm in that case — prepare would kill live FastDDS peers.
if [ "${ATTACH_ROS_DDS_ONLY:-0}" = "1" ] || [ "${VOICE_DEMO_ATTACH_DDS:-0}" = "1" ]; then
  attach_ros_dds_env
else
  prepare_ros_dds_env
fi

log() {
  echo "[$(date +%H:%M:%S)] $*"
}

publish_zero_cmd() {
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
}

write_sensor_base_stack_json() {
  mkdir -p "$(dirname "$SENSOR_BASE_STACK_JSON")"
  python3 - "$SENSOR_BASE_STACK_JSON" \
    "${NAMED_PIDS[lidar]:-}" \
    "${NAMED_PIDS[scan_filter]:-}" \
    "${NAMED_PIDS[static_tf]:-}" \
    "${NAMED_PIDS[foxglove_bridge]:-}" \
    "${NAMED_PIDS[slam_toolbox]:-}" <<'PY'
import json, sys, time
from pathlib import Path
out = Path(sys.argv[1])
payload = {
    "schema_version": 1,
    "status": "sensor_base_owned_by_nav_session",
    "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "pids": {
        "lidar": int(sys.argv[2]) if sys.argv[2] else None,
        "scan_filter": int(sys.argv[3]) if sys.argv[3] else None,
        "static_tf": int(sys.argv[4]) if sys.argv[4] else None,
        "foxglove_bridge": int(sys.argv[5]) if sys.argv[5] else None,
        "slam_toolbox_stopped": True,
        "slam_toolbox_last_pid": int(sys.argv[6]) if sys.argv[6] else None,
    },
    "keep": ["lidar", "scan_filter", "chassis_bridge", "odom", "static_tf", "foxglove_bridge"],
    "stopped": ["slam_toolbox"],
}
tmp = out.with_suffix(".json.tmp")
tmp.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
tmp.replace(out)
print(out)
PY
}

write_nav_handoff_ack_json() {
  # Session-scoped ack for Qwen Nav2 reuse pipeline (process identity, atomic write).
  local session_id="${QWEN_NAV_HANDOFF_SESSION_ID:-}"
  if [[ -z "$session_id" && -f "${PROJECT_DIR}/runtime/nav_handoff_active_session" ]]; then
    session_id="$(tr -d '[:space:]' < "${PROJECT_DIR}/runtime/nav_handoff_active_session" || true)"
  fi
  if [[ -z "$session_id" && -f "${PROJECT_DIR}/runtime/request_nav_handoff" ]]; then
    session_id="$(tr -d '[:space:]' < "${PROJECT_DIR}/runtime/request_nav_handoff" || true)"
  fi
  if [[ -z "$session_id" ]]; then
    log "[handoff] WARN: no session_id for ack.json"
    return 0
  fi
  local ack_dir="${PROJECT_DIR}/runtime/nav_handoff/${session_id}"
  mkdir -p "$ack_dir"
  local chassis_pid=""
  chassis_pid="$(pgrep -f 'm1_pwm_cmd_vel_bridge.py' 2>/dev/null | head -1 || true)"
  local joy_gone=0 teleop_gone=0 slam_gone=0 frontier_gone=0
  pgrep -f 'joy_node' >/dev/null 2>&1 || joy_gone=1
  pgrep -f 'teleop_twist_joy' >/dev/null 2>&1 || teleop_gone=1
  pgrep -f 'slam_toolbox' >/dev/null 2>&1 || slam_gone=1
  pgrep -f 'frontier_region_debug' >/dev/null 2>&1 || frontier_gone=1
  python3 - "${ack_dir}/ack.json" "$session_id" \
    "${NAMED_PIDS[lidar]:-}" \
    "${NAMED_PIDS[scan_filter]:-}" \
    "${chassis_pid:-}" \
    "${NAMED_PIDS[static_tf]:-}" \
    "${NAMED_PIDS[foxglove_bridge]:-}" \
    "$joy_gone" "$teleop_gone" "$slam_gone" "$frontier_gone" \
    "${PROJECT_DIR}/scripts/nav" <<'PY'
import json, os, sys
from pathlib import Path
sys.path.insert(0, sys.argv[12])
from qwen_nav2_common import build_process_identity, time_now

out, sid = Path(sys.argv[1]), sys.argv[2]

def maybe_int(s):
    return int(s) if s else None

roles = {
    "lidar": maybe_int(sys.argv[3]),
    "scan_filter": maybe_int(sys.argv[4]),
    "chassis": maybe_int(sys.argv[5]),
    "static_tf": maybe_int(sys.argv[6]),
    "foxglove": maybe_int(sys.argv[7]),
}
processes = {}
for role, pid in roles.items():
    ident = build_process_identity(role, pid)
    if ident is not None:
        processes[role] = ident

payload = {
    "session_id": sid,
    "state": "SENSOR_BASE_HELD",
    "completed_epoch": time_now(),
    "processes": processes,
    "stopped": {
        "joy": bool(int(sys.argv[8])),
        "teleop": bool(int(sys.argv[9])),
        "slam": bool(int(sys.argv[10])),
        "frontier": bool(int(sys.argv[11])),
    },
    "static_tf_present_via_tf": "static_tf" in processes,
    "foxglove_optional": True,
    "source": "run_corridor_mapping_live_foxglove",
}
# Legacy flat fields kept only for diagnostics — not used by validate_handoff_ack.
payload["_legacy_diagnostic_pids"] = {k: v for k, v in roles.items()}
tmp = out.with_suffix(".tmp")
with open(tmp, "w", encoding="utf-8") as fh:
    fh.write(json.dumps(payload, indent=2) + "\n")
    fh.flush()
    os.fsync(fh.fileno())
os.replace(tmp, out)
print(out)
PY
}

perform_nav_handoff() {
  if [[ "$HANDOFF_DONE" -eq 1 ]]; then
    return 0
  fi
  CONTROLLED_NAV_HANDOFF=1
  log "[handoff] controlled handoff to Nav2: stop slam_toolbox only, keep sensors"
  publish_zero_cmd

  pkill -TERM -f "joy_node|teleop_twist_joy" 2>/dev/null || true
  if [[ -x "${PROJECT_DIR}/scripts/nav/stop_frontier_region_debug.sh" ]]; then
    bash "${PROJECT_DIR}/scripts/nav/stop_frontier_region_debug.sh" >/dev/null 2>&1 || true
  fi
  # Exact-PID stop for any remaining frontier_region_debug_node.py in this project.
  local fpid fcwd fcmd
  for fpid in $(pgrep -f "frontier_region_debug_node.py" 2>/dev/null || true); do
    fcwd="$(readlink -f "/proc/${fpid}/cwd" 2>/dev/null || true)"
    fcmd="$(tr '\0' ' ' < "/proc/${fpid}/cmdline" 2>/dev/null || true)"
    if [[ "$fcwd" == "$PROJECT_DIR" || "$fcmd" == *"${PROJECT_DIR}/"* ]]; then
      kill -TERM "$fpid" 2>/dev/null || true
    fi
  done

  local slam_pid="${NAMED_PIDS[slam_toolbox]:-}"
  if [[ -n "$slam_pid" ]] && kill -0 "$slam_pid" 2>/dev/null; then
    kill -TERM "$slam_pid" 2>/dev/null || true
    sleep 0.4
    kill -KILL "$slam_pid" 2>/dev/null || true
  fi
  # Narrow slam_toolbox node kill only (not fuzzy python).
  pkill -TERM -f "async_slam_toolbox_node|sync_slam_toolbox_node" 2>/dev/null || true
  sleep 0.4
  pkill -KILL -f "async_slam_toolbox_node|sync_slam_toolbox_node" 2>/dev/null || true

  # Drop slam from PIDS so EXIT cleanup won't kill sensors via full list semantics
  local filtered=()
  local pid
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "$slam_pid" && "$pid" == "$slam_pid" ]]; then
      continue
    fi
    filtered+=("$pid")
  done
  PIDS=("${filtered[@]:-}")
  unset 'NAMED_PIDS[slam_toolbox]'

  # Wait briefly until stopped roles are actually gone before writing ack.
  local w still_f
  for w in $(seq 1 20); do
    pgrep -f "joy_node" >/dev/null 2>&1 && { sleep 0.2; continue; }
    pgrep -f "teleop_twist_joy" >/dev/null 2>&1 && { sleep 0.2; continue; }
    pgrep -f "async_slam_toolbox_node|sync_slam_toolbox_node" >/dev/null 2>&1 && { sleep 0.2; continue; }
    still_f=0
    for fpid in $(pgrep -f "frontier_region_debug_node.py" 2>/dev/null || true); do
      fcwd="$(readlink -f "/proc/${fpid}/cwd" 2>/dev/null || true)"
      fcmd="$(tr '\0' ' ' < "/proc/${fpid}/cmdline" 2>/dev/null || true)"
      if [[ "$fcwd" == "$PROJECT_DIR" || "$fcmd" == *"${PROJECT_DIR}/"* ]]; then
        kill -KILL "$fpid" 2>/dev/null || true
        still_f=1
      fi
    done
    [[ "$still_f" -eq 1 ]] && { sleep 0.2; continue; }
    break
  done

  write_sensor_base_stack_json || true
  write_nav_handoff_ack_json || true
  rm -f "$HANDOFF_REQUEST_FILE"
  HANDOFF_DONE=1
  log "[handoff] sensor_base_stack written: $SENSOR_BASE_STACK_JSON"
  log "[handoff] ack written for session=${QWEN_NAV_HANDOFF_SESSION_ID:-unknown}"
  log "[handoff] exiting wrapper without stopping lidar/chassis/scan_filter"
  # Disarm full cleanup; exit 0 and leave sensors running for Nav2 ownership.
  trap - EXIT TERM
  exit 0
}

on_usr1_handoff() {
  log "[handoff] received SIGUSR1"
  perform_nav_handoff
}

cleanup() {
  if [[ "$HANDOFF_DONE" -eq 1 || "$CONTROLLED_NAV_HANDOFF" -eq 1 ]]; then
    log "[cleanup] handoff done — skip stopping sensor base stack"
    return 0
  fi
  echo ""
  log "[cleanup] publishing zero /cmd_vel..."
  publish_zero_cmd

  log "[cleanup] stopping processes started by this script..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  pkill -f "async_slam_toolbox_node" 2>/dev/null || true
  pkill -f "sync_slam_toolbox_node" 2>/dev/null || true
  pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
  pkill -f "ydlidar_ros2_driver_node" 2>/dev/null || true
  pkill -f "start_lidar_only.sh" 2>/dev/null || true
}

# Do not trap INT: when started under setsid from run_joy_mapping_all.sh,
# parent Ctrl+C should not tear down slam_toolbox before map save.
trap cleanup EXIT TERM
trap on_usr1_handoff USR1

ensure_slam_config() {
  if [ -f "$SLAM_CONFIG" ]; then
    log "SLAM config OK: $SLAM_CONFIG"
    return 0
  fi

  mkdir -p "$(dirname "$SLAM_CONFIG")"

  cat > "$SLAM_CONFIG" <<'YAML'
slam_toolbox:
  ros__parameters:
    use_sim_time: false

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan_filtered

    mode: mapping
    resolution: 0.05
    max_laser_range: 4.0

    minimum_time_interval: 0.1
    transform_timeout: 1.0
    tf_buffer_duration: 60.0
    map_update_interval: 1.0
    throttle_scans: 1
    transform_publish_period: 0.02

    debug_logging: false
    enable_interactive_mode: true
    stack_size_to_use: 40000000
YAML

  log "SLAM config ready: $SLAM_CONFIG"
}

start_background() {
  local name="$1"
  shift
  local log_file="${LOG_DIR}/${name}.log"
  local pid_file="${PID_DIR}/${name}.pid"

  log "Starting ${name} -> ${log_file}"
  "$@" > "$log_file" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  NAMED_PIDS["$name"]="$pid"
  mkdir -p "$PID_DIR"
  echo "$pid" > "$pid_file"
  sleep 0.5
}

wait_topic_exists() {
  local topic="$1"
  local timeout_sec="${2:-60}"
  local probe="${PROJECT_DIR}/scripts/lib/ros_topic_probe.py"
  local start now
  start="$(date +%s)"

  log "Waiting for ${topic} ..."
  while true; do
    # Prefer rclpy probe: avoids hung/stale ros2cli daemon false negatives.
    if [[ -f "$probe" ]] && [[ "$topic" =~ ^/(scan|scan_filtered|odom|map|tf|tf_static)$ ]]; then
      if python3 "$probe" has-samples "$topic" 1 2 >/dev/null 2>&1; then
        log "OK: ${topic}"
        return 0
      fi
    elif timeout 3 ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      log "OK: ${topic}"
      return 0
    fi
    # Lidar can be scanning while CLI discovery is broken; accept fresh driver log for /scan.
    if [[ "$topic" == "/scan" ]] \
      && pgrep -f 'ydlidar_ros2_driver_node' >/dev/null 2>&1 \
      && [[ -f "${PROJECT_DIR}/logs/lidar_driver.log" ]] \
      && find "${PROJECT_DIR}/logs/lidar_driver.log" -mmin -2 >/dev/null 2>&1 \
      && tail -n 40 "${PROJECT_DIR}/logs/lidar_driver.log" 2>/dev/null \
           | grep -q "Now lidar is scanning"; then
      log "OK: ${topic} (lidar driver scanning; CLI discovery flaky)"
      return 0
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_sec )); then
      break
    fi
    sleep 1
  done

  log "FAIL: timeout waiting for ${topic}"
  return 1
}

# Reuse wait_tf_chain.py for a single link by making the other lookups trivial.
# Example: wait_tf_link odom laser 30  -> waits odom <- laser
wait_tf_link() {
  local parent="$1" child="$2" timeout_sec="${3:-45}"
  local waiter="${PROJECT_DIR}/scripts/nav/wait_tf_chain.py"
  log "Waiting TF ${parent} <- ${child} (timeout ${timeout_sec}s) ..."
  if [[ ! -f "$waiter" ]]; then
    log "WARN: missing $waiter; sleep ${timeout_sec}s fallback"
    sleep "$timeout_sec"
    return 0
  fi
  if python3 "$waiter" \
    --map-frame "$parent" \
    --odom-frame "$child" \
    --base-frame "$child" \
    --timeout "$timeout_sec" \
    --need-ok 2 \
    --poll 0.5; then
    log "OK: TF ${parent} <- ${child}"
    return 0
  fi
  log "FAIL: TF ${parent} <- ${child} not ready"
  return 1
}

show_status() {
  echo ""
  echo "========== ROS TOPICS =========="
  ros2 topic list | sort | egrep "scan|odom|tf|map|cmd_vel|joy|chassis" || true

  echo ""
  echo "========== /scan info =========="
  ros2 topic info /scan -v 2>/dev/null | head -40 || true

  echo ""
  echo "========== /odom info =========="
  ros2 topic info /odom -v 2>/dev/null | head -40 || true

  echo ""
  echo "========== /map_metadata once =========="
  timeout 5 ros2 topic echo /map_metadata --once 2>/dev/null | head -25 || true

  echo ""
  echo "========== TF odom -> laser =========="
  timeout 8 ros2 run tf2_ros tf2_echo odom laser 2>/dev/null | head -25 || true

  echo ""
  echo "========== Logs if something is missing =========="
  echo "LiDAR log:         ${LOG_DIR}/lidar.log"
  echo "Scan filter log:   ${LOG_DIR}/scan_filter.log"
  echo "Chassis log:       ${LOG_DIR}/chassis_bridge.log"
  echo "SLAM log:          ${LOG_DIR}/slam_toolbox.log"
  echo "Foxglove log:      ${LOG_DIR}/foxglove_bridge.log"
}

main() {
  log "===== Corridor SLAM Live (Foxglove, known-good style) ====="
  log "PROJECT_DIR=$PROJECT_DIR"
  log "No auto motion. Drive manually via joystick /cmd_vel."
  log "Handoff: touch $HANDOFF_REQUEST_FILE or SIGUSR1 for controlled Nav2 handoff"

  mkdir -p "$LOG_DIR"
  PID_DIR="${LOG_DIR}/pids"
  mkdir -p "$PID_DIR" "$(dirname "$HANDOFF_REQUEST_FILE")"
  rm -f "$HANDOFF_REQUEST_FILE"

  if [ ! -e "$LIDAR_DEV" ]; then
    echo "FAIL: missing $LIDAR_DEV" >&2
    ls -l /dev/ydlidar /dev/rosmaster /dev/ttyUSB* 2>/dev/null || true
    exit 1
  fi

  if [ ! -e "$CHASSIS_DEV" ]; then
    echo "FAIL: missing $CHASSIS_DEV" >&2
    ls -l /dev/ydlidar /dev/rosmaster /dev/ttyUSB* 2>/dev/null || true
    exit 1
  fi

  ensure_slam_config

  log "Stopping old SLAM-related processes..."
  publish_zero_cmd
  cleanup_lidar_slam_nav_processes
  pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true
  pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
  pkill -f "ydlidar_ros2_driver_node" 2>/dev/null || true
  pkill -f "start_lidar_only.sh" 2>/dev/null || true
  sleep 1

  log "[1/6] LiDAR -> /scan"
  start_background lidar bash "${PROJECT_DIR}/scripts/lidar/start_lidar_only.sh" --foreground
  sleep 6

  wait_topic_exists /scan 30 || {
    log "FAIL: /scan not found"
    exit 1
  }

  log "[scan_filter] Start /scan -> /scan_filtered"
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

  wait_topic_exists /scan_filtered 30 || {
    log "FAIL: /scan_filtered not found"
    exit 1
  }

  log "[2/6] Chassis PWM bridge + /odom"
  source "${PROJECT_DIR}/scripts/lib/load_mvp_tune.sh"
  if [ "${SLAM_USE_CALIBRATION:-0}" = "1" ]; then
    # load_mvp_tune 会覆盖 CHASSIS_MAX_VX 等；校准模式下重新应用建图专用参数
    # shellcheck source=scripts/lib/slam_calibrated_env.sh
    source "${PROJECT_DIR}/scripts/lib/slam_calibrated_env.sh"
    log "Calibration mode: re-applied slam_calibrated_env after mvp_tune"
  fi
  source "${PROJECT_DIR}/scripts/lib/run_chassis_bridge.sh"
  export CHASSIS_PORT="${CHASSIS_DEV}"
  run_chassis_bridge "${LOG_DIR}/chassis_bridge.log"
  sleep 3
  # slam_toolbox needs odom->base_link before any scan; starting too early fills
  # its MessageFilter and it never publishes map->odom (hub then TF-timeouts).
  wait_tf_link odom base_link 40 || {
    log "FAIL: odom<-base_link missing (chassis odom TF)"
    exit 1
  }

  log "[3/6] Static TF base_link -> ${LASER_FRAME}"
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
  wait_tf_link odom "${LASER_FRAME}" 30 || {
    log "FAIL: odom<-${LASER_FRAME} missing (static TF / odom)"
    exit 1
  }

  log "[4/6] slam_toolbox online_async (scan_topic=/scan_filtered)"
  start_background slam_toolbox \
    ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:="${SLAM_CONFIG}"
  # Give toolbox time to bind before hammering the graph with status CLI.
  sleep 5

  if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    log "[5/6] foxglove_bridge port ${FOXGLOVE_PORT}"
    if ! ensure_foxglove_port_free "${FOXGLOVE_PORT}" 12; then
      log "ERROR: Foxglove port ${FOXGLOVE_PORT} busy; /scan will NOT show in Foxglove"
      log "HINT: stop nav2/semantic explore, or: pkill -f run_nav2_foxglove_click_goal.sh"
      FOXGLOVE_STARTED=0
    else
      start_background foxglove_bridge \
        bash "${PROJECT_DIR}/scripts/lidar/start_foxglove.sh"
      sleep 3
      FOXGLOVE_STARTED=0
      for _ in 1 2 3 4 5 6; do
        if foxglove_bridge_log_looks_healthy "${LOG_DIR}/foxglove_bridge.log"; then
          FOXGLOVE_STARTED=1
          break
        fi
        sleep 1
      done
      if [ "$FOXGLOVE_STARTED" = "1" ]; then
        log "OK: foxglove_bridge listening on ${FOXGLOVE_PORT}"
      else
        FOXGLOVE_STARTED=0
        log "ERROR: foxglove_bridge failed (see ${LOG_DIR}/foxglove_bridge.log)"
        if grep -q "Bind Error" "${LOG_DIR}/foxglove_bridge.log" 2>/dev/null; then
          log "ERROR: Bind Error — another process still owns port ${FOXGLOVE_PORT}"
        fi
      fi
    fi
  else
    log "[5/6] foxglove_bridge not installed, skipping"
  fi

  # Real readiness = map->odom TF (not merely /map publisher advertisement).
  log "Waiting for TF map <- odom (slam_toolbox pose) ..."
  if ! wait_tf_link map odom 60; then
    log "FAIL: map<-odom never appeared — slam is dropping scans; see ${LOG_DIR}/slam_toolbox.log"
    if grep -q 'discarding message because the queue is full' "${LOG_DIR}/slam_toolbox.log" 2>/dev/null; then
      log "HINT: MessageFilter queue full — odom/laser TF was late or CPU starved at slam start"
    fi
    exit 1
  fi

  # Heavy ros2 CLI status dump AFTER TF is healthy (was starving slam at boot).
  if [ "${SLAM_SKIP_STATUS_DUMP:-0}" != "1" ]; then
    show_status
  fi

  log "===== SLAM live stack started ====="
  log "Do NOT close this terminal."
  if [ "$FOXGLOVE_STARTED" = "1" ]; then
    BOARD_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
    log "Foxglove: ws://${BOARD_IP}:${FOXGLOVE_PORT}"
    log "Foxglove 3D: Fixed frame=map, enable /scan_filtered + /map + TF"
  else
    log "WARN: Foxglove bridge NOT running — lidar/SLAM OK but no ws://${FOXGLOVE_PORT}"
    log "WARN: tail -f ${LOG_DIR}/foxglove_bridge.log"
  fi
  log "Now open terminal 2: joy_node."
  log "Then terminal 3: teleop_twist_joy."
  log "Terminal 4: monitoring and map saving."
  log "Nav handoff: touch ${HANDOFF_REQUEST_FILE}  OR  kill -USR1 $$"

  while true; do
    if [[ -f "$HANDOFF_REQUEST_FILE" ]]; then
      perform_nav_handoff
    fi
    sleep 1
  done
}

main "$@"
