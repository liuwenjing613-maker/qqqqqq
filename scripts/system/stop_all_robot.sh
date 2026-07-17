#!/usr/bin/env bash
# Stop every RDK X5 VLN / SLAM / Nav2 / voice / camera / chassis process
# started by this repository. Safe to run repeatedly.
set +e
set +u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SELF_PID="$$"
PARENT_PID="${PPID:-}"

log() { echo "[STOP] $*"; }

# All matches use pgrep/pkill -f (cmdline regex).
# Do NOT use pkill -x: Linux comm is truncated to 15 chars
# (waypoint_follower / velocity_smoother / lifecycle_manager never match -x).
PATTERN_LIST=(
  # Launchers
  'start_v1_map_qwen_simple_voice_v2\.sh'
  'start_v1_map_qwen_fullflow_v2\.sh'
  'start_live_servo_voice\.sh'
  'start_live_servo\.sh'
  'start_live_debug\.sh'
  'start_debug_node\.sh'
  'run_slam_calibrated\.sh'
  'run_corridor_mapping'
  'start_foxglove\.sh'
  'run_voice_instruction_once'
  # Fusion / handoff
  'simple_handoff_supervisor'
  'simple_handoff_event_console'
  'online_map_plan_bridge_node'
  'online_map_qwen_nav_backend'
  'cmd_vel_intervention_mux'
  'log_fan_in_v2\.py'
  'nav2_online_slam_fusion_navigation_launch'
  # Qwen / camera
  'qwen_visual_servo_node'
  'qwen_vln_debug_node'
  'opencv_compressed_cam'
  'compressed_to_raw_image'
  'cmd_vel_priority_mux'
  # Chassis / lidar / SLAM
  'm1_pwm_cmd_vel_bridge'
  'cmd_vel_to_rosmaster'
  'simple_scan_filter'
  'ydlidar_ros2_driver'
  'async_slam_toolbox'
  'online_async_launch\.py'
  'slam_toolbox'
  # Nav2 (match binary path / argv0; -x is unreliable for long names)
  'nav2_controller/controller_server'
  'nav2_planner/planner_server'
  'nav2_smoother/smoother_server'
  'nav2_behaviors/behavior_server'
  'nav2_bt_navigator/bt_navigator'
  'nav2_waypoint_follower/waypoint_follower'
  'nav2_velocity_smoother/velocity_smoother'
  'nav2_lifecycle_manager/lifecycle_manager'
  'controller_server --ros-args'
  'planner_server --ros-args'
  'smoother_server --ros-args'
  'behavior_server --ros-args'
  'bt_navigator --ros-args'
  'waypoint_follower --ros-args'
  'velocity_smoother --ros-args'
  'lifecycle_manager --ros-args'
  # Foxglove / USB cam node
  'foxglove_bridge'
  'hobot_usb_cam'
  # Legacy MVP
  'run_mvp_task\.py'
  'red_target_servo_ros'
  'yolo_world_servo_ros'
  'keyboard_cmd_vel\.py'
)

is_protected_pid() {
  local pid="$1"
  [[ -z "$pid" ]] && return 0
  [[ "$pid" == "$SELF_PID" || "$pid" == "$PARENT_PID" ]] && return 0
  local cmd=""
  if [[ -r "/proc/$pid/cmdline" ]]; then
    cmd="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
  fi
  [[ "$cmd" == *stop_all_robot.sh* ]] && return 0
  return 1
}

list_match_pids() {
  local pat pid
  for pat in "${PATTERN_LIST[@]}"; do
    for pid in $(pgrep -f -- "$pat" 2>/dev/null || true); do
      is_protected_pid "$pid" && continue
      printf '%s\n' "$pid"
    done
  done
}

kill_pid_tree() {
  local sig="$1"
  local pid="$2"
  local child
  [[ -z "$pid" ]] && return 0
  kill -0 "$pid" 2>/dev/null || return 0
  is_protected_pid "$pid" && return 0
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    kill_pid_tree "$sig" "$child"
  done
  kill "-$sig" -- "-$pid" 2>/dev/null || true
  kill "-$sig" -- "$pid" 2>/dev/null || true
}

signal_all() {
  local sig="$1"
  local pid
  local -a pids=()
  while read -r pid; do
    [[ -n "$pid" ]] && pids+=("$pid")
  done < <(list_match_pids | sort -u)

  if [[ "${#pids[@]}" -eq 0 ]]; then
    log "no matching PIDs for $sig"
    return 0
  fi
  log "$sig ${#pids[@]} process(es): ${pids[*]}"
  for pid in "${pids[@]}"; do
    kill_pid_tree "$sig" "$pid"
  done
}

publish_zero_cmd() {
  # Skip when nothing is up — ros2 CLI timeouts dominate empty runs.
  if ! pgrep -f -- 'm1_pwm_cmd_vel_bridge|qwen_visual_servo|controller_server|cmd_vel_priority_mux|cmd_vel_intervention_mux' >/dev/null 2>&1; then
    return 0
  fi
  (
    set +e
    set +u
    if [[ -f /opt/tros/humble/setup.bash ]]; then
      # shellcheck disable=SC1091
      source /opt/tros/humble/setup.bash >/dev/null 2>&1
    elif [[ -f /opt/ros/humble/setup.bash ]]; then
      # shellcheck disable=SC1091
      source /opt/ros/humble/setup.bash >/dev/null 2>&1
    else
      exit 0
    fi
    local twist='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
    timeout 1 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "$twist" >/dev/null 2>&1
    timeout 1 ros2 topic pub --once /cmd_vel_autonomy geometry_msgs/msg/Twist "$twist" >/dev/null 2>&1
    timeout 1 ros2 topic pub --once /qwen_vln/servo/command std_msgs/msg/String \
      "{data: 'disable'}" >/dev/null 2>&1
  ) || true
}

clear_runtime_junk() {
  rm -f /tmp/rdk_x5_vln_fullflow_v2_*/pids.env 2>/dev/null || true
  rm -rf /tmp/rdk_x5_simple_handoff_v2_* 2>/dev/null || true
  rm -rf /tmp/rdk_x5_vln_fullflow_v2_* 2>/dev/null || true
  rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null || true
  # Hard-timeout daemon stop; a wedged daemon must not block this script.
  (
    set +e
    set +u
    if [[ -f /opt/tros/humble/setup.bash ]]; then
      # shellcheck disable=SC1091
      source /opt/tros/humble/setup.bash >/dev/null 2>&1
    elif [[ -f /opt/ros/humble/setup.bash ]]; then
      # shellcheck disable=SC1091
      source /opt/ros/humble/setup.bash >/dev/null 2>&1
    fi
    timeout 2 ros2 daemon stop >/dev/null 2>&1
  ) || true
}

remaining_robot_procs() {
  ps -eo pid,cmd 2>/dev/null | awk '
    /rdk_x5_vln_robot|async_slam_toolbox|ydlidar_ros2|foxglove_bridge|hobot_usb_cam|nav2_|bt_navigator|controller_server|planner_server|smoother_server|behavior_server|waypoint_follower|velocity_smoother|lifecycle_manager|opencv_compressed_cam|qwen_visual_servo|qwen_vln_debug|simple_handoff|online_map_qwen|m1_pwm_cmd_vel|simple_scan_filter|start_v1_map_qwen|start_live_servo|run_slam_calibrated|run_corridor_mapping|slam_toolbox/ &&
    !/awk/ && !/stop_all_robot\.sh/ { print }
  ' || true
}

log "project=$PROJECT_DIR"
log "publish zero velocity / disable servo"
publish_zero_cmd

HAD_TARGETS=0
if list_match_pids | grep -q '[0-9]'; then
  HAD_TARGETS=1
fi

signal_all TERM
if [[ "$HAD_TARGETS" == "1" ]]; then
  sleep 1.0
fi
signal_all KILL
if [[ "$HAD_TARGETS" == "1" ]]; then
  sleep 0.3
fi

# Final pattern sweep.
for pat in "${PATTERN_LIST[@]}"; do
  pkill -KILL -f -- "$pat" 2>/dev/null || true
done

if command -v fuser >/dev/null 2>&1; then
  fuser -k /dev/video0 /dev/video1 >/dev/null 2>&1 || true
fi

# Only wipe DDS/runtime junk when we actually tore something down.
if [[ "$HAD_TARGETS" == "1" ]]; then
  clear_runtime_junk
fi

LEFT="$(remaining_robot_procs)"
if [[ -n "${LEFT}" ]]; then
  log "WARN: still alive — force kill survivors"
  printf '%s\n' "$LEFT"
  while read -r pid _rest; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    is_protected_pid "$pid" && continue
    kill -KILL "$pid" 2>/dev/null || true
  done <<<"$LEFT"
  sleep 0.3
  LEFT="$(remaining_robot_procs)"
  if [[ -n "${LEFT}" ]]; then
    log "ERROR: could not stop:"
    printf '%s\n' "$LEFT"
    exit 1
  fi
fi

log "all robot processes stopped"
exit 0
