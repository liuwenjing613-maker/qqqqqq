#!/usr/bin/env bash
# Stop every RDK X5 VLN / SLAM / Nav2 / voice / camera / chassis process
# started by this repository. Safe to run repeatedly.
#
# Goals for next cold start:
#   - no orphan lidar/chassis/ROS nodes holding serial or DDS participants
#   - no stale FastDDS shm / ros2-daemon (common cause of "Waiting for /scan")
#   - no stale hub locks / runtime pid+handoff files that confuse reuse logic
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
  # Voice demo hub — kill the python controller (and KWS/prewarm). Matching
  # start_voice_demo_hub_v1.sh is intentionally omitted: stop&&start chains
  # embed that path in the parent cmdline and would get killed mid-restart.
  'voice_demo_hub_v1\.py'
  'voice_demo_hub_web_view_v1\.py'
  'prewarm_qwen_ego_v1\.sh'
  'kws_function_event_server_v1\.py'
  'run_voice_function_event_server_v1\.sh'
  # Standalone camera preview (conflicts with hub qwen prewarm on /dev/video0)
  'start_foxglove_camera\.sh'
  'web_mjpeg_stream\.py'
  'foxglove_image_throttle'
  # Launchers
  'start_v1_map_qwen_simple_voice_v2\.sh'
  'start_v1_map_qwen_fullflow_v2\.sh'
  'start_live_servo_voice\.sh'
  'start_live_servo\.sh'
  'start_live_debug\.sh'
  'start_debug_node\.sh'
  'run_slam_calibrated\.sh'
  'run_corridor_mapping'
  'run_joy_mapping_calibrated'
  'run_joy_mapping_qwen_plan'
  'run_joy_map_qwen_plan_session'
  'run_nav2_saved_map\.sh'
  'run_nav2_foxglove_click_goal\.sh'
  'run_qwen_target_nav2_reuse\.sh'
  'run_qwen_session_nav2_goal\.sh'
  'start_lidar_only\.sh'
  'start_foxglove\.sh'
  'run_voice_instruction_once'
  'wait_tf_chain\.py'
  # Fusion / handoff
  'simple_handoff_supervisor'
  'simple_handoff_event_console'
  'online_map_plan_bridge_node'
  'online_map_qwen_nav_backend'
  'cmd_vel_intervention_mux'
  'log_fan_in_v2\.py'
  'nav2_online_slam_fusion_navigation_launch'
  'foxglove_click_goal_bridge'
  'pose_memory_node'
  'nav2_s_diagnose'
  # Qwen / camera
  'qwen_visual_servo_node'
  'qwen_vln_debug_node'
  'opencv_compressed_cam'
  'compressed_to_raw_image'
  'cmd_vel_priority_mux'
  # Chassis / lidar / SLAM / TF
  'm1_pwm_cmd_vel_bridge'
  'cmd_vel_to_rosmaster'
  'run_chassis_bridge\.sh'
  'simple_scan_filter'
  'ydlidar_ros2_driver'
  'start_lidar_only'
  'static_transform_publisher.*base_link.*laser'
  'async_slam_toolbox'
  'online_async_launch\.py'
  'slam_toolbox'
  # Teleop (often left after mapping)
  'teleop_twist_joy'
  'joy_node'
  # Nav2 (match binary path / argv0; -x is unreliable for long names)
  'nav2_controller/controller_server'
  'nav2_planner/planner_server'
  'nav2_smoother/smoother_server'
  'nav2_behaviors/behavior_server'
  'nav2_bt_navigator/bt_navigator'
  'nav2_waypoint_follower/waypoint_follower'
  'nav2_velocity_smoother/velocity_smoother'
  'nav2_lifecycle_manager/lifecycle_manager'
  'nav2_map_server/map_server'
  'nav2_amcl/amcl'
  'controller_server --ros-args'
  'planner_server --ros-args'
  'smoother_server --ros-args'
  'behavior_server --ros-args'
  'bt_navigator --ros-args'
  'waypoint_follower --ros-args'
  'velocity_smoother --ros-args'
  'lifecycle_manager --ros-args'
  'map_server --ros-args'
  'amcl --ros-args'
  'nav2_bringup'
  'nav2_click_nav_bringup'
  # Explore / semantic
  'run_shared_nav_semantic_explore'
  'semantic_mapper_node'
  'explore_goal_selector'
  # Foxglove / USB cam node
  'foxglove_bridge'
  'hobot_usb_cam'
  # Hung ros2 CLI probes (hold DDS participants after Ctrl-C)
  'ros2 topic info'
  'ros2 topic list'
  'ros2 topic echo'
  'ros2 topic hz'
  'ros2 topic pub'
  'ros2 run tf2_ros'
  # Legacy MVP
  'run_mvp_task\.py'
  'red_target_servo_ros'
  'yolo_world_servo_ros'
  'keyboard_cmd_vel\.py'
)

is_protected_pid() {
  local pid="$1"
  [[ -z "$pid" ]] && return 0
  # Protect this script and every ancestor shell. Critical when callers run:
  #   bash stop_all_robot.sh; bash start_voice_demo_hub_v1.sh
  # because pgrep -f 'start_voice_demo_hub_v1\.sh' matches the parent cmdline
  # before start even runs, and would suicide the restart chain.
  local walk="$SELF_PID"
  local depth=0
  while [[ -n "$walk" && "$walk" != "0" && "$depth" -lt 12 ]]; do
    [[ "$pid" == "$walk" ]] && return 0
    walk="$(ps -o ppid= -p "$walk" 2>/dev/null | tr -d ' ')"
    depth=$((depth + 1))
  done
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
    # Prefer project UDP-only profile so this pub does not recreate SHM segments.
    if [[ -f "${PROJECT_DIR}/configs/fastdds_no_shm.xml" ]]; then
      export FASTRTPS_DEFAULT_PROFILES="${PROJECT_DIR}/configs/fastdds_no_shm.xml"
    fi
    local twist='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
    timeout 1 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "$twist" >/dev/null 2>&1
    timeout 1 ros2 topic pub --once /cmd_vel_autonomy geometry_msgs/msg/Twist "$twist" >/dev/null 2>&1
    timeout 1 ros2 topic pub --once /cmd_vel_ego geometry_msgs/msg/Twist "$twist" >/dev/null 2>&1
    timeout 1 ros2 topic pub --once /cmd_vel_map geometry_msgs/msg/Twist "$twist" >/dev/null 2>&1
    timeout 1 ros2 topic pub --once /qwen_vln/servo/command std_msgs/msg/String \
      "{data: 'disable'}" >/dev/null 2>&1
  ) || true
}

release_device_holds() {
  # Free camera + serial so next lidar/chassis open is not EBUSY.
  if command -v fuser >/dev/null 2>&1; then
    fuser -k /dev/video0 /dev/video1 >/dev/null 2>&1 || true
    fuser -k /dev/ydlidar /dev/rosmaster \
      /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2 \
      /dev/ttyACM0 /dev/ttyACM1 >/dev/null 2>&1 || true
  fi
}

clear_fastdds_shm() {
  # Leftover FastDDS shared-memory / port locks cause slow or failed topic
  # discovery on the next start (hub stuck on Waiting for /scan).
  rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null || true
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
}

stop_ros2_daemon() {
  # Stale ros2-daemon is a common source of bogus topic list / discovery.
  local pid
  for pid in $(pgrep -f -- 'ros2cli\.daemon|ros2-daemon' 2>/dev/null || true); do
    is_protected_pid "$pid" && continue
    kill -TERM "$pid" 2>/dev/null || true
  done
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
    if [[ -f "${PROJECT_DIR}/configs/fastdds_no_shm.xml" ]]; then
      export FASTRTPS_DEFAULT_PROFILES="${PROJECT_DIR}/configs/fastdds_no_shm.xml"
    fi
    timeout 2 ros2 daemon stop >/dev/null 2>&1
  ) || true
  sleep 0.2
  for pid in $(pgrep -f -- 'ros2cli\.daemon|ros2-daemon' 2>/dev/null || true); do
    is_protected_pid "$pid" && continue
    kill -KILL "$pid" 2>/dev/null || true
  done
}

clear_runtime_junk() {
  log "clear DDS shm + ros2 daemon + runtime markers"
  clear_fastdds_shm
  stop_ros2_daemon
  # Second pass after daemon dies — daemon may recreate shm on stop race.
  clear_fastdds_shm

  rm -f /tmp/rdk_x5_vln_fullflow_v2_*/pids.env 2>/dev/null || true
  rm -rf /tmp/rdk_x5_simple_handoff_v2_* 2>/dev/null || true
  rm -rf /tmp/rdk_x5_vln_fullflow_v2_* 2>/dev/null || true
  # Hub singleton lock — leftover after Ctrl-C blocks next start_voice_demo_hub.
  rm -f /tmp/rdk_x5_voice_demo_hub_v1.lock 2>/dev/null || true
  # Stale lidar pid / handoff ownership confuse reuse and next cold start.
  rm -f "${PROJECT_DIR}/runtime/ydlidar_driver.pid" 2>/dev/null || true
  rm -f "${PROJECT_DIR}/runtime/request_nav_handoff" 2>/dev/null || true
  rm -f "${PROJECT_DIR}/runtime/nav_handoff_active_session" 2>/dev/null || true
  rm -f "${PROJECT_DIR}/runtime/sensor_base_stack.json" 2>/dev/null || true
}

remaining_robot_procs() {
  ps -eo pid,cmd 2>/dev/null | awk '
    /voice_demo_hub_v1|prewarm_qwen_ego|kws_function_event_server_v1|async_slam_toolbox|ydlidar_ros2|foxglove_bridge|hobot_usb_cam|nav2_|bt_navigator|controller_server|planner_server|smoother_server|behavior_server|waypoint_follower|velocity_smoother|lifecycle_manager|map_server|amcl|opencv_compressed_cam|qwen_visual_servo|qwen_vln_debug|simple_handoff|online_map_qwen|m1_pwm_cmd_vel|simple_scan_filter|start_v1_map_qwen|start_live_servo|run_slam_calibrated|run_corridor_mapping|run_nav2_saved_map|run_nav2_foxglove|start_lidar_only|slam_toolbox|static_transform_publisher|teleop_twist_joy|joy_node|\/rdk_x5_vln_robot\// &&
    !/awk/ && !/stop_all_robot\.sh/ && !/cursor-server/ && !/ripgrep/ && !/\/rg / { print }
  ' || true
}

log "project=$PROJECT_DIR"
if pgrep -f -- 'voice_demo_hub_v1\.py' >/dev/null 2>&1; then
  log "WARN: voice_demo_hub_v1 is running — stopping hub + owned stacks together (avoid partial kill conflict)"
fi
log "publish zero velocity / disable servo"
publish_zero_cmd

HAD_TARGETS=0
if list_match_pids | grep -q '[0-9]'; then
  HAD_TARGETS=1
fi

signal_all TERM
if [[ "$HAD_TARGETS" == "1" ]]; then
  # Give lidar/chassis a moment to release serial before KILL/fuser.
  sleep 1.5
fi
signal_all KILL
if [[ "$HAD_TARGETS" == "1" ]]; then
  sleep 0.4
fi

# Final pattern sweep (still respect ancestor protection — never raw pkill -f).
while read -r pid; do
  [[ -n "$pid" ]] || continue
  is_protected_pid "$pid" && continue
  kill -KILL "$pid" 2>/dev/null || true
done < <(list_match_pids | sort -u)

release_device_holds

# Always wipe DDS/runtime junk — orphans may already be dead with shm left behind,
# which is exactly what breaks the next /scan discovery.
clear_runtime_junk

# Brief settle so USB-serial re-enumerate before the next launcher opens ports.
sleep 0.5

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
  release_device_holds
  clear_fastdds_shm
  LEFT="$(remaining_robot_procs)"
  if [[ -n "${LEFT}" ]]; then
    log "ERROR: could not stop:"
    printf '%s\n' "$LEFT"
    exit 1
  fi
fi

# Final shm check (best-effort report).
if ls /dev/shm/fastrtps* /dev/shm/sem.fastrtps* >/dev/null 2>&1; then
  log "WARN: FastDDS shm still present after cleanup:"
  ls -la /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null || true
  clear_fastdds_shm
fi

log "all robot processes stopped; DDS/runtime cleaned for next start"
exit 0
