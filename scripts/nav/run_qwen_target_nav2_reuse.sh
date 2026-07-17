#!/usr/bin/env bash
# Qwen target -> Nav2 reuse pipeline (post navigation_goal_proposal.json).
# Stages: handoff -> repair_sensor_base_once -> read-only health -> Nav2 overlay -> AMCL -> path/nav
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_DIR"

# shellcheck source=scripts/lib/ros_dds_env.sh
source "${PROJECT_DIR}/scripts/lib/ros_dds_env.sh"
# shellcheck source=scripts/lib/nav2_stack_reuse.sh
source "${PROJECT_DIR}/scripts/lib/nav2_stack_reuse.sh"
# shellcheck source=scripts/lib/slam_calibrated_env.sh
source "${PROJECT_DIR}/scripts/lib/slam_calibrated_env.sh"
# shellcheck source=scripts/lib/lidar_frame_config.sh
source "${PROJECT_DIR}/scripts/lib/lidar_frame_config.sh"

SESSION_ID=""
SESSION_DIR=""
MAP_YAML=""
POSE_JSON=""
GOAL_JSON=""
CANDIDATE_BUNDLE=""
MAPPING_PID=""
START_ONLY=0
COMPUTE_PATH_ONLY=0
STOP_NAV2_ON_EXIT=0
NAV2_PARAMS="${PROJECT_DIR}/configs/nav2_params.yaml"
NAV2_BRINGUP="${PROJECT_DIR}/configs/nav2_click_nav_bringup_launch.py"

NAV2_LAUNCH_PID=""
NAV2_PGID=""
RUNTIME_DIR=""
HANDOFF_DIR=""
PROBE="${PROJECT_DIR}/scripts/nav/qwen_nav2_runtime_probe.py"

usage() { sed -n '1,40p' "$0" | tail -n +2; }

log() {
  echo "[$(date +%H:%M:%S)][QWEN_NAV2] $*" | tee -a "${RUNTIME_DIR}/orchestrator.log"
}

source_ros_environment() {
  set +u
  if [[ -f /opt/tros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/tros/humble/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "$HOME/ydlidar_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/ydlidar_ws/install/setup.bash"
  fi
  attach_ros_dds_env
  set -u
}

publish_zero_velocity() {
  local duration="${1:-1.0}"
  source_ros_environment
  timeout "${duration}" ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
    >/dev/null 2>&1 || true
}

wait_robot_stopped() {
  local timeout=5 start now
  start="$(date +%s)"
  while true; do
    now="$(date +%s)"
    if (( now - start >= timeout )); then
      log "FAIL: robot stop timeout"
      return 1
    fi
    if python3 - <<'PY'
import time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
samples = []
rclpy.init()
node = Node("qwen_nav_stop_probe")
node.create_subscription(Odometry, "/odom", lambda m: samples.append(m), qos_profile_sensor_data)
end = time.time() + 1.2
while time.time() < end and len(samples) < 5:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node(); rclpy.shutdown()
if len(samples) < 5:
    raise SystemExit(1)
ok = 0
for msg in samples[-5:]:
    if abs(msg.twist.twist.linear.x) < 0.015 and abs(msg.twist.twist.linear.y) < 0.015 and abs(msg.twist.twist.angular.z) < 0.03:
        ok += 1
raise SystemExit(0 if ok >= 5 else 1)
PY
    then
      log "CONTROL_STOPPED"
      return 0
    fi
    sleep 0.3
  done
}

write_state() {
  local state="$1" reason="${2:-}"
  python3 - "$RUNTIME_DIR/nav2_state.json" "$SESSION_ID" "$state" "$MAP_YAML" "$reason" <<'PY'
import json, os, sys, time
from pathlib import Path
path, sid, state, my, reason = sys.argv[1:6]
payload = {"session_id": sid, "state": state, "updated_epoch": time.time(), "map_yaml": my, "failure_reason": reason or None}
tmp = Path(path).with_suffix(".json.tmp")
tmp.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
os.replace(tmp, path)
PY
}

slam_exit_confirmed_thrice() {
  local i ok
  for i in 1 2 3; do
    ok=1
    if slam_toolbox_running; then ok=0; fi
    if ! python3 - <<'PY'
import rclpy
from rclpy.node import Node
rclpy.init()
node = Node("qwen_map_pub_gate")
infos = node.get_publishers_info_by_topic("/map")
slam = [p for p in infos if "slam" in (p.node_name or "").lower()]
node.destroy_node(); rclpy.shutdown()
raise SystemExit(1 if slam else 0)
PY
    then ok=0; fi
    if pgrep -f "teleop_twist_joy|joy_node" >/dev/null 2>&1; then ok=0; fi
    if [[ "$ok" -ne 1 ]]; then return 1; fi
    sleep 0.7
  done
  return 0
}

stop_mapping_control_handoff() {
  local request_json ack_json requested_epoch cmdline
  HANDOFF_DIR="${PROJECT_DIR}/runtime/nav_handoff/${SESSION_ID}"
  mkdir -p "$HANDOFF_DIR"
  request_json="${HANDOFF_DIR}/request.json"
  ack_json="${HANDOFF_DIR}/ack.json"
  rm -f "$ack_json"

  log "MAPPING_HANDOFF: validate MAPPING_PID then USR1"
  publish_zero_velocity 1.0

  if [[ -z "${MAPPING_PID:-}" ]]; then
    log "FAIL: MAPPING_PID empty"
    return 1
  fi
  if ! kill -0 "$MAPPING_PID" 2>/dev/null; then
    log "FAIL: MAPPING_PID=$MAPPING_PID not alive"
    return 1
  fi
  cmdline="$(tr '\0' ' ' < "/proc/${MAPPING_PID}/cmdline" 2>/dev/null || true)"
  if [[ "$cmdline" != *run_joy_mapping_calibrated* && "$cmdline" != *run_corridor_mapping* ]]; then
    log "FAIL: MAPPING_PID cmdline unexpected: $cmdline"
    return 1
  fi

  # Explicit frontier stop (required)
  bash "${PROJECT_DIR}/scripts/nav/stop_frontier_region_debug.sh" >> "${RUNTIME_DIR}/orchestrator.log" 2>&1 || true

  requested_epoch="$(python3 -c 'import time; print(time.time())')"
  python3 - "$request_json" "$SESSION_ID" "$requested_epoch" "$MAPPING_PID" "$cmdline" <<'PY'
import json, os, sys, time
from pathlib import Path
path, sid, epoch, pid, cmd = sys.argv[1:6]
payload = {
  "session_id": sid,
  "requested_epoch": float(epoch),
  "mapping_pid": int(pid),
  "mapping_cmdline": cmd,
  "expected_cmdlines": {
    "lidar_pid": "ydlidar",
    "scan_filter_pid": "simple_scan_filter",
    "chassis_pid": "m1_pwm_cmd_vel_bridge",
    "static_tf_pid": "static_transform_publisher",
    "foxglove_pid": "foxglove_bridge",
  },
}
tmp = Path(path).with_suffix(".tmp")
tmp.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
os.replace(tmp, path)
# also write legacy touch file for older wrappers
Path("/root/rdk_x5_vln_robot/runtime/request_nav_handoff").parent.mkdir(parents=True, exist_ok=True)
Path("/root/rdk_x5_vln_robot/runtime/request_nav_handoff").write_text(sid + "\n", encoding="utf-8")
# export session for corridor/joy wrappers
Path("/root/rdk_x5_vln_robot/runtime/nav_handoff_active_session").write_text(sid + "\n", encoding="utf-8")
PY
  export QWEN_NAV_HANDOFF_SESSION_ID="$SESSION_ID"
  export QWEN_NAV_HANDOFF_DIR="$HANDOFF_DIR"

  kill -USR1 "$MAPPING_PID" 2>/dev/null || true
  local pid
  for pid in $(pgrep -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true); do
    kill -USR1 "$pid" 2>/dev/null || true
  done

  # Wait for ack.json
  local i
  for i in $(seq 1 40); do
    [[ -f "$ack_json" ]] && break
    sleep 0.3
  done
  if [[ ! -f "$ack_json" ]]; then
    # Fallback: if legacy sensor_base_stack.json exists, synthesize ack (best effort)
    if [[ -f "${PROJECT_DIR}/runtime/sensor_base_stack.json" ]]; then
      python3 - "$ack_json" "$SESSION_ID" "${PROJECT_DIR}/runtime/sensor_base_stack.json" <<'PY'
import json, os, sys, time
from pathlib import Path
out, sid, legacy = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
d = json.loads(legacy.read_text(encoding="utf-8"))
pids = d.get("pids") or {}
# chassis from pgrep
import subprocess
def one(pat):
    try:
        o = subprocess.check_output(["pgrep","-f",pat], text=True).strip().splitlines()
        return int(o[0]) if o else None
    except Exception:
        return None
payload = {
  "session_id": sid,
  "state": "SENSOR_BASE_HELD",
  "completed_epoch": time.time(),
  "lidar_pid": pids.get("lidar"),
  "scan_filter_pid": pids.get("scan_filter"),
  "chassis_pid": one("m1_pwm_cmd_vel_bridge.py"),
  "static_tf_pid": pids.get("static_tf"),
  "foxglove_pid": pids.get("foxglove_bridge"),
  "stopped": {"joy": True, "teleop": True, "slam": True, "frontier": True},
  "source": "legacy_sensor_base_stack",
}
tmp = out.with_suffix(".tmp")
tmp.write_text(json.dumps(payload, indent=2)+"\n", encoding="utf-8")
os.replace(tmp, out)
PY
    else
      log "FAIL: handoff ack missing"
      return 1
    fi
  fi

  if ! python3 - <<PY
import json, sys
sys.path.insert(0, "${PROJECT_DIR}/scripts/nav")
from pathlib import Path
from qwen_nav2_common import validate_handoff_ack
req = json.loads(Path("${request_json}").read_text())
ack = json.loads(Path("${ack_json}").read_text())
ok, reason = validate_handoff_ack(req, ack, expected_session_id="${SESSION_ID}")
print(reason)
raise SystemExit(0 if ok else 1)
PY
  then
    log "FAIL: handoff ack validation"
    return 1
  fi

  # TERM remaining teleop/slam if any
  pkill -TERM -f "joy_node|teleop_twist_joy" 2>/dev/null || true
  pkill -TERM -f "async_slam_toolbox_node|sync_slam_toolbox_node" 2>/dev/null || true
  local j
  for j in $(seq 1 16); do
    if slam_exit_confirmed_thrice; then
      break
    fi
    sleep 0.5
  done
  if ! slam_exit_confirmed_thrice; then
    log "FAIL: SLAM exit gate (3x) not satisfied"
    return 1
  fi

  # /cmd_vel teleop publishers
  if ! python3 "$PROBE" cmd-vel-publishers >> "${RUNTIME_DIR}/orchestrator.log" 2>&1; then
    log "FAIL: teleop still publishing /cmd_vel"
    return 1
  fi

  log "MAPPING_HANDOFF_COMPLETE"
  return 0
}

repair_sensor_base_once() {
  log "repair_sensor_base_once"
  local health_json="${RUNTIME_DIR}/sensor_health_pre_repair.json"
  # snapshot process counts via read-only probe first (writes full health)
  python3 "$PROBE" sensor-health \
    --session-id "$SESSION_ID" \
    --runtime-dir "$RUNTIME_DIR" \
    --window 2.5 \
    > "${RUNTIME_DIR}/sensor_health_attempt1.log" 2>&1 || true
  cp -f "${RUNTIME_DIR}/sensor_health.json" "$health_json" 2>/dev/null || true

  # Use explicit Python repair decision (scan_filter / static_tf / foxglove only)
  python3 - "${RUNTIME_DIR}/sensor_health.json" "${RUNTIME_DIR}" "${PROJECT_DIR}" "${SESSION_ID}" <<'PY'
import json, os, subprocess, sys, time
from pathlib import Path
sys.path.insert(0, sys.argv[2] + "/scripts/nav")
from qwen_nav2_common import (
    decide_foxglove_action, decide_scan_filter_action, decide_static_tf_action,
    atomic_write_json, pid_alive,
)

health = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
runtime = Path(sys.argv[2])
project = Path(sys.argv[3])
session = sys.argv[4]
repairs = {"actions": [], "started_pids": {}}

# scan_filter
sf = health.get("scan_filter_process") or {}
count = int(sf.get("count") or 0)
fresh = bool(sf.get("topic_fresh"))
action, reason = decide_scan_filter_action(count, fresh)
repairs["actions"].append({"target": "scan_filter", "action": action, "reason": reason, "pids": sf.get("pids")})
if action == "fail":
    atomic_write_json(runtime / "repair_sensor_base.json", {"status": "FAIL", **repairs})
    print(f"FAIL scan_filter: {reason} pids={sf.get('pids')}")
    raise SystemExit(2)
if action == "start_once":
    script = project / "scripts/slam/simple_scan_filter.py"
    proc = subprocess.Popen(["python3", str(script)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    repairs["started_pids"]["scan_filter"] = proc.pid
    deadline = time.time() + 8.0
    while time.time() < deadline:
        time.sleep(0.5)
        # rely on later health recheck
        break
    time.sleep(2.0)

# static TF: only if missing
tf = health.get("tf") or {}
tf_ok = bool(tf.get("base_link_laser"))
owner_pid = None
ack = Path(f"{project}/runtime/nav_handoff/{session}/ack.json")
if ack.is_file():
    owner_pid = (json.loads(ack.read_text()).get("static_tf_pid"))
alive = pid_alive(int(owner_pid)) if owner_pid else False
st_action, st_reason = decide_static_tf_action(tf_exists=tf_ok, owner_pid=int(owner_pid) if owner_pid else None, owner_alive=alive)
repairs["actions"].append({"target": "static_tf", "action": st_action, "reason": st_reason, "owner_pid": owner_pid})
if st_action == "fail":
    atomic_write_json(runtime / "repair_sensor_base.json", {"status": "FAIL", **repairs})
    print(f"FAIL static_tf: {st_reason}")
    raise SystemExit(3)
if st_action == "start_once":
    scan_frame = health.get("scan_frame") or "laser"
    env = os.environ
    cmd = [
        "ros2","run","tf2_ros","static_transform_publisher",
        "--x", env.get("LASER_X","0.10"), "--y", env.get("LASER_Y","0.0"), "--z", env.get("LASER_Z","0.12"),
        "--roll", env.get("LASER_ROLL","0.0"), "--pitch", env.get("LASER_PITCH","0.0"),
        "--yaw", env.get("LASER_YAW","3.141592653589793"),
        "--frame-id","base_link","--child-frame-id", scan_frame,
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    repairs["started_pids"]["static_tf"] = proc.pid
    time.sleep(1.0)

# foxglove: never fails overall
fox = health.get("foxglove") or {}
fx_action, fx_reason = decide_foxglove_action(
    port_listening=bool(fox.get("port")),
    bridge_count=int(fox.get("bridge_count") or 0),
)
repairs["actions"].append({"target": "foxglove", "action": fx_action, "reason": fx_reason})
if fx_action == "start_once":
    # best-effort start once
    try:
        proc = subprocess.Popen(
            ["ros2","run","foxglove_bridge","foxglove_bridge","--port","8765"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
        )
        repairs["started_pids"]["foxglove"] = proc.pid
        time.sleep(1.5)
    except Exception as exc:
        repairs["foxglove_warn"] = str(exc)
elif fx_action == "warn":
    repairs["foxglove_warn"] = fx_reason

atomic_write_json(runtime / "repair_sensor_base.json", {"status": "OK", **repairs})
print("repair_sensor_base_once OK")
raise SystemExit(0)
PY
  local rc=$?
  if [[ "$rc" -eq 2 ]]; then
    log "FAIL: scan_filter policy"
    cat "${RUNTIME_DIR}/repair_sensor_base.json" 2>/dev/null || true
    return 1
  fi
  if [[ "$rc" -eq 3 ]]; then
    log "FAIL: static_tf policy"
    return 1
  fi
  if [[ "$rc" -ne 0 ]]; then
    log "FAIL: repair_sensor_base_once rc=$rc"
    return 1
  fi
  return 0
}

stop_nav2_overlay_owned() {
  local owner="${RUNTIME_DIR}/nav2_owner.json"
  log "stop_nav2_overlay_owned"
  publish_zero_velocity 0.5

  if [[ ! -f "$owner" ]]; then
    # Discover only — no fuzzy kill
    python3 - <<'PY'
import subprocess
patterns = ["nav2_click_nav_bringup", "controller_server", "planner_server", "bt_navigator", "amcl", "map_server"]
found = []
for p in patterns:
    try:
        out = subprocess.check_output(["pgrep","-af",p], text=True).strip()
        if out:
            found.append(out)
    except Exception:
        pass
if found:
    print("[QWEN_NAV2] WARN: found leftover Nav2 processes but no owner file; refuse fuzzy cleanup:")
    print("\n".join(found))
    raise SystemExit(2)
print("[QWEN_NAV2] no owner and no leftover Nav2")
PY
    local disc=$?
    if [[ "$disc" -eq 2 ]]; then
      log "FAIL: leftover Nav2 without owner — manual confirmation required"
      return 1
    fi
    return 0
  fi

  if ! python3 - <<PY
import json, sys
sys.path.insert(0, "${PROJECT_DIR}/scripts/nav")
from pathlib import Path
from qwen_nav2_common import validate_nav2_owner
owner = json.loads(Path("${owner}").read_text())
ok, reason = validate_nav2_owner(owner, expected_session_id="${SESSION_ID}")
print(reason)
raise SystemExit(0 if ok else 1)
PY
  then
    log "FAIL: nav2 owner validation — refuse kill"
    return 1
  fi

  # cancel NavigateToPose via action cancel service (best effort)
  python3 - <<'PY' || true
import uuid
import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID
rclpy.init()
node = Node("qwen_nav_cancel_all")
cli = node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
if cli.wait_for_service(timeout_sec=2.0):
    req = CancelGoal.Request()
    # empty UUID + zero stamp = cancel all goals (ROS 2 convention)
    req.goal_info.goal_id = UUID(uuid=[0]*16)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
node.destroy_node(); rclpy.shutdown()
PY

  local pgid launch_pid
  launch_pid="$(python3 -c "import json; print(json.load(open('${owner}'))['launch_pid'])")"
  pgid="$(python3 -c "import json; print(json.load(open('${owner}')).get('launch_pgid') or json.load(open('${owner}'))['launch_pid'])")"
  if [[ -n "$pgid" ]]; then
    kill -TERM "-${pgid}" 2>/dev/null || true
  fi
  [[ -n "$launch_pid" ]] && kill -TERM "$launch_pid" 2>/dev/null || true
  local i
  for i in $(seq 1 10); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      break
    fi
    sleep 0.5
  done
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -KILL "-${pgid}" 2>/dev/null || true
    kill -KILL "$launch_pid" 2>/dev/null || true
  fi
  rm -f "$owner"
  return 0
}

write_nav2_owner() {
  python3 - "$RUNTIME_DIR/nav2_owner.json" "$SESSION_ID" "$NAV2_LAUNCH_PID" "$NAV2_PGID" <<'PY'
import json, os, sys
from pathlib import Path
path, sid, pid, pgid = sys.argv[1:5]
pid = int(pid); pgid = int(pgid)
cmd = ""
start_ticks = None
try:
    cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\x00", b" ").decode()
    start_ticks = int(Path(f"/proc/{pid}/stat").read_text().split()[21])
except Exception:
    pass
payload = {
  "session_id": sid,
  "launch_pid": pid,
  "launch_pgid": pgid,
  "start_ticks": start_ticks,
  "cmdline": cmd,
}
tmp = Path(path).with_suffix(".tmp")
tmp.write_text(json.dumps(payload, indent=2)+"\n", encoding="utf-8")
os.replace(tmp, path)
PY
}

cleanup_on_exit() {
  local code=$?
  publish_zero_velocity 0.5
  if [[ "$STOP_NAV2_ON_EXIT" -eq 1 ]]; then
    stop_nav2_overlay_owned || true
  fi
  exit "$code"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session-id) SESSION_ID="$2"; shift 2 ;;
    --session-dir) SESSION_DIR="$2"; shift 2 ;;
    --map-yaml) MAP_YAML="$2"; shift 2 ;;
    --pose-json) POSE_JSON="$2"; shift 2 ;;
    --goal-json) GOAL_JSON="$2"; shift 2 ;;
    --candidate-bundle) CANDIDATE_BUNDLE="$2"; shift 2 ;;
    --mapping-pid) MAPPING_PID="$2"; shift 2 ;;
    --start-only) START_ONLY=1; shift ;;
    --compute-path-only) COMPUTE_PATH_ONLY=1; shift ;;
    --stop-nav2-on-exit) STOP_NAV2_ON_EXIT=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[FATAL] unknown arg: $1"; exit 1 ;;
  esac
done

for v in SESSION_ID SESSION_DIR MAP_YAML POSE_JSON GOAL_JSON CANDIDATE_BUNDLE; do
  if [[ -z "${!v}" ]]; then
    echo "[FATAL] missing required arg for $v"
    exit 1
  fi
done

RUNTIME_DIR="${SESSION_DIR}/nav2_runtime"
mkdir -p "$RUNTIME_DIR"
trap cleanup_on_exit INT TERM EXIT
source_ros_environment

MAP_YAML="$(readlink -f "$MAP_YAML")"
POSE_JSON="$(readlink -f "$POSE_JSON")"
CANDIDATE_BUNDLE="$(readlink -f "$CANDIDATE_BUNDLE")"
SESSION_DIR="$(readlink -f "$SESSION_DIR")"
RUNTIME_DIR="${SESSION_DIR}/nav2_runtime"
mkdir -p "$RUNTIME_DIR"

SESSION_GOAL="${SESSION_DIR}/navigation_goal_proposal.json"
if [[ "$(readlink -f "$GOAL_JSON")" != "$(readlink -f "$SESSION_GOAL")" ]]; then
  cp -f "$GOAL_JSON" "$SESSION_GOAL"
  log "copied goal -> $SESSION_GOAL"
fi
GOAL_JSON="$SESSION_GOAL"

T0="$(date +%s)"
if ! python3 - <<PY
import sys
sys.path.insert(0, "${PROJECT_DIR}/scripts/nav")
from pathlib import Path
from qwen_nav2_common import validate_goal_inputs
validate_goal_inputs(
    session_id="${SESSION_ID}",
    map_yaml=Path("${MAP_YAML}"),
    goal_json=Path("${GOAL_JSON}"),
    candidate_bundle=Path("${CANDIDATE_BUNDLE}"),
    pose_json=Path("${POSE_JSON}"),
)
print("TARGET_RECEIVED validation PASS")
PY
then
  write_state "FAILED" "goal validation"
  exit 1
fi
log "TARGET_RECEIVED session=${SESSION_ID}"

publish_zero_velocity 1.0
if ! wait_robot_stopped; then
  write_state "FAILED" "robot not stopped"
  exit 1
fi

# Pose consistency while SLAM map->base_link still available (before handoff)
if ! python3 "$PROBE" pose-compare --pose-json "$POSE_JSON" --timeout 5 \
  > "${RUNTIME_DIR}/pose_compare.json" 2>&1; then
  log "FAIL: frozen pose vs live map->base_link exceeds 0.20m/12deg"
  cat "${RUNTIME_DIR}/pose_compare.json" || true
  write_state "FAILED" "pose_delta"
  exit 1
fi

if ! stop_mapping_control_handoff; then
  write_state "FAILED" "mapping handoff"
  exit 1
fi

if ! repair_sensor_base_once; then
  write_state "FAILED" "repair sensor base"
  exit 1
fi

T_SENSOR="$(date +%s)"
if ! python3 "$PROBE" sensor-health \
  --session-id "$SESSION_ID" \
  --runtime-dir "$RUNTIME_DIR" \
  --window 3.0; then
  write_state "FAILED" "sensor health"
  exit 1
fi
log "SENSOR_BASE_VERIFIED in $(( $(date +%s) - T_SENSOR ))s"

if ! stop_nav2_overlay_owned; then
  write_state "FAILED" "old nav2 cleanup"
  exit 1
fi

T_NAV2="$(date +%s)"
setsid ros2 launch \
  "$NAV2_BRINGUP" \
  use_sim_time:=False \
  autostart:=True \
  "map:=${MAP_YAML}" \
  "params_file:=${NAV2_PARAMS}" \
  use_composition:=False \
  >> "${RUNTIME_DIR}/nav2_launch.log" 2>&1 &
NAV2_LAUNCH_PID=$!
NAV2_PGID="$(ps -o pgid= -p "$NAV2_LAUNCH_PID" 2>/dev/null | tr -d ' ' || echo "$NAV2_LAUNCH_PID")"
write_nav2_owner
log "NAV2_OVERLAY_STARTED pid=${NAV2_LAUNCH_PID} pgid=${NAV2_PGID}"

sleep 1
if ! kill -0 "$NAV2_LAUNCH_PID" 2>/dev/null; then
  log "FAIL: Nav2 launch exited early"
  tail -80 "${RUNTIME_DIR}/nav2_launch.log" || true
  write_state "FAILED" "nav2 launch exit"
  exit 1
fi

SETTLE_ARGS=(--settle-timeout 30)
[[ "$START_ONLY" -eq 1 ]] || true
if ! python3 "$PROBE" wait-nav-ready \
  --map-yaml "$MAP_YAML" \
  --pose-json "$POSE_JSON" \
  --runtime-dir "$RUNTIME_DIR" \
  --loc-timeout 25 \
  --nav-timeout 30 \
  "${SETTLE_ARGS[@]}"; then
  write_state "LOCALIZATION_UNSETTLED" "amcl settle"
  if [[ -f "${RUNTIME_DIR}/amcl_settle_metrics.json" ]]; then
    cat "${RUNTIME_DIR}/amcl_settle_metrics.json" || true
  fi
  publish_zero_velocity 1.0
  exit 1
fi
log "LOCALIZATION_SETTLED"

EXEC_ARGS=(
  --session-id "$SESSION_ID"
  --session-dir "$SESSION_DIR"
  --map-yaml "$MAP_YAML"
  --pose-json "$POSE_JSON"
  --goal-json "$GOAL_JSON"
  --candidate-bundle "$CANDIDATE_BUNDLE"
  --runtime-dir "$RUNTIME_DIR"
  --nav-launch-pid "$NAV2_LAUNCH_PID"
)
[[ "$START_ONLY" -eq 1 ]] && EXEC_ARGS+=(--start-only)
[[ "$COMPUTE_PATH_ONLY" -eq 1 ]] && EXEC_ARGS+=(--compute-path-only)

T_EXEC="$(date +%s)"
set +e
python3 "${PROJECT_DIR}/scripts/nav/qwen_nav2_goal_executor.py" "${EXEC_ARGS[@]}"
RC=$?
set -e

python3 - "${RUNTIME_DIR}/nav2_timing.json" "$T0" "$T_NAV2" "$T_EXEC" "$RC" <<'PY'
import json, os, sys, time
from pathlib import Path
path, t0, t_nav2, t_exec, rc = sys.argv[1:6]
payload = {
  "pipeline_start_epoch": float(t0),
  "nav2_launch_epoch": float(t_nav2),
  "executor_start_epoch": float(t_exec),
  "finished_epoch": time.time(),
  "exit_code": int(rc),
  "nav2_boot_s": float(t_exec) - float(t_nav2),
  "total_s": time.time() - float(t0),
  "goal_to_navigate_s": time.time() - float(t0),
}
tmp = Path(path).with_suffix(".tmp")
tmp.write_text(json.dumps(payload, indent=2)+"\n", encoding="utf-8")
os.replace(tmp, path)
PY

if [[ "$RC" -ne 0 ]]; then
  log "FAIL exit=$RC"
  [[ -f "${RUNTIME_DIR}/nav2_state.json" ]] && cat "${RUNTIME_DIR}/nav2_state.json" || true
  exit "$RC"
fi
log "pipeline OK in $(( $(date +%s) - T0 ))s"
exit 0
