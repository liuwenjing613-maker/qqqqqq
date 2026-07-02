#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"

GOAL_X="${1:-}"
GOAL_Y="${2:-}"
GOAL_YAW="${3:-0.0}"

START_X="${4:-0.0}"
START_Y="${5:-0.0}"
START_YAW="${6:-0.0}"

if [ -z "$GOAL_X" ] || [ -z "$GOAL_Y" ]; then
  echo "Usage:"
  echo "  bash scripts/slam/nav2_plan_preview.sh GOAL_X GOAL_Y [GOAL_YAW] [START_X START_Y START_YAW]"
  echo
  echo "Examples:"
  echo "  bash scripts/slam/nav2_plan_preview.sh 0.60 0.00 0.00"
  echo "  bash scripts/slam/nav2_plan_preview.sh 1.80 0.00 0.00 0.05 0.03 0.00"
  exit 2
fi

MAP_YAML="${MAP_YAML:-$PROJECT_DIR/maps/joy_calibrated_corridor_map.yaml}"
PARAMS_FILE="${PARAMS_FILE:-$PROJECT_DIR/configs/nav2_params.yaml}"
PLANNER_ID="${PLANNER_ID:-GridBased}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
FOXGLOVE_LAYOUT="${FOXGLOVE_LAYOUT:-$PROJECT_DIR/configs/foxglove_nav2_oneclick.layout.json}"
START_FOXGLOVE="${START_FOXGLOVE:-1}"
KEEP_SEC="${KEEP_SEC:-600}"

LOG_DIR="$PROJECT_DIR/logs/nav2_plan_preview_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
PREVIEW_PARAMS="$LOG_DIR/nav2_preview_params.yaml"

log() {
  echo "[NAV2-PLAN-PREVIEW] $*"
}

warn() {
  echo "[NAV2-PLAN-PREVIEW][WARN] $*" >&2
}

fail() {
  echo "[NAV2-PLAN-PREVIEW][FAIL] $*" >&2
  echo "[NAV2-PLAN-PREVIEW][FAIL] logs=$LOG_DIR" >&2
  exit 1
}

PIDS=()

source_ros() {
  set +u
  [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
  [ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
  set -u
}

start_bg() {
  local name="$1"
  shift
  log "start $name: $*"
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=("$!")
}

cleanup() {
  log "cleanup preview processes..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  pkill -f "nav2_compute_plan_once.py" 2>/dev/null || true
  pkill -f "static_transform_publisher.*map.*base_link" 2>/dev/null || true
  pkill -f "m1_pwm_cmd_vel_bridge.py|cmd_vel_to_rosmaster.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cd "$PROJECT_DIR"
source_ros

[ -f "$MAP_YAML" ] || fail "MAP_YAML not found: $MAP_YAML"
[ -f "$PARAMS_FILE" ] || fail "PARAMS_FILE not found: $PARAMS_FILE"
[ -f "$PROJECT_DIR/ros2_bridge/nav2_compute_plan_once.py" ] || fail "missing ros2_bridge/nav2_compute_plan_once.py"

python3 - <<PY
import yaml
with open("$PARAMS_FILE", "r", encoding="utf-8") as f:
    yaml.safe_load(f)
print("[OK] YAML parse: $PARAMS_FILE")
PY

patch_preview_params() {
  python3 - <<PY
import yaml
from pathlib import Path

src = Path("$PARAMS_FILE")
dst = Path("$PREVIEW_PARAMS")
data = yaml.safe_load(src.read_text(encoding="utf-8"))

gc = data.setdefault("global_costmap", {}).setdefault("global_costmap", {}).setdefault("ros__parameters", {})
# Preview mode: static map only, no lidar obstacle layer required.
gc["plugins"] = ["static_layer", "inflation_layer"]
gc.pop("obstacle_layer", None)

dst.parent.mkdir(parents=True, exist_ok=True)
dst.write_text(yaml.dump(data, sort_keys=False), encoding="utf-8")
print(f"[OK] preview params written: {dst}")
PY
}

wait_map_topic() {
  local timeout_sec="${1:-60}"
  log "wait /map with transient_local QoS"
  python3 - "$timeout_sec" <<'PY'
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

timeout = float(sys.argv[1])
rclpy.init()
node = Node("nav2_plan_preview_wait_map")
qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
got = {"ok": False, "w": 0, "h": 0}

def cb(msg: OccupancyGrid) -> None:
    if msg.info.width > 0 and msg.info.height > 0:
        got["ok"] = True
        got["w"] = msg.info.width
        got["h"] = msg.info.height

node.create_subscription(OccupancyGrid, "/map", cb, qos)
start = time.time()
while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.5)
    if got["ok"]:
        print(f"[OK] /map received: {got['w']} x {got['h']}", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    time.sleep(0.2)

node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
PY
}

wait_tf_map_base_link() {
  local timeout_sec="${1:-30}"
  log "wait TF map -> base_link (timeout=${timeout_sec}s)"
  python3 - "$timeout_sec" <<'PY'
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros

timeout = float(sys.argv[1])
rclpy.init()
node = Node("nav2_plan_preview_wait_tf")
buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
tf2_ros.TransformListener(buf, node)
start = time.time()
ok = False
while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        buf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.2))
        ok = True
        break
    except Exception:
        pass
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ok else 1)
PY
}

print_foxglove_help() {
  local host
  host="$(hostname -I 2>/dev/null | awk '{print $1}')"
  [ -n "$host" ] || host="127.0.0.1"
  echo "========== FOXGLOVE PLAN PREVIEW =========="
  echo "Connect: ws://${host}:${FOXGLOVE_PORT}"
  if [ -f "$FOXGLOVE_LAYOUT" ]; then
    echo "Layout -> Import -> $FOXGLOVE_LAYOUT"
  fi
  echo "View in 3D panel:"
  echo "  /map               occupancy grid"
  echo "  /plan              planned path (green)"
  echo "  /nav2_viz/global_plan"
  echo "  /nav2_plan_markers path line + start(blue) + goal(red)"
  echo "  /goal_pose /start_pose"
  echo "Press Ctrl+C here to stop preview."
  echo "==========================================="
}

log "GOAL=($GOAL_X, $GOAL_Y, $GOAL_YAW), START=($START_X, $START_Y, $START_YAW)"
log "MAP_YAML=$MAP_YAML"
log "PARAMS_FILE=$PARAMS_FILE"
log "PREVIEW_PARAMS=$PREVIEW_PARAMS"
log "PLANNER_ID=$PLANNER_ID"
log "logs=$LOG_DIR"
log "SAFE MODE: no controller_server, no bt_navigator, no chassis_bridge, no /cmd_vel"

pkill -f "nav2_compute_plan_once.py" 2>/dev/null || true
pkill -f "/opt/ros/.*/lib/nav2_map_server/map_server" 2>/dev/null || true
pkill -f "/opt/ros/.*/lib/nav2_planner/planner_server" 2>/dev/null || true
pkill -f "static_transform_publisher.*map.*base_link" 2>/dev/null || true
pkill -f "m1_pwm_cmd_vel_bridge.py|cmd_vel_to_rosmaster.py" 2>/dev/null || true
sleep 1

patch_preview_params

if [ "$START_FOXGLOVE" = "1" ] && ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  start_bg foxglove ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="$FOXGLOVE_PORT"
  sleep 2
else
  warn "foxglove_bridge not installed; path topics still publish on ROS"
fi

wait_lifecycle_service() {
  local node="$1"
  local timeout_sec="${2:-60}"
  local i=0
  log "wait lifecycle service: $node"
  while [ "$i" -lt "$timeout_sec" ]; do
    if ros2 lifecycle get "$node" >/tmp/nav2_plan_preview_lifecycle.txt 2>&1; then
      sed "s|^|[NAV2-PLAN-PREVIEW] ${node} state: |" /tmp/nav2_plan_preview_lifecycle.txt || true
      return 0
    fi
    sleep 1
    i=$((i + 1))
  done
  return 1
}

activate_lifecycle_node() {
  local node="$1"

  wait_lifecycle_service "$node" 80 || fail "$node lifecycle service not available"

  local state
  state="$(ros2 lifecycle get "$node" 2>/dev/null || true)"

  if echo "$state" | grep -q "active"; then
    log "$node already active"
    return 0
  fi

  if echo "$state" | grep -q "unconfigured"; then
    log "configure $node"
    ros2 lifecycle set "$node" configure || fail "configure $node failed"
    sleep 2
  fi

  state="$(ros2 lifecycle get "$node" 2>/dev/null || true)"
  if echo "$state" | grep -q "inactive"; then
    log "activate $node"
    timeout 45 ros2 lifecycle set "$node" activate || fail "activate $node failed (timeout 45s; check TF and $LOG_DIR/${node#/}.log)"
    sleep 2
  fi

  state="$(ros2 lifecycle get "$node" 2>/dev/null || true)"
  echo "$state" | grep -q "active" || fail "$node is not active; current state: $state"

  log "$node active"
}

# map -> base_link at preview start pose (planner global costmap needs robot frame)
start_bg static_tf ros2 run tf2_ros static_transform_publisher \
  --x "$START_X" --y "$START_Y" --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw "$START_YAW" \
  --frame-id map --child-frame-id base_link
sleep 2
wait_tf_map_base_link 20 || fail "map->base_link TF missing; stop chassis/odom nodes and retry"

start_bg map_server ros2 run nav2_map_server map_server \
  --ros-args \
  -r __node:=map_server \
  --params-file "$PREVIEW_PARAMS" \
  -p "yaml_filename:=$MAP_YAML"

activate_lifecycle_node /map_server
wait_map_topic 60 || fail "no /map received; check $LOG_DIR/map_server.log"
log "/map received"

start_bg planner_server ros2 run nav2_planner planner_server \
  --ros-args \
  -r __node:=planner_server \
  --params-file "$PREVIEW_PARAMS"

activate_lifecycle_node /planner_server

log "wait action server: /compute_path_to_pose"
start_ts=$(date +%s)
while true; do
  if ros2 action list 2>/dev/null | grep -qx "/compute_path_to_pose"; then
    break
  fi
  if [ "$(($(date +%s) - start_ts))" -ge 60 ]; then
    fail "/compute_path_to_pose not available; check $LOG_DIR/planner_server.log"
  fi
  sleep 1
done
log "/compute_path_to_pose available"

print_foxglove_help
log "compute and publish preview path"

python3 "$PROJECT_DIR/ros2_bridge/nav2_compute_plan_once.py" \
  --goal-x "$GOAL_X" \
  --goal-y "$GOAL_Y" \
  --goal-yaw "$GOAL_YAW" \
  --start-x "$START_X" \
  --start-y "$START_Y" \
  --start-yaw "$START_YAW" \
  --planner-id "$PLANNER_ID" \
  --keep-sec "$KEEP_SEC" \
  || fail "plan compute failed; check planner_server.log and goal/start pose"

log "preview finished"
