#!/usr/bin/env bash
# Localization bootstrap helpers for saved-map Nav2 (ported from nav2_oneclick_goal.sh).

_LIFECYCLE_PROBE="${PROJECT_DIR:-/root/rdk_x5_vln_robot}/scripts/lib/lifecycle_probe.py"

export_ros_dds_env() {
  local project_dir="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
  export FASTRTPS_DEFAULT_PROFILES="${project_dir}/configs/fastdds_no_shm.xml"
}

lifecycle_get() {
  # Legacy helper; prefer lifecycle_probe_rclpy for readiness checks.
  timeout 20 ros2 lifecycle get "$1" 2>/dev/null || true
}

lifecycle_probe_rclpy() {
  local node="$1"
  local timeout_sec="${2:-120}"
  python3 "$_LIFECYCLE_PROBE" wait "$node" "$timeout_sec"
}

lifecycle_primary_state() {
  sed 's/\x1b\[[0-9;]*m//g' <<<"$1" \
    | grep -oE '(unconfigured|inactive|active|configured|finalized|no_service|call_failed|NO RESPONSE) ?(\[[0-9]+\])?' \
    | tail -1
}

lifecycle_is_active_label() {
  local label="$1"
  [[ "$label" == active\ \[3\] ]]
}

retry_navigation_bringup() {
  local timeout_sec="${1:-90}"
  echo "[NAV2_BOOT] WARN: navigation stack not fully active; requesting lifecycle_manager_navigation STARTUP"
  timeout 15 ros2 service call /lifecycle_manager_navigation/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" >/dev/null 2>&1 || true
  sleep 3
  wait_nav_actions_ready "$timeout_sec"
}

wait_nav_actions_ready() {
  local timeout_sec="${1:-180}"
  echo "[NAV2_BOOT] wait Nav2 actions: /navigate_to_pose + /compute_path_to_pose (timeout=${timeout_sec}s)"
  if python3 "$_LIFECYCLE_PROBE" nav-actions "$timeout_sec"; then
    echo "[NAV2_BOOT] Nav2 navigation actions ready"
    return 0
  fi
  echo "[NAV2_BOOT] ERROR: Nav2 navigation actions not ready after ${timeout_sec}s"
  return 1
}

wait_lifecycle_active() {
  local node="$1"
  local timeout_sec="${2:-120}"
  local start now label rc

  echo "[NAV2_BOOT] wait lifecycle active: $node (timeout=${timeout_sec}s, rclpy)"
  start="$(date +%s)"
  while true; do
    label="$(python3 "$_LIFECYCLE_PROBE" wait "$node" 15 2>/dev/null || true)"
    if lifecycle_is_active_label "$label"; then
      echo "[NAV2_BOOT] $node active: ${label}"
      return 0
    fi
    now="$(date +%s)"
    if [ $((now - start)) -ge "$timeout_sec" ]; then
      echo "[NAV2_BOOT] ERROR: $node not active after ${timeout_sec}s (last: ${label:-NO RESPONSE})"
      return 1
    fi
    sleep 1
  done
}

wait_map_topic_data() {
  local timeout_sec="${1:-60}"
  python3 - "$timeout_sec" <<'PY'
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

timeout = float(sys.argv[1])
rclpy.init()
node = Node("nav2_wait_map")
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
        print(f"[NAV2_BOOT] /map received: {got['w']} x {got['h']}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    time.sleep(0.2)

node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
PY
}

wait_odom_base_link_tf() {
  local timeout_sec="${1:-60}"
  python3 - "$timeout_sec" <<'PY'
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

timeout = float(sys.argv[1])
rclpy.init()
node = Node("nav2_wait_odom_base")
buf = Buffer(cache_time=Duration(seconds=10.0))
TransformListener(buf, node, spin_thread=False)
start = time.time()
while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        buf.lookup_transform("odom", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.2))
        print("[NAV2_BOOT] TF odom -> base_link OK")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    except Exception:
        pass
node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
PY
}

wait_map_base_link_tf_rclpy() {
  local timeout_sec="${1:-120}"
  local log_tag="${2:-NAV2_BOOT}"
  python3 - "$timeout_sec" "$log_tag" <<'PY'
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

timeout = float(sys.argv[1])
log_tag = sys.argv[2]
rclpy.init()
node = Node("nav2_wait_map_base_link")
buf = Buffer(cache_time=Duration(seconds=30.0))
TransformListener(buf, node, spin_thread=False)
start = time.time()
last_report = start
while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.1)
    now = time.time()
    if now - last_report >= 10.0:
        print(f"[{log_tag}] ... waiting TF map -> base_link ({int(now - start)}s)", flush=True)
        last_report = now
    try:
        tf = buf.lookup_transform(
            "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.3)
        )
        t = tf.transform.translation
        print(
            f"[{log_tag}] TF map -> base_link OK  x={t.x:.3f} y={t.y:.3f}",
            flush=True,
        )
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    except Exception:
        pass
node.destroy_node()
rclpy.shutdown()
print(f"[{log_tag}] ERROR: map -> base_link TF not available after {timeout:.0f}s")
raise SystemExit(1)
PY
}

wait_amcl_scan_subscription() {
  local timeout_sec="${1:-90}"
  local start
  start="$(date +%s)"
  echo "[NAV2] wait AMCL /scan_filtered subscription (timeout=${timeout_sec}s) ..."
  while true; do
    local sub_count
    sub_count="$(ros2 topic info /scan_filtered -v 2>/dev/null | awk '/Subscription count:/{print $3; exit}')"
    if [ "${sub_count:-0}" -ge 1 ]; then
      if ros2 topic info /scan_filtered -v 2>/dev/null | grep -qi amcl; then
        echo "[NAV2] AMCL laser subscription ready (scan_filtered subs=${sub_count})"
        return 0
      fi
      if [ "${sub_count:-0}" -ge 1 ]; then
        echo "[NAV2] /scan_filtered has subscriber(s=${sub_count}); proceed with AMCL bootstrap"
        return 0
      fi
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_sec" ]; then
      echo "[NAV2] WARN: AMCL /scan_filtered subscription not confirmed after ${timeout_sec}s"
      return 1
    fi
    sleep 1
  done
}

wait_initialpose_subscriber_ready() {
  local timeout_sec="${1:-60}"
  local start
  start="$(date +%s)"
  echo "[NAV2] wait /initialpose subscriber (timeout=${timeout_sec}s) ..."
  while true; do
    local sub_count
    sub_count="$(ros2 topic info /initialpose -v 2>/dev/null | awk '/Subscription count:/{print $3; exit}')"
    if [ "${sub_count:-0}" -ge 1 ]; then
      echo "[NAV2] /initialpose subscriber OK (count=${sub_count})"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_sec" ]; then
      echo "[NAV2] WARN: no /initialpose subscriber yet; AMCL bootstrap may need retry"
      return 1
    fi
    sleep 1
  done
}

bootstrap_amcl_from_pose() {
  local x="$1"
  local y="$2"
  local yaw="$3"
  local timeout_sec="${4:-90}"
  echo "[NAV2_BOOT] bootstrap AMCL: x=$x y=$y yaw=$yaw timeout=${timeout_sec}s"
  python3 - "$x" "$y" "$yaw" "$timeout_sec" <<'PY'
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

x, y, yaw = map(float, sys.argv[1:4])
timeout = float(sys.argv[4])

rclpy.init()
node = Node("nav2_bootstrap_amcl")
initial_qos = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
amcl_qos = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
)
pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", initial_qos)
tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
TransformListener(tf_buffer, node, spin_thread=False)
amcl_samples = []

def on_amcl_pose(msg: PoseWithCovarianceStamped) -> None:
    amcl_samples.append(time.time())

node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_amcl_pose, amcl_qos)

msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.pose.pose.position.x = x
msg.pose.pose.position.y = y
msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
msg.pose.covariance = [0.0] * 36
msg.pose.covariance[0] = 0.25
msg.pose.covariance[7] = 0.25
msg.pose.covariance[35] = 0.0685

start = time.time()
odom_ok = False
while time.time() - start < min(timeout, 30.0):
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.2))
        odom_ok = True
        break
    except Exception:
        pass

if not odom_ok:
    print("[NAV2_BOOT] ERROR: odom -> base_link TF not ready before AMCL bootstrap", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)

# Publish initialpose periodically until AMCL converges (not just 2 bursts at t=0).
last_publish = 0.0
publish_interval = 3.0
publish_count = 0

map_base_ok = False
while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.1)
    now = time.time()
    if now - last_publish >= publish_interval:
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        last_publish = now
        publish_count += 1
        if publish_count == 1 or publish_count % 3 == 0:
            print(
                f"[NAV2_BOOT] publish /initialpose #{publish_count} "
                f"({len(amcl_samples)} /amcl_pose samples)",
                flush=True,
            )
    try:
        tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.2))
        map_base_ok = True
    except Exception:
        map_base_ok = False
    if map_base_ok and len(amcl_samples) >= 1:
        print(
            f"[NAV2_BOOT] AMCL bootstrap OK: map->base_link TF + {len(amcl_samples)} /amcl_pose samples",
            flush=True,
        )
        break

if map_base_ok:
    print("[NAV2_BOOT] TF map -> base_link OK", flush=True)
else:
    print("[NAV2_BOOT] ERROR: map -> base_link TF missing after AMCL bootstrap", flush=True)

node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if map_base_ok else 1)
PY
}

load_pose_from_json() {
  local json_file="$1"
  python3 - "$json_file" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)
print(float(data.get("x", 0.0)))
print(float(data.get("y", 0.0)))
print(float(data.get("yaw", 0.0)))
PY
}

print_pose_state_summary() {
  local json_file="$1"
  if [ ! -f "$json_file" ]; then
    echo "[NAV2_BOOT] WARN: pose state file not found: $json_file"
    return 1
  fi
  python3 - "$json_file" <<'PY'
import json
import math
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)
x = float(data.get("x", 0.0))
y = float(data.get("y", 0.0))
yaw = float(data.get("yaw", 0.0))
qx = float(data.get("qx", 0.0))
qy = float(data.get("qy", 0.0))
qz = float(data.get("qz", 0.0))
qw = float(data.get("qw", 1.0))
has_quat = any(abs(v) > 1e-9 for v in (qx, qy, qz)) or abs(qw - 1.0) > 1e-9
print(f"[NAV2_BOOT] saved pose: x={x:.3f} y={y:.3f} yaw={yaw:.3f} rad ({math.degrees(yaw):.1f} deg)")
if has_quat:
    print(f"[NAV2_BOOT] saved orientation quaternion: qx={qx:.4f} qy={qy:.4f} qz={qz:.4f} qw={qw:.4f}")
else:
    print("[NAV2_BOOT] WARN: quaternion fields missing; only yaw is used for AMCL bootstrap")
print("[NAV2_BOOT] If scan and map walls do not overlap in Foxglove, set initial pose via /initialpose")
PY
}

wait_amcl_localization_settle() {
  local timeout_sec="${1:-25}"
  python3 - "$timeout_sec" <<'PY'
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

timeout = float(sys.argv[1])
rclpy.init()
node = Node("nav2_wait_amcl_settle")
qos = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
)
samples = []

def cb(msg: PoseWithCovarianceStamped) -> None:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    cov_xy = float(msg.pose.covariance[0]) + float(msg.pose.covariance[7])
    samples.append((time.time(), p.x, p.y, yaw, cov_xy))

node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", cb, qos)
start = time.time()
min_samples = 5
stable_need = 3
stable_count = 0

while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.2)
    if len(samples) < min_samples:
        continue
    recent = samples[-stable_need:]
    xs = [s[1] for s in recent]
    ys = [s[2] for s in recent]
    yaws = [s[3] for s in recent]
    if (
        max(xs) - min(xs) < 0.08
        and max(ys) - min(ys) < 0.08
        and max(yaws) - min(yaws) < 0.12
    ):
        stable_count += 1
        if stable_count >= 4:
            last = samples[-1]
            print(
                f"[NAV2_BOOT] AMCL pose settled: x={last[1]:.3f} y={last[2]:.3f} "
                f"yaw={last[3]:.3f} rad ({math.degrees(last[3]):.1f} deg), "
                f"cov_xy_sum={last[4]:.3f}, samples={len(samples)}",
                flush=True,
            )
            node.destroy_node()
            rclpy.shutdown()
            raise SystemExit(0)
    else:
        stable_count = 0

if samples:
    last = samples[-1]
    print(
        f"[NAV2_BOOT] WARN: AMCL pose not fully settled after {timeout:.0f}s; "
        f"last x={last[1]:.3f} y={last[2]:.3f} yaw={math.degrees(last[3]):.1f} deg "
        f"(samples={len(samples)}). Check scan/map overlap in Foxglove.",
        flush=True,
    )
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0)

print(f"[NAV2_BOOT] WARN: no /amcl_pose received in {timeout:.0f}s", flush=True)
node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
PY
}

bootstrap_amcl_from_state_file() {
  local state_file="$1"
  local timeout_sec="${2:-90}"
  if [ ! -f "$state_file" ]; then
    echo "[NAV2_BOOT] WARN: pose state file not found: $state_file"
    return 1
  fi
  local pose_vals
  pose_vals="$(load_pose_from_json "$state_file")" || return 1
  local x y yaw
  x="$(echo "$pose_vals" | sed -n '1p')"
  y="$(echo "$pose_vals" | sed -n '2p')"
  yaw="$(echo "$pose_vals" | sed -n '3p')"
  bootstrap_amcl_from_pose "$x" "$y" "$yaw" "$timeout_sec"
}

verify_nav2_navigation_ready() {
  local timeout_sec="${1:-90}"
  wait_map_topic_data 30 || return 1
  wait_nav_actions_ready "$timeout_sec" || return 1
  return 0
}

write_nav2_ready_json() {
  local log_dir="$1"
  local map_yaml="$2"
  local started_at="${3:-}"
  local reuse_scan="${4:-0}"
  local reuse_scan_filtered="${5:-0}"
  local reuse_chassis="${6:-0}"
  local reuse_static_tf="${7:-0}"
  python3 - "$log_dir" "$map_yaml" "$started_at" "$reuse_scan" "$reuse_scan_filtered" "$reuse_chassis" "$reuse_static_tf" <<'PY'
import json
import sys
import time
from pathlib import Path

log_dir = Path(sys.argv[1])
map_yaml = sys.argv[2]
started_at = sys.argv[3]
payload = {
    "map_yaml": str(Path(map_yaml).resolve()),
    "started_at": started_at or time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "ready_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "reuse_scan": sys.argv[4] == "1",
    "reuse_scan_filtered": sys.argv[5] == "1",
    "reuse_chassis": sys.argv[6] == "1",
    "reuse_static_tf": sys.argv[7] == "1",
    "map_ready": True,
    "amcl_active": True,
    "map_to_base_link": True,
    "navigate_to_pose_ready": True,
    "compute_path_to_pose_ready": True,
}
out = log_dir / "ready.json"
out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
# legacy empty ready marker
(log_dir / "ready").write_text("", encoding="utf-8")
print(str(out))
PY
}

wait_nav2_lifecycle_parallel() {
  local timeout_sec="${1:-120}"
  python3 - "$timeout_sec" "$_LIFECYCLE_PROBE" <<'PY'
import subprocess
import sys
import time

timeout = float(sys.argv[1])
probe = sys.argv[2]
nodes = ["/map_server", "/amcl", "/planner_server", "/controller_server", "/bt_navigator"]
start = time.time()
pending = set(nodes)
while time.time() - start < timeout and pending:
    for node in list(pending):
        try:
            out = subprocess.check_output(
                ["python3", probe, "wait", node, "5"], text=True, stderr=subprocess.DEVNULL
            )
            if "active [3]" in out:
                pending.discard(node)
        except subprocess.CalledProcessError:
            pass
    if pending:
        time.sleep(0.5)
if pending:
    print(f"[NAV2_BOOT] ERROR: lifecycle not active: {sorted(pending)}", flush=True)
    raise SystemExit(1)
print("[NAV2_BOOT] Nav2 lifecycle servers active (parallel wait)", flush=True)
raise SystemExit(0)
PY
}
