#!/usr/bin/env bash
# Localization bootstrap helpers for saved-map Nav2 (ported from nav2_oneclick_goal.sh).

lifecycle_get() {
  timeout 5 ros2 lifecycle get "$1" 2>/dev/null || true
}

wait_lifecycle_active() {
  local node="$1"
  local timeout_sec="${2:-120}"
  local start now state

  echo "[NAV2_BOOT] wait lifecycle active: $node (timeout=${timeout_sec}s)"
  start="$(date +%s)"
  while true; do
    state="$(lifecycle_get "$node")"
    if echo "$state" | grep -q "active"; then
      echo "[NAV2_BOOT] $node active: ${state}"
      return 0
    fi
    now="$(date +%s)"
    if [ $((now - start)) -ge "$timeout_sec" ]; then
      echo "[NAV2_BOOT] ERROR: $node not active after ${timeout_sec}s (last: ${state:-NO RESPONSE})"
      return 1
    fi
    sleep 2
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
qos = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)
tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
TransformListener(tf_buffer, node, spin_thread=False)

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
last_pub = 0.0
map_base_ok = False
tf_ok_since = None

while time.time() - start < timeout:
    now = time.time()
    if now - last_pub >= 0.4:
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        last_pub = now
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.2))
        if not map_base_ok:
            map_base_ok = True
            tf_ok_since = now
            print("[NAV2_BOOT] TF map -> base_link OK", flush=True)
        # Keep publishing initialpose briefly so AMCL can ingest scans.
        if tf_ok_since is not None and now - tf_ok_since >= 4.0:
            break
    except Exception:
        tf_ok_since = None
        map_base_ok = False

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
  local timeout_sec="${1:-30}"
  wait_lifecycle_active /map_server "$timeout_sec" || return 1
  wait_lifecycle_active /amcl "$timeout_sec" || return 1
  wait_lifecycle_active /controller_server "$timeout_sec" || return 1
  wait_lifecycle_active /planner_server "$timeout_sec" || return 1
  wait_lifecycle_active /bt_navigator "$timeout_sec" || return 1
  return 0
}
