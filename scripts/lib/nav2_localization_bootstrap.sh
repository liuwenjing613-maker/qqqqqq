#!/usr/bin/env bash
# Localization bootstrap helpers for saved-map Nav2 (ported from nav2_oneclick_goal.sh).

_LIFECYCLE_PROBE="${PROJECT_DIR:-/root/rdk_x5_vln_robot}/scripts/lib/lifecycle_probe.py"
_ROS_TOPIC_PROBE="${PROJECT_DIR:-/root/rdk_x5_vln_robot}/scripts/lib/ros_topic_probe.py"

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

get_map_publisher_count() {
  ros2 topic info /map -v 2>/dev/null | awk '/Publisher count:/{print $3; exit}' || echo 0
}

wait_map_publisher_count() {
  local expected="$1"
  local timeout_sec="${2:-60}"
  local start now count
  start="$(date +%s)"
  echo "[NAV2_BOOT] wait /map publisher count == ${expected} (timeout=${timeout_sec}s)"
  while true; do
    count="$(get_map_publisher_count)"
    if [ "${count:-0}" -eq "$expected" ]; then
      echo "[NAV2_BOOT] /map publisher count OK: ${count}"
      return 0
    fi
    now="$(date +%s)"
    if [ $((now - start)) -ge "$timeout_sec" ]; then
      echo "[NAV2_BOOT] ERROR: /map publisher count=${count:-0}, expected ${expected} after ${timeout_sec}s"
      return 1
    fi
    sleep 1
  done
}

update_nav2_stage_timing_json() {
  local timing_file="$1"
  shift
  python3 - "$timing_file" "$@" <<'PY'
import json
import os
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
updates: dict[str, float] = {}
idx = 2
while idx + 1 < len(sys.argv):
    updates[sys.argv[idx]] = float(sys.argv[idx + 1])
    idx += 2

data: dict = {}
if path.is_file():
    data = json.loads(path.read_text(encoding="utf-8"))

data.update(updates)
path.parent.mkdir(parents=True, exist_ok=True)
tmp = path.with_suffix(path.suffix + ".tmp")
tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
os.replace(tmp, path)
PY
}

wait_amcl_localization_settle() {
  local timeout_sec="${1:-${AMCL_SETTLE_TIMEOUT_S:-60}}"
  export AMCL_SETTLE_TIMEOUT_S="$timeout_sec"
  python3 - "$timeout_sec" "$_LIFECYCLE_PROBE" <<'PY'
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

timeout = float(sys.argv[1])
lifecycle_probe = sys.argv[2]

min_samples = int(os.environ.get("AMCL_SETTLE_MIN_SAMPLES", "5"))
max_x_spread = float(os.environ.get("AMCL_SETTLE_MAX_X_SPREAD_M", "0.08"))
max_y_spread = float(os.environ.get("AMCL_SETTLE_MAX_Y_SPREAD_M", "0.08"))
max_yaw_spread_deg = float(os.environ.get("AMCL_SETTLE_MAX_YAW_SPREAD_DEG", "7.0"))
max_x_cov = float(os.environ.get("AMCL_SETTLE_MAX_X_COV", "0.25"))
max_y_cov = float(os.environ.get("AMCL_SETTLE_MAX_Y_COV", "0.25"))
max_yaw_cov = float(os.environ.get("AMCL_SETTLE_MAX_YAW_COV", "0.15"))
metrics_out = os.environ.get("AMCL_SETTLE_METRICS_FILE", "")

def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def yaw_spread_deg(yaws: list[float]) -> float:
    if len(yaws) < 2:
        return 0.0
    yaws = sorted(yaws)
    max_gap = max(yaws[i + 1] - yaws[i] for i in range(len(yaws) - 1))
    wrap_gap = (yaws[0] + 2.0 * math.pi) - yaws[-1]
    return math.degrees(max(max_gap, wrap_gap))

def probe_amcl_active() -> bool | None:
    probe = Path(lifecycle_probe)
    if not probe.is_file():
        return None
    try:
        out = subprocess.check_output(
            ["python3", str(probe), "wait", "/amcl", "5"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=8,
        )
        return "active [3]" in out
    except Exception:
        return None

def build_metrics(samples, scan_fresh: bool, map_tf_fresh: bool, amcl_active) -> dict:
    xs = [s["x"] for s in samples]
    ys = [s["y"] for s in samples]
    yaws = [s["yaw"] for s in samples]
    last = samples[-1] if samples else {"x_cov": 0.0, "y_cov": 0.0, "yaw_cov": 0.0}
    x_spread = (max(xs) - min(xs)) if xs else 0.0
    y_spread = (max(ys) - min(ys)) if ys else 0.0
    yaw_spread = yaw_spread_deg(yaws) if yaws else 0.0
    return {
        "sample_count": len(samples),
        "x_spread_m": x_spread,
        "y_spread_m": y_spread,
        "yaw_spread_deg": yaw_spread,
        "x_cov": float(last.get("x_cov", 0.0)),
        "y_cov": float(last.get("y_cov", 0.0)),
        "yaw_cov": float(last.get("yaw_cov", 0.0)),
        "scan_filtered_fresh": scan_fresh,
        "map_to_base_link_fresh": map_tf_fresh,
        "amcl_active": amcl_active,
    }

def checks_pass(metrics: dict) -> tuple[bool, list[str]]:
    failures: list[str] = []
    if metrics["sample_count"] < min_samples:
        failures.append(f"samples={metrics['sample_count']} < {min_samples}")
    if metrics["x_spread_m"] > max_x_spread:
        failures.append(f"x_spread={metrics['x_spread_m']:.4f}m > {max_x_spread}")
    if metrics["y_spread_m"] > max_y_spread:
        failures.append(f"y_spread={metrics['y_spread_m']:.4f}m > {max_y_spread}")
    if metrics["yaw_spread_deg"] > max_yaw_spread_deg:
        failures.append(f"yaw_spread={metrics['yaw_spread_deg']:.2f}deg > {max_yaw_spread_deg}")
    if metrics["x_cov"] > max_x_cov:
        failures.append(f"x_cov={metrics['x_cov']:.4f} > {max_x_cov}")
    if metrics["y_cov"] > max_y_cov:
        failures.append(f"y_cov={metrics['y_cov']:.4f} > {max_y_cov}")
    if metrics["yaw_cov"] > max_yaw_cov:
        failures.append(f"yaw_cov={metrics['yaw_cov']:.4f} > {max_yaw_cov}")
    if not metrics["scan_filtered_fresh"]:
        failures.append("scan_filtered not fresh")
    if not metrics["map_to_base_link_fresh"]:
        failures.append("map->base_link TF not fresh")
    if metrics["amcl_active"] is False:
        failures.append("amcl lifecycle not active")
    return (len(failures) == 0, failures)

def print_metrics(metrics: dict, failures: list[str] | None = None) -> None:
    print(
        "[NAV2_BOOT] AMCL settle metrics: "
        f"samples={metrics['sample_count']} "
        f"x_spread={metrics['x_spread_m']:.4f}m "
        f"y_spread={metrics['y_spread_m']:.4f}m "
        f"yaw_spread={metrics['yaw_spread_deg']:.2f}deg "
        f"x_cov={metrics['x_cov']:.4f} "
        f"y_cov={metrics['y_cov']:.4f} "
        f"yaw_cov={metrics['yaw_cov']:.4f} "
        f"scan_filtered_fresh={metrics['scan_filtered_fresh']} "
        f"map_to_base_link_fresh={metrics['map_to_base_link_fresh']} "
        f"amcl_active={metrics['amcl_active']}",
        flush=True,
    )
    if failures:
        print(f"[NAV2_BOOT] AMCL settle FAIL: {'; '.join(failures)}", flush=True)

def write_metrics_file(metrics: dict, passed: bool) -> None:
    if not metrics_out:
        return
    payload = dict(metrics)
    payload["passed"] = passed
    path = Path(metrics_out)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, path)

rclpy.init()
node = Node("nav2_wait_amcl_settle")
amcl_qos = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
)
samples: list[dict] = []
scan_last = {"t": 0.0}

def on_amcl_pose(msg: PoseWithCovarianceStamped) -> None:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    cov = msg.pose.covariance
    samples.append(
        {
            "t": time.time(),
            "x": float(p.x),
            "y": float(p.y),
            "yaw": yaw_from_quat(q),
            "x_cov": float(cov[0]),
            "y_cov": float(cov[7]),
            "yaw_cov": float(cov[35]),
        }
    )

def on_scan(_msg: LaserScan) -> None:
    scan_last["t"] = time.time()

node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_amcl_pose, amcl_qos)
node.create_subscription(LaserScan, "/scan_filtered", on_scan, qos_profile_sensor_data)
tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
TransformListener(tf_buffer, node, spin_thread=False)

start = time.time()
last_report = start
passed = False
final_metrics: dict = {}

while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.2)
    scan_fresh = (time.time() - scan_last["t"]) <= 3.0 if scan_last["t"] > 0 else False
    map_tf_fresh = False
    try:
        tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.2))
        map_tf_fresh = True
    except Exception:
        map_tf_fresh = False
    amcl_active = probe_amcl_active()
    metrics = build_metrics(samples, scan_fresh, map_tf_fresh, amcl_active)
    ok, failures = checks_pass(metrics)
    now = time.time()
    if now - last_report >= 5.0:
        print_metrics(metrics, failures if not ok else None)
        last_report = now
    if ok:
        passed = True
        final_metrics = metrics
        break

if not passed:
    scan_fresh = (time.time() - scan_last["t"]) <= 3.0 if scan_last["t"] > 0 else False
    map_tf_fresh = False
    try:
        tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.2))
        map_tf_fresh = True
    except Exception:
        pass
    amcl_active = probe_amcl_active()
    final_metrics = build_metrics(samples, scan_fresh, map_tf_fresh, amcl_active)
    _, failures = checks_pass(final_metrics)

node.destroy_node()
rclpy.shutdown()

if passed:
    print_metrics(final_metrics)
    print("[NAV2_BOOT] AMCL localization settled (hard gate PASS)", flush=True)
    write_metrics_file(final_metrics, True)
    raise SystemExit(0)

print_metrics(final_metrics, failures)
print(f"[NAV2_BOOT] ERROR: AMCL localization NOT settled after {timeout:.0f}s (hard gate FAIL)", flush=True)
write_metrics_file(final_metrics, False)
raise SystemExit(1)
PY
}

amcl_settle_metrics_json() {
  local metrics_file="${AMCL_SETTLE_METRICS_FILE:-}"
  python3 - "$metrics_file" <<'PY'
import json
import sys
from pathlib import Path

empty = {
    "x_spread_m": 0.0,
    "y_spread_m": 0.0,
    "yaw_spread_deg": 0.0,
    "x_cov": 0.0,
    "y_cov": 0.0,
    "yaw_cov": 0.0,
}
path = Path(sys.argv[1]) if sys.argv[1] else None
if not path or not path.is_file():
    print(json.dumps(empty))
    raise SystemExit(0)
data = json.loads(path.read_text(encoding="utf-8"))
out = {k: float(data.get(k, 0.0)) for k in empty}
print(json.dumps(out))
PY
}

amcl_settle_sample_count() {
  local metrics_file="${AMCL_SETTLE_METRICS_FILE:-}"
  python3 - "$metrics_file" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1]) if sys.argv[1] else None
if not path or not path.is_file():
    print(0)
    raise SystemExit(0)
data = json.loads(path.read_text(encoding="utf-8"))
print(int(data.get("sample_count", 0)))
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
  local status="$3"
  local amcl_settled="$4"
  local amcl_metrics_json="$5"
  local amcl_sample_count="$6"
  local reuse_scan="${7:-0}"
  local reuse_scan_filtered="${8:-0}"
  local reuse_chassis="${9:-0}"
  python3 - "$log_dir" "$map_yaml" "$status" "$amcl_settled" "$amcl_metrics_json" \
    "$amcl_sample_count" "$reuse_scan" "$reuse_scan_filtered" "$reuse_chassis" \
    "$_LIFECYCLE_PROBE" "$_ROS_TOPIC_PROBE" <<'PY'
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

log_dir = Path(sys.argv[1])
map_yaml = sys.argv[2]
status = sys.argv[3]
amcl_settled = sys.argv[4] == "1"
amcl_metrics = json.loads(sys.argv[5] or "{}")
amcl_sample_count = int(sys.argv[6] or "0")
reuse_scan = sys.argv[7] == "1"
reuse_scan_filtered = sys.argv[8] == "1"
reuse_chassis = sys.argv[9] == "1"
lifecycle_probe = sys.argv[10]
topic_probe = sys.argv[11]

def map_publisher_count() -> int:
    try:
        out = subprocess.check_output(
            ["ros2", "topic", "info", "/map", "-v"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=8,
        )
        for line in out.splitlines():
            if "Publisher count:" in line:
                return int(line.split(":", 1)[1].strip())
    except Exception:
        pass
    return 0

def lifecycle_active(node_name: str) -> bool:
    try:
        out = subprocess.check_output(
            ["python3", lifecycle_probe, "wait", node_name, "5"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
        return "active [3]" in out
    except Exception:
        return False

def topic_fresh(topic: str, sensor_qos: bool = False) -> bool:
    qos_flags = ["--sensor-qos"] if sensor_qos else []
    try:
        subprocess.check_call(
            [
                "python3",
                topic_probe,
                "has-samples",
                topic,
                "1",
                "8",
                *qos_flags,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=12,
        )
        return True
    except Exception:
        return False

def nav_actions_ready() -> tuple[bool, bool]:
    try:
        subprocess.check_call(
            ["python3", lifecycle_probe, "nav-actions", "15"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=20,
        )
        return True, True
    except Exception:
        return False, False

map_tf_fresh = False
rclpy.init()
node = Node("nav2_ready_json_probe")
tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
TransformListener(tf_buffer, node, spin_thread=False)
deadline = time.time() + 5.0
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.2))
        map_tf_fresh = True
        break
    except Exception:
        pass
node.destroy_node()
rclpy.shutdown()

compute_ready, navigate_ready = nav_actions_ready()
ready_for_goal = status == "READY_FOR_GOAL" and amcl_settled and compute_ready and navigate_ready and map_tf_fresh

payload = {
    "schema_version": 2,
    "status": status,
    "ready_for_goal": ready_for_goal,
    "map_yaml": str(Path(map_yaml).resolve()),
    "map_publisher_count": map_publisher_count(),
    "map_server_active": lifecycle_active("/map_server"),
    "amcl_active": lifecycle_active("/amcl"),
    "amcl_settled": amcl_settled,
    "amcl_sample_count": amcl_sample_count,
    "amcl_metrics": {
        "x_spread_m": float(amcl_metrics.get("x_spread_m", 0.0)),
        "y_spread_m": float(amcl_metrics.get("y_spread_m", 0.0)),
        "yaw_spread_deg": float(amcl_metrics.get("yaw_spread_deg", 0.0)),
        "x_cov": float(amcl_metrics.get("x_cov", 0.0)),
        "y_cov": float(amcl_metrics.get("y_cov", 0.0)),
        "yaw_cov": float(amcl_metrics.get("yaw_cov", 0.0)),
    },
    "map_to_base_link_fresh": map_tf_fresh,
    "scan_fresh": topic_fresh("/scan", sensor_qos=True),
    "scan_filtered_fresh": topic_fresh("/scan_filtered", sensor_qos=True),
    "odom_fresh": topic_fresh("/odom"),
    "compute_path_to_pose_ready": compute_ready,
    "navigate_to_pose_ready": navigate_ready,
    "reused_lidar": reuse_scan,
    "reused_scan_filter": reuse_scan_filtered,
    "reused_chassis": reuse_chassis,
    "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
}

out = log_dir / "ready.json"
tmp = out.with_suffix(".json.tmp")
tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
os.replace(tmp, out)
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
