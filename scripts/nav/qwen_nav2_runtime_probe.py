#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rclpy-only runtime probes for Qwen target Nav2 reuse pipeline.

IMPORTANT: This module is READ-ONLY for sensor nodes.
It must NEVER Popen/restart lidar, scan_filter, chassis, or static TF.
Repair belongs in run_qwen_target_nav2_reuse.sh::repair_sensor_base_once.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_DIR = _SCRIPT_DIR.parents[1]
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

_DEFAULT_FASTDDS = str(_PROJECT_DIR / "configs" / "fastdds_no_shm.xml")
if not os.environ.get("FASTRTPS_DEFAULT_PROFILES"):
    os.environ["FASTRTPS_DEFAULT_PROFILES"] = _DEFAULT_FASTDDS

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
    qos_profile_system_default,
)
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

from qwen_nav2_common import (
    amcl_settle_near_threshold,
    atomic_write_json,
    evaluate_amcl_settle_window,
    normalize_yaw,
    read_proc_start_ticks,
    realpath,
    sensor_health_overall_pass_v2,
    time_now,
)

PRIMARY_STATE_ACTIVE = 3
LIFECYCLE_NODES = [
    "map_server",
    "amcl",
    "planner_server",
    "controller_server",
    "bt_navigator",
    "behavior_server",
]
OPTIONAL_LIFECYCLE_NODES = ["velocity_smoother", "smoother_server"]


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return normalize_yaw(math.atan2(siny, cosy))


def _pgrep_af(pattern: str) -> List[Tuple[int, str]]:
    try:
        out = subprocess.check_output(
            ["pgrep", "-af", pattern], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except subprocess.CalledProcessError:
        return []
    rows: List[Tuple[int, str]] = []
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        try:
            rows.append((int(parts[0]), parts[1] if len(parts) > 1 else ""))
        except ValueError:
            continue
    return rows


def _proc_inventory(pattern: str) -> Dict[str, Any]:
    rows = _pgrep_af(pattern)
    pids = [p for p, _ in rows]
    return {
        "count": len(rows),
        "pids": pids,
        "start_ticks": {str(p): read_proc_start_ticks(p) for p in pids},
        "cmdlines": [c for _, c in rows],
    }


def _chassis_device_holder(device: str, expected_pid: Optional[int]) -> bool:
    if expected_pid is None:
        return False
    try:
        out = subprocess.check_output(["lsof", "-t", device], text=True, stderr=subprocess.DEVNULL)
        holders = {int(x) for x in out.split() if x.strip().isdigit()}
    except (subprocess.CalledProcessError, FileNotFoundError, ValueError):
        # Best effort: if lsof unavailable, accept unique chassis PID.
        return True
    return expected_pid in holders and holders == {expected_pid}


def _port_listening(port: int = 8765) -> bool:
    try:
        import socket

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.3)
        ok = sock.connect_ex(("127.0.0.1", port)) == 0
        sock.close()
        return ok
    except Exception:
        return False


class RuntimeProbe(Node):
    """Parallel subscribers for scan/scan_filtered/odom + TF buffer."""

    def __init__(self, name: str = "qwen_nav2_runtime_probe") -> None:
        super().__init__(name)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._scan: Deque[Tuple[float, LaserScan]] = deque(maxlen=30)
        self._scan_filtered: Deque[Tuple[float, LaserScan]] = deque(maxlen=30)
        self._odom: Deque[Tuple[float, Odometry]] = deque(maxlen=40)
        self._map_msg: Optional[OccupancyGrid] = None
        self._scan_frame = ""
        self._last_scan_mono = 0.0
        self._last_filt_mono = 0.0
        self._last_odom_mono = 0.0
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "/scan_filtered", self._on_filt, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        self._scan.append((now, msg))
        self._last_scan_mono = now
        if msg.header.frame_id:
            self._scan_frame = msg.header.frame_id.lstrip("/")

    def _on_filt(self, msg: LaserScan) -> None:
        now = time.monotonic()
        self._scan_filtered.append((now, msg))
        self._last_filt_mono = now

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        self._odom.append((now, msg))
        self._last_odom_mono = now

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_msg = msg

    def spin_until(self, duration_s: float) -> None:
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    @property
    def scan_frame(self) -> str:
        return self._scan_frame or "laser"

    def _valid_scan(
        self, samples: Deque[Tuple[float, LaserScan]], min_count: int, max_age: float
    ) -> Tuple[bool, str, Dict[str, Any]]:
        if len(samples) < min_count:
            return False, f"need {min_count} samples, got {len(samples)}", {"ok": False, "sample_count": len(samples)}
        stamps = [
            s[1].header.stamp.sec + s[1].header.stamp.nanosec * 1e-9 for s in list(samples)[-min_count:]
        ]
        if any(stamps[i] <= stamps[i - 1] for i in range(1, len(stamps))):
            return False, "timestamps not increasing", {"ok": False}
        last_mono, last = samples[-1]
        if not last.ranges:
            return False, "empty ranges", {"ok": False}
        if not last.header.frame_id:
            return False, "empty frame_id", {"ok": False}
        valid = sum(1 for r in last.ranges if math.isfinite(r) and r > 0.0)
        if valid <= 0:
            return False, "no valid ranges", {"ok": False}
        age = time.monotonic() - last_mono
        if age > max_age:
            return False, f"stale age={age:.2f}s", {"ok": False, "age_s": age}
        return True, "ok", {
            "ok": True,
            "frame": last.header.frame_id.lstrip("/"),
            "age_s": round(age, 3),
            "sample_count": len(samples),
        }

    def collect_parallel_health(self, window_s: float = 3.0) -> Dict[str, Any]:
        """Subscribe all topics in parallel for window_s, then evaluate."""
        self.spin_until(window_s)
        ok_scan, reason_scan, scan_info = self._valid_scan(self._scan, 3, 1.0)
        scan_frame = str(scan_info.get("frame", self.scan_frame))
        ok_filt, reason_filt, filt_info = self._valid_scan(self._scan_filtered, 3, 1.0)
        if ok_filt and filt_info.get("frame") and filt_info["frame"] != scan_frame:
            ok_filt = False
            reason_filt = f"frame mismatch {filt_info.get('frame')} != {scan_frame}"
            filt_info["ok"] = False

        ok_odom = False
        reason_odom = "need samples"
        odom_info: Dict[str, Any] = {"ok": False}
        if len(self._odom) >= 5:
            last_mono, last = self._odom[-1]
            stamps = [
                s[1].header.stamp.sec + s[1].header.stamp.nanosec * 1e-9
                for s in list(self._odom)[-5:]
            ]
            if any(stamps[i] <= stamps[i - 1] for i in range(1, len(stamps))):
                reason_odom = "timestamps not increasing"
            elif last.header.frame_id != "odom":
                reason_odom = f"frame_id={last.header.frame_id!r}"
            elif last.child_frame_id.lstrip("/") != "base_link":
                reason_odom = f"child={last.child_frame_id!r}"
            else:
                p = last.pose.pose.position
                q = last.pose.pose.orientation
                if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                    reason_odom = "non-finite"
                else:
                    age = time.monotonic() - last_mono
                    if age > 0.5:
                        reason_odom = f"stale age={age:.2f}s"
                    else:
                        ok_odom = True
                        reason_odom = "ok"
                        odom_info = {"ok": True, "age_s": round(age, 3), "sample_count": len(self._odom)}
        if not ok_odom and not odom_info.get("ok"):
            odom_info = {"ok": False, "reason": reason_odom, "sample_count": len(self._odom)}

        # TF (up to 5s remaining budget already collected; quick lookup)
        odom_tf = False
        laser_tf = False
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                self.tf_buffer.lookup_transform(
                    "odom", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.15)
                )
                odom_tf = True
            except Exception:
                pass
            try:
                self.tf_buffer.lookup_transform(
                    "base_link", scan_frame, rclpy.time.Time(), timeout=Duration(seconds=0.15)
                )
                laser_tf = True
            except Exception:
                pass
            if odom_tf and laser_tf:
                break
        ok_tf = odom_tf and laser_tf
        tf_info = {"odom_base_link": odom_tf, "base_link_laser": laser_tf, "ok": ok_tf}

        lidar_inv = _proc_inventory("ydlidar")
        filt_inv = _proc_inventory("simple_scan_filter.py")
        chassis_inv = _proc_inventory("m1_pwm_cmd_vel_bridge.py")
        device = os.environ.get("CHASSIS_DEV", "/dev/rosmaster")
        chassis_pid = chassis_inv["pids"][0] if chassis_inv["count"] == 1 else None
        chassis_ok = (
            chassis_inv["count"] == 1
            and _chassis_device_holder(device, chassis_pid)
        )
        chassis = {
            "ok": chassis_ok,
            "count": chassis_inv["count"],
            "pids": chassis_inv["pids"],
            "start_ticks": chassis_inv["start_ticks"],
            "cmdlines": chassis_inv["cmdlines"],
            "device": device,
            "device_held_by_pid": chassis_pid,
        }
        if chassis_pid is not None:
            chassis["pid"] = chassis_pid

        fox_inv = _proc_inventory("foxglove_bridge")
        port_ok = _port_listening(8765)
        fox = {
            "ok": fox_inv["count"] >= 1 and port_ok,
            "status": "OK" if (fox_inv["count"] >= 1 and port_ok) else "WARN",
            "reused": fox_inv["count"] >= 1 and port_ok,
            "port": 8765 if port_ok else None,
            "bridge_count": fox_inv["count"],
            "count": fox_inv["count"],
            "pids": fox_inv["pids"],
            "start_ticks": fox_inv["start_ticks"],
        }

        scan_filter = {
            "count": filt_inv["count"],
            "pids": filt_inv["pids"],
            "start_ticks": filt_inv["start_ticks"],
            "cmdlines": filt_inv["cmdlines"],
            "topic_fresh": bool(ok_filt),
        }
        lidar = {
            "count": lidar_inv["count"],
            "pids": lidar_inv["pids"],
            "start_ticks": lidar_inv["start_ticks"],
            "cmdlines": lidar_inv["cmdlines"],
        }

        # Duplicate static TF is common after mapping handoff; warn only, do not block nav.
        static_tf_inv = _proc_inventory("static_transform_publisher")
        static_tf_dup = static_tf_inv["count"] > 1
        tf_info["static_tf"] = static_tf_inv
        if static_tf_dup:
            tf_info["static_tf_duplicate"] = True
            tf_info["static_tf_duplicate_warn"] = True

        overall = sensor_health_overall_pass_v2(
            scan_ok=ok_scan,
            scan_filtered_ok=ok_filt,
            odom_ok=ok_odom,
            tf_ok=ok_tf,
            chassis_ok=chassis_ok,
            lidar_count=int(lidar["count"]),
            scan_filter_count=int(scan_filter["count"]),
            chassis_count=int(chassis["count"]),
            foxglove_ok=bool(fox.get("ok")),
        )
        return {
            "status": "PASS" if overall else "FAIL",
            "checked_epoch": time_now(),
            "scan": {
                **(scan_info if scan_info.get("ok") else {"ok": False, "reason": reason_scan, **scan_info}),
                "receive_age_s": scan_info.get("age_s"),
            },
            "scan_filtered": {
                **(filt_info if filt_info.get("ok") else {"ok": False, "reason": reason_filt, **filt_info}),
                "receive_age_s": filt_info.get("age_s"),
            },
            "odom": {**odom_info, "receive_age_s": odom_info.get("age_s")},
            "tf": tf_info,
            "chassis": chassis,
            "foxglove": fox,
            "lidar_process": lidar,
            "scan_filter_process": scan_filter,
            "process_counts": {
                "lidar": lidar["count"],
                "scan_filter": scan_filter["count"],
                "chassis": chassis["count"],
                "foxglove": fox["count"],
                "static_tf": static_tf_inv["count"],
            },
            "actual_scan_frame": scan_frame,
            "scan_frame": scan_frame,
            "reasons": {
                "scan": reason_scan,
                "scan_filtered": reason_filt,
                "odom": reason_odom,
                "tf": (
                    "ok_with_static_tf_duplicate_warn"
                    if (ok_tf and static_tf_dup)
                    else ("ok" if ok_tf else "tf missing")
                ),
                "lidar_count": lidar["count"],
                "scan_filter_count": scan_filter["count"],
                "chassis_count": chassis["count"],
            },
        }

    def get_lifecycle_state(self, target: str) -> Tuple[Optional[int], str]:
        client = self.create_client(GetState, f"/{target}/get_state")
        if not client.wait_for_service(timeout_sec=2.0):
            return None, "no_service"
        fut = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return None, "call_failed"
        st = fut.result().current_state
        return int(st.id), f"{st.label} [{st.id}]"

    def wait_lifecycle_nodes(self, nodes: List[str], timeout_s: float) -> Tuple[bool, Dict[str, str]]:
        deadline = time.monotonic() + timeout_s
        status: Dict[str, str] = {n: "pending" for n in nodes}
        while time.monotonic() < deadline:
            all_ok = True
            for node in nodes:
                state_id, label = self.get_lifecycle_state(node)
                status[node] = label
                if state_id != PRIMARY_STATE_ACTIVE:
                    all_ok = False
            if all_ok:
                return True, status
            time.sleep(1.0)
        return False, status

    def verify_map(self, map_yaml: Path) -> Tuple[bool, str, Dict[str, Any]]:
        import yaml
        from PIL import Image

        self.spin_until(5.0)
        if self._map_msg is None:
            return False, "no /map message", {}
        grid = self._map_msg
        if grid.header.frame_id != "map":
            return False, f"map frame_id={grid.header.frame_id!r}", {}
        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution
        if w <= 0 or h <= 0 or res <= 0:
            return False, "invalid map dimensions", {}
        if len(grid.data) != w * h:
            return False, "map data length mismatch", {}
        meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
        yaml_res = float(meta.get("resolution", 0))
        origin = meta.get("origin") or [0, 0, 0]
        image_name = str(meta.get("image", ""))
        pgm_path = map_yaml.parent / image_name
        if not pgm_path.is_file():
            pgm_path = map_yaml.with_suffix(".pgm")
        with Image.open(pgm_path) as img:
            pw, ph = img.size
        if abs(yaml_res - res) > 1e-4:
            return False, f"resolution mismatch yaml={yaml_res} map={res}", {}
        if pw != w or ph != h:
            return False, f"size mismatch pgm=({pw},{ph}) map=({w},{h})", {}
        if abs(float(origin[0]) - grid.info.origin.position.x) > 1e-3:
            return False, "origin x mismatch", {}
        if abs(float(origin[1]) - grid.info.origin.position.y) > 1e-3:
            return False, "origin y mismatch", {}
        return True, "ok", {"width": w, "height": h, "resolution": res}

    def lookup_map_base(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.3)
            )
            t = tf.transform.translation
            return float(t.x), float(t.y), yaw_from_quat(tf.transform.rotation)
        except Exception:
            return None

    def tf_exists(self, parent: str, child: str, timeout_s: float = 1.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.15)
                )
                return True
            except Exception:
                pass
        return False


def cmd_sensor_health(args: argparse.Namespace) -> int:
    """Read-only parallel sensor health. Never repairs nodes."""
    runtime_dir = Path(args.runtime_dir)
    runtime_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = RuntimeProbe()
    try:
        payload = node.collect_parallel_health(window_s=float(args.window))
        payload["session_id"] = args.session_id
        atomic_write_json(runtime_dir / "sensor_health.json", payload)
        if payload["status"] != "PASS":
            print(f"[PROBE] sensor health FAIL reasons={payload.get('reasons')}")
            return 1
        if not payload.get("foxglove", {}).get("ok"):
            print("[PROBE] WARN: foxglove not healthy (non-blocking)")
        print("[PROBE] sensor health PASS")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def cmd_tf_check(args: argparse.Namespace) -> int:
    rclpy.init()
    node = RuntimeProbe("qwen_tf_check")
    try:
        node.spin_until(1.0)
        ok = node.tf_exists(args.parent, args.child, timeout_s=float(args.timeout))
        print(json.dumps({"ok": ok, "parent": args.parent, "child": args.child}))
        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def cmd_pose_compare(args: argparse.Namespace) -> int:
    from qwen_nav2_common import pose_delta_ok

    pose = json.loads(Path(args.pose_json).read_text(encoding="utf-8"))
    rclpy.init()
    node = RuntimeProbe("qwen_pose_compare")
    try:
        node.spin_until(2.0)
        live = None
        deadline = time.monotonic() + float(args.timeout)
        while time.monotonic() < deadline and live is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            live = node.lookup_map_base()
        if live is None:
            print("[PROBE] FAIL: cannot read live map->base_link before AMCL")
            return 1
        ok, metrics = pose_delta_ok(
            float(pose["x"]),
            float(pose["y"]),
            float(pose.get("yaw", 0.0)),
            live[0],
            live[1],
            live[2],
        )
        print(json.dumps({"ok": ok, "metrics": metrics, "live": {"x": live[0], "y": live[1], "yaw": live[2]}}))
        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def cmd_wait_nav_ready(args: argparse.Namespace) -> int:
    """Combined: lifecycle + map verify + amcl init + amcl settle."""
    runtime_dir = Path(args.runtime_dir)
    runtime_dir.mkdir(parents=True, exist_ok=True)
    pose = json.loads(Path(args.pose_json).read_text(encoding="utf-8"))
    if str(pose.get("frame_id", "map")) != "map":
        print("[PROBE] pose frame_id must be map")
        return 1
    x, y = float(pose["x"]), float(pose["y"])
    yaw = float(pose.get("yaw", 0.0))
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        print("[PROBE] pose not finite")
        return 1

    rclpy.init()
    node = RuntimeProbe("qwen_wait_nav_ready")
    try:
        # 1) localization lifecycle
        ok_loc, st_loc = node.wait_lifecycle_nodes(["map_server", "amcl"], float(args.loc_timeout))
        if not ok_loc:
            print(f"[PROBE] localization lifecycle FAIL: {st_loc}")
            return 1

        # 2) map received + verify (warn on mismatch — still publish initialpose)
        ok_map, reason_map, map_info = node.verify_map(realpath(Path(args.map_yaml)))
        if not ok_map:
            print(f"[PROBE] WARN map verify: {reason_map} — continue")
        else:
            print(f"[PROBE] map verify PASS {map_info}")

        # 3) navigation lifecycle
        nav_nodes = [n for n in LIFECYCLE_NODES if n not in ("map_server", "amcl")]
        for opt in OPTIONAL_LIFECYCLE_NODES:
            state_id, _ = node.get_lifecycle_state(opt)
            if state_id is not None:
                nav_nodes.append(opt)
        ok_nav, st_nav = node.wait_lifecycle_nodes(nav_nodes, float(args.nav_timeout))
        if not ok_nav:
            print(f"[PROBE] navigation lifecycle FAIL: {st_nav}")
            return 1
        print("[PROBE] LOCALIZATION_ACTIVE")

        # 4) require scan_filtered fresh before initialpose (warn only — do not block)
        node.spin_until(1.0)
        if time.monotonic() - node._last_filt_mono > 1.0:
            print("[PROBE] WARN: /scan_filtered not fresh before initialpose — continue")

        # 5) initialpose with SystemDefaultsQoS, 3x @ 0.25s
        pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos_profile_system_default)
        cov = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467,
        ]
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance = cov
        for _ in range(3):
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.25)
        print("[PROBE] AMCL initialpose published x3")

        # 6) request_nomotion_update once
        client = node.create_client(Empty, "/request_nomotion_update")
        if client.wait_for_service(timeout_sec=3.0):
            fut = client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
            print("[PROBE] request_nomotion_update called")
        else:
            print("[PROBE] WARN: /request_nomotion_update unavailable")

        if args.skip_settle:
            atomic_write_json(
                runtime_dir / "nav2_ready_probe.json",
                {"status": "ACTIVE_NO_SETTLE", "lifecycle": {**st_loc, **st_nav}},
            )
            return 0

        # 7) AMCL settle — TransientLocal Reliable KeepLast(1)
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        samples: Deque[Dict[str, float]] = deque(maxlen=12)
        stable_hits = 0

        def on_amcl(msg: PoseWithCovarianceStamped) -> None:
            p = msg.pose.pose.position
            yaw_v = yaw_from_quat(msg.pose.pose.orientation)
            cov_m = msg.pose.covariance
            samples.append(
                {
                    "x": float(p.x),
                    "y": float(p.y),
                    "yaw": yaw_v,
                    "x_cov": float(cov_m[0]),
                    "y_cov": float(cov_m[7]),
                    "yaw_cov": float(cov_m[35]),
                }
            )

        node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_amcl, amcl_qos)
        base_timeout = float(args.settle_timeout)
        deadline = time.monotonic() + base_timeout
        extended_once = False
        last_reasons: List[str] = []
        last_metrics: Dict[str, Any] = {}
        prev_sample_count = 0
        samples_growing = False
        while True:
            now = time.monotonic()
            if now >= deadline:
                # One optional +15s extension only if samples grow + near threshold + TF/scan OK.
                if (
                    not extended_once
                    and samples_growing
                    and amcl_settle_near_threshold(last_metrics)
                    and last_metrics.get("map_tf_ok")
                    and float(last_metrics.get("scan_age_s", 99)) <= 1.0
                    and float(last_metrics.get("odom_age_s", 99)) <= 0.5
                ):
                    extended_once = True
                    deadline = time.monotonic() + 15.0
                    print("[PROBE] AMCL settle near threshold — extend once +15s")
                    continue
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            scan_age = (
                time.monotonic() - node._last_filt_mono if node._last_filt_mono else 999.0
            )
            odom_age = (
                time.monotonic() - node._last_odom_mono if node._last_odom_mono else 999.0
            )
            map_tf_ok = node.lookup_map_base() is not None
            ok, reasons, metrics = evaluate_amcl_settle_window(
                list(samples),
                scan_age_s=scan_age,
                odom_age_s=odom_age,
                map_tf_ok=map_tf_ok,
            )
            cur_count = int(metrics.get("sample_count", 0))
            if cur_count > prev_sample_count:
                samples_growing = True
            prev_sample_count = cur_count
            last_reasons = reasons
            last_metrics = metrics
            if ok:
                stable_hits += 1
                if stable_hits >= 2:
                    metrics["extended_once"] = extended_once
                    atomic_write_json(runtime_dir / "amcl_settle_metrics.json", metrics)
                    print(f"[PROBE] AMCL settled {metrics}")
                    return 0
            else:
                stable_hits = 0

        fail_payload = {
            "status": "LOCALIZATION_UNSETTLED",
            "sample_count": last_metrics.get("sample_count"),
            "x_spread_m": last_metrics.get("x_spread_m"),
            "y_spread_m": last_metrics.get("y_spread_m"),
            "yaw_spread_deg": last_metrics.get("yaw_spread_deg"),
            "covariance": {
                "x": last_metrics.get("x_cov"),
                "y": last_metrics.get("y_cov"),
                "yaw": last_metrics.get("yaw_cov"),
            },
            "map_tf_ok": last_metrics.get("map_tf_ok"),
            "scan_age_s": last_metrics.get("scan_age_s"),
            "odom_age_s": last_metrics.get("odom_age_s"),
            "reasons": last_reasons,
            "metrics": last_metrics,
            "extended_once": extended_once,
            "samples_growing": samples_growing,
        }
        atomic_write_json(runtime_dir / "amcl_settle_metrics.json", fail_payload)
        print(f"[PROBE] AMCL settle FAIL reasons={last_reasons} metrics={last_metrics}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def cmd_cmd_vel_publishers(args: argparse.Namespace) -> int:
    """Report /cmd_vel and /cmd_vel_nav publishers (rclpy)."""
    rclpy.init()
    node = Node("qwen_cmd_vel_pub_probe")
    try:
        cmd = node.get_publishers_info_by_topic("/cmd_vel")
        nav = node.get_publishers_info_by_topic("/cmd_vel_nav")
        payload = {
            "cmd_vel": [
                {"node": p.node_name, "topic_type": p.topic_type} for p in cmd
            ],
            "cmd_vel_nav": [
                {"node": p.node_name, "topic_type": p.topic_type} for p in nav
            ],
        }
        print(json.dumps(payload))
        teleop_names = ("teleop", "joy")
        bad = [
            p
            for p in payload["cmd_vel"]
            if any(t in p["node"].lower() for t in teleop_names)
        ]
        return 1 if bad else 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Qwen Nav2 runtime probe (read-only)")
    sub = p.add_subparsers(dest="cmd", required=True)

    sh = sub.add_parser("sensor-health")
    sh.add_argument("--session-id", required=True)
    sh.add_argument("--runtime-dir", required=True)
    sh.add_argument("--window", type=float, default=3.0)
    sh.set_defaults(func=cmd_sensor_health)

    tf = sub.add_parser("tf-check")
    tf.add_argument("--parent", required=True)
    tf.add_argument("--child", required=True)
    tf.add_argument("--timeout", type=float, default=5.0)
    tf.set_defaults(func=cmd_tf_check)

    pc = sub.add_parser("pose-compare")
    pc.add_argument("--pose-json", required=True)
    pc.add_argument("--timeout", type=float, default=5.0)
    pc.set_defaults(func=cmd_pose_compare)

    wr = sub.add_parser("wait-nav-ready")
    wr.add_argument("--map-yaml", required=True)
    wr.add_argument("--pose-json", required=True)
    wr.add_argument("--runtime-dir", required=True)
    wr.add_argument("--loc-timeout", type=float, default=25.0)
    wr.add_argument("--nav-timeout", type=float, default=30.0)
    wr.add_argument("--settle-timeout", type=float, default=30.0)
    wr.add_argument("--skip-settle", action="store_true")
    wr.set_defaults(func=cmd_wait_nav_ready)

    cv = sub.add_parser("cmd-vel-publishers")
    cv.set_defaults(func=cmd_cmd_vel_publishers)
    return p


def main() -> int:
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
