#!/usr/bin/env python3
"""Count ROS topic samples via rclpy (handles sensor BEST_EFFORT QoS reliably)."""

from __future__ import annotations

import importlib
import os
import sys
import time

_DEFAULT_FASTDDS = "/root/rdk_x5_vln_robot/configs/fastdds_no_shm.xml"
if not os.environ.get("FASTRTPS_DEFAULT_PROFILES"):
    os.environ["FASTRTPS_DEFAULT_PROFILES"] = _DEFAULT_FASTDDS

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

_TOPIC_TYPES: dict[str, tuple[str, str]] = {
    "/scan": ("sensor_msgs.msg", "LaserScan"),
    "/scan_filtered": ("sensor_msgs.msg", "LaserScan"),
    "/odom": ("nav_msgs.msg", "Odometry"),
    "/tf": ("tf2_msgs.msg", "TFMessage"),
    "/map": ("nav_msgs.msg", "OccupancyGrid"),
}

_SENSOR_TOPICS = {"/scan", "/scan_filtered"}


def _load_msg_class(module_name: str, class_name: str):
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


def _qos_for_topic(topic: str, sensor_qos: bool) -> QoSProfile:
    if sensor_qos or topic in _SENSOR_TOPICS:
        return qos_profile_sensor_data
    if topic == "/map":
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def has_samples(
    topic: str,
    min_samples: int,
    timeout_sec: float,
    *,
    sensor_qos: bool = False,
) -> int:
    min_samples = max(1, int(min_samples))
    timeout_sec = max(0.5, float(timeout_sec))
    sensor_qos = sensor_qos or topic in _SENSOR_TOPICS

    if topic not in _TOPIC_TYPES:
        print(f"unknown topic type for {topic}", file=sys.stderr)
        return 2

    if not rclpy.ok():
        rclpy.init()

    node = Node("ros_topic_probe")
    msg_cls = _load_msg_class(*_TOPIC_TYPES[topic])
    qos = _qos_for_topic(topic, sensor_qos)
    count = {"n": 0}

    def _cb(_msg) -> None:
        count["n"] += 1

    node.create_subscription(msg_cls, topic, _cb, qos)
    deadline = time.time() + timeout_sec
    last_report = time.time()
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if count["n"] >= min_samples:
                print(
                    f"topic publishing OK (rclpy): {topic} samples={count['n']}",
                    flush=True,
                )
                return 0
            now = time.time()
            if now - last_report >= 10.0:
                print(
                    f"topic wait (rclpy): {topic} samples={count['n']}/{min_samples} "
                    f"({int(now - (deadline - timeout_sec))}s)",
                    flush=True,
                )
                last_report = now
            time.sleep(0.1)
        print(
            f"topic not publishing (rclpy): {topic} samples={count['n']}/{min_samples}",
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def scan_frame_id(topic: str, timeout_sec: float) -> int:
    timeout_sec = max(0.5, float(timeout_sec))
    if topic not in _TOPIC_TYPES:
        print(f"unknown topic type for {topic}", file=sys.stderr)
        return 2

    if not rclpy.ok():
        rclpy.init()

    node = Node("ros_topic_probe_scan_frame")
    msg_cls = _load_msg_class(*_TOPIC_TYPES[topic])
    qos = _qos_for_topic(topic, True)
    frame = {"id": ""}

    def _cb(msg) -> None:
        frame["id"] = str(getattr(msg.header, "frame_id", "") or "").strip()

    node.create_subscription(msg_cls, topic, _cb, qos)
    deadline = time.time() + timeout_sec
    try:
        while time.time() < deadline and not frame["id"]:
            rclpy.spin_once(node, timeout_sec=0.2)
            time.sleep(0.05)
        if frame["id"]:
            print(frame["id"], flush=True)
            return 0
        print(f"no frame_id on {topic} within {timeout_sec}s", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    if len(sys.argv) < 2:
        print(
            "usage: ros_topic_probe.py has-samples <topic> <min_samples> <timeout_sec> "
            "[--sensor-qos|--reliable]\n"
            "       ros_topic_probe.py scan-frame-id <topic> <timeout_sec>",
            file=sys.stderr,
        )
        return 2

    cmd = sys.argv[1]
    if cmd == "scan-frame-id":
        if len(sys.argv) < 4:
            print("usage: ros_topic_probe.py scan-frame-id <topic> <timeout_sec>", file=sys.stderr)
            return 2
        return scan_frame_id(sys.argv[2], float(sys.argv[3]))

    if cmd != "has-samples":
        print(f"unknown command: {cmd}", file=sys.stderr)
        return 2

    if len(sys.argv) < 5:
        print("usage: ros_topic_probe.py has-samples <topic> <min> <timeout> [qos]", file=sys.stderr)
        return 2

    topic = sys.argv[2]
    min_samples = int(sys.argv[3])
    timeout_sec = float(sys.argv[4])
    sensor_qos = topic in _SENSOR_TOPICS
    for flag in sys.argv[5:]:
        if flag == "--sensor-qos":
            sensor_qos = True
        elif flag == "--reliable":
            sensor_qos = False

    return has_samples(topic, min_samples, timeout_sec, sensor_qos=sensor_qos)


if __name__ == "__main__":
    raise SystemExit(main())
