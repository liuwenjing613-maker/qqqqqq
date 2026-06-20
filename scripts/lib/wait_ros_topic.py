#!/usr/bin/env python3
"""Wait until a ROS2 topic publishes at least one message (correct QoS, no huge echo)."""

from __future__ import annotations

import argparse
import importlib
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data


def _msg_class(type_str: str):
    pkg, name = type_str.split("/msg/", 1)
    mod = importlib.import_module(f"{pkg}.msg")
    return getattr(mod, name)


def _pick_qos(type_str: str) -> QoSProfile:
    if "LaserScan" in type_str:
        return qos_profile_sensor_data
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


def _find_topic_types(node: Node, topic: str) -> list[str]:
    for name, types in node.get_topic_names_and_types():
        if name == topic:
            return list(types)
    return []


def wait_for_topic(topic: str, timeout_sec: float, min_msgs: int = 1) -> bool:
    rclpy.init(args=None)
    node = Node("wait_ros_topic")
    received = {"n": 0}
    sub = None
    deadline = time.time() + max(timeout_sec, 0.5)

    try:
        while time.time() < deadline:
            types = _find_topic_types(node, topic)
            if types and sub is None:
                msg_class = _msg_class(types[0])
                qos = _pick_qos(types[0])

                def _cb(_msg, counter=received):
                    counter["n"] += 1

                sub = node.create_subscription(msg_class, topic, _cb, qos)

            if received["n"] >= min_msgs:
                return True

            remaining = max(0.0, deadline - time.time())
            if remaining <= 0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.25, remaining))

        return received["n"] >= min_msgs
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Block until topic publishes")
    parser.add_argument("--topic", required=True)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--min-msgs", type=int, default=1)
    args = parser.parse_args()

    ok = wait_for_topic(args.topic, args.timeout, max(1, args.min_msgs))
    if not ok:
        print(f"TIMEOUT: no message on {args.topic} within {args.timeout}s", file=sys.stderr)
        return 1
    print(f"OK: {args.topic}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
