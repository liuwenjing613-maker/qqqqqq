#!/usr/bin/env python3
"""Probe commanded vx vs encoder-reported motion for M1."""

import argparse
import json
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class VxHoldProbe(Node):
    def __init__(self, target_vx: float, duration: float):
        super().__init__("m1_vx_hold_probe")
        self.target_vx = float(target_vx)
        self.duration = float(duration)
        self.sent_samples = []
        self.odom_samples = []
        self.bridge_state = None
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel_sent", self._on_sent, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/chassis_bridge_state", self._on_state, 10)
        self.timer = self.create_timer(0.05, self._publish_cmd)

    def _on_sent(self, msg: Twist):
        self.sent_samples.append(float(msg.linear.x))

    def _on_odom(self, msg: Odometry):
        self.odom_samples.append(float(msg.twist.twist.linear.x))

    def _on_state(self, msg: String):
        try:
            self.bridge_state = json.loads(msg.data)
        except Exception:
            pass

    def _publish_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.target_vx
        self.pub.publish(cmd)


def summarize(name, values):
    if not values:
        return {"name": name, "count": 0}
    abs_vals = [abs(v) for v in values]
    return {
        "name": name,
        "count": len(values),
        "mean": statistics.mean(values),
        "mean_abs": statistics.mean(abs_vals),
        "median_abs": statistics.median(abs_vals),
        "max_abs": max(abs_vals),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vx", type=float, default=0.03)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = VxHoldProbe(args.vx, args.duration)
    print(f"Publishing /cmd_vel vx={args.vx} for {args.duration}s ...")
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    stop = Twist()
    node.pub.publish(stop)
    time.sleep(0.2)

    warmup_n = max(1, int(args.warmup / 0.05))
    sent = node.sent_samples[warmup_n:]
    odom = node.odom_samples[warmup_n:]

    report = {
        "target_vx": args.vx,
        "sanitize": {
            "all_ok": (node.bridge_state or {}).get("sanitize_all_ok"),
            "steps": (node.bridge_state or {}).get("sanitize_steps"),
            "max_vx": (node.bridge_state or {}).get("last_raw_vx"),
        },
        "cmd_vel_sent": summarize("cmd_vel_sent", sent),
        "odom_vx": summarize("odom_vx", odom),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
