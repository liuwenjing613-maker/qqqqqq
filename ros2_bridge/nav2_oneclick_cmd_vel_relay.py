#!/usr/bin/env python3
"""One-click Nav2 only: invert vx/wz from a raw cmd_vel topic before chassis."""

from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Nav2OneclickCmdVelRelay(Node):
    def __init__(self, in_topic: str, out_topic: str) -> None:
        super().__init__("nav2_oneclick_cmd_vel_relay")
        self._pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._on_cmd, 10)
        self.get_logger().info(
            f"cmd_vel invert relay: {in_topic} -> {out_topic} (linear.x, angular.z negated)"
        )

    def _on_cmd(self, msg: Twist) -> None:
        out = Twist()
        out.linear.x = -float(msg.linear.x)
        out.linear.y = float(msg.linear.y)
        out.linear.z = float(msg.linear.z)
        out.angular.x = float(msg.angular.x)
        out.angular.y = float(msg.angular.y)
        out.angular.z = -float(msg.angular.z)
        self._pub.publish(out)


def main() -> None:
    parser = argparse.ArgumentParser(description="Invert Nav2 cmd_vel for oneclick navigation only")
    parser.add_argument("--in-topic", default="/nav2_oneclick/raw_cmd_vel")
    parser.add_argument("--out-topic", default="/cmd_vel")
    args = parser.parse_args()

    rclpy.init()
    node = Nav2OneclickCmdVelRelay(args.in_topic, args.out_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
