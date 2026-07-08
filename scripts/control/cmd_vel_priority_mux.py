#!/usr/bin/env python3
"""Merge autonomy and joystick cmd_vel; joystick has highest priority."""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class CmdVelPriorityMux(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("cmd_vel_priority_mux")
        self.axis_linear = int(args.axis_linear)
        self.axis_angular = int(args.axis_angular)
        self.joy_deadzone = float(args.joy_deadzone)
        self.release_hold_sec = float(args.release_hold_sec)

        self.autonomy_cmd = Twist()
        self.joy_cmd = Twist()
        self.joy_axes: list[float] = []
        self.last_joy_active_at = 0.0

        self.create_subscription(Twist, args.autonomy_topic, self._on_autonomy, 10)
        self.create_subscription(Twist, args.joy_cmd_topic, self._on_joy_cmd, 10)
        self.create_subscription(Joy, args.joy_topic, self._on_joy, 10)
        self.pub = self.create_publisher(Twist, args.output_topic, 10)
        self.create_timer(1.0 / max(5.0, float(args.rate_hz)), self._publish)

        self.get_logger().info(
            "cmd_vel mux: autonomy=%s joy_cmd=%s output=%s joy=%s"
            % (args.autonomy_topic, args.joy_cmd_topic, args.output_topic, args.joy_topic)
        )

    def _axes_active(self) -> bool:
        if not self.joy_axes:
            return False
        max_idx = max(self.axis_linear, self.axis_angular)
        if len(self.joy_axes) <= max_idx:
            return False
        lin = abs(float(self.joy_axes[self.axis_linear]))
        ang = abs(float(self.joy_axes[self.axis_angular]))
        return lin > self.joy_deadzone or ang > self.joy_deadzone

    def _cmd_active(self, msg: Twist) -> bool:
        return (
            abs(float(msg.linear.x)) > self.joy_deadzone
            or abs(float(msg.angular.z)) > self.joy_deadzone
        )

    def _joy_active(self) -> bool:
        if self._axes_active() or self._cmd_active(self.joy_cmd):
            return True
        return (time.time() - self.last_joy_active_at) < self.release_hold_sec

    def _mark_joy_active(self) -> None:
        self.last_joy_active_at = time.time()

    def _on_autonomy(self, msg: Twist) -> None:
        self.autonomy_cmd = msg

    def _on_joy_cmd(self, msg: Twist) -> None:
        self.joy_cmd = msg
        if self._cmd_active(msg) or self._axes_active():
            self._mark_joy_active()

    def _on_joy(self, msg: Joy) -> None:
        self.joy_axes = [float(v) for v in msg.axes]
        if self._axes_active():
            self._mark_joy_active()

    def _publish(self) -> None:
        out = self.joy_cmd if self._joy_active() else self.autonomy_cmd
        self.pub.publish(out)


def main() -> None:
    parser = argparse.ArgumentParser(description="cmd_vel priority mux (joy > autonomy)")
    parser.add_argument("--autonomy-topic", default="/cmd_vel_autonomy")
    parser.add_argument("--joy-cmd-topic", default="/cmd_vel_joy")
    parser.add_argument("--output-topic", default="/cmd_vel")
    parser.add_argument("--joy-topic", default="/joy")
    parser.add_argument("--axis-linear", type=int, default=1)
    parser.add_argument("--axis-angular", type=int, default=0)
    parser.add_argument("--joy-deadzone", type=float, default=0.08)
    parser.add_argument("--release-hold-sec", type=float, default=0.20)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelPriorityMux(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
