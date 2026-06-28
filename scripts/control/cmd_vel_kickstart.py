#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def sgn(x: float) -> float:
    if x > 0:
        return 1.0
    if x < 0:
        return -1.0
    return 0.0


class CmdVelKickstart(Node):
    def __init__(self, args):
        super().__init__("cmd_vel_kickstart")

        self.in_topic = args.in_topic
        self.out_topic = args.out_topic

        self.deadband_vx = args.deadband_vx
        self.deadband_wz = args.deadband_wz

        self.kick_vx = args.kick_vx
        self.kick_wz = args.kick_wz
        self.kick_duration = args.kick_duration

        self.max_vx = args.max_vx
        self.max_wz = args.max_wz

        self.rate_hz = args.rate_hz
        self.input_timeout = args.input_timeout

        self.target = Twist()
        self.last_input_time = 0.0

        self.was_idle = True
        self.kick_until = 0.0
        self.kick_sign_vx = 0.0
        self.kick_sign_wz = 0.0

        self.sub = self.create_subscription(
            Twist,
            self.in_topic,
            self.on_cmd,
            10,
        )
        self.pub = self.create_publisher(Twist, self.out_topic, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(
            f"cmd_vel kickstart: {self.in_topic} -> {self.out_topic}, "
            f"kick_vx={self.kick_vx}, kick_wz={self.kick_wz}, "
            f"kick_duration={self.kick_duration}s"
        )

    def is_zero_cmd(self, msg: Twist) -> bool:
        return (
            abs(msg.linear.x) < self.deadband_vx
            and abs(msg.angular.z) < self.deadband_wz
        )

    def clamp_cmd(self, msg: Twist) -> Twist:
        out = Twist()
        out.linear.x = max(-self.max_vx, min(self.max_vx, msg.linear.x))
        out.angular.z = max(-self.max_wz, min(self.max_wz, msg.angular.z))
        return out

    def on_cmd(self, msg: Twist):
        now = time.time()
        self.last_input_time = now

        self.target = self.clamp_cmd(msg)

        if self.is_zero_cmd(self.target):
            self.was_idle = True
            self.kick_until = 0.0
            return

        if self.was_idle:
            self.was_idle = False
            self.kick_until = now + self.kick_duration
            self.kick_sign_vx = sgn(self.target.linear.x)
            self.kick_sign_wz = sgn(self.target.angular.z)

            self.get_logger().info(
                f"kickstart triggered: "
                f"target_vx={self.target.linear.x:.3f}, "
                f"target_wz={self.target.angular.z:.3f}"
            )

    def zero(self) -> Twist:
        return Twist()

    def on_timer(self):
        now = time.time()

        if now - self.last_input_time > self.input_timeout:
            self.was_idle = True
            self.kick_until = 0.0
            self.pub.publish(self.zero())
            return

        if self.is_zero_cmd(self.target):
            self.pub.publish(self.zero())
            return

        out = Twist()

        if now < self.kick_until:
            if abs(self.target.linear.x) >= self.deadband_vx:
                out.linear.x = self.kick_sign_vx * max(abs(self.target.linear.x), self.kick_vx)
            else:
                out.linear.x = 0.0

            if abs(self.target.angular.z) >= self.deadband_wz:
                out.angular.z = self.kick_sign_wz * max(abs(self.target.angular.z), self.kick_wz)
            else:
                out.angular.z = 0.0
        else:
            out.linear.x = self.target.linear.x
            out.angular.z = self.target.angular.z

        out = self.clamp_cmd(out)
        self.pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-topic", default="/cmd_vel_joy")
    parser.add_argument("--out-topic", default="/cmd_vel")

    parser.add_argument("--deadband-vx", type=float, default=0.004)
    parser.add_argument("--deadband-wz", type=float, default=0.015)

    parser.add_argument("--kick-vx", type=float, default=0.060)
    parser.add_argument("--kick-wz", type=float, default=0.220)
    parser.add_argument("--kick-duration", type=float, default=0.22)

    parser.add_argument("--max-vx", type=float, default=0.080)
    parser.add_argument("--max-wz", type=float, default=0.350)

    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--input-timeout", type=float, default=0.35)

    args = parser.parse_args()

    rclpy.init()
    node = CmdVelKickstart(args)

    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
