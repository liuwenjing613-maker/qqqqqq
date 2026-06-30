#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_q(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Check(Node):
    def __init__(self):
        super().__init__("nav_frame_forward_check")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(Odometry, "/odom", self.cb, 10)
        self.x = None
        self.y = None
        self.yaw = None
        self.vx = None

    def cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_q(msg.pose.pose.orientation)
        self.vx = msg.twist.twist.linear.x

    def stop(self):
        msg = Twist()
        for _ in range(20):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    node = Check()

    print("[INFO] waiting /odom...")
    t0 = time.time()
    while node.x is None and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.x is None:
        print("[ERROR] no /odom received in 5s")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    x0 = node.x
    y0 = node.y
    yaw0 = node.yaw

    print(
        f"[START] x={x0:.4f}, y={y0:.4f}, "
        f"yaw={yaw0:.4f} rad, yaw_deg={math.degrees(yaw0):.1f}"
    )

    msg = Twist()
    msg.linear.x = 0.03

    print("[MOVE] publishing /cmd_vel linear.x=+0.03 for 2.0s")
    start = time.time()
    while time.time() - start < 2.0:
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)

    node.stop()

    time.sleep(0.2)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.02)

    dx = node.x - x0
    dy = node.y - y0

    body_forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    body_left = -dx * math.sin(yaw0) + dy * math.cos(yaw0)

    print(
        f"[END] x={node.x:.4f}, y={node.y:.4f}, "
        f"yaw={node.yaw:.4f} rad, yaw_deg={math.degrees(node.yaw):.1f}"
    )
    print(f"[DELTA] dx={dx:.4f}, dy={dy:.4f}")
    print(f"[BODY] forward={body_forward:.4f}, left={body_left:.4f}")
    print(f"[TWIST] odom.twist.twist.linear.x={node.vx}")

    if body_forward > 0 and (node.vx is None or node.vx > 0):
        print("[OK] NAV frame is consistent: forward command -> +base_link X -> positive odom twist.")
    elif body_forward > 0 and node.vx is not None and node.vx < 0:
        print("[WARN] pose motion is forward, but odom twist vx is negative. Nav may be unstable.")
    else:
        print("[BAD] pose motion is opposite to base_link +X. Need flip VX sign or XY yaw offset.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
