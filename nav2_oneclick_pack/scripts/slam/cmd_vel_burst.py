#!/usr/bin/env python3
"""Short /cmd_vel burst tester for RDK X5 ROSMASTER M1.
Usage:
  python3 scripts/slam/cmd_vel_burst.py 0.15 0.0 2.0
  python3 scripts/slam/cmd_vel_burst.py 0.0 0.6 1.5
"""
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main() -> None:
    vx = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
    wz = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    hz = float(sys.argv[4]) if len(sys.argv) > 4 else 10.0

    rclpy.init()
    node = Node("cmd_vel_burst_tester")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    print(f"[CMD_TEST] vx={vx}, wz={wz}, duration={duration}s, hz={hz}")
    deadline = time.monotonic() + 3.0
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        print("[CMD_TEST] waiting for /cmd_vel subscriber...")
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f"[CMD_TEST] /cmd_vel subscriber_count={pub.get_subscription_count()}")
    msg = Twist()
    msg.linear.x = vx
    msg.angular.z = wz

    period = 1.0 / hz
    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)

    stop = Twist()
    for _ in range(10):
        pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.05)

    print("[CMD_TEST] stop sent")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
