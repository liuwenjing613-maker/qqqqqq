#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelSequenceTester(Node):
    def __init__(self):
        super().__init__("cmd_vel_sequence_tester")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        time.sleep(0.5)

    def publish_cmd(self, vx=0.0, wz=0.0, duration=1.0, name=""):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)

        self.get_logger().info(f"{name}: vx={vx:.3f}, wz={wz:.3f}, duration={duration:.1f}s")

        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(msg)
            time.sleep(0.1)

        self.stop()
        time.sleep(0.8)

    def stop(self):
        msg = Twist()
        self.pub.publish(msg)
        self.get_logger().info("STOP")


def main():
    rclpy.init()
    node = CmdVelSequenceTester()

    input("确认小车轮子架空后，按 Enter 开始 /cmd_vel 自动测试：")

    try:
        node.publish_cmd(vx=0.04, wz=0.0, duration=1.0, name="1. forward")
        node.publish_cmd(vx=-0.04, wz=0.0, duration=1.0, name="2. backward")
        node.publish_cmd(vx=0.0, wz=0.20, duration=1.0, name="3. rotate left")
        node.publish_cmd(vx=0.0, wz=-0.20, duration=1.0, name="4. rotate right")
        node.publish_cmd(vx=0.04, wz=0.15, duration=1.0, name="5. forward + left")
        node.publish_cmd(vx=0.04, wz=-0.15, duration=1.0, name="6. forward + right")

        node.get_logger().info("All tests done.")

    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted.")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
