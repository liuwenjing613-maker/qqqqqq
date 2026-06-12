#!/usr/bin/env python3
import sys
import time
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP = """
键盘控制 /cmd_vel：

w：前进
s：后退
a：左转
d：右转
x：停止
q：退出

注意：
当前阶段禁用横移，不使用 A/D 横移。
a/d 是原地旋转，不是平移。
"""


def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class KeyboardCmdVel(Node):
    def __init__(self):
        super().__init__("keyboard_cmd_vel")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.vx_step = 0.04
        self.wz_step = 0.25

        self.get_logger().info("keyboard_cmd_vel started.")
        print(HELP)

    def publish_cmd(self, vx=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def loop(self):
        while rclpy.ok():
            key = get_key(timeout=0.1)

            if key == "w":
                print("forward")
                self.publish_cmd(self.vx_step, 0.0)
            elif key == "s":
                print("backward")
                self.publish_cmd(-self.vx_step, 0.0)
            elif key == "a":
                print("rotate left")
                self.publish_cmd(0.0, self.wz_step)
            elif key == "d":
                print("rotate right")
                self.publish_cmd(0.0, -self.wz_step)
            elif key == "x":
                print("stop")
                self.stop()
            elif key == "q":
                print("quit")
                self.stop()
                break
            else:
                # 没有按键时不持续发命令，让 watchdog 自动停车
                pass


def main():
    rclpy.init()
    node = KeyboardCmdVel()

    try:
        node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
