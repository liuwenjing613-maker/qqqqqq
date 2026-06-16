#!/usr/bin/env python3
"""
经 ROS /cmd_vel + cmd_vel_to_rosmaster 测试前进/左转/右转。
与 MVP 实际走的路径一致，用于排查底盘桥与平滑参数。

需先启动 cmd_vel_to_rosmaster（本脚本不自动启动）。
"""

import argparse
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.config.mvp_tune import DEFAULT_TUNE_PATH, load_mvp_tune


class CmdVelGroundTester(Node):
    def __init__(self, cmd_topic="/cmd_vel"):
        super().__init__("cmd_vel_ground_tester")
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        time.sleep(0.5)

    def hold(self, vx, wz, duration, name):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.get_logger().info(f"{name}: vx={vx:+.3f} wz={wz:+.3f} duration={duration:.1f}s")
        end = time.time() + duration
        while time.time() < end:
            self.pub.publish(msg)
            time.sleep(0.05)
        self.stop()

    def stop(self):
        self.pub.publish(Twist())
        self.get_logger().info("STOP")


def main():
    tune = load_mvp_tune(DEFAULT_TUNE_PATH)
    parser = argparse.ArgumentParser(description="经 /cmd_vel 测试前进/左转/右转")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--vx", type=float, default=tune["max_vx"])
    parser.add_argument("--wz", type=float, default=tune["recovery_scan_wz"])
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument(
        "--action",
        choices=["forward", "left", "right", "all"],
        default="all",
    )
    parser.add_argument("--yes", action="store_true")
    args = parser.parse_args()

    print("=== /cmd_vel 落地测试 ===")
    print(f"vx={args.vx} wz(left)={args.wz} duration={args.duration}")
    print("请先运行: bash scripts/test_chassis_ground_ros.sh bridge")
    print("或手动启动 cmd_vel_to_rosmaster.py")
    if not args.yes:
        input("确认底盘桥已启动、周围安全，按 Enter：")

    rclpy.init()
    node = CmdVelGroundTester(args.cmd_topic)
    tests = []
    if args.action in ("forward", "all"):
        tests.append(("前进", args.vx, 0.0))
    if args.action in ("left", "all"):
        tests.append(("左转", 0.0, args.wz))
    if args.action in ("right", "all"):
        tests.append(("右转", 0.0, -args.wz))

    try:
        for label, vx, wz in tests:
            if args.action == "all" and not args.yes:
                input(f"准备 {label}，按 Enter：")
            node.hold(vx, wz, args.duration, label)
            time.sleep(0.5)
        node.get_logger().info("全部完成")
    except KeyboardInterrupt:
        node.get_logger().warn("中断")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
