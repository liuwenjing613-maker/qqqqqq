#!/usr/bin/env python3
"""
Non-destructive arrow calibration probe for RDK X5 VLN robot.

What it DOES:
- Publishes a slow forward /cmd_vel.
- Watches /odom displacement.
- Computes a visualization-only yaw offset so a display arrow can point along
  the measured forward travel direction.
- Saves configs/arrow_calibration.env.

What it DOES NOT do:
- Does not edit m1_pwm_cmd_vel_bridge.py.
- Does not edit run_chassis_bridge.sh.
- Does not change /odom, map->odom, odom->base_link, or base_link->laser.
- Does not change the LiDAR driver or LaserScan angles.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    # ROS yaw from quaternion, assuming planar robot.
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    stamp_sec: float


class ArrowCalibrationProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("arrow_calibration_probe")
        self.args = args
        self.latest_pose: Optional[Pose2D] = None
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 10)
        self.get_logger().info(
            f"probe: odom={args.odom_topic}, cmd={args.cmd_topic}, "
            f"target_distance={args.target_distance:.3f}m, speed={args.speed:.3f}m/s"
        )

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        self.latest_pose = Pose2D(float(p.x), float(p.y), yaw, stamp)

    def publish_cmd(self, vx: float, wz: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def stop_robot(self, repeat: int = 20) -> None:
        for _ in range(repeat):
            self.publish_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_odom(self, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_pose is not None:
                return True
        return False

    def run_probe(self) -> int:
        if self.args.speed <= 0:
            self.get_logger().error("--speed must be positive; positive /cmd_vel.linear.x is the tested forward command.")
            return 2
        if self.args.target_distance <= 0:
            self.get_logger().error("--target-distance must be positive.")
            return 2

        self.get_logger().info("waiting for /odom ...")
        if not self.wait_for_odom(self.args.odom_wait):
            self.get_logger().error("timeout waiting for /odom; calibration aborted.")
            return 3

        self.stop_robot(repeat=10)
        time.sleep(self.args.settle)
        rclpy.spin_once(self, timeout_sec=0.1)
        start = self.latest_pose
        assert start is not None

        self.get_logger().info(
            f"start odom: x={start.x:.4f}, y={start.y:.4f}, yaw={start.yaw:.4f} rad"
        )
        self.get_logger().warn(
            "Robot will move forward slowly. Make sure there is a clear path, then keep hands ready to stop."
        )

        t0 = time.time()
        end = start
        rate_sleep = 1.0 / max(5.0, self.args.rate_hz)
        try:
            while rclpy.ok() and time.time() - t0 < self.args.max_time:
                rclpy.spin_once(self, timeout_sec=0.01)
                if self.latest_pose is not None:
                    end = self.latest_pose
                dx = end.x - start.x
                dy = end.y - start.y
                dist = math.hypot(dx, dy)
                if dist >= self.args.target_distance:
                    break
                self.publish_cmd(self.args.speed, 0.0)
                time.sleep(rate_sleep)
        finally:
            self.stop_robot(repeat=30)
            time.sleep(self.args.settle)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_pose is not None:
                end = self.latest_pose

        dx = end.x - start.x
        dy = end.y - start.y
        dist = math.hypot(dx, dy)
        if dist < self.args.min_valid_distance:
            self.get_logger().error(
                f"movement too small: {dist:.3f}m < {self.args.min_valid_distance:.3f}m. "
                "No calibration file written."
            )
            return 4

        travel_yaw = math.atan2(dy, dx)
        base_yaw = end.yaw
        base_arrow_yaw_offset = wrap_pi(travel_yaw - base_yaw)

        # Visualization-only laser arrow: by default it is drawn at the laser origin
        # and points in the same calibrated physical-forward direction as the base arrow.
        laser_arrow_yaw_offset = base_arrow_yaw_offset + float(self.args.laser_arrow_extra_yaw)
        laser_arrow_yaw_offset = wrap_pi(laser_arrow_yaw_offset)

        self.write_env(
            start=start,
            end=end,
            dx=dx,
            dy=dy,
            dist=dist,
            travel_yaw=travel_yaw,
            base_yaw=base_yaw,
            base_arrow_yaw_offset=base_arrow_yaw_offset,
            laser_arrow_yaw_offset=laser_arrow_yaw_offset,
        )

        self.get_logger().info("calibration result:")
        self.get_logger().info(f"  dx={dx:.4f}, dy={dy:.4f}, distance={dist:.4f} m")
        self.get_logger().info(f"  forward travel yaw in odom = {travel_yaw:.6f} rad ({math.degrees(travel_yaw):.2f} deg)")
        self.get_logger().info(f"  base_link yaw from odom    = {base_yaw:.6f} rad ({math.degrees(base_yaw):.2f} deg)")
        self.get_logger().info(f"  BASE_ARROW_YAW_OFFSET      = {base_arrow_yaw_offset:.6f} rad ({math.degrees(base_arrow_yaw_offset):.2f} deg)")
        self.get_logger().info(f"wrote: {self.args.output}")
        return 0

    def write_env(
        self,
        *,
        start: Pose2D,
        end: Pose2D,
        dx: float,
        dy: float,
        dist: float,
        travel_yaw: float,
        base_yaw: float,
        base_arrow_yaw_offset: float,
        laser_arrow_yaw_offset: float,
    ) -> None:
        out = Path(self.args.output).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(out.suffix + ".tmp")
        now = time.strftime("%Y-%m-%d %H:%M:%S")
        text = f"""# Auto-generated by ros2_bridge/arrow_calibration_probe.py
# Time: {now}
# This file is for visualization-only arrow frames/markers.
# It must not be used to modify odom->base_link or base_link->laser used by SLAM.
ARROW_CALIBRATION_VALID=1
ARROW_CALIBRATION_TIME=\"{now}\"
BASE_ARROW_YAW_OFFSET={base_arrow_yaw_offset:.12f}
LASER_ARROW_YAW_OFFSET={laser_arrow_yaw_offset:.12f}
BASE_FORWARD_ODOM_YAW={travel_yaw:.12f}
BASE_ODOM_END_YAW={base_yaw:.12f}
BASE_TRAVEL_DX={dx:.12f}
BASE_TRAVEL_DY={dy:.12f}
BASE_TRAVEL_DISTANCE={dist:.12f}
BASE_START_X={start.x:.12f}
BASE_START_Y={start.y:.12f}
BASE_START_YAW={start.yaw:.12f}
BASE_END_X={end.x:.12f}
BASE_END_Y={end.y:.12f}
BASE_END_YAW={end.yaw:.12f}
LASER_X={self.args.laser_x:.12f}
LASER_Y={self.args.laser_y:.12f}
LASER_Z={self.args.laser_z:.12f}
"""
        tmp.write_text(text, encoding="utf-8")
        os.replace(tmp, out)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Calibrate visualization arrows from a straight odom motion probe.")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--cmd-topic", default="/cmd_vel")
    p.add_argument("--output", default="/root/rdk_x5_vln_robot/configs/arrow_calibration.env")
    p.add_argument("--target-distance", type=float, default=1.0)
    p.add_argument("--min-valid-distance", type=float, default=0.25)
    p.add_argument("--speed", type=float, default=0.04)
    p.add_argument("--max-time", type=float, default=35.0)
    p.add_argument("--rate-hz", type=float, default=20.0)
    p.add_argument("--odom-wait", type=float, default=20.0)
    p.add_argument("--settle", type=float, default=0.5)
    p.add_argument("--laser-x", type=float, default=0.10)
    p.add_argument("--laser-y", type=float, default=0.0)
    p.add_argument("--laser-z", type=float, default=0.12)
    p.add_argument("--laser-arrow-extra-yaw", type=float, default=0.0)
    return p


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    node = ArrowCalibrationProbe(args)

    def handle_signal(_signum, _frame):
        node.get_logger().warn("signal received; stopping robot")
        node.stop_robot(repeat=30)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        return node.run_probe()
    except KeyboardInterrupt:
        node.stop_robot(repeat=30)
        return 130
    finally:
        node.stop_robot(repeat=10)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
