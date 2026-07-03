#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Listen for Foxglove 3D panel mouse clicks published to /foxglove_goal_point.

Use this to verify Foxglove Publish -> 2D point -> /foxglove_goal_point works.
Does NOT start navigation; only prints received click coordinates.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class FoxgloveClickDetectNode(Node):
    def __init__(self, point_topic: str, pose_topic: str) -> None:
        super().__init__('foxglove_mouse_click_detect')
        self.point_topic = point_topic
        self.pose_topic = pose_topic
        self.click_count = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PointStamped, point_topic, self._on_point, qos)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Foxglove mouse click detector READY')
        self.get_logger().info(f'Listening point topic: {point_topic}')
        self.get_logger().info(f'Listening pose topic:  {pose_topic} (compat)')
        self.get_logger().info('Foxglove setup:')
        self.get_logger().info('  3D panel -> Publish tool -> 2D point')
        self.get_logger().info(f'  Topic must be exactly: {point_topic}')
        self.get_logger().info('  Fixed frame: map')
        self.get_logger().info('Click on the map now. Coordinates will print below.')
        self.get_logger().info('=' * 60)

    def _on_point(self, msg: PointStamped) -> None:
        self.click_count += 1
        frame = msg.header.frame_id or '(empty)'
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        stamp = msg.header.stamp
        line = (
            f'[CLICK #{self.click_count}] POINT frame={frame} '
            f'x={x:.4f} y={y:.4f} z={z:.4f} '
            f'stamp={stamp.sec}.{stamp.nanosec:09d}'
        )
        print(line, flush=True)
        self.get_logger().info(line)

    def _on_pose(self, msg: PoseStamped) -> None:
        self.click_count += 1
        frame = msg.header.frame_id or '(empty)'
        x = msg.pose.position.x
        y = msg.pose.position.y
        line = (
            f'[CLICK #{self.click_count}] POSE  frame={frame} '
            f'x={x:.4f} y={y:.4f} (legacy / pose drag mode)'
        )
        print(line, flush=True)
        self.get_logger().info(line)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Detect Foxglove map mouse clicks on /foxglove_goal_point.')
    parser.add_argument('--point-topic', default='/foxglove_goal_point')
    parser.add_argument('--pose-topic', default='/foxglove_goal_pose')
    parser.add_argument('--timeout-sec', type=float, default=0.0, help='Exit after N seconds; 0 = run forever.')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = FoxgloveClickDetectNode(args.point_topic, args.pose_topic)
    start = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.timeout_sec > 0 and (time.time() - start) >= args.timeout_sec:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if node.click_count == 0:
            msg = 'NO CLICK detected. Check Foxglove topic /foxglove_goal_point and Publish->2D point mode.'
            print(msg, flush=True)
            node.get_logger().warn(msg)
        else:
            msg = f'Total clicks detected: {node.click_count}'
            print(msg, flush=True)
            node.get_logger().info(msg)
        node.destroy_node()
        rclpy.shutdown()
        if node.click_count == 0 and args.timeout_sec > 0:
            sys.exit(1)


if __name__ == '__main__':
    main()
