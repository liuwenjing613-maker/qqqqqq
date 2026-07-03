#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sniff ALL common Foxglove click topics to find where clicks actually go."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

TOPICS = [
    ('/foxglove_goal_point', PointStamped, 'POINT'),
    ('/foxglove_goal_pose', PoseStamped, 'POSE'),
    ('/clicked_point', PointStamped, 'POINT'),
    ('/move_base_simple/goal', PoseStamped, 'POSE'),
    ('/goal_pose', PoseStamped, 'POSE'),
    ('/initialpose', PoseWithCovarianceStamped, 'INIT'),
]


class ClickSniffer(Node):
    def __init__(self) -> None:
        super().__init__('foxglove_click_sniffer')
        self.count = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        for topic, msg_type, kind in TOPICS:
            self.create_subscription(
                msg_type,
                topic,
                lambda m, t=topic, k=kind: self._cb(m, t, k),
                qos if 'foxglove' in topic else volatile_qos,
            )
        self.get_logger().info('Sniffing click topics: ' + ', '.join(t for t, _, _ in TOPICS))
        print('=' * 60, flush=True)
        print('CLICK SNIFFER READY - click map in Foxglove now', flush=True)
        print('Must use: 3D panel toolbar -> Publish icon -> 2D point', flush=True)
        print('=' * 60, flush=True)

    def _cb(self, msg, topic: str, kind: str) -> None:
        self.count += 1
        if kind == 'POINT':
            x, y = msg.point.x, msg.point.y
        elif kind == 'INIT':
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
        else:
            x = msg.pose.position.x
            y = msg.pose.position.y
        frame = msg.header.frame_id or '(empty)'
        line = f'[SNIFF #{self.count}] topic={topic} kind={kind} frame={frame} x={x:.4f} y={y:.4f}'
        print(line, flush=True)
        self.get_logger().info(line)


def main() -> None:
    rclpy.init()
    node = ClickSniffer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f'Total messages: {node.count}', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
