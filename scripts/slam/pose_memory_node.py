#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Map-frame pose memory for SLAM mapping and saved-map Nav2 navigation.

- Periodically saves map->base_link (or base_footprint) TF to JSON.
- Optionally publishes saved pose to /initialpose once at startup.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

INITIAL_COVARIANCE = [0.0] * 36
INITIAL_COVARIANCE[0] = 0.25
INITIAL_COVARIANCE[7] = 0.25
INITIAL_COVARIANCE[35] = 0.0685


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def load_pose_json(path: Path) -> Optional[Dict[str, Any]]:
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text())
        if not isinstance(data, dict):
            return None
        return data
    except (json.JSONDecodeError, OSError):
        return None


def save_pose_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(path.name + '.tmp')
    tmp_path.write_text(json.dumps(data, indent=2) + '\n')
    os.replace(str(tmp_path), str(path))


def pose_msg_from_json(data: Dict[str, Any]) -> PoseWithCovarianceStamped:
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = float(data.get('x', 0.0))
    msg.pose.pose.position.y = float(data.get('y', 0.0))
    msg.pose.pose.position.z = float(data.get('z', 0.0))
    msg.pose.pose.orientation.x = float(data.get('qx', 0.0))
    msg.pose.pose.orientation.y = float(data.get('qy', 0.0))
    msg.pose.pose.orientation.z = float(data.get('qz', 0.0))
    msg.pose.pose.orientation.w = float(data.get('qw', 1.0))
    msg.pose.covariance = list(INITIAL_COVARIANCE)
    return msg


def pose_dict_from_transform(map_frame: str, child_frame: str, tf: TransformStamped) -> Dict[str, Any]:
    t = tf.transform.translation
    r = tf.transform.rotation
    stamp = tf.header.stamp
    stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return {
        'frame_id': map_frame,
        'child_frame_id': child_frame,
        'x': t.x,
        'y': t.y,
        'z': t.z,
        'qx': r.x,
        'qy': r.y,
        'qz': r.z,
        'qw': r.w,
        'yaw': yaw_from_quaternion(r.x, r.y, r.z, r.w),
        'stamp': stamp_sec,
        'source': 'tf',
    }


class PoseMemoryNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('pose_memory_node')
        self.state_file = Path(args.state_file)
        self.map_frame = args.map_frame
        self.base_frame = args.base_frame
        self.fallback_base_frame = args.fallback_base_frame
        self.initial_repeat = args.initial_repeat
        self.initial_interval = args.initial_interval

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)

        self._loaded_pose_msg: Optional[PoseWithCovarianceStamped] = None
        self._initial_remaining = 0
        self._initial_timer = None

        if args.publish_initial:
            pose_data = load_pose_json(self.state_file)
            if pose_data is None:
                self.get_logger().warn(
                    f'--publish-initial set but state file not found: {self.state_file}. '
                    'Will continue and save pose once TF is available.'
                )
            else:
                self.get_logger().info(f'Loaded last pose from {self.state_file}')
                self._loaded_pose_msg = pose_msg_from_json(pose_data)
                self._initial_remaining = self.initial_repeat
                self._initial_timer = self.create_timer(self.initial_interval, self._publish_initial_tick)

        self.create_timer(args.save_period, self._save_tick)

    def _lookup_transform(self) -> Optional[Tuple[str, TransformStamped]]:
        for child_frame in (self.base_frame, self.fallback_base_frame):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    child_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
                return child_frame, tf
            except TransformException:
                continue
        return None

    def _save_tick(self) -> None:
        result = self._lookup_transform()
        if result is None:
            return

        child_frame, tf = result
        data = pose_dict_from_transform(self.map_frame, child_frame, tf)
        try:
            save_pose_json(self.state_file, data)
        except OSError as exc:
            self.get_logger().warn(f'Failed to save pose to {self.state_file}: {exc}')

    def _publish_initial_tick(self) -> None:
        if self._initial_remaining <= 0 or self._loaded_pose_msg is None:
            if self._initial_timer is not None:
                self._initial_timer.cancel()
            return

        self._loaded_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.initial_pub.publish(self._loaded_pose_msg)
        sent = self.initial_repeat - self._initial_remaining + 1
        self.get_logger().info(
            f'Published initial pose to /initialpose ({sent}/{self.initial_repeat})'
        )
        self._initial_remaining -= 1
        if self._initial_remaining <= 0 and self._initial_timer is not None:
            self._initial_timer.cancel()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Save map-frame robot pose and optionally publish /initialpose.')
    parser.add_argument(
        '--state-file',
        default='/root/rdk_x5_vln_robot/state/last_pose_map.json',
        help='JSON file for last known map-frame pose.',
    )
    parser.add_argument('--map-frame', default='map', help='Parent TF frame.')
    parser.add_argument('--base-frame', default='base_link', help='Primary child TF frame.')
    parser.add_argument('--fallback-base-frame', default='base_footprint', help='Fallback child TF frame.')
    parser.add_argument('--save-period', type=float, default=1.0, help='Seconds between TF save attempts.')
    parser.add_argument(
        '--publish-initial',
        action='store_true',
        help='Publish saved pose to /initialpose once at startup.',
    )
    parser.add_argument('--initial-repeat', type=int, default=5, help='Number of /initialpose publishes at startup.')
    parser.add_argument('--initial-interval', type=float, default=0.3, help='Seconds between startup /initialpose publishes.')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = PoseMemoryNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
