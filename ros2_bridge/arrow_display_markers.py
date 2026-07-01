#!/usr/bin/env python3
"""
Publish visualization-only calibrated forward arrows for Foxglove/RViz.

It publishes:
- /arrow_calibration_markers: MarkerArray with base and laser forward arrows.
- /tf_static child frames:
    base_forward_calibrated  under base_link
    laser_forward_calibrated under base_link

These frames are deliberately NEW child frames. They do not replace base_link,
laser, odom, or map, so they do not affect SLAM or navigation transforms.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Dict

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def parse_env_file(path: str) -> Dict[str, str]:
    data: Dict[str, str] = {}
    p = Path(path).expanduser()
    if not p.exists():
        return data
    for raw in p.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key.strip()] = value.strip().strip('"')
    return data


def get_float(data: Dict[str, str], key: str, default: float) -> float:
    try:
        return float(data.get(key, default))
    except Exception:
        return float(default)


def yaw_to_quat(yaw: float):
    # x,y,z,w tuple.
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class ArrowDisplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("arrow_display_markers")
        self.args = args
        self.env = parse_env_file(args.config)
        self.base_yaw = get_float(self.env, "BASE_ARROW_YAW_OFFSET", args.base_yaw_offset)
        self.laser_yaw = get_float(self.env, "LASER_ARROW_YAW_OFFSET", args.laser_yaw_offset)
        self.laser_x = get_float(self.env, "LASER_X", args.laser_x)
        self.laser_y = get_float(self.env, "LASER_Y", args.laser_y)
        self.laser_z = get_float(self.env, "LASER_Z", args.laser_z)

        self.pub = self.create_publisher(MarkerArray, args.marker_topic, 10)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static_frames()
        self.timer = self.create_timer(1.0 / max(1.0, args.rate_hz), self.publish_markers)
        self.get_logger().info(
            f"display arrows from {args.config}: base_offset={self.base_yaw:.6f} rad, "
            f"laser_offset={self.laser_yaw:.6f} rad"
        )
        self.get_logger().info(
            "Foxglove: add /arrow_calibration_markers, or show TF frames "
            "base_forward_calibrated and laser_forward_calibrated."
        )

    def make_tf(self, parent: str, child: str, x: float, y: float, z: float, yaw: float) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t

    def publish_static_frames(self) -> None:
        frames = [
            self.make_tf(self.args.base_frame, "base_forward_calibrated", 0.0, 0.0, self.args.base_z, self.base_yaw),
            self.make_tf(self.args.base_frame, "laser_forward_calibrated", self.laser_x, self.laser_y, self.laser_z, self.laser_yaw),
        ]
        self.static_tf.sendTransform(frames)

    def arrow_marker(
        self,
        marker_id: int,
        frame_id: str,
        ns: str,
        start_x: float,
        start_y: float,
        start_z: float,
        yaw: float,
        length: float,
        diameter: float,
        head_diameter: float,
        r: float,
        g: float,
        b: float,
        text: str | None = None,
    ) -> Marker:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame_id
        m.ns = ns
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.points = [
            Point(x=float(start_x), y=float(start_y), z=float(start_z)),
            Point(
                x=float(start_x + length * math.cos(yaw)),
                y=float(start_y + length * math.sin(yaw)),
                z=float(start_z),
            ),
        ]
        # For ARROW with points: scale.x shaft diameter, scale.y head diameter, scale.z head length.
        m.scale.x = float(diameter)
        m.scale.y = float(head_diameter)
        m.scale.z = float(head_diameter * 1.5)
        m.color.r = float(r)
        m.color.g = float(g)
        m.color.b = float(b)
        m.color.a = 1.0
        m.lifetime = Duration(sec=1, nanosec=0)
        return m

    def text_marker(self, marker_id: int, frame_id: str, ns: str, x: float, y: float, z: float, text: str) -> Marker:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame_id
        m.ns = ns
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.z = 0.07
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.text = text
        m.lifetime = Duration(sec=1, nanosec=0)
        return m

    def publish_markers(self) -> None:
        arr = MarkerArray()
        # Green: calibrated chassis physical-forward display arrow, drawn in base_link.
        arr.markers.append(
            self.arrow_marker(
                1,
                self.args.base_frame,
                "calibrated_base_forward",
                0.0,
                0.0,
                self.args.base_z,
                self.base_yaw,
                self.args.base_arrow_length,
                0.025,
                0.07,
                0.1,
                1.0,
                0.1,
            )
        )
        arr.markers.append(
            self.text_marker(
                2,
                self.args.base_frame,
                "calibrated_base_forward",
                self.args.base_arrow_length * math.cos(self.base_yaw),
                self.args.base_arrow_length * math.sin(self.base_yaw),
                self.args.base_z + 0.08,
                "base forward calibrated",
            )
        )
        # Blue: calibrated LiDAR physical-forward display arrow, drawn at laser mounting origin in base_link.
        arr.markers.append(
            self.arrow_marker(
                3,
                self.args.base_frame,
                "calibrated_laser_forward",
                self.laser_x,
                self.laser_y,
                self.laser_z + 0.02,
                self.laser_yaw,
                self.args.laser_arrow_length,
                0.018,
                0.055,
                0.1,
                0.4,
                1.0,
            )
        )
        arr.markers.append(
            self.text_marker(
                4,
                self.args.base_frame,
                "calibrated_laser_forward",
                self.laser_x + self.args.laser_arrow_length * math.cos(self.laser_yaw),
                self.laser_y + self.args.laser_arrow_length * math.sin(self.laser_yaw),
                self.laser_z + 0.10,
                "laser forward calibrated",
            )
        )
        # Red: raw laser frame +X axis. This is diagnostic only and uses the actual laser frame.
        arr.markers.append(
            self.arrow_marker(
                5,
                self.args.laser_frame,
                "raw_laser_x_axis",
                0.0,
                0.0,
                0.03,
                0.0,
                self.args.raw_laser_arrow_length,
                0.012,
                0.040,
                1.0,
                0.1,
                0.1,
            )
        )
        arr.markers.append(
            self.text_marker(6, self.args.laser_frame, "raw_laser_x_axis", self.args.raw_laser_arrow_length, 0.0, 0.10, "raw laser +X"))
        self.pub.publish(arr)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Publish non-destructive calibrated arrow markers/frames.")
    p.add_argument("--config", default="/root/rdk_x5_vln_robot/configs/arrow_calibration.env")
    p.add_argument("--marker-topic", default="/arrow_calibration_markers")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--laser-frame", default="laser")
    p.add_argument("--rate-hz", type=float, default=5.0)
    p.add_argument("--base-yaw-offset", type=float, default=0.0)
    p.add_argument("--laser-yaw-offset", type=float, default=0.0)
    p.add_argument("--laser-x", type=float, default=0.10)
    p.add_argument("--laser-y", type=float, default=0.0)
    p.add_argument("--laser-z", type=float, default=0.12)
    p.add_argument("--base-z", type=float, default=0.10)
    p.add_argument("--base-arrow-length", type=float, default=0.45)
    p.add_argument("--laser-arrow-length", type=float, default=0.32)
    p.add_argument("--raw-laser-arrow-length", type=float, default=0.25)
    return p


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    node = ArrowDisplay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
