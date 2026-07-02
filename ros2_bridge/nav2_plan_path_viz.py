#!/usr/bin/env python3
"""Visualize Nav2 global/local plans in Foxglove via MarkerArray + Path republish."""

from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _path_to_line_marker(path: Path, marker_id: int, ns: str, color: ColorRGBA, width: float) -> Marker:
    marker = Marker()
    marker.header = path.header
    if not marker.header.frame_id:
        marker.header.frame_id = "map"
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    marker.color = color
    for pose in path.poses:
        pt = Point()
        pt.x = pose.pose.position.x
        pt.y = pose.pose.position.y
        pt.z = 0.06
        marker.points.append(pt)
    return marker


def _goal_marker(goal: PoseStamped, marker_id: int) -> Marker:
    marker = Marker()
    marker.header = goal.header
    if not marker.header.frame_id:
        marker.header.frame_id = "map"
    marker.ns = "nav2_goal"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = goal.pose
    marker.pose.position.z = 0.08
    marker.scale.x = 0.14
    marker.scale.y = 0.14
    marker.scale.z = 0.14
    marker.color = _color(1.0, 0.2, 0.2, 0.9)
    return marker


class Nav2PlanPathViz(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("nav2_plan_path_viz")
        self.args = args
        self._global_path: Path | None = None
        self._local_path: Path | None = None
        self._goal_pose: PoseStamped | None = None

        self._marker_pub = self.create_publisher(MarkerArray, args.marker_topic, 10)
        self._global_pub = self.create_publisher(Path, args.global_topic, 10)
        self._local_pub = self.create_publisher(Path, args.local_topic, 10)

        self.create_subscription(Path, args.global_in, self._on_global, 10)
        self.create_subscription(Path, args.local_in, self._on_local, 10)
        self.create_subscription(PoseStamped, args.goal_in, self._on_goal, 10)

        period = 1.0 / max(1.0, float(args.rate_hz))
        self.create_timer(period, self._publish)
        self.get_logger().info(
            f"plan viz: {args.global_in}->{args.global_topic}, "
            f"{args.local_in}->{args.local_topic}, markers={args.marker_topic}"
        )

    def _on_global(self, msg: Path) -> None:
        self._global_path = msg

    def _on_local(self, msg: Path) -> None:
        self._local_path = msg

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal_pose = msg

    def _publish(self) -> None:
        markers = MarkerArray()

        if self._global_path is not None and self._global_path.poses:
            self._global_pub.publish(self._global_path)
            markers.markers.append(
                _path_to_line_marker(
                    self._global_path,
                    1,
                    "global_plan",
                    _color(0.1, 1.0, 0.35, 0.95),
                    self.args.global_width,
                )
            )
        else:
            clear_global = Marker()
            clear_global.action = Marker.DELETE
            clear_global.ns = "global_plan"
            clear_global.id = 1
            markers.markers.append(clear_global)

        if self._local_path is not None and self._local_path.poses:
            self._local_pub.publish(self._local_path)
            markers.markers.append(
                _path_to_line_marker(
                    self._local_path,
                    2,
                    "local_plan",
                    _color(0.2, 0.55, 1.0, 0.95),
                    self.args.local_width,
                )
            )
        else:
            clear_local = Marker()
            clear_local.action = Marker.DELETE
            clear_local.ns = "local_plan"
            clear_local.id = 2
            markers.markers.append(clear_local)

        if self._goal_pose is not None:
            markers.markers.append(_goal_marker(self._goal_pose, 3))

        if markers.markers:
            self._marker_pub.publish(markers)


def main() -> None:
    parser = argparse.ArgumentParser(description="Nav2 plan path Foxglove visualization")
    parser.add_argument("--global-in", default="/plan")
    parser.add_argument("--local-in", default="/local_plan")
    parser.add_argument("--goal-in", default="/goal_pose")
    parser.add_argument("--global-topic", default="/nav2_viz/global_plan")
    parser.add_argument("--local-topic", default="/nav2_viz/local_plan")
    parser.add_argument("--marker-topic", default="/nav2_plan_markers")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--global-width", type=float, default=0.06)
    parser.add_argument("--local-width", type=float, default=0.04)
    args = parser.parse_args()

    rclpy.init()
    node = Nav2PlanPathViz(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
