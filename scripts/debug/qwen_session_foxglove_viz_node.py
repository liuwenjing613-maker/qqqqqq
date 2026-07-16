#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Qwen 会话 Foxglove 可视化：发布机器人位姿箭头与 Qwen 目标点 Marker。

- 不发布 /cmd_vel
- 不调用 Nav2
- 仅读取 TF 与 navigation_goal_proposal.json
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def load_goal_payload(path: Path) -> Optional[Dict[str, Any]]:
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None
    return data if isinstance(data, dict) else None


class QwenSessionFoxgloveVizNode(Node):
    def __init__(self) -> None:
        super().__init__("qwen_session_foxglove_viz")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("goal_json_path", "")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("robot_arrow_length_m", 0.55)
        self.declare_parameter("robot_arrow_shaft_diameter_m", 0.10)
        self.declare_parameter("goal_sphere_diameter_m", 0.28)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._robot_frame = str(self.get_parameter("robot_frame").value)
        self._goal_json_path = Path(str(self.get_parameter("goal_json_path").value)).expanduser()
        self._robot_arrow_len = float(self.get_parameter("robot_arrow_length_m").value)
        self._robot_arrow_shaft = float(self.get_parameter("robot_arrow_shaft_diameter_m").value)
        self._goal_sphere = float(self.get_parameter("goal_sphere_diameter_m").value)

        self._goal_payload: Optional[Dict[str, Any]] = None
        self._goal_mtime: float = 0.0

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub_robot = self.create_publisher(MarkerArray, "/qwen_session/robot_pose_markers", 10)
        self._pub_goal = self.create_publisher(MarkerArray, "/qwen_session/qwen_goal_markers", 10)
        self._pub_goal_pose = self.create_publisher(PoseStamped, "/qwen_session/qwen_goal_pose", 10)

        rate_hz = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._on_timer)
        self.get_logger().info(
            f"qwen_session_foxglove_viz started map={self._map_frame} "
            f"robot={self._robot_frame} goal_json={self._goal_json_path}"
        )

    def _stamp(self):
        msg = self.get_clock().now().to_msg()
        return msg

    def _lookup_robot_xy_yaw(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
        cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return float(t.x), float(t.y), float(yaw)

    def _maybe_reload_goal(self) -> None:
        path = self._goal_json_path
        if not path:
            return
        if not path.is_file():
            return
        mtime = path.stat().st_mtime
        if mtime <= self._goal_mtime and self._goal_payload is not None:
            return
        payload = load_goal_payload(path)
        if payload is None:
            return
        self._goal_payload = payload
        self._goal_mtime = mtime
        status = payload.get("selection_status", "unknown")
        goal = payload.get("goal_pose_map")
        if goal:
            self.get_logger().info(
                f"loaded Qwen goal json status={status} "
                f"x={goal.get('x')} y={goal.get('y')} yaw_deg={goal.get('yaw_deg')}"
            )
        else:
            self.get_logger().info(f"loaded Qwen goal json status={status} (no goal)")

    def _delete_all(self, ns: str, mid: int) -> Marker:
        m = Marker()
        m.header.frame_id = self._map_frame
        m.header.stamp = self._stamp()
        m.ns = ns
        m.id = mid
        m.action = Marker.DELETEALL
        return m

    def _publish_robot_markers(self, x: float, y: float, yaw: float) -> None:
        stamp = self._stamp()
        arr = MarkerArray()
        arr.markers.append(self._delete_all("robot_pose", 0))

        body = Marker()
        body.header.frame_id = self._map_frame
        body.header.stamp = stamp
        body.ns = "robot_pose"
        body.id = 1
        body.type = Marker.SPHERE
        body.action = Marker.ADD
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.position.z = 0.12
        body.pose.orientation = yaw_to_quaternion(yaw)
        body.scale.x = body.scale.y = body.scale.z = 0.22
        body.color = ColorRGBA(r=0.1, g=0.45, b=1.0, a=0.95)
        arr.markers.append(body)

        arrow = Marker()
        arrow.header.frame_id = self._map_frame
        arrow.header.stamp = stamp
        arrow.ns = "robot_pose"
        arrow.id = 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points.append(Point(x=x, y=y, z=0.12))
        arrow.points.append(
            Point(
                x=x + self._robot_arrow_len * math.cos(yaw),
                y=y + self._robot_arrow_len * math.sin(yaw),
                z=0.12,
            )
        )
        arrow.scale.x = self._robot_arrow_shaft
        arrow.scale.y = self._robot_arrow_shaft * 2.2
        arrow.scale.z = self._robot_arrow_shaft * 2.8
        arrow.color = ColorRGBA(r=0.0, g=0.35, b=1.0, a=0.98)
        arr.markers.append(arrow)

        label = Marker()
        label.header.frame_id = self._map_frame
        label.header.stamp = stamp
        label.ns = "robot_pose"
        label.id = 3
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = x + 0.15
        label.pose.position.y = y + 0.15
        label.pose.position.z = 0.45
        label.scale.z = 0.18
        label.color = ColorRGBA(r=0.85, g=0.92, b=1.0, a=1.0)
        label.text = f"ROBOT\nyaw={math.degrees(yaw):.0f}°"
        arr.markers.append(label)

        self._pub_robot.publish(arr)

    def _publish_goal_markers(self) -> None:
        payload = self._goal_payload
        stamp = self._stamp()
        arr = MarkerArray()
        arr.markers.append(self._delete_all("qwen_goal", 0))

        if not payload or payload.get("selection_status") != "REGION_PROPOSED":
            self._pub_goal.publish(arr)
            return

        goal = payload.get("goal_pose_map") or {}
        center = payload.get("region_center_map") or {}
        gx = float(goal.get("x", 0.0))
        gy = float(goal.get("y", 0.0))
        gyaw = float(goal.get("yaw_rad", 0.0))
        cx = float(center.get("x", gx))
        cy = float(center.get("y", gy))

        sphere = Marker()
        sphere.header.frame_id = self._map_frame
        sphere.header.stamp = stamp
        sphere.ns = "qwen_goal"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = gx
        sphere.pose.position.y = gy
        sphere.pose.position.z = 0.18
        sphere.scale.x = sphere.scale.y = sphere.scale.z = self._goal_sphere
        sphere.color = ColorRGBA(r=1.0, g=0.15, b=0.15, a=0.95)
        arr.markers.append(sphere)

        ring = Marker()
        ring.header.frame_id = self._map_frame
        ring.header.stamp = stamp
        ring.ns = "qwen_goal"
        ring.id = 2
        ring.type = Marker.CYLINDER
        ring.action = Marker.ADD
        ring.pose.position.x = cx
        ring.pose.position.y = cy
        ring.pose.position.z = 0.05
        ring.scale.x = ring.scale.y = 0.35
        ring.scale.z = 0.04
        ring.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.55)
        arr.markers.append(ring)

        cross_h = Marker()
        cross_h.header.frame_id = self._map_frame
        cross_h.header.stamp = stamp
        cross_h.ns = "qwen_goal"
        cross_h.id = 3
        cross_h.type = Marker.LINE_LIST
        cross_h.action = Marker.ADD
        cross_h.scale.x = 0.05
        cross_h.color = ColorRGBA(r=1.0, g=0.1, b=0.2, a=1.0)
        s = self._goal_sphere * 0.65
        cross_h.points.append(Point(x=gx - s, y=gy, z=0.18))
        cross_h.points.append(Point(x=gx + s, y=gy, z=0.18))
        cross_h.points.append(Point(x=gx, y=gy - s, z=0.18))
        cross_h.points.append(Point(x=gx, y=gy + s, z=0.18))
        arr.markers.append(cross_h)

        text = Marker()
        text.header.frame_id = self._map_frame
        text.header.stamp = stamp
        text.ns = "qwen_goal"
        text.id = 4
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = gx
        text.pose.position.y = gy
        text.pose.position.z = 0.55
        text.scale.z = 0.20
        text.color = ColorRGBA(r=1.0, g=0.85, b=0.85, a=1.0)
        summary = payload.get("proposal_summary") or {}
        conf = summary.get("confidence")
        conf_txt = f"{float(conf):.2f}" if conf is not None else "?"
        text.text = (
            "QWEN GOAL\n"
            f"x={gx:.2f} y={gy:.2f}\n"
            f"conf={conf_txt}"
        )
        arr.markers.append(text)

        self._pub_goal.publish(arr)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self._map_frame
        pose_msg.header.stamp = stamp
        pose_msg.pose.position.x = gx
        pose_msg.pose.position.y = gy
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = yaw_to_quaternion(gyaw)
        self._pub_goal_pose.publish(pose_msg)

    def _on_timer(self) -> None:
        self._maybe_reload_goal()
        robot = self._lookup_robot_xy_yaw()
        if robot is not None:
            self._publish_robot_markers(*robot)
        self._publish_goal_markers()


def main() -> None:
    rclpy.init()
    node = QwenSessionFoxgloveVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
