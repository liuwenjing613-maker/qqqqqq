#!/usr/bin/env python3
import json
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class FrontierPointMarkerViz(Node):
    def __init__(self):
        super().__init__("frontier_point_marker_viz")

        self.declare_parameter("point_topic", "/failsafe_nav_point")
        self.declare_parameter("state_topic", "/failsafe_nav_state")
        self.declare_parameter("sent_cmd_topic", "/cmd_vel_sent")
        self.declare_parameter("marker_topic", "/failsafe_nav/markers")
        self.declare_parameter("frame_id", "laser")

        self.point_topic = self.get_parameter("point_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.sent_cmd_topic = self.get_parameter("sent_cmd_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.latest_point = {}
        self.latest_state = {}
        self.latest_sent_cmd = Twist()

        self.sub_point = self.create_subscription(
            String,
            self.point_topic,
            self.on_point,
            10,
        )

        self.sub_state = self.create_subscription(
            String,
            self.state_topic,
            self.on_state,
            10,
        )

        self.sub_sent = self.create_subscription(
            Twist,
            self.sent_cmd_topic,
            self.on_sent_cmd,
            10,
        )

        self.pub_markers = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10,
        )

        self.timer = self.create_timer(0.1, self.publish_markers)

        self.get_logger().info(
            f"FrontierPointMarkerViz started: "
            f"point={self.point_topic}, state={self.state_topic}, "
            f"markers={self.marker_topic}, frame_id={self.frame_id}"
        )

    def on_point(self, msg: String):
        try:
            self.latest_point = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"failed to parse point json: {e}")

    def on_state(self, msg: String):
        try:
            self.latest_state = json.loads(msg.data)
        except Exception:
            pass

    def on_sent_cmd(self, msg: Twist):
        self.latest_sent_cmd = msg

    def _make_delete_all(self):
        m = Marker()
        m.action = Marker.DELETEALL
        return m

    def publish_markers(self):
        arr = MarkerArray()
        arr.markers.append(self._make_delete_all())

        if not self.latest_point:
            self.pub_markers.publish(arr)
            return

        now = self.get_clock().now().to_msg()

        source = str(self.latest_point.get("source", "unknown"))
        mode = str(self.latest_state.get("mode", self.latest_state.get("fsm_state", "unknown")))
        reason = str(self.latest_state.get("reason", ""))
        active_reason = str(self.latest_state.get("active_cmd_reason", ""))
        sent_vx = float(self.latest_sent_cmd.linear.x)
        sent_wz = float(self.latest_sent_cmd.angular.z)
        nav_vx = float(self.latest_state.get("cmd_vx", 0.0))
        nav_wz = float(self.latest_state.get("cmd_wz", 0.0))

        heading_deg = self.latest_point.get("heading_deg", None)
        clearance = self.latest_point.get("clearance", None)
        front_distance = self.latest_point.get("front_distance", None)
        score = self.latest_point.get("score", None)

        # 如果没有 heading_deg，就根据 u 粗略反推一个角度，避免完全画不出来。
        if heading_deg is None:
            u = float(self.latest_point.get("u", 320.0))
            image_width = float(self.latest_point.get("image_width", 640.0))
            fov_deg = 70.0
            heading_deg = (u - image_width / 2.0) / image_width * fov_deg

        heading_rad = math.radians(float(heading_deg))

        if clearance is None:
            clearance = 1.0

        length = max(0.25, min(float(clearance), 1.5))
        end_x = length * math.cos(heading_rad)
        end_y = length * math.sin(heading_rad)

        blocked = ("BLOCK" in mode.upper()) or ("STOP" in mode.upper())

        if blocked:
            color = (1.0, 0.1, 0.1, 1.0)
        elif source == "target":
            color = (0.1, 0.3, 1.0, 1.0)
        else:
            color = (0.1, 1.0, 0.2, 1.0)

        # Arrow marker
        arrow = Marker()
        arrow.header.frame_id = self.frame_id
        arrow.header.stamp = now
        arrow.ns = "frontier_point"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0

        p0 = Point()
        p0.x = 0.0
        p0.y = 0.0
        p0.z = 0.05

        p1 = Point()
        p1.x = float(end_x)
        p1.y = float(end_y)
        p1.z = 0.05

        arrow.points = [p0, p1]
        arrow.scale.x = 0.04
        arrow.scale.y = 0.10
        arrow.scale.z = 0.10
        arrow.color.r = color[0]
        arrow.color.g = color[1]
        arrow.color.b = color[2]
        arrow.color.a = color[3]
        arr.markers.append(arrow)

        # Sphere marker at selected point
        sphere = Marker()
        sphere.header.frame_id = self.frame_id
        sphere.header.stamp = now
        sphere.ns = "frontier_point"
        sphere.id = 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(end_x)
        sphere.pose.position.y = float(end_y)
        sphere.pose.position.z = 0.05
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.14
        sphere.scale.y = 0.14
        sphere.scale.z = 0.14
        sphere.color.r = color[0]
        sphere.color.g = color[1]
        sphere.color.b = color[2]
        sphere.color.a = 0.9
        arr.markers.append(sphere)

        # Text marker
        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = now
        text.ns = "frontier_point"
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(end_x)
        text.pose.position.y = float(end_y)
        text.pose.position.z = 0.35
        text.pose.orientation.w = 1.0
        text.scale.z = 0.16
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0

        text.text = (
            f"{mode}\n"
            f"reason: {reason[:40]}\n"
            f"nav vx/wz: {nav_vx:.3f}/{nav_wz:.3f}\n"
            f"sent vx/wz: {sent_vx:.3f}/{sent_wz:.3f}"
        )
        if active_reason:
            text.text += f"\nactive: {active_reason[:30]}"
        if score is not None:
            text.text += f"\nscore: {float(score):.2f}"

        arr.markers.append(text)

        self.pub_markers.publish(arr)


def main():
    rclpy.init()
    node = FrontierPointMarkerViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
