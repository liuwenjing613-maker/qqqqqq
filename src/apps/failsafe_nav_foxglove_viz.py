#!/usr/bin/env python3
"""
Foxglove visualization bridge for P0 failsafe navigation.

Publishes:
  /failsafe_nav/markers      visualization_msgs/MarkerArray  (3D panel)
  /failsafe_nav/debug_image  sensor_msgs/Image               (Image panel)

Subscribe:
  /failsafe_nav_state, /failsafe_nav_point, /target_bbox_json, /image_raw
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

DEFAULT_CONFIG = str(ROOT / "configs" / "yolo_lidar_failsafe_nav.yaml")


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _color(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _polar_xy(heading_deg: float, dist: float):
    rad = math.radians(float(heading_deg))
    return dist * math.cos(rad), dist * math.sin(rad)


class FailsafeNavFoxgloveViz(Node):
    def __init__(self, cfg: Dict[str, Any]):
        super().__init__("failsafe_nav_foxglove_viz")

        self.frame_id = str(cfg.get("viz_frame_id", "laser"))
        self.camera_hfov_deg = float(cfg.get("camera_hfov_deg", 70.0))
        self.lidar_front_deg = float(cfg.get("lidar_front_deg", 18.0))
        self.emergency_stop_distance = float(cfg.get("emergency_stop_distance", 0.22))
        self.hard_stop_distance = float(cfg.get("hard_stop_distance", 0.32))
        self.slow_distance = float(cfg.get("slow_distance", 0.55))

        self.state_topic = cfg.get("state_topic", "/failsafe_nav_state")
        self.point_topic = cfg.get("point_topic", "/failsafe_nav_point")
        self.bbox_topic = cfg.get("target_bbox_topic", "/target_bbox_json")
        self.image_topic = cfg.get("image_topic", "/image_raw")
        self.markers_topic = cfg.get("viz_markers_topic", "/failsafe_nav/markers")
        self.debug_image_topic = cfg.get("viz_debug_image_topic", "/failsafe_nav/debug_image")

        self.bridge = CvBridge()
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_state: Dict[str, Any] = {}
        self.latest_point: Dict[str, Any] = {}
        self.latest_bbox: Dict[str, Any] = {}
        self.image_width = int(cfg.get("image_width", 640))
        self.image_height = int(cfg.get("image_height", 480))

        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)
        self.image_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.create_subscription(String, self.state_topic, self.state_cb, 10)
        self.create_subscription(String, self.point_topic, self.point_cb, 10)
        self.create_subscription(String, self.bbox_topic, self.bbox_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)

        hz = float(cfg.get("viz_rate_hz", 5.0))
        self.create_timer(1.0 / max(hz, 1.0), self.publish_all)

        self.get_logger().info("===== failsafe_nav_foxglove_viz =====")
        self.get_logger().info(f"markers={self.markers_topic} debug_image={self.debug_image_topic}")
        self.get_logger().info(f"frame_id={self.frame_id}")

    def state_cb(self, msg: String):
        try:
            self.latest_state = json.loads(msg.data)
        except Exception:
            pass

    def point_cb(self, msg: String):
        try:
            self.latest_point = json.loads(msg.data)
        except Exception:
            pass

    def bbox_cb(self, msg: String):
        try:
            self.latest_bbox = json.loads(msg.data)
        except Exception:
            pass

    def image_cb(self, msg: Image):
        if msg.width and msg.height:
            self.image_width = int(msg.width)
            self.image_height = int(msg.height)
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge failed: {exc}")

    def publish_all(self):
        stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self._build_markers(stamp))
        debug = self._build_debug_image()
        if debug is not None:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    def _build_markers(self, stamp) -> MarkerArray:
        arr = MarkerArray()
        mid = 0

        def add(marker: Marker):
            nonlocal mid
            marker.header.stamp = stamp
            marker.header.frame_id = self.frame_id
            marker.id = mid
            mid += 1
            arr.markers.append(marker)

        state = self.latest_state or {}
        mode = str(state.get("mode", "UNKNOWN"))
        front = state.get("front_distance")
        front_f = float(front) if front is not None else None

        # --- safety rings ---
        for dist, color, ns in (
            (self.emergency_stop_distance, (1.0, 0.1, 0.1, 0.25), "safety_emergency"),
            (self.hard_stop_distance, (1.0, 0.5, 0.0, 0.20), "safety_hard"),
            (self.slow_distance, (1.0, 1.0, 0.0, 0.12), "safety_slow"),
        ):
            m = Marker()
            m.ns = ns
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.015
            m.color = _color(*color)
            for deg in np.linspace(-70, 70, 29):
                x, y = _polar_xy(deg, dist)
                m.points.append(Point(x=float(x), y=float(y), z=0.0))
            add(m)

        # --- front distance wedge ---
        if front_f is not None and front_f > 0:
            m = Marker()
            m.ns = "front_distance"
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.03
            m.color = _color(0.2, 1.0, 0.4, 0.9)
            from geometry_msgs.msg import Point as _Point
            for deg in (-self.lidar_front_deg, self.lidar_front_deg):
                x, y = _polar_xy(deg, front_f)
                m.points.append(_Point(x=0.0, y=0.0, z=0.0))
                m.points.append(_Point(x=float(x), y=float(y), z=0.0))
            add(m)

            m2 = Marker()
            m2.ns = "front_center"
            m2.type = Marker.ARROW
            m2.action = Marker.ADD
            m2.scale.x = 0.04
            m2.scale.y = 0.08
            m2.scale.z = 0.08
            m2.color = _color(0.2, 1.0, 0.4, 0.95)
            m2.points.append(_Point(x=0.0, y=0.0, z=0.0))
            m2.points.append(_Point(x=float(front_f), y=0.0, z=0.0))
            add(m2)

        # --- free-space waypoint ray ---
        wp = state.get("waypoint") if isinstance(state.get("waypoint"), dict) else {}
        heading = state.get("heading_deg", wp.get("heading_deg"))
        clearance = state.get("clearance", wp.get("clearance"))
        if heading is not None and clearance is not None:
            hx, hy = _polar_xy(float(heading), float(clearance))
            m = Marker()
            m.ns = "free_space_ray"
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.scale.y = 0.10
            m.scale.z = 0.10
            m.color = _color(0.2, 0.6, 1.0, 0.95)
            m.points.append(Point(x=0.0, y=0.0, z=0.0))
            m.points.append(Point(x=float(hx), y=float(hy), z=0.0))
            add(m)

            m2 = Marker()
            m2.ns = "free_space_point"
            m2.type = Marker.SPHERE
            m2.action = Marker.ADD
            m2.pose.position.x = float(hx)
            m2.pose.position.y = float(hy)
            m2.pose.position.z = 0.05
            m2.scale.x = m2.scale.y = m2.scale.z = 0.12
            m2.color = _color(0.2, 0.6, 1.0, 0.85)
            add(m2)

        # --- target direction from bbox center (approximate) ---
        target = state.get("target") if isinstance(state.get("target"), dict) else self.latest_bbox
        if target and target.get("visible", False):
            u = target.get("u", target.get("cx"))
            if u is not None:
                angle_deg = (float(u) - self.image_width / 2.0) / max(self.image_width, 1.0) * self.camera_hfov_deg
                dist = front_f if front_f is not None else 1.0
                tx, ty = _polar_xy(angle_deg, dist)
                m = Marker()
                m.ns = "target_ray"
                m.type = Marker.ARROW
                m.action = Marker.ADD
                m.scale.x = 0.04
                m.scale.y = 0.09
                m.scale.z = 0.09
                m.color = _color(1.0, 0.2, 0.9, 0.95)
                m.points.append(Point(x=0.0, y=0.0, z=0.0))
                m.points.append(Point(x=float(tx), y=float(ty), z=0.0))
                add(m)

        # --- status text ---
        cmd_vx = state.get("cmd_vx", 0.0)
        cmd_wz = state.get("cmd_wz", 0.0)
        reason = str(state.get("reason", ""))
        text = f"mode={mode}\n"
        text += f"front={front_f:.2f}m\n" if front_f is not None else "front=?\n"
        text += f"vx={float(cmd_vx):.3f} wz={float(cmd_wz):.3f}\n{reason[:60]}"

        m = Marker()
        m.ns = "status_text"
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.6
        m.scale.z = 0.12
        m.color = _color(1.0, 1.0, 1.0, 1.0)
        m.text = text
        add(m)

        return arr

    def _build_debug_image(self) -> Optional[np.ndarray]:
        if self.latest_frame is None:
            return None

        vis = self.latest_frame.copy()
        h, w = vis.shape[:2]

        # bbox
        bbox = self.latest_bbox
        if bbox.get("visible", False) and isinstance(bbox.get("bbox"), (list, tuple)) and len(bbox["bbox"]) == 4:
            x1, y1, x2, y2 = [int(v) for v in bbox["bbox"]]
            if x2 > x1 and y2 > y1:
                cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 0, 255), 2)
                label = f"{bbox.get('class_name', '')} {float(bbox.get('score', 0)):.3f}"
                cv2.putText(vis, label, (x1, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # active nav point
        pt = self.latest_point
        if pt.get("u") is not None and pt.get("v") is not None:
            u, v = int(float(pt["u"])), int(float(pt["v"]))
            color = (255, 180, 0) if pt.get("source") == "target" else (0, 180, 255)
            cv2.drawMarker(vis, (u, v), color, cv2.MARKER_CROSS, 24, 2)
            cv2.circle(vis, (u, v), 10, color, 1)
            cv2.putText(vis, str(pt.get("source", "point")), (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        # state overlay
        st = self.latest_state
        mode = str(st.get("mode", "?"))
        front = st.get("front_distance")
        front_s = f"{float(front):.2f}m" if front is not None else "?"
        lines = [
            f"mode={mode}",
            f"front={front_s}  vx={float(st.get('cmd_vx', 0)):.3f}  wz={float(st.get('cmd_wz', 0)):.3f}",
            f"reason={str(st.get('reason', ''))[:50]}",
        ]
        wp = st.get("waypoint") if isinstance(st.get("waypoint"), dict) else {}
        if wp:
            lines.append(
                f"free_space h={wp.get('heading_deg', '?')} clear={wp.get('clearance', '?')}"
            )
        y0 = 22
        for i, line in enumerate(lines):
            cv2.putText(vis, line, (8, y0 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(vis, line, (8, y0 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 40, 0), 1)

        cv2.line(vis, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
        return vis


def main():
    parser = argparse.ArgumentParser(description="Foxglove viz for failsafe nav")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    rclpy.init()
    node = FailsafeNavFoxgloveViz(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
