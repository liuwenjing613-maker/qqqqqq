#!/usr/bin/env python3
"""
Visualize LiDAR free-space / frontier decision for RDK X5 VLN robot.

Subscribes:
  - /scan
  - /failsafe_nav_state
  - /failsafe_nav_point
  - /cmd_vel or /cmd_vel_debug

Publishes:
  - /failsafe_nav/debug_image   sensor_msgs/Image, BGR8
  - /failsafe_nav/markers       visualization_msgs/MarkerArray

Usage:
  python3 debug_tools/visualize_lidar_frontier.py \
    --config configs/yolo_lidar_failsafe_nav.yaml \
    --cmd-topic /cmd_vel_debug

Foxglove:
  - Add Image panel: /failsafe_nav/debug_image
  - Add 3D MarkerArray layer: /failsafe_nav/markers
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = str(ROOT / "configs" / "yolo_lidar_failsafe_nav.yaml")


def load_yaml(path: str) -> Dict[str, Any]:
    path = os.path.expanduser(path)
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        return {}
    return cfg


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def safe_float(v: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        if v is None:
            return default
        x = float(v)
        if not math.isfinite(x):
            return default
        return x
    except Exception:
        return default


def parse_json_string(data: str) -> Dict[str, Any]:
    try:
        obj = json.loads(data)
        if isinstance(obj, dict):
            return obj
        return {}
    except Exception:
        return {}


def color_by_distance(r: float, max_range: float) -> Tuple[int, int, int]:
    """
    Return BGR color. Near obstacle is red/orange, far obstacle is green/blue.
    """
    t = clamp(r / max(max_range, 1e-6), 0.0, 1.0)

    if t < 0.25:
        # red -> orange
        return (0, int(120 + 100 * t / 0.25), 255)
    if t < 0.55:
        # orange -> yellow/green
        k = (t - 0.25) / 0.30
        return (0, 220, int(255 * (1.0 - 0.4 * k)))
    if t < 0.80:
        # green -> cyan
        k = (t - 0.55) / 0.25
        return (int(180 * k), 255, 0)
    # blue-ish for far points
    k = (t - 0.80) / 0.20
    return (255, int(180 * (1.0 - k)), 80)


class LidarFrontierVisualizer(Node):
    def __init__(self, cfg: Dict[str, Any], args: argparse.Namespace):
        super().__init__("lidar_frontier_visualizer")

        self.cfg = cfg

        self.scan_topic = args.scan_topic or cfg.get("scan_topic", "/scan")
        self.state_topic = args.state_topic or cfg.get("state_topic", "/failsafe_nav_state")
        self.point_topic = args.point_topic or cfg.get("point_topic", "/failsafe_nav_point")
        self.cmd_topic = args.cmd_topic or cfg.get("cmd_topic", "/cmd_vel")
        self.extra_cmd_topic = args.extra_cmd_topic

        self.debug_image_topic = args.debug_image_topic or cfg.get(
            "viz_debug_image_topic", "/failsafe_nav/debug_image"
        )
        self.marker_topic = args.marker_topic or cfg.get(
            "viz_markers_topic", "/failsafe_nav/markers"
        )

        self.camera_hfov_deg = float(cfg.get("camera_hfov_deg", 70.0))
        self.camera_lidar_yaw_offset_deg = float(cfg.get("camera_lidar_yaw_offset_deg", 0.0))

        self.lidar_min_range = float(cfg.get("lidar_min_range", 0.08))
        self.lidar_max_range = float(args.max_range or cfg.get("lidar_max_range", 6.0))
        self.lidar_front_deg = float(cfg.get("lidar_front_deg", 25.0))

        self.emergency_stop_distance = float(cfg.get("emergency_stop_distance", 0.22))
        self.hard_stop_distance = float(cfg.get("hard_stop_distance", 0.32))
        self.slow_distance = float(cfg.get("slow_distance", 0.55))
        self.free_space_sector_deg = float(cfg.get("free_space_sector_deg", 70.0))
        self.free_space_window_deg = float(cfg.get("free_space_window_deg", 10.0))
        self.free_space_min_clearance = float(cfg.get("free_space_min_clearance", 0.45))

        self.canvas_width = int(args.canvas_width)
        self.canvas_height = int(args.canvas_height)
        self.plot_size = int(min(self.canvas_height - 80, self.canvas_width - 360))
        self.plot_radius_px = int(self.plot_size * 0.46)
        self.center_x = int(self.plot_size * 0.52)
        self.center_y = int(self.canvas_height * 0.52)

        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time: float = 0.0
        self.latest_state: Dict[str, Any] = {}
        self.latest_state_time: float = 0.0
        self.latest_point: Dict[str, Any] = {}
        self.latest_point_time: float = 0.0
        self.latest_cmd: Optional[Twist] = None
        self.latest_cmd_time: float = 0.0

        self.image_pub = self.create_publisher(Image, self.debug_image_topic, 2)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 2)

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, self.state_topic, self.state_cb, 10)
        self.create_subscription(String, self.point_topic, self.point_cb, 10)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 10)

        if self.extra_cmd_topic and self.extra_cmd_topic != self.cmd_topic:
            self.create_subscription(Twist, self.extra_cmd_topic, self.cmd_cb, 10)

        rate = float(args.rate or cfg.get("viz_rate_hz", 5.0))
        self.timer = self.create_timer(1.0 / max(rate, 0.5), self.timer_cb)

        self.get_logger().info("===== LiDAR Frontier Visualizer START =====")
        self.get_logger().info(f"scan_topic={self.scan_topic}")
        self.get_logger().info(f"state_topic={self.state_topic}")
        self.get_logger().info(f"point_topic={self.point_topic}")
        self.get_logger().info(f"cmd_topic={self.cmd_topic}")
        if self.extra_cmd_topic:
            self.get_logger().info(f"extra_cmd_topic={self.extra_cmd_topic}")
        self.get_logger().info(f"debug_image_topic={self.debug_image_topic}")
        self.get_logger().info(f"marker_topic={self.marker_topic}")

    def scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = time.time()

    def state_cb(self, msg: String) -> None:
        data = parse_json_string(msg.data)
        if data:
            self.latest_state = data
            self.latest_state_time = time.time()

    def point_cb(self, msg: String) -> None:
        data = parse_json_string(msg.data)
        if data:
            self.latest_point = data
            self.latest_point_time = time.time()

    def cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_time = time.time()

    def timer_cb(self) -> None:
        img = self.render_debug_image()
        self.publish_image(img)
        self.publish_markers()

    def meters_to_px(self, r: float) -> float:
        return r * self.plot_radius_px / max(self.lidar_max_range, 1e-6)

    def heading_to_image_xy(self, heading_deg: float, radius_m: float) -> Tuple[int, int]:
        """
        Display convention:
          heading 0 deg: upward/front
          heading >0: right side on image
          heading <0: left side on image
        """
        theta = math.radians(heading_deg)
        x_m = radius_m * math.sin(theta)
        y_m = radius_m * math.cos(theta)

        px = int(round(self.center_x + self.meters_to_px(x_m)))
        py = int(round(self.center_y - self.meters_to_px(y_m)))
        return px, py

    def scan_angle_to_display_xy(self, theta_rad: float, r: float) -> Tuple[int, int]:
        """
        Convert raw LaserScan angle to the same display convention used by planner.

        The navigation code treats heading=0 as the camera/front direction, while
        camera_lidar_yaw_offset_deg shifts camera-front to LiDAR angle.
        Therefore display_heading = scan_angle - yaw_offset.
        """
        display_theta = theta_rad - math.radians(self.camera_lidar_yaw_offset_deg)

        x_m = r * math.sin(display_theta)
        y_m = r * math.cos(display_theta)

        px = int(round(self.center_x + self.meters_to_px(x_m)))
        py = int(round(self.center_y - self.meters_to_px(y_m)))
        return px, py

    def infer_heading_from_point_u(self) -> Optional[float]:
        u = safe_float(self.latest_point.get("u"), None)
        w = safe_float(self.latest_point.get("image_width"), None)
        if u is None:
            u = safe_float(self.latest_state.get("u"), None)
        if w is None:
            w = safe_float(self.latest_state.get("image_width"), None)
        if u is None or w is None or w <= 1:
            return None
        return (u / w - 0.5) * self.camera_hfov_deg

    def get_selected_heading(self) -> Optional[float]:
        h = safe_float(self.latest_state.get("heading_deg"), None)
        if h is not None:
            return h

        wp = self.latest_state.get("waypoint")
        if isinstance(wp, dict):
            h = safe_float(wp.get("heading_deg"), None)
            if h is not None:
                return h

        h = safe_float(self.latest_point.get("heading_deg"), None)
        if h is not None:
            return h

        return self.infer_heading_from_point_u()

    def get_selected_clearance(self) -> Optional[float]:
        c = safe_float(self.latest_state.get("clearance"), None)
        if c is not None:
            return c

        wp = self.latest_state.get("waypoint")
        if isinstance(wp, dict):
            c = safe_float(wp.get("clearance"), None)
            if c is not None:
                return c

        c = safe_float(self.latest_point.get("clearance"), None)
        return c

    def get_selected_score(self) -> Optional[float]:
        s = safe_float(self.latest_state.get("score"), None)
        if s is not None:
            return s

        wp = self.latest_state.get("waypoint")
        if isinstance(wp, dict):
            s = safe_float(wp.get("score"), None)
            if s is not None:
                return s

        return safe_float(self.latest_point.get("score"), None)

    def get_front_distance(self) -> Optional[float]:
        f = safe_float(self.latest_state.get("front_distance"), None)
        if f is not None:
            return f

        wp = self.latest_state.get("waypoint")
        if isinstance(wp, dict):
            f = safe_float(wp.get("front_distance"), None)
            if f is not None:
                return f

        return self.compute_front_distance_from_scan()

    def compute_front_distance_from_scan(self) -> Optional[float]:
        scan = self.latest_scan
        if scan is None:
            return None

        values = []
        angle_min = float(scan.angle_min)
        angle_inc = float(scan.angle_increment)
        if abs(angle_inc) < 1e-9:
            return None

        center_deg = self.camera_lidar_yaw_offset_deg
        half = self.lidar_front_deg / 2.0

        for deg in np.linspace(center_deg - half, center_deg + half, num=max(5, int(self.lidar_front_deg) + 1)):
            rad = math.radians(float(deg))
            idx = int(round((rad - angle_min) / angle_inc))
            if idx < 0 or idx >= len(scan.ranges):
                continue
            r = float(scan.ranges[idx])
            if not math.isfinite(r):
                continue
            if r < self.lidar_min_range or r > self.lidar_max_range:
                continue
            values.append(r)

        if not values:
            return None

        return float(np.percentile(np.array(values, dtype=np.float32), 20))

    def draw_grid(self, img: np.ndarray) -> None:
        # Dark background grid, intentionally subtle. Foxglove's blue grid already bullied us enough.
        grid_color = (45, 45, 45)
        axis_color = (90, 90, 90)

        max_r = self.lidar_max_range
        for r in np.arange(0.5, max_r + 1e-6, 0.5):
            radius = int(round(self.meters_to_px(float(r))))
            cv2.circle(img, (self.center_x, self.center_y), radius, grid_color, 1)

        for deg in range(-180, 181, 30):
            p = self.heading_to_image_xy(float(deg), self.lidar_max_range)
            cv2.line(img, (self.center_x, self.center_y), p, grid_color, 1)

        # Front axis.
        front = self.heading_to_image_xy(0.0, self.lidar_max_range)
        cv2.line(img, (self.center_x, self.center_y), front, axis_color, 2)

        # Left/right axis.
        left = self.heading_to_image_xy(-90.0, self.lidar_max_range)
        right = self.heading_to_image_xy(90.0, self.lidar_max_range)
        cv2.line(img, left, right, axis_color, 1)

        cv2.putText(img, "FRONT", (front[0] - 25, max(20, front[1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1, cv2.LINE_AA)

    def draw_safety_rings(self, img: np.ndarray) -> None:
        rings = [
            (self.emergency_stop_distance, (0, 0, 255), "emergency"),
            (self.hard_stop_distance, (0, 80, 255), "hard stop"),
            (self.slow_distance, (0, 200, 255), "slow"),
            (self.free_space_min_clearance, (0, 180, 0), "min free"),
        ]

        for dist, color, label in rings:
            if dist <= 0:
                continue
            radius = int(round(self.meters_to_px(dist)))
            cv2.circle(img, (self.center_x, self.center_y), radius, color, 1)
            cv2.putText(
                img,
                f"{label} {dist:.2f}m",
                (self.center_x + radius + 5, self.center_y - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                color,
                1,
                cv2.LINE_AA,
            )

    def draw_free_space_sector(self, img: np.ndarray) -> None:
        left = self.heading_to_image_xy(-self.free_space_sector_deg, self.lidar_max_range)
        right = self.heading_to_image_xy(self.free_space_sector_deg, self.lidar_max_range)

        cv2.line(img, (self.center_x, self.center_y), left, (70, 120, 70), 1)
        cv2.line(img, (self.center_x, self.center_y), right, (70, 120, 70), 1)

        cv2.putText(
            img,
            f"free-space sector +/-{self.free_space_sector_deg:.0f}deg",
            (20, self.canvas_height - 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (120, 180, 120),
            1,
            cv2.LINE_AA,
        )

    def draw_scan_points(self, img: np.ndarray) -> int:
        scan = self.latest_scan
        if scan is None:
            return 0

        angle = float(scan.angle_min)
        angle_inc = float(scan.angle_increment)
        n = len(scan.ranges)

        count = 0
        for i in range(n):
            r = float(scan.ranges[i])
            if not math.isfinite(r):
                angle += angle_inc
                continue
            if r < self.lidar_min_range or r > self.lidar_max_range:
                angle += angle_inc
                continue

            px, py = self.scan_angle_to_display_xy(angle, r)
            if 0 <= px < self.canvas_width and 0 <= py < self.canvas_height:
                color = color_by_distance(r, self.lidar_max_range)
                cv2.circle(img, (px, py), 3, color, -1)
                count += 1

            angle += angle_inc

        return count

    def draw_robot(self, img: np.ndarray) -> None:
        # Robot body.
        cv2.circle(img, (self.center_x, self.center_y), 8, (230, 230, 230), -1)
        cv2.circle(img, (self.center_x, self.center_y), 14, (170, 170, 170), 1)

        # Heading arrow.
        p_front = self.heading_to_image_xy(0.0, 0.35)
        cv2.arrowedLine(
            img,
            (self.center_x, self.center_y),
            p_front,
            (255, 255, 255),
            2,
            tipLength=0.25,
        )
        cv2.putText(
            img,
            "robot/lidar",
            (self.center_x + 16, self.center_y + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )

    def draw_selected_heading(self, img: np.ndarray) -> None:
        heading = self.get_selected_heading()
        clearance = self.get_selected_clearance()
        mode = str(self.latest_state.get("mode", self.latest_point.get("mode", "")))

        if heading is None:
            return

        if clearance is None:
            arrow_len = min(self.lidar_max_range * 0.65, 2.0)
        else:
            arrow_len = clamp(clearance, 0.3, self.lidar_max_range * 0.85)

        end = self.heading_to_image_xy(float(heading), arrow_len)

        if "TARGET" in mode:
            color = (255, 180, 0)
            label = "TARGET"
        elif "BLOCK" in mode or "STOP" in mode or "EMERGENCY" in mode:
            color = (0, 0, 255)
            label = "RECOVERY"
        else:
            color = (0, 255, 0)
            label = "FREE"

        cv2.arrowedLine(
            img,
            (self.center_x, self.center_y),
            end,
            color,
            4,
            tipLength=0.18,
        )
        cv2.circle(img, end, 8, color, 2)

        cv2.putText(
            img,
            f"{label} heading={heading:.1f}deg",
            (end[0] + 8, end[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            1,
            cv2.LINE_AA,
        )

        # Draw selected window around heading.
        half = self.free_space_window_deg / 2.0
        p1 = self.heading_to_image_xy(float(heading) - half, arrow_len)
        p2 = self.heading_to_image_xy(float(heading) + half, arrow_len)
        cv2.line(img, (self.center_x, self.center_y), p1, (80, 180, 80), 1)
        cv2.line(img, (self.center_x, self.center_y), p2, (80, 180, 80), 1)

    def draw_info_panel(self, img: np.ndarray, valid_scan_count: int) -> None:
        x0 = self.plot_size + 35
        y = 40
        line_h = 26

        cv2.rectangle(img, (self.plot_size + 15, 20), (self.canvas_width - 20, self.canvas_height - 20), (28, 28, 28), -1)
        cv2.rectangle(img, (self.plot_size + 15, 20), (self.canvas_width - 20, self.canvas_height - 20), (70, 70, 70), 1)

        now = time.time()
        scan_age = now - self.latest_scan_time if self.latest_scan_time > 0 else None
        state_age = now - self.latest_state_time if self.latest_state_time > 0 else None
        point_age = now - self.latest_point_time if self.latest_point_time > 0 else None
        cmd_age = now - self.latest_cmd_time if self.latest_cmd_time > 0 else None

        mode = str(self.latest_state.get("mode", "NO_STATE"))
        reason = str(self.latest_state.get("reason", ""))
        instruction = str(self.latest_state.get("instruction", self.cfg.get("instruction", "")))
        front = self.get_front_distance()
        heading = self.get_selected_heading()
        clearance = self.get_selected_clearance()
        score = self.get_selected_score()

        cmd_vx = None
        cmd_wz = None
        if self.latest_cmd is not None:
            cmd_vx = float(self.latest_cmd.linear.x)
            cmd_wz = float(self.latest_cmd.angular.z)
        else:
            cmd_vx = safe_float(self.latest_state.get("cmd_vx"), 0.0)
            cmd_wz = safe_float(self.latest_state.get("cmd_wz"), 0.0)

        scan_frame = "none"
        scan_hz_hint = ""
        if self.latest_scan is not None:
            scan_frame = self.latest_scan.header.frame_id
            scan_hz_hint = f"ranges={len(self.latest_scan.ranges)} valid={valid_scan_count}"

        def put(text: str, color=(230, 230, 230), scale=0.50, thick=1):
            nonlocal y
            cv2.putText(img, text, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)
            y += line_h

        put("LiDAR Frontier Debug", (255, 255, 255), 0.62, 2)
        y += 6

        mode_color = (0, 255, 0)
        if "WAIT" in mode:
            mode_color = (0, 180, 255)
        if "BLOCK" in mode or "STOP" in mode or "EMERGENCY" in mode:
            mode_color = (0, 0, 255)
        if "TARGET" in mode:
            mode_color = (255, 180, 0)

        put(f"mode: {mode}", mode_color, 0.58, 2)
        put(f"instruction: {instruction}", (210, 210, 210))
        put(f"reason: {reason[:34]}", (180, 180, 180), 0.44)

        y += 8
        put(f"front_distance: {front:.3f} m" if front is not None else "front_distance: null", (0, 220, 255))
        put(f"heading_deg: {heading:.2f}" if heading is not None else "heading_deg: null", (0, 255, 0))
        put(f"clearance: {clearance:.3f} m" if clearance is not None else "clearance: null", (0, 255, 0))
        put(f"score: {score:.3f}" if score is not None else "score: null", (0, 255, 0))

        y += 8
        put(f"cmd_vx: {cmd_vx:.3f} m/s" if cmd_vx is not None else "cmd_vx: null", (255, 220, 150))
        put(f"cmd_wz: {cmd_wz:.3f} rad/s" if cmd_wz is not None else "cmd_wz: null", (255, 220, 150))

        y += 8
        put(f"scan_topic: {self.scan_topic}", (180, 210, 255), 0.44)
        put(f"scan_frame: {scan_frame}", (180, 210, 255), 0.44)
        put(scan_hz_hint, (180, 210, 255), 0.44)

        y += 8
        put(f"scan_age: {scan_age:.2f}s" if scan_age is not None else "scan_age: never", (180, 180, 180), 0.44)
        put(f"state_age: {state_age:.2f}s" if state_age is not None else "state_age: never", (180, 180, 180), 0.44)
        put(f"point_age: {point_age:.2f}s" if point_age is not None else "point_age: never", (180, 180, 180), 0.44)
        put(f"cmd_age: {cmd_age:.2f}s" if cmd_age is not None else "cmd_age: never", (180, 180, 180), 0.44)

        y += 8
        put("Legend:", (255, 255, 255), 0.50, 1)
        put("red/orange: near obstacle", (0, 140, 255), 0.42)
        put("green/blue: farther obstacle", (220, 220, 120), 0.42)
        put("green arrow: selected free-space", (0, 255, 0), 0.42)
        put("red rings: stop thresholds", (0, 0, 255), 0.42)

        if self.latest_scan is None:
            cv2.putText(
                img,
                "NO /scan RECEIVED",
                (70, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                3,
                cv2.LINE_AA,
            )

    def render_debug_image(self) -> np.ndarray:
        img = np.zeros((self.canvas_height, self.canvas_width, 3), dtype=np.uint8)
        img[:] = (10, 10, 10)

        self.draw_grid(img)
        self.draw_free_space_sector(img)
        self.draw_safety_rings(img)
        valid_scan_count = self.draw_scan_points(img)
        self.draw_selected_heading(img)
        self.draw_robot(img)
        self.draw_info_panel(img, valid_scan_count)

        return img

    def publish_image(self, img: np.ndarray) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.latest_scan is not None:
            msg.header.frame_id = self.latest_scan.header.frame_id
        else:
            msg.header.frame_id = str(self.cfg.get("viz_frame_id", "laser"))

        msg.height = int(img.shape[0])
        msg.width = int(img.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(img.shape[1] * 3)
        msg.data = img.tobytes()

        self.image_pub.publish(msg)

    def marker_frame_id(self) -> str:
        if self.latest_scan is not None and self.latest_scan.header.frame_id:
            return self.latest_scan.header.frame_id
        return str(self.cfg.get("viz_frame_id", "laser"))

    def publish_markers(self) -> None:
        heading = self.get_selected_heading()
        if heading is None:
            # Still publish DELETEALL so old arrows disappear.
            arr = MarkerArray()
            delete = Marker()
            delete.action = Marker.DELETEALL
            arr.markers.append(delete)
            self.marker_pub.publish(arr)
            return

        clearance = self.get_selected_clearance()
        if clearance is None:
            length = 1.0
        else:
            length = clamp(float(clearance), 0.25, min(self.lidar_max_range, 2.5))

        # Planner heading is camera/front heading.
        # Convert it back to LiDAR raw angle for marker geometry.
        theta = math.radians(float(heading) + self.camera_lidar_yaw_offset_deg)

        end_x = length * math.cos(theta)
        end_y = length * math.sin(theta)

        frame_id = self.marker_frame_id()
        stamp = self.get_clock().now().to_msg()

        arr = MarkerArray()

        delete = Marker()
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)

        arrow = Marker()
        arrow.header.frame_id = frame_id
        arrow.header.stamp = stamp
        arrow.ns = "frontier_debug"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [
            Point(x=0.0, y=0.0, z=0.08),
            Point(x=float(end_x), y=float(end_y), z=0.08),
        ]
        arrow.scale.x = 0.035
        arrow.scale.y = 0.09
        arrow.scale.z = 0.14
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 0.0
        arrow.color.a = 0.95
        arr.markers.append(arrow)

        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = stamp
        sphere.ns = "frontier_debug"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(end_x)
        sphere.pose.position.y = float(end_y)
        sphere.pose.position.z = 0.08
        sphere.scale.x = 0.12
        sphere.scale.y = 0.12
        sphere.scale.z = 0.12
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 0.95
        arr.markers.append(sphere)

        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = "frontier_debug"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(end_x)
        text.pose.position.y = float(end_y)
        text.pose.position.z = 0.35
        text.scale.z = 0.18
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 0.95

        mode = str(self.latest_state.get("mode", ""))
        front = self.get_front_distance()
        clearance = self.get_selected_clearance()
        text.text = (
            f"{mode}\n"
            f"heading={heading:.1f} deg\n"
            f"front={front:.2f}m" if front is not None else f"{mode}\nheading={heading:.1f} deg\nfront=null"
        )
        if clearance is not None:
            text.text += f"\nclearance={clearance:.2f}m"

        arr.markers.append(text)

        self.marker_pub.publish(arr)


def main() -> None:
    parser = argparse.ArgumentParser(description="LiDAR frontier/free-space visualizer")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="YAML config path")
    parser.add_argument("--scan-topic", default=None)
    parser.add_argument("--state-topic", default=None)
    parser.add_argument("--point-topic", default=None)
    parser.add_argument("--cmd-topic", default=None)
    parser.add_argument("--extra-cmd-topic", default="/cmd_vel_debug")
    parser.add_argument("--debug-image-topic", default=None)
    parser.add_argument("--marker-topic", default=None)
    parser.add_argument("--rate", type=float, default=None)
    parser.add_argument("--max-range", type=float, default=None)
    parser.add_argument("--canvas-width", type=int, default=1200)
    parser.add_argument("--canvas-height", type=int, default=850)
    args = parser.parse_args()

    cfg = load_yaml(args.config)

    rclpy.init()
    node = LidarFrontierVisualizer(cfg, args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
