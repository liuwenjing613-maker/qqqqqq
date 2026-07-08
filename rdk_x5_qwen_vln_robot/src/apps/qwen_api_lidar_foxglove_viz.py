#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Foxglove visualization for Qwen API + LiDAR navigation.

Subscribes:
  /image_raw, /scan, /qwen_api_json, /qwen_api_state

Publishes (RELIABLE QoS for foxglove_bridge):
  /qwen_api_viz/image/compressed   annotated camera (Image panel)
  /qwen_api_viz/markers            LiDAR depth rays + safety rings (3D panel)
  /qwen_api_viz/hud                std_msgs/String JSON summary (Raw Messages panel)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any, Dict, Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

DEFAULT_CONFIG = os.path.join(PROJECT_ROOT, "configs/qwen_api_lidar_nav.yaml")


def load_yaml(path: str) -> Dict[str, Any]:
    with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    return cfg if isinstance(cfg, dict) else {}


def _nested_get(cfg: Dict[str, Any], block: str, key: str, default: Any) -> Any:
    section = cfg.get(block)
    if isinstance(section, dict) and key in section:
        return section[key]
    if key in cfg:
        return cfg[key]
    return default


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _polar_xy(heading_deg: float, dist: float) -> tuple:
    rad = math.radians(float(heading_deg))
    return dist * math.cos(rad), dist * math.sin(rad)


def _fmt(v: Any, digits: int = 3) -> str:
    if v is None:
        return "n/a"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def _draw_text_block(img: np.ndarray, lines: list, origin=(8, 24), line_h=22) -> None:
    x, y = origin
    for line in lines:
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1, cv2.LINE_AA)
        y += line_h


class QwenApiLidarFoxgloveViz(Node):
    def __init__(self, cfg: Dict[str, Any]):
        super().__init__("qwen_api_lidar_foxglove_viz")
        self.cfg = cfg
        viz = cfg.get("foxglove_viz") or {}

        self.image_topic = str(cfg.get("image_topic", "/image_raw"))
        self.scan_topic = str(cfg.get("scan_topic", "/scan"))
        self.json_topic = str(cfg.get("qwen_json_topic", "/qwen_api_json"))
        self.state_topic = str(cfg.get("state_topic", "/qwen_api_state"))

        self.markers_topic = str(viz.get("markers_topic", "/qwen_api_viz/markers"))
        self.compressed_topic = str(viz.get("debug_compressed_topic", "/qwen_api_viz/image/compressed"))
        self.hud_topic = str(viz.get("hud_topic", "/qwen_api_viz/hud"))
        self.frame_id = str(viz.get("frame_id", "laser"))

        self.image_width = int(cfg.get("image_width", 1280))
        self.image_height = int(cfg.get("image_height", 720))
        self.lidar_front_deg = float(cfg.get("lidar_front_deg", 30.0))
        self.emergency_stop_distance = float(cfg.get("emergency_stop_distance", 0.28))
        self.hard_stop_distance = float(cfg.get("hard_stop_distance", 0.42))
        self.slow_distance = float(cfg.get("slow_distance", 0.65))
        self.arrive_distance = float(
            _nested_get(cfg, "success", "lidar_target_arrive_distance", cfg.get("lidar_target_arrive_distance", 0.6))
        )

        self.viz_image_max_fps = float(viz.get("viz_image_max_fps", 8.0))
        self.viz_image_max_width = int(viz.get("viz_image_max_width", 960))
        self.viz_jpeg_quality = int(viz.get("viz_jpeg_quality", 80))
        self.marker_hz = float(viz.get("viz_rate_hz", 5.0))

        self.bridge = CvBridge()
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_header: Optional[Header] = None
        self.latest_json: Dict[str, Any] = {}
        self.latest_state: Dict[str, Any] = {}
        self.latest_scan: Optional[LaserScan] = None
        self._last_process_time = 0.0
        self._published_frames = 0
        self._dropped_frames = 0

        foxglove_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, foxglove_qos)
        self.compressed_pub = self.create_publisher(CompressedImage, self.compressed_topic, foxglove_qos)
        self.hud_pub = self.create_publisher(String, self.hud_topic, 10)

        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, self.json_topic, self._json_cb, 10)
        self.create_subscription(String, self.state_topic, self._state_cb, 10)

        self.create_timer(1.0 / max(self.marker_hz, 1.0), self._publish_markers)
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info("===== qwen_api_lidar_foxglove_viz =====")
        self.get_logger().info(f"image in={self.image_topic} out={self.compressed_topic}")
        self.get_logger().info(f"markers={self.markers_topic} frame_id={self.frame_id}")
        self.get_logger().info(f"qwen json={self.json_topic} state={self.state_topic}")

    def _merged(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        out.update(self.latest_json)
        out.update(self.latest_state)
        return out

    def _json_cb(self, msg: String) -> None:
        try:
            self.latest_json = json.loads(msg.data)
        except Exception:
            pass
        self._maybe_publish_image()

    def _state_cb(self, msg: String) -> None:
        try:
            self.latest_state = json.loads(msg.data)
        except Exception:
            pass
        self._maybe_publish_image()

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    def _image_cb(self, msg: Image) -> None:
        if not self._should_process_frame():
            return
        if msg.width and msg.height:
            self.image_width = int(msg.width)
            self.image_height = int(msg.height)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge failed: {exc}")
            return
        self.latest_frame = self._resize_for_viz(frame)
        self.latest_header = msg.header
        self._publish_debug_image()

    def _should_process_frame(self) -> bool:
        if self.viz_image_max_fps <= 0.0:
            return True
        now = time.time()
        if now - self._last_process_time < 1.0 / self.viz_image_max_fps:
            self._dropped_frames += 1
            return False
        self._last_process_time = now
        return True

    def _resize_for_viz(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        if self.viz_image_max_width > 0 and w > self.viz_image_max_width:
            nh = max(1, int(h * (float(self.viz_image_max_width) / float(w))))
            return cv2.resize(frame, (self.viz_image_max_width, nh), interpolation=cv2.INTER_AREA)
        return frame

    def _maybe_publish_image(self) -> None:
        if self.latest_frame is not None:
            self._publish_debug_image()

    def _output_header(self) -> Header:
        out = Header()
        out.stamp = self.get_clock().now().to_msg()
        if self.latest_header is not None and self.latest_header.frame_id:
            out.frame_id = self.latest_header.frame_id
        else:
            out.frame_id = "usb_camera"
        return out

    def _build_debug_image(self) -> Optional[np.ndarray]:
        if self.latest_frame is None:
            return None
        vis = self.latest_frame.copy()
        h, w = vis.shape[:2]
        scale_x = float(w) / max(1.0, float(self.image_width))
        scale_y = float(h) / max(1.0, float(self.image_height))
        data = self._merged()
        phase = str(data.get("phase", ""))
        point_kind = str(data.get("point_kind", ""))

        def px_u(u_val: Any) -> Optional[int]:
            if u_val is None:
                return None
            return int(float(u_val) * scale_x)

        def px_v(v_val: Any) -> Optional[int]:
            if v_val is None:
                return None
            return int(float(v_val) * scale_y)

        filtered_u = data.get("filtered_u", data.get("u"))
        raw_u = data.get("raw_u")
        v = data.get("v")
        path_point = data.get("path_point")
        path_locked = (
            (
                bool(data.get("path_point_locked"))
                or phase in ("EXPLORE_ALIGN", "EXPLORE_FORWARD")
            )
            and isinstance(path_point, (list, tuple))
            and len(path_point) >= 2
            and path_point[0] is not None
            and path_point[1] is not None
        )

        cx = int(w / 2)
        cy = int(h / 2)

        if path_locked:
            pu, pv = px_u(path_point[0]), px_v(path_point[1])
            if pu is not None and pv is not None:
                # Locked Qwen path waypoint: orange cross only (no line to center).
                cv2.drawMarker(vis, (pu, pv), (0, 165, 255), cv2.MARKER_CROSS, 28, 2)
                cv2.circle(vis, (pu, pv), 10, (0, 165, 255), 1, cv2.LINE_AA)
        elif point_kind != "path":
            fu, fv = px_u(filtered_u), px_v(v)
            if fu is not None and fv is not None:
                cv2.drawMarker(vis, (fu, fv), (0, 0, 255), cv2.MARKER_CROSS, 28, 2)
                cv2.circle(vis, (fu, fv), 10, (0, 0, 255), 1, cv2.LINE_AA)

            ru = px_u(raw_u)
            if ru is not None and fv is not None and (fu is None or abs(ru - fu) > 2):
                cv2.drawMarker(vis, (ru, fv), (255, 128, 0), cv2.MARKER_TILTED_CROSS, 20, 2)

        cv2.drawMarker(vis, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 18, 1)

        deadband = float(_nested_get(self.cfg, "servo", "center_deadband", 0.10))
        band_px = int(deadband * w)
        cv2.rectangle(vis, (cx - band_px, 0), (cx + band_px, h - 1), (0, 255, 255), 1)

        lines = [
            f"action={data.get('action', data.get('state', 'n/a'))}  servo={data.get('servo_state', 'n/a')}",
            f"phase={phase or 'n/a'}  status={data.get('status', 'n/a')}  mode={data.get('qwen_mode', 'n/a')}  kind={point_kind or 'n/a'}",
            f"usable={data.get('usable')}  conf={_fmt(data.get('confidence'), 2)}  step={data.get('step', 'n/a')}",
        ]
        if path_locked:
            lines.append(
                f"path_locked=({_fmt(path_point[0], 1)}, {_fmt(path_point[1], 1)}) conf={_fmt(data.get('path_confidence'), 2)}"
            )
        else:
            lines.append(
                f"u raw={_fmt(raw_u, 1)} filt={_fmt(filtered_u, 1)} v={_fmt(v, 1)}  ex={_fmt(data.get('center_error'), 3)}"
            )
        lines.extend([
            f"reason={data.get('reason') or '-'}",
            f"coord_reason={data.get('coord_reason') or '-'}",
            f"front={_fmt(data.get('front_distance'))}m  target={_fmt(data.get('target_distance_fused', data.get('target_distance')))}m",
            f"target_raw={_fmt(data.get('target_distance_raw'))}m  arrive={_fmt(data.get('target_distance_arrive'))}m",
            f"vx={_fmt(data.get('cmd_vx'), 3)}  wz={_fmt(data.get('cmd_wz'), 3)}  latency={_fmt(data.get('latency_sec'), 2)}s",
        ])
        _draw_text_block(vis, lines)

        hud = {k: data.get(k) for k in (
            "step", "action", "servo_state", "status", "qwen_mode", "point_kind",
            "usable", "confidence", "reason", "coord_reason",
            "raw_u", "filtered_u", "u", "v", "center_error",
            "front_distance", "target_distance", "target_distance_raw",
            "target_distance_fused", "target_distance_arrive", "target_angle_deg",
            "cmd_vx", "cmd_wz", "latency_sec",
        )}
        self.hud_pub.publish(String(data=json.dumps(hud, ensure_ascii=False)))
        return vis

    def _publish_debug_image(self) -> None:
        debug = self._build_debug_image()
        if debug is None:
            return
        ok, encoded = cv2.imencode(
            ".jpg", debug, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.viz_jpeg_quality)]
        )
        if not ok:
            return
        out = CompressedImage()
        out.header = self._output_header()
        out.format = "jpeg"
        out.data = encoded.tobytes()
        self.compressed_pub.publish(out)
        self._published_frames += 1

    def _publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self._build_markers(stamp))

    def _build_markers(self, stamp) -> MarkerArray:
        arr = MarkerArray()
        mid = 0
        data = self._merged()

        def add(marker: Marker) -> None:
            nonlocal mid
            marker.header.stamp = stamp
            marker.header.frame_id = self.frame_id
            marker.id = mid
            mid += 1
            arr.markers.append(marker)

        del_m = Marker()
        del_m.action = Marker.DELETEALL
        add(del_m)

        for dist, color, ns in (
            (self.emergency_stop_distance, (1.0, 0.1, 0.1, 0.35), "safety_emergency"),
            (self.hard_stop_distance, (1.0, 0.5, 0.0, 0.28), "safety_hard"),
            (self.slow_distance, (1.0, 1.0, 0.0, 0.18), "safety_slow"),
            (self.arrive_distance, (0.2, 1.0, 0.2, 0.22), "safety_arrive"),
        ):
            m = Marker()
            m.ns = ns
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.02
            m.color = _color(*color)
            half = self.lidar_front_deg
            for deg in np.linspace(-half, half, 25):
                x, y = _polar_xy(deg, dist)
                m.points.append(Point(x=float(x), y=float(y), z=0.0))
            add(m)

        front = data.get("front_distance")
        if front is not None and float(front) > 0.0:
            m = Marker()
            m.ns = "front_hit"
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.04
            m.color = _color(0.1, 0.9, 1.0, 0.95)
            fd = float(front)
            for deg in (-self.lidar_front_deg, self.lidar_front_deg):
                x, y = _polar_xy(deg, fd)
                m.points.append(Point(x=0.0, y=0.0, z=0.0))
                m.points.append(Point(x=float(x), y=float(y), z=0.0))
            add(m)
            sm = Marker()
            sm.ns = "front_sphere"
            sm.type = Marker.SPHERE
            sm.action = Marker.ADD
            sm.pose.position.x = float(fd)
            sm.pose.position.y = 0.0
            sm.pose.position.z = 0.05
            sm.scale.x = sm.scale.y = sm.scale.z = 0.08
            sm.color = _color(0.1, 0.9, 1.0, 0.9)
            add(sm)

        target_angle = data.get("target_angle_deg")
        target_dist = data.get("target_distance_fused", data.get("target_distance"))
        arrive_dist = data.get("target_distance_arrive")
        if target_angle is not None and target_dist is not None and float(target_dist) > 0.0:
            ang = float(target_angle)
            td = float(target_dist)
            x, y = _polar_xy(ang, td)
            m = Marker()
            m.ns = "target_ray"
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.points.append(Point(x=0.0, y=0.0, z=0.0))
            m.points.append(Point(x=float(x), y=float(y), z=0.0))
            m.scale.x = 0.03
            m.scale.y = 0.06
            m.scale.z = 0.06
            m.color = _color(1.0, 0.2, 0.2, 0.95)
            add(m)
            sm = Marker()
            sm.ns = "target_hit"
            sm.type = Marker.SPHERE
            sm.action = Marker.ADD
            sm.pose.position.x = float(x)
            sm.pose.position.y = float(y)
            sm.pose.position.z = 0.06
            sm.scale.x = sm.scale.y = sm.scale.z = 0.10
            sm.color = _color(1.0, 0.2, 0.2, 0.9)
            add(sm)

        if target_angle is not None and arrive_dist is not None and float(arrive_dist) > 0.0:
            ang = float(target_angle)
            ad = float(arrive_dist)
            x, y = _polar_xy(ang, ad)
            m = Marker()
            m.ns = "arrive_ray"
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.04
            m.scale.x = m.scale.y = m.scale.z = 0.07
            m.color = _color(0.2, 1.0, 0.3, 0.85)
            add(m)

        tm = Marker()
        tm.ns = "qwen_hud"
        tm.type = Marker.TEXT_VIEW_FACING
        tm.action = Marker.ADD
        tm.pose.position.x = 0.0
        tm.pose.position.y = 0.0
        tm.pose.position.z = 0.45
        tm.scale.z = 0.08
        tm.color = _color(1.0, 1.0, 1.0, 1.0)
        tm.text = (
            f"{data.get('action', data.get('state', '?'))} | "
            f"conf={_fmt(data.get('confidence'), 2)} | "
            f"front={_fmt(data.get('front_distance'))}m | "
            f"target={_fmt(target_dist)}m | "
            f"arrive={_fmt(arrive_dist)}m\n"
            f"u={_fmt(data.get('filtered_u', data.get('u')), 0)} "
            f"status={data.get('status', '-')} "
            f"{(data.get('reason') or '')[:40]}"
        )
        add(tm)

        return arr

    def _log_stats(self) -> None:
        if self._published_frames or self._dropped_frames:
            self.get_logger().info(
                f"viz frames published={self._published_frames} dropped={self._dropped_frames}"
            )
            self._published_frames = 0
            self._dropped_frames = 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Foxglove viz for Qwen API LiDAR nav")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    args = parser.parse_args()
    cfg = load_yaml(args.config)
    rclpy.init()
    node = QwenApiLidarFoxgloveViz(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
