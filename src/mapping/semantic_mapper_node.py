#!/usr/bin/env python3
"""Semantic mapping overlay node for joystick SLAM sessions."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.mapping.semantic_config import load_semantic_config
from src.mapping.semantic_object_filter import SemanticObjectFilter
from src.mapping.semantic_projection import (
    RobotPose,
    box_center_uv,
    estimate_lidar_range_median,
    laser_scan_to_dict,
    parse_bbox_xyxy,
    pixel_to_bearing_rad,
    project_object_xy,
)
from src.mapping.semantic_store import SemanticStore
from src.mapping.semantic_types import SemanticObservation, json_dumps

try:
    from tf2_ros import Buffer, TransformListener
    import tf_transformations
except ImportError:
    Buffer = None  # type: ignore
    TransformListener = None  # type: ignore
    tf_transformations = None  # type: ignore


def yaw_from_quaternion(q: Quaternion) -> float:
    if tf_transformations is not None:
        return float(tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2])
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SemanticMapperNode(Node):
    def __init__(self, config_path: str, map_name: str):
        super().__init__("semantic_mapper")
        self.cfg = load_semantic_config(config_path)
        self.frames = self.cfg["frames"]
        self.topics = self.cfg["topics"]
        self.map_name = map_name

        cam = self.cfg["camera"]
        self.object_filter = SemanticObjectFilter(
            self.cfg, image_width=cam["width"], image_height=cam["height"]
        )
        self.store = SemanticStore(self.cfg, map_name=map_name)

        self.latest_scan: Optional[Dict[str, Any]] = None
        self.latest_image: Optional[Image] = None
        self.latest_odom: Optional[Odometry] = None
        self.map_info: Optional[Dict[str, Any]] = None

        if Buffer is not None:
            self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None
            self.get_logger().warn("tf2_ros unavailable; pose lookup disabled")

        qos = 10
        foxglove_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.sub_bbox = self.create_subscription(
            String, self.topics.get("bbox_json", "/target_bbox_json"), self.on_bbox, qos
        )
        self.sub_scan = self.create_subscription(
            LaserScan, self.topics.get("scan", "/scan_filtered"), self.on_scan, qos
        )
        scan_fb = self.topics.get("scan_fallback", "/scan")
        if scan_fb and scan_fb != self.topics.get("scan"):
            self.sub_scan_fb = self.create_subscription(LaserScan, scan_fb, self.on_scan, qos)

        self.sub_image = self.create_subscription(
            Image, self.topics.get("image_raw", "/image_raw"), self.on_image, qos
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, self.topics.get("map", "/map"), self.on_map, 1
        )
        self.sub_odom = self.create_subscription(
            Odometry, self.topics.get("odom", "/odom"), self.on_odom, qos
        )
        save_topic = self.topics.get("save_trigger", "/semantic_map/save")
        self.sub_save = self.create_subscription(String, save_topic, self.on_save, qos)

        self.pub_json = self.create_publisher(
            String, self.topics.get("semantic_map_json", "/semantic_map_json"), qos
        )
        self.pub_status = self.create_publisher(
            String, self.topics.get("semantic_status", "/semantic_status"), qos
        )
        self.pub_obs = self.create_publisher(
            String, self.topics.get("semantic_observations", "/semantic_observations"), qos
        )
        self.pub_landmarks = self.create_publisher(
            MarkerArray, self.topics.get("semantic_landmarks", "/semantic_landmarks"), foxglove_qos
        )
        self.pub_viewpoints = self.create_publisher(
            MarkerArray, self.topics.get("semantic_viewpoints", "/semantic_viewpoints"), foxglove_qos
        )
        self.pub_cones = self.create_publisher(
            MarkerArray, self.topics.get("semantic_observed_cones", "/semantic_observed_cones"), foxglove_qos
        )
        self.pub_loop = self.create_publisher(
            String, self.topics.get("semantic_loop_error", "/semantic_loop_error"), qos
        )
        self.pub_save_event = self.create_publisher(
            String, self.topics.get("semantic_save_event", "/semantic_save_event"), qos
        )

        self._last_marker_pub = 0.0
        self._marker_pub_min_interval = 0.25  # max ~4 Hz markers over Foxglove

        autosave = float(self.cfg.get("storage", {}).get("autosave_sec", 2.0))
        if autosave > 0:
            self.create_timer(autosave, self._autosave_cb)

        self.get_logger().info(f"semantic_mapper session={self.store.session_dir}")
        self._publish_markers(force=True)

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = laser_scan_to_dict(msg)

    def on_image(self, msg: Image) -> None:
        self.latest_image = msg

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_info = {
            "resolution": float(msg.info.resolution),
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "origin": [
                float(msg.info.origin.position.x),
                float(msg.info.origin.position.y),
            ],
        }

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        speed = math.hypot(
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z),
        )
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = RobotPose(
            frame_id=self.frames.get("odom_frame", "odom"),
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw_from_quaternion(msg.pose.pose.orientation),
            provisional=True,
        )
        map_pose = self.lookup_robot_pose()
        if map_pose is not None:
            self.store.update_pose(map_pose, stamp, odom_speed=speed)
            self._publish_loop_error()

    def lookup_robot_pose(self) -> Optional[RobotPose]:
        if self.tf_buffer is None:
            if self.latest_odom is not None:
                msg = self.latest_odom
                return RobotPose(
                    frame_id=self.frames.get("fallback_frame", "odom"),
                    x=float(msg.pose.pose.position.x),
                    y=float(msg.pose.pose.position.y),
                    yaw=yaw_from_quaternion(msg.pose.pose.orientation),
                    provisional=True,
                )
            return None

        fixed = self.frames.get("fixed_frame", "map")
        fallback = self.frames.get("fallback_frame", "odom")
        base = self.frames.get("base_frame", "base_link")

        for frame_id, provisional in ((fixed, False), (fallback, True)):
            try:
                tf = self.tf_buffer.lookup_transform(
                    frame_id,
                    base,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                t = tf.transform.translation
                q = tf.transform.rotation
                return RobotPose(
                    frame_id=frame_id,
                    x=float(t.x),
                    y=float(t.y),
                    yaw=yaw_from_quaternion(q),
                    provisional=provisional,
                )
            except Exception:
                continue
        return None

    def on_bbox(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        image_w = int(data.get("image_width") or self.cfg["camera"]["width"])
        image_h = int(data.get("image_height") or self.cfg["camera"]["height"])
        self.object_filter.set_image_size(image_w, image_h)

        boxes = data.get("boxes") or []
        if not boxes and data.get("visible"):
            boxes = [data]

        pose = self.lookup_robot_pose()
        if pose is None:
            self._publish_status({"reason": "no_pose"})
            return

        stamp = time.time()
        self.store.update_pose(pose, stamp)

        cam = self.cfg["camera"]
        proj = self.cfg["projection"]
        observed_classes: List[str] = []
        keyframe_path: Optional[str] = None

        for box in boxes:
            bbox_xyxy = parse_bbox_xyxy(box)
            if bbox_xyxy is None:
                continue
            u, v = box_center_uv(box, bbox_xyxy)
            area_ratio = float(
                box.get("area_ratio")
                or ((bbox_xyxy[2] - bbox_xyxy[0]) * (bbox_xyxy[3] - bbox_xyxy[1]))
                / max(image_w * image_h, 1)
            )

            track = self.object_filter.update(box, bbox_xyxy, u, v, area_ratio, stamp)
            if track is None:
                continue
            if not track.is_candidate:
                continue

            class_name = track.class_name
            observed_classes.append(class_name)

            bearing_rad = pixel_to_bearing_rad(
                u, image_w, cam["hfov_deg"], cam["yaw_offset_deg"]
            )
            range_m = estimate_lidar_range_median(
                self.latest_scan,
                bearing_rad,
                min_range_m=proj["min_range_m"],
                max_range_m=proj["max_range_m"],
                target_window_deg=proj["target_window_deg"],
            )

            object_x = object_y = None
            range_source = "none"
            if range_m is not None:
                object_x, object_y = project_object_xy(
                    pose.x, pose.y, pose.yaw, bearing_rad, range_m
                )
                range_source = "lidar_median"

            quality = "confirmed_input" if track.is_confirmed else "voted"
            if track.is_dynamic:
                quality = "raw"

            obs = SemanticObservation(
                obs_id=self.store._next_obs_id(),
                stamp=stamp,
                fixed_frame=pose.frame_id,
                robot_x=pose.x,
                robot_y=pose.y,
                robot_yaw=pose.yaw,
                class_name=class_name,
                score=track.score,
                bbox_xyxy=bbox_xyxy,
                u=u,
                v=v,
                area_ratio=area_ratio,
                bearing_rad=bearing_rad,
                range_m=range_m,
                object_x=object_x,
                object_y=object_y,
                position_sigma=proj["default_position_sigma_m"] if range_m is None else 0.35,
                source=str(data.get("source", "yolov5s_bpu")),
                quality=quality,
                range_source=range_source,
                raw_json=box,
            )
            self.store.add_observation(obs)
            self.pub_obs.publish(String(data=json_dumps(obs.to_dict())))

            if (
                self.object_filter.can_landmark(track)
                and range_m is not None
                and not track.is_dynamic
            ):
                classes = self.cfg.get("classes", {})
                self.store.fuse_landmark(
                    obs,
                    small_objects=set(classes.get("small_objects", [])),
                    large_objects=set(classes.get("large_objects", [])),
                )

        if observed_classes:
            vp = self.store.maybe_add_viewpoint(pose, observed_classes, stamp, keyframe_path)
            if vp and self.store.save_keyframes and self.latest_image is not None:
                self._save_keyframe(vp.node_id)

        self.publish_json_and_markers()
        self.store.maybe_autosave()

    def _save_keyframe(self, vp_id: str) -> None:
        try:
            import cv2
            from cv_bridge import CvBridge

            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
            path = self.store.keyframe_path(vp_id)
            quality = int(self.cfg.get("storage", {}).get("save_keyframe_jpeg_quality", 70))
            cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        except Exception as exc:
            self.get_logger().debug(f"keyframe save skipped: {exc}")

    def publish_json_and_markers(self) -> None:
        payload = self.store.build_semantic_map_json()
        self.pub_json.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self._publish_status(
            {
                "landmarks": len(self.store.landmarks),
                "viewpoints": len(self.store.viewpoints),
                "observations": len(self.store.observations),
            }
        )
        if self.cfg.get("visualization", {}).get("publish_markers", True):
            self._publish_markers()

    def _publish_markers(self, force: bool = False) -> None:
        now = time.time()
        if not force and now - self._last_marker_pub < self._marker_pub_min_interval:
            return
        self._last_marker_pub = now

        frame_id = self.frames.get("fixed_frame", "map")
        lifetime = float(self.cfg.get("visualization", {}).get("marker_lifetime_sec", 0.0))
        dur = rclpy.duration.Duration(seconds=lifetime) if lifetime > 0 else rclpy.duration.Duration(seconds=0)

        lm_markers = MarkerArray()
        for i, lm in enumerate(self.store.landmarks.values()):
            m = Marker()
            m.header.frame_id = lm.frame_id or frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "semantic_landmarks"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = lm.x
            m.pose.position.y = lm.y
            m.pose.position.z = 0.15
            m.scale.x = m.scale.y = m.scale.z = 0.18
            color = ColorRGBA(r=0.2, g=0.8, b=0.3, a=0.9)
            if lm.state != "confirmed":
                color = ColorRGBA(r=0.9, g=0.7, b=0.1, a=0.7)
            m.color = color
            if lifetime > 0:
                m.lifetime = dur.to_msg()
            lm_markers.markers.append(m)

            if self.cfg.get("visualization", {}).get("landmark_text", True):
                t = Marker()
                t.header = m.header
                t.ns = "semantic_landmark_labels"
                t.id = i
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = lm.x
                t.pose.position.y = lm.y
                t.pose.position.z = 0.35
                t.text = f"{lm.class_name} ({lm.state})"
                t.scale.z = 0.12
                t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
                if lifetime > 0:
                    t.lifetime = dur.to_msg()
                lm_markers.markers.append(t)
        self.pub_landmarks.publish(lm_markers)

        vp_markers = MarkerArray()
        for i, vp in enumerate(self.store.viewpoints):
            m = Marker()
            m.header.frame_id = vp.frame_id or frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "semantic_viewpoints"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = vp.x
            m.pose.position.y = vp.y
            m.pose.position.z = 0.05
            m.pose.orientation = self._yaw_to_quaternion(vp.yaw)
            m.scale.x = 0.35
            m.scale.y = 0.08
            m.scale.z = 0.08
            m.color = ColorRGBA(r=0.3, g=0.5, b=1.0, a=0.8)
            if lifetime > 0:
                m.lifetime = dur.to_msg()
            vp_markers.markers.append(m)
        self.pub_viewpoints.publish(vp_markers)

        if self.cfg.get("visualization", {}).get("observed_cone_markers", True):
            cones = MarkerArray()
            for i, obs in enumerate(self.store.observations[-12:]):
                if obs.range_m is None:
                    continue
                m = Marker()
                m.header.frame_id = obs.fixed_frame or frame_id
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "semantic_observed_cones"
                m.id = i
                m.type = Marker.LINE_LIST
                m.action = Marker.ADD
                m.pose.orientation.w = 1.0
                m.scale.x = 0.03
                m.color = ColorRGBA(r=0.9, g=0.4, b=0.1, a=0.5)
                p0 = Point(x=obs.robot_x, y=obs.robot_y, z=0.05)
                if obs.object_x is not None and obs.object_y is not None:
                    p1 = Point(x=obs.object_x, y=obs.object_y, z=0.05)
                else:
                    import math as _m

                    p1 = Point(
                        x=obs.robot_x + obs.range_m * _m.cos(obs.robot_yaw + obs.bearing_rad),
                        y=obs.robot_y + obs.range_m * _m.sin(obs.robot_yaw + obs.bearing_rad),
                        z=0.05,
                    )
                m.points = [p0, p1]
                if lifetime > 0:
                    m.lifetime = dur.to_msg()
                cones.markers.append(m)
            self.pub_cones.publish(cones)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _publish_status(self, extra: Dict[str, Any]) -> None:
        status = {
            "session_id": self.store.session_id,
            "session_dir": self.store.session_dir,
            "map_name": self.map_name,
            **extra,
        }
        self.pub_status.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def _publish_loop_error(self) -> None:
        if not self.store.loop_quality:
            return
        self.pub_loop.publish(String(data=json.dumps(self.store.loop_quality, ensure_ascii=False)))

    def _autosave_cb(self) -> None:
        self.store.maybe_autosave()

    def on_save(self, msg: String) -> None:
        path = self.store.save_all(final=True)
        self.get_logger().info(f"semantic map saved: {path}")
        self.pub_save_event.publish(
            String(data=json.dumps({"event": "saved", "path": path}, ensure_ascii=False))
        )
        self.publish_json_and_markers()


def parse_args():
    ap = argparse.ArgumentParser(description="Semantic mapper overlay for joystick SLAM")
    ap.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs", "semantic_mapping.yaml"))
    ap.add_argument("--map-name", default="joy_semantic_calibrated_map")
    return ap.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = SemanticMapperNode(args.config, args.map_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.store.save_all(final=True)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
