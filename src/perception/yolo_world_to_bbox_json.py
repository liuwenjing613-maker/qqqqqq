#!/usr/bin/env python3
"""
Bridge hobot_yolo_world detections to /target_bbox_json for P0 failsafe navigation.

Uses the same extract_yolo_target + MultiFrameTargetVoter pipeline as
debug_tools/yolo_live_browser_preview.py (aligned with start_yolo_live_preview.sh).
"""
import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.perception.multi_frame_voter import MultiFrameTargetVoter
from src.perception.stamp_sync import StampSyncBuffer
from src.perception.target_backend_yolo import extract_yolo_target, parse_target_classes


def _stamp_valid(stamp):
    return bool(stamp.sec or stamp.nanosec)


def _bbox_xywh_to_xyxy(bbox):
    x, y, w, h = [float(v) for v in bbox]
    return [x, y, x + w, y + h]


class YoloWorldToBBoxJson(Node):
    def __init__(
        self,
        image_topic="/image_raw",
        det_topic="/hobot_yolo_world",
        out_topic="/target_bbox_json",
        target_classes="bottle,cup",
        image_width=640,
        image_height=480,
        min_score=0.002,
        min_red_ratio=0.06,
        max_area_ratio=0.24,
        require_red_verify=False,
        sync_max_delta_sec=0.5,
        sync_buffer_len=80,
        publish_rate_hz=10.0,
    ):
        super().__init__("yolo_world_to_bbox_json")

        self.image_topic = image_topic
        self.det_topic = det_topic
        self.out_topic = out_topic
        self.target_classes = parse_target_classes(target_classes)
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.min_score = float(min_score)
        self.min_red_ratio = float(min_red_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.require_red_verify = bool(require_red_verify)
        self.sync_max_delta_sec = float(sync_max_delta_sec)
        self.publish_period = 1.0 / max(1.0, float(publish_rate_hz))

        self.bridge = CvBridge()
        self.frame_buffer = StampSyncBuffer(max_len=sync_buffer_len, max_delta_sec=self.sync_max_delta_sec)
        self.latest_frame = None
        self.latest_det_msg = None
        self.last_publish_time = 0.0
        self.det_count = 0
        self.publish_count = 0

        self.target_voter = MultiFrameTargetVoter(
            window_size=10,
            min_votes=3,
            lost_hold_frames=3,
            iou_threshold=0.20,
            center_dist_threshold=0.18,
            smooth_alpha=0.65,
            image_width=self.image_width,
            image_height=self.image_height,
        )

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos)
        self.det_sub = self.create_subscription(PerceptionTargets, self.det_topic, self.det_callback, qos)
        self.pub = self.create_publisher(String, self.out_topic, 10)

        self.get_logger().info("===== yolo_world_to_bbox_json =====")
        self.get_logger().info(f"image_topic={self.image_topic} det_topic={self.det_topic}")
        self.get_logger().info(f"out_topic={self.out_topic} target_classes={self.target_classes}")
        self.get_logger().info(
            f"min_score={self.min_score} max_area_ratio={self.max_area_ratio} "
            f"require_red_verify={self.require_red_verify}"
        )

    def image_callback(self, msg: Image):
        if msg.width and msg.height:
            self.image_width = int(msg.width)
            self.image_height = int(msg.height)
            self.target_voter.image_width = self.image_width
            self.target_voter.image_height = self.image_height

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge image failed: {repr(exc)}")
            return

        self.latest_frame = frame
        if _stamp_valid(msg.header.stamp):
            self.frame_buffer.push(msg.header.stamp, frame)

    def det_callback(self, msg: PerceptionTargets):
        self.det_count += 1
        self.latest_det_msg = msg

        now = time.time()
        if now - self.last_publish_time < self.publish_period:
            return

        frame = None
        if _stamp_valid(msg.header.stamp):
            frame, _ = self.frame_buffer.find_closest(msg.header.stamp)
        if frame is None:
            frame = self.latest_frame
        if frame is None:
            return

        try:
            mvp_target = extract_yolo_target(
                msg,
                target_classes=self.target_classes,
                image_width=self.image_width,
                image_height=self.image_height,
                min_score=self.min_score,
                max_area_ratio=self.max_area_ratio,
                frame=frame,
                min_red_ratio=self.min_red_ratio,
                require_red_verify=self.require_red_verify,
                min_red_iou=0.10,
            )
            mvp_target = self.target_voter.update(mvp_target)
        except Exception as exc:
            self.get_logger().warn(f"process detections failed: {repr(exc)}")
            return

        self.last_publish_time = now
        payload = self._target_to_json(mvp_target)
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(out)
        self.publish_count += 1

        if mvp_target.get("visible", False):
            self.get_logger().info(
                f"[BBOX_JSON] class={payload.get('class')} score={payload.get('score', 0.0):.4f} "
                f"bbox={payload.get('bbox')} vote={mvp_target.get('vote_count')}/{mvp_target.get('vote_window')}"
            )
        elif self.publish_count % 20 == 0:
            self.get_logger().info(
                f"[BBOX_JSON] lost reason={payload.get('reason')} "
                f"vote={mvp_target.get('vote_count')}/{mvp_target.get('vote_window')}"
            )

    def _target_to_json(self, target):
        if not target or not target.get("visible", False):
            return {
                "visible": False,
                "reason": str(target.get("reason", "not_visible") if target else "not_visible"),
                "vote_count": target.get("vote_count") if target else 0,
                "vote_window": target.get("vote_window") if target else 0,
            }

        bbox = target.get("bbox")
        if bbox and len(bbox) == 4:
            bbox_out = _bbox_xywh_to_xyxy(bbox)
        else:
            bbox_out = None

        return {
            "visible": True,
            "bbox": bbox_out,
            "score": float(target.get("score", 0.0)),
            "class": str(target.get("class_name", "")),
            "class_name": str(target.get("class_name", "")),
            "cx": float(target.get("cx", 0.0)),
            "cy": float(target.get("cy", 0.0)),
            "area_ratio": float(target.get("area_ratio", 0.0)),
            "vote_count": target.get("vote_count"),
            "vote_window": target.get("vote_window"),
            "reason": target.get("vote_reason", "ok"),
        }


def main():
    parser = argparse.ArgumentParser(description="Bridge /hobot_yolo_world -> /target_bbox_json")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--det-topic", default="/hobot_yolo_world")
    parser.add_argument("--out-topic", default="/target_bbox_json")
    parser.add_argument("--target-classes", default="bottle,cup")
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--image-height", type=int, default=480)
    parser.add_argument("--min-score", type=float, default=0.002)
    parser.add_argument("--min-red-ratio", type=float, default=0.06)
    parser.add_argument("--max-area-ratio", type=float, default=0.24)
    parser.add_argument("--require-red-verify", action="store_true")
    parser.add_argument("--sync-max-delta-sec", type=float, default=0.5)
    parser.add_argument("--publish-rate-hz", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = YoloWorldToBBoxJson(
        image_topic=args.image_topic,
        det_topic=args.det_topic,
        out_topic=args.out_topic,
        target_classes=args.target_classes,
        image_width=args.image_width,
        image_height=args.image_height,
        min_score=args.min_score,
        min_red_ratio=args.min_red_ratio,
        max_area_ratio=args.max_area_ratio,
        require_red_verify=args.require_red_verify,
        sync_max_delta_sec=args.sync_max_delta_sec,
        publish_rate_hz=args.publish_rate_hz,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"summary: dets={node.det_count} published={node.publish_count}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
