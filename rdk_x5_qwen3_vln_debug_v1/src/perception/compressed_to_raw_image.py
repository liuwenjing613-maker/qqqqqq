#!/usr/bin/env python3
"""Decode the project's compressed camera topic into a reliable raw Image topic."""

import argparse
import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class CompressedToRawImage(Node):
    def __init__(self, in_topic: str, out_topic: str, frame_id: str,
                 max_fps: float, status_topic: str):
        super().__init__("qwen_compressed_to_raw_image")
        self.in_topic = in_topic
        self.out_topic = out_topic
        self.frame_id = frame_id
        self.max_fps = max(0.0, float(max_fps))
        self.bridge = CvBridge()
        self.last_pub_monotonic = 0.0
        self.window_start = time.monotonic()
        self.window_count = 0
        self.total_count = 0
        self.last_width = 0
        self.last_height = 0

        raw_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.raw_pub = self.create_publisher(Image, out_topic, raw_qos)
        self.status_pub = self.create_publisher(String, status_topic, 1)
        self.sub = self.create_subscription(
            CompressedImage, in_topic, self.callback, qos_profile_sensor_data
        )
        self.create_timer(2.0, self.publish_status)
        self.get_logger().info(
            f"bridge subscribe {in_topic} CompressedImage BEST_EFFORT -> "
            f"publish {out_topic} Image RELIABLE, max_fps={self.max_fps:g}"
        )

    def callback(self, msg: CompressedImage) -> None:
        try:
            now = time.monotonic()
            if self.max_fps > 0.0:
                min_interval = 1.0 / self.max_fps
                if now - self.last_pub_monotonic < min_interval:
                    return
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warning("imdecode returned None")
                return
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw_msg.header = msg.header
            if not raw_msg.header.frame_id:
                raw_msg.header.frame_id = self.frame_id
            self.raw_pub.publish(raw_msg)
            self.last_pub_monotonic = now
            self.window_count += 1
            self.total_count += 1
            self.last_height, self.last_width = frame.shape[:2]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"convert failed: {exc!r}")

    def publish_status(self) -> None:
        now = time.monotonic()
        elapsed = max(1e-6, now - self.window_start)
        fps = self.window_count / elapsed
        payload = {
            "status": "publishing" if self.total_count > 0 else "waiting_compressed_input",
            "input_topic": self.in_topic,
            "output_topic": self.out_topic,
            "encoding": "bgr8",
            "width": self.last_width,
            "height": self.last_height,
            "window_fps": round(fps, 2),
            "total_frames": self.total_count,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        if self.window_count > 0:
            self.get_logger().info(
                f"published {self.out_topic}: {self.last_width}x{self.last_height}, "
                f"approx_fps={fps:.2f}, total={self.total_count}"
            )
        self.window_start = now
        self.window_count = 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-topic", default="/image")
    parser.add_argument("--out-topic", default="/image_raw")
    parser.add_argument("--frame-id", default="usb_camera")
    parser.add_argument("--max-fps", type=float, default=8.0)
    parser.add_argument("--status-topic", default="/image_raw_bridge/status")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CompressedToRawImage(
        in_topic=args.in_topic,
        out_topic=args.out_topic,
        frame_id=args.frame_id,
        max_fps=args.max_fps,
        status_topic=args.status_topic,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
