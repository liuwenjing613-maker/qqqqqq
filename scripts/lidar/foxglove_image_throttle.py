#!/usr/bin/env python3
"""Republish /image -> /image_viz for Foxglove (throttle + optional downscale).

Keeps the full-rate /image for Qwen while Foxglove gets a cheap preview stream
so Wi-Fi + foxglove_bridge stop saturating.
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class FoxgloveImageThrottle(Node):
    def __init__(
        self,
        in_topic: str,
        out_topic: str,
        max_fps: float,
        max_width: int,
        jpeg_quality: int,
    ) -> None:
        super().__init__("foxglove_image_throttle")
        self._max_period = 1.0 / max(0.5, float(max_fps))
        self._max_width = max(160, int(max_width))
        self._jpeg_quality = int(np.clip(jpeg_quality, 20, 95))
        self._last_pub = 0.0
        self._in_count = 0
        self._out_count = 0
        self._pub = self.create_publisher(
            CompressedImage, out_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            CompressedImage,
            in_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"{in_topic} -> {out_topic} max_fps={max_fps:.1f} "
            f"max_width={self._max_width} jpeg_q={self._jpeg_quality}"
        )

    def _on_image(self, msg: CompressedImage) -> None:
        self._in_count += 1
        now = time.monotonic()
        if now - self._last_pub < self._max_period:
            return
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
            h, w = frame.shape[:2]
            if w > self._max_width:
                scale = self._max_width / float(w)
                frame = cv2.resize(
                    frame,
                    (self._max_width, max(1, int(round(h * scale)))),
                    interpolation=cv2.INTER_AREA,
                )
            ok, buf = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
            )
            if not ok:
                return
            out = CompressedImage()
            out.header = msg.header
            out.format = "jpeg"
            out.data = buf.tobytes()
            self._pub.publish(out)
            self._last_pub = now
            self._out_count += 1
            if self._out_count % 25 == 1:
                self.get_logger().info(
                    f"viz frames in={self._in_count} out={self._out_count} "
                    f"shape={frame.shape[1]}x{frame.shape[0]} "
                    f"jpeg_kb={len(out.data) / 1024.0:.1f}"
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"viz republish failed: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-topic", default="/image")
    parser.add_argument("--out-topic", default="/image_viz")
    parser.add_argument("--max-fps", type=float, default=5.0)
    parser.add_argument("--max-width", type=int, default=640)
    parser.add_argument("--jpeg-quality", type=int, default=50)
    args = parser.parse_args()

    rclpy.init()
    node = FoxgloveImageThrottle(
        in_topic=args.in_topic,
        out_topic=args.out_topic,
        max_fps=args.max_fps,
        max_width=args.max_width,
        jpeg_quality=args.jpeg_quality,
    )
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
