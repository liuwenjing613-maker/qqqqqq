#!/usr/bin/env python3
"""Publish USB camera frames as sensor_msgs/CompressedImage on /image.

Fallback for hobot_usb_cam which often advertises a publisher under a busy
DDS graph but never delivers frames (FastDDS SHM / buffer queue issues).

Stable profile (same as start_live_servo_voice / camera_stack):
  /dev/video0, MJPG, 640x480@15.
"""

from __future__ import annotations

import argparse
import time
from typing import Any, List, Union

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def _device_candidates(device: str) -> List[Union[str, int]]:
    """Prefer path, then V4L index (some OpenCV builds fail open-by-name)."""
    out: List[Union[str, int]] = [device]
    if device.startswith("/dev/video"):
        suffix = device[len("/dev/video") :]
        if suffix.isdigit():
            out.append(int(suffix))
    return out


def open_usb_capture(
    device: str,
    width: int,
    height: int,
    fps: float,
    retries: int = 5,
    retry_sleep_s: float = 0.4,
) -> Any:
    """Open V4L2 capture with short retries; configure MJPG before first read."""
    last_err = ""
    for attempt in range(1, retries + 1):
        for candidate in _device_candidates(device):
            cap = cv2.VideoCapture(candidate, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                last_err = f"open failed candidate={candidate!r}"
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
            cap.set(cv2.CAP_PROP_FPS, float(fps))
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap
            cap.release()
            last_err = f"opened but first read failed candidate={candidate!r}"
        time.sleep(retry_sleep_s)
        if attempt < retries:
            print(
                f"[opencv_cam] retry {attempt}/{retries} open {device}: {last_err}",
                flush=True,
            )
    raise RuntimeError(f"cannot open camera device {device} ({last_err})")


class OpenCvCompressedCam(Node):
    def __init__(
        self,
        cap: Any,
        device: str,
        topic: str,
        width: int,
        height: int,
        fps: float,
        jpeg_quality: int,
        frame_id: str,
    ) -> None:
        super().__init__("opencv_compressed_cam")
        self._pub = self.create_publisher(CompressedImage, topic, qos_profile_sensor_data)
        self._cap = cap
        self._jpeg_quality = int(jpeg_quality)
        self._frame_id = frame_id
        self._period = 1.0 / max(fps, 1.0)
        self._last_log = 0.0
        self._ok = 0
        self._fail = 0
        self.create_timer(self._period, self._tick)
        self.get_logger().info(
            f"opencv cam -> {topic}: {device} target={width}x{height}@{fps} jpeg_q={jpeg_quality}"
        )

    def _tick(self) -> None:
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self._fail += 1
            return
        ok_enc, buf = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        )
        if not ok_enc:
            self._fail += 1
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self._pub.publish(msg)
        self._ok += 1
        now = time.monotonic()
        if now - self._last_log >= 2.0:
            self._last_log = now
            self.get_logger().info(
                f"published {self._ok} frames (fail={self._fail}) shape={frame.shape[1]}x{frame.shape[0]}"
            )

    def destroy_node(self) -> bool:
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--topic", default="/image")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--frame-id", default="usb_camera")
    args = parser.parse_args()

    # Open V4L2 before rclpy/Node so a busy DDS graph cannot race device open.
    cap = open_usb_capture(args.device, args.width, args.height, args.fps)

    rclpy.init()
    node = OpenCvCompressedCam(
        cap=cap,
        device=args.device,
        topic=args.topic,
        width=args.width,
        height=args.height,
        fps=args.fps,
        jpeg_quality=args.jpeg_quality,
        frame_id=args.frame_id,
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
