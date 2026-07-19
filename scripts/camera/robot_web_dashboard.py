#!/usr/bin/env python3
"""ROS camera web stream — subscribes to /qwen_vln/annotated_image/compressed only.

Does NOT open /dev/video0. Requires Qwen node publishing annotated frames.

Open: http://<board_ip>:8090/
"""
from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

DEFAULT_TOPIC = "/qwen_vln/annotated_image/compressed"

INDEX_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Qwen 标注相机</title>
  <style>
  * { box-sizing: border-box; }
  body {
    margin: 0;
    min-height: 100vh;
    font-family: system-ui, sans-serif;
    background: #0f1115;
    color: #e8eaed;
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 20px 16px 32px;
  }
  h1 { margin: 0 0 6px; font-size: 1.25rem; }
  .meta { color: #9aa0a6; font-size: 0.88rem; margin-bottom: 16px; }
  .viewer {
    width: min(100%, 1280px);
    background: #1a1d23;
    border: 1px solid #2d3139;
    border-radius: 12px;
    overflow: hidden;
  }
  img { display: block; width: 100%; height: auto; background: #000; }
  code { background: #1f2937; padding: 2px 6px; border-radius: 4px; }
  </style>
</head>
<body>
  <h1>Qwen 标注相机</h1>
  <div class="meta">话题 <code>/qwen_vln/annotated_image/compressed</code> · 不占用 USB 相机</div>
  <div class="viewer">
    <img src="/stream.mjpg" alt="annotated camera">
  </div>
</body>
</html>
"""


class AnnotatedCameraStream(Node):
    def __init__(self, topic: str, jpeg_quality: int, fps: float, max_width: int) -> None:
        super().__init__("robot_web_dashboard")
        self._topic = topic
        self._jpeg_quality = int(np.clip(jpeg_quality, 40, 95))
        self._period = 1.0 / max(1.0, float(fps))
        self._max_width = max(0, int(max_width))
        self._lock = threading.Lock()
        self._latest_jpeg: Optional[bytes] = None
        self._frame_count = 0
        self._last_pub = 0.0

        self.create_subscription(CompressedImage, topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(f"subscribed {topic} (no /dev/video0)")

    def _on_image(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        if now - self._last_pub < self._period:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        h, w = frame.shape[:2]
        if self._max_width > 0 and w > self._max_width:
            scale = self._max_width / w
            frame = cv2.resize(frame, (self._max_width, max(1, int(h * scale))))
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            return
        with self._lock:
            self._latest_jpeg = buf.tobytes()
            self._frame_count += 1
            self._last_pub = now

    def get_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_jpeg


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def make_handler(node: AnnotatedCameraStream, period: float):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                body = INDEX_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/health":
                body = b'{"ok":true}\n'
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path != "/stream.mjpg":
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                jpeg = node.get_jpeg()
                if jpeg is None:
                    time.sleep(0.05)
                    continue
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    time.sleep(period)
                except (BrokenPipeError, ConnectionResetError):
                    break

        def log_message(self, fmt, *args):
            return

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="Qwen annotated camera web stream (ROS only)")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--max-width", type=int, default=1280, help="0 = full resolution")
    args = parser.parse_args()

    rclpy.init()
    node = AnnotatedCameraStream(
        topic=args.topic,
        jpeg_quality=args.jpeg_quality,
        fps=args.fps,
        max_width=args.max_width,
    )
    server = ThreadingHTTPServer((args.host, args.port), make_handler(node, node._period))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"http://0.0.0.0:{args.port}/  topic={args.topic}")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
