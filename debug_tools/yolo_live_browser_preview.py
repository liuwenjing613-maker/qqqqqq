#!/usr/bin/env python3
"""
YOLO-World live browser preview.

功能：
- 订阅摄像头原图话题，默认 /image_raw
- 订阅 hobot_yolo_world 检测话题，默认 /hobot_yolo_world
- 绘制 raw candidates、MVP target、score、area_ratio、reject_reason
- 启动 MJPEG 网页服务，浏览器访问 http://<board_ip>:8088 实时观看
- 不发布 /cmd_vel，不控制底盘，只做视觉诊断
"""

import argparse
import sys
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import cv2
import rclpy
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception.multi_frame_voter import MultiFrameTargetVoter
from src.perception.stamp_sync import StampSyncBuffer
from src.perception.target_backend_yolo import (
    extract_yolo_target,
    list_yolo_candidates,
    parse_target_classes,
    _pick_best_rejected,
)


def _stamp_valid(stamp):
    return bool(stamp.sec or stamp.nanosec)


class YoloLiveBrowserPreview(Node):
    def __init__(
        self,
        image_topic="/image_raw",
        det_topic="/hobot_yolo_world",
        target_classes="bottle,cup",
        image_width=1280,
        image_height=720,
        min_score=0.002,
        raw_min_score=0.0,
        min_red_ratio=0.06,
        max_area_ratio=0.24,
        require_red_verify=False,
        show_all_boxes=True,
        sync_max_delta_sec=0.5,
        sync_buffer_len=80,
        jpeg_quality=80,
        publish_fps=15.0,
    ):
        super().__init__("yolo_live_browser_preview")

        self.image_topic = image_topic
        self.det_topic = det_topic
        self.target_classes = parse_target_classes(target_classes)
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.min_score = float(min_score)
        self.raw_min_score = float(raw_min_score)
        self.min_red_ratio = float(min_red_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.require_red_verify = bool(require_red_verify)
        self.show_all_boxes = bool(show_all_boxes)
        self.sync_max_delta_sec = float(sync_max_delta_sec)
        self.jpeg_quality = int(jpeg_quality)
        self.publish_period = 1.0 / max(1.0, float(publish_fps))

        self.bridge = CvBridge()
        self.frame_buffer = StampSyncBuffer(max_len=sync_buffer_len, max_delta_sec=self.sync_max_delta_sec)

        self.latest_frame = None
        self.latest_det_msg = None
        self.latest_jpeg = None
        self.lock = threading.Lock()

        self.frame_count = 0
        self.det_count = 0
        self.mvp_found_count = 0
        self.last_render_time = 0.0
        self.last_log_time = 0.0
        self.last_status = "WAITING"
        self.last_raw_count = 0
        self.last_reject_reason = ""

        self.target_voter = MultiFrameTargetVoter(
            window_size=6,
            min_votes=2,
            lost_hold_frames=1,
            iou_threshold=0.05,
            center_dist_threshold=0.35,
            smooth_alpha=0.20,
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

        self.get_logger().info("===== yolo_live_browser_preview =====")
        self.get_logger().info(f"image_topic={self.image_topic}")
        self.get_logger().info(f"det_topic={self.det_topic}")
        self.get_logger().info(f"target_classes={self.target_classes}")
        self.get_logger().info(
            f"min_score={self.min_score} raw_min_score={self.raw_min_score} "
            f"max_area_ratio={self.max_area_ratio}"
        )
        self.get_logger().info(
            f"require_red_verify={self.require_red_verify} min_red_ratio={self.min_red_ratio}"
        )
        self.get_logger().info(
            f"sync_max_delta_sec={self.sync_max_delta_sec} show_all_boxes={self.show_all_boxes}"
        )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge image failed: {repr(exc)}")
            return

        self.frame_count += 1
        self.latest_frame = frame
        if _stamp_valid(msg.header.stamp):
            self.frame_buffer.push(msg.header.stamp, frame)

        now = time.time()
        if self.latest_det_msg is None and now - self.last_render_time >= self.publish_period:
            self.last_render_time = now
            self._render_and_store_jpeg(frame, None, None, [], status="NO_DET")

    def det_callback(self, msg: PerceptionTargets):
        self.det_count += 1
        self.latest_det_msg = msg

        frame = None
        if _stamp_valid(msg.header.stamp):
            frame, delta = self.frame_buffer.find_closest(msg.header.stamp)
            if frame is None:
                frame = self.latest_frame
                if time.time() - self.last_log_time > 1.0:
                    self.get_logger().warn(
                        f"stamp sync failed; fallback to latest_frame "
                        f"(frame_buf={len(self.frame_buffer)})"
                    )
        else:
            frame = self.latest_frame
            if time.time() - self.last_log_time > 1.0:
                self.get_logger().warn("[YOLO_DET] empty stamp; fallback to latest_frame")

        if frame is None:
            return

        now = time.time()
        if now - self.last_render_time < self.publish_period:
            return
        self.last_render_time = now

        raw_dets = []
        mvp_target = {"visible": False, "reason": "not_processed"}
        try:
            raw_dets = list_yolo_candidates(
                msg,
                target_classes=self.target_classes,
                image_width=self.image_width,
                image_height=self.image_height,
                min_score=self.raw_min_score,
                frame=frame,
                require_red_verify=False,
                max_area_ratio=self.max_area_ratio,
                min_area_ratio=0.0,
            )
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

            single_frame_target = mvp_target
            mvp_target = self.target_voter.update(mvp_target)
        except Exception as exc:
            self.get_logger().warn(f"process detections failed: {repr(exc)}")

        status = "MVP_FOUND" if mvp_target and mvp_target.get("visible", False) else "NO_MVP"
        if status == "MVP_FOUND":
            self.mvp_found_count += 1

        self.last_raw_count = len(raw_dets or [])
        self.last_status = status
        self.last_reject_reason = ""
        if mvp_target and not mvp_target.get("visible", False):
            self.last_reject_reason = str(mvp_target.get("reason", ""))

        if now - self.last_log_time >= 1.0:
            self.last_log_time = now
            summary = "none"
            if raw_dets:
                summary = ", ".join(
                    f"{d.get('class_name')}:{float(d.get('score', 0.0)):.4f}"
                    f"(area={float(d.get('area_ratio', 0.0)):.3f},"
                    f"rej={d.get('reject_reason') or 'ok'})"
                    for d in raw_dets[:5]
                )
            if status == "MVP_FOUND":
                self.get_logger().info(
                    f"[MVP_TARGET] class={mvp_target.get('class_name')} "
                    f"score={float(mvp_target.get('score', 0.0)):.4f} "
                    f"bbox={mvp_target.get('bbox')} "
                    f"area={float(mvp_target.get('area_ratio', 0.0)):.4f} "
                    f"vote={mvp_target.get('vote_count')}/{mvp_target.get('vote_window')} "
                    f"reason={mvp_target.get('reason')} "
                    f"raw={len(raw_dets or [])} [{summary}]"
                )
            else:
                self.get_logger().warn(
                    f"[MVP_REJECT] reason={mvp_target.get('reason')} "
                    f"vote={mvp_target.get('vote_count')}/{mvp_target.get('vote_window')} "
                    f"raw={len(raw_dets or [])} [{summary}]"
                )

        self._render_and_store_jpeg(frame, msg, mvp_target, raw_dets, status=status)

    def _render_and_store_jpeg(self, frame, det_msg, mvp_target, raw_dets, status):
        canvas = frame.copy()
        h, w = canvas.shape[:2]

        def draw_box(det, color, thickness, tag):
            try:
                x, y, bw, bh = [int(v) for v in det["bbox"]]
                x = max(0, min(x, w - 1))
                y = max(0, min(y, h - 1))
                bw = max(1, min(bw, w - x))
                bh = max(1, min(bh, h - y))
                cv2.rectangle(canvas, (x, y), (x + bw, y + bh), color, thickness)
                label = (
                    f"{tag} {det.get('class_name')} "
                    f"{float(det.get('score', 0.0)):.3f} "
                    f"a={float(det.get('area_ratio', 0.0)):.3f}"
                )
                if det.get("red_ratio") is not None:
                    label += f" r={float(det.get('red_ratio', 0.0)):.2f}"
                if det.get("reject_reason"):
                    label += f" {det.get('reject_reason')}"
                cv2.putText(
                    canvas,
                    label,
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )
            except Exception:
                return

        if self.show_all_boxes:
            for det in raw_dets or []:
                if det.get("area_ratio", 0.0) > self.max_area_ratio:
                    color = (0, 0, 255)
                    tag = "REJ_AREA"
                elif det.get("reject_reason"):
                    color = (0, 0, 255)
                    tag = "REJ"
                elif det.get("visible", True) and det.get("score", 0.0) >= self.min_score:
                    color = (0, 255, 255)
                    tag = "RAW"
                else:
                    color = (255, 0, 0)
                    tag = "LOW"
                draw_box(det, color, 1, tag)

        if mvp_target and mvp_target.get("visible", False):
            draw_box(mvp_target, (0, 255, 0), 3, "MVP")

        best_raw = "none"
        if raw_dets:
            best = _pick_best_rejected(raw_dets, self.max_area_ratio) or raw_dets[0]
            best_raw = (
                f"{best.get('class_name')} score={float(best.get('score', 0.0)):.4f} "
                f"area={float(best.get('area_ratio', 0.0)):.3f} "
                f"rej={best.get('reject_reason') or 'ok'}"
            )

        vote_text = ""
        stale_text = ""
        if mvp_target:
            vote_count = mvp_target.get("vote_count")
            vote_window = mvp_target.get("vote_window")
            vote_reason = mvp_target.get("vote_reason") or mvp_target.get("reason")

            if vote_count is not None and vote_window is not None:
                vote_text = f" vote={vote_count}/{vote_window} vote_reason={vote_reason}"
            stale_text = (
                f" stale={mvp_target.get('stale', False)} "
                f"source={mvp_target.get('source')}"
            )

        lines = [
            f"YOLO-World LIVE | status={status} raw={len(raw_dets or [])}{vote_text}{stale_text}",
            f"target_classes={','.join(self.target_classes) if self.target_classes else 'ALL'}",
            f"min={self.min_score} raw_min={self.raw_min_score} max_area={self.max_area_ratio}",
            f"frames={self.frame_count} dets={self.det_count} mvp_found={self.mvp_found_count}",
            f"best_raw={best_raw}",
        ]
        if status != "MVP_FOUND" and self.last_reject_reason:
            lines.append(f"reject={self.last_reject_reason}")

        self._draw_text_block(canvas, lines)

        ok, jpeg = cv2.imencode(
            ".jpg",
            canvas,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if ok:
            with self.lock:
                self.latest_jpeg = jpeg.tobytes()

    @staticmethod
    def _draw_text_block(img, lines, origin=(12, 12), line_height=24, font_scale=0.58):
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = 1
        max_width = 0
        for line in lines:
            (tw, _), _ = cv2.getTextSize(line, font, font_scale, thickness)
            max_width = max(max_width, tw)

        x0, y0 = origin
        panel_w = max_width + 24
        panel_h = line_height * len(lines) + 16

        overlay = img.copy()
        cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.58, img, 0.42, 0, img)

        y = y0 + 22
        for line in lines:
            cv2.putText(
                img,
                line,
                (x0 + 10, y),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )
            y += line_height

    def get_latest_jpeg(self):
        with self.lock:
            return self.latest_jpeg


def make_handler(node: YoloLiveBrowserPreview):
    class MJPEGHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/" or self.path.startswith("/index"):
                html = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>YOLO-World Live Preview</title>
  <style>
    body { margin: 0; background: #111; color: #eee; font-family: Arial, sans-serif; }
    header { padding: 12px 18px; background: #1e1e1e; border-bottom: 1px solid #333; }
    main { padding: 16px; }
    img { max-width: 100%; border: 2px solid #444; background: #000; }
    code { color: #9cdcfe; }
  </style>
</head>
<body>
  <header>
    <h2>YOLO-World Live Preview</h2>
    <div>Green = MVP target, Yellow = accepted raw, Red = rejected, Blue = low score.</div>
  </header>
  <main>
    <img src="/stream.mjpg" />
    <p>Stream endpoint: <code>/stream.mjpg</code></p>
  </main>
</body>
</html>
"""
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(html.encode("utf-8"))
                return

            if self.path.startswith("/stream.mjpg"):
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()

                while True:
                    jpeg = node.get_latest_jpeg()
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("utf-8"))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        time.sleep(node.publish_period)
                    except (BrokenPipeError, ConnectionResetError):
                        break
                return

            self.send_response(404)
            self.end_headers()

        def log_message(self, fmt, *args):
            return

    return MJPEGHandler


def run_http_server(node, host, port):
    server = HTTPServer((host, port), make_handler(node))
    node.get_logger().info(f"HTTP server listening on {host}:{port}")
    try:
        server.serve_forever()
    finally:
        server.server_close()


def main():
    parser = argparse.ArgumentParser(description="YOLO-World live browser preview.")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--det-topic", default="/hobot_yolo_world")
    parser.add_argument("--target-classes", default="bottle,cup")
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--min-score", type=float, default=0.002)
    parser.add_argument("--raw-min-score", type=float, default=0.0)
    parser.add_argument("--min-red-ratio", type=float, default=0.06)
    parser.add_argument("--max-area-ratio", type=float, default=0.24)
    parser.add_argument("--no-red-verify", action="store_true")
    parser.add_argument("--show-all-boxes", action="store_true")
    parser.add_argument("--sync-max-delta-sec", type=float, default=0.5)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--fps", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = YoloLiveBrowserPreview(
        image_topic=args.image_topic,
        det_topic=args.det_topic,
        target_classes=args.target_classes,
        image_width=args.image_width,
        image_height=args.image_height,
        min_score=args.min_score,
        raw_min_score=args.raw_min_score,
        min_red_ratio=args.min_red_ratio,
        max_area_ratio=args.max_area_ratio,
        require_red_verify=not args.no_red_verify,
        show_all_boxes=args.show_all_boxes,
        sync_max_delta_sec=args.sync_max_delta_sec,
        jpeg_quality=args.jpeg_quality,
        publish_fps=args.fps,
    )

    http_thread = threading.Thread(
        target=run_http_server,
        args=(node, args.host, args.port),
        daemon=True,
    )
    http_thread.start()

    node.get_logger().info(
        f"Open browser: http://<board_ip>:{args.port}  "
        f"(example: http://192.168.137.100:{args.port})"
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"summary: frames={node.frame_count} dets={node.det_count} "
            f"mvp_found={node.mvp_found_count}"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
