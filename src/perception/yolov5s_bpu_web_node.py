#!/usr/bin/env python3
import os
import sys
import cv2
import json
import time
import argparse
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.nav.nav_video_overlay import NavOverlayContext, annotate_nav_frame, safe_json_load


COCO_NAMES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
    "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
    "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
    "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch",
    "potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone",
    "microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush"
]


def parse_target_classes(text):
    text = (text or "").lower()
    out = set()

    # 直接 COCO 类别
    for part in text.replace(";", ",").split(","):
        p = part.strip()
        if p in COCO_NAMES:
            out.add(p)

    # 简单自然语言映射
    if any(k in text for k in ["bottle", "water", "drink", "drinking"]):
        out.add("bottle")
    if any(k in text for k in ["cup", "mug", "glass"]):
        out.add("cup")
    if any(k in text for k in ["bag", "backpack", "schoolbag", "school bag"]):
        out.add("backpack")
    if "book" in text:
        out.add("book")
    if "chair" in text:
        out.add("chair")
    if any(k in text for k in ["table", "desk", "dining table"]):
        out.add("dining table")

    return out


class SharedFrame:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.status = {
            "message": "waiting for first frame",
            "fps": 0,
            "target": "",
            "score": 0,
            "infer_ms": 0,
        }


shared = SharedFrame()


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            html = b"""
            <html>
            <head>
              <title>YOLOv5s-BPU Viewer</title>
              <style>
                body { font-family: Arial, sans-serif; background:#111; color:#eee; margin:20px; }
                img { max-width: 100%; border: 2px solid #444; }
                .box { margin-bottom: 12px; }
                code { background:#222; padding:2px 5px; }
              </style>
            </head>
            <body>
              <h2>YOLOv5s-BPU Realtime Viewer</h2>
              <div class="box">Stream: <code>/stream.mjpg</code></div>
              <img src="/stream.mjpg">
            </body>
            </html>
            """
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
            return

        if self.path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        while True:
            with shared.lock:
                jpeg = shared.jpeg

            if jpeg is None:
                time.sleep(0.05)
                continue

            try:
                self.wfile.write(b"--frame\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                time.sleep(0.05)
            except BrokenPipeError:
                break
            except ConnectionResetError:
                break

    def log_message(self, fmt, *args):
        return


class Yolov5sBpuWebNode(Node):
    def __init__(self, args):
        super().__init__("yolov5s_bpu_web_node")
        self.args = args
        self.bridge = CvBridge()

        self.target_classes = parse_target_classes(args.target_classes)
        if not self.target_classes:
            self.target_classes = {"bottle", "cup"}

        self.last_infer_time = 0.0
        self.min_interval = 1.0 / max(0.1, args.max_hz)
        self.frame_count = 0
        self.fps_t0 = time.time()
        self.current_fps = 0.0
        self.last_infer_ms = 0.0
        self.overlay_ctx = NavOverlayContext()

        self.model = self.load_model(args)

        # foxglove_bridge subscribes with RELIABLE; match it for annotated overlay stream.
        foxglove_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.pub_json = self.create_publisher(String, args.out_topic, 10)
        self.pub_debug = self.create_publisher(String, args.debug_topic, 10)
        self.pub_img = self.create_publisher(CompressedImage, args.annotated_topic, foxglove_qos)

        self.sub_words = self.create_subscription(
            String, args.target_words_topic, self.on_target_words, 10
        )

        if args.input_type == "compressed":
            self.sub_img = self.create_subscription(
                CompressedImage, args.image_topic, self.on_compressed_image, 10
            )
            self.get_logger().info(f"subscribing compressed image: {args.image_topic}")
        else:
            self.sub_img = self.create_subscription(
                Image, args.image_topic, self.on_raw_image, 10
            )
            self.get_logger().info(f"subscribing raw image: {args.image_topic}")

        self.create_subscription(String, args.nav_state_topic, self.on_nav_state, 10)
        self.create_subscription(Twist, args.cmd_vel_topic, self.on_cmd_vel, 10)
        self.create_subscription(String, args.nav_point_topic, self.on_nav_point, 10)

        self.start_web_server(args.web_host, args.web_port)

        self.get_logger().info("===== YOLOv5s-BPU web node started =====")
        self.get_logger().info(f"model={args.model}")
        self.get_logger().info(f"target_classes={sorted(self.target_classes)}")
        self.get_logger().info(f"out_topic={args.out_topic}")
        self.get_logger().info(f"annotated_topic={args.annotated_topic}")
        self.get_logger().info(f"web=http://<RDK_IP>:{args.web_port}/")
        self.get_logger().info(
            f"nav overlay topics state={args.nav_state_topic} cmd={args.cmd_vel_topic}"
        )

    def on_nav_state(self, msg: String) -> None:
        self.overlay_ctx.update_nav_state(safe_json_load(msg.data))

    def on_cmd_vel(self, msg: Twist) -> None:
        self.overlay_ctx.update_cmd(float(msg.linear.x), float(msg.angular.z))

    def on_nav_point(self, msg: String) -> None:
        data = safe_json_load(msg.data)
        if data:
            self.overlay_ctx.target_point = data

    def _overlay_header(self) -> list[str]:
        return [
            f"YOLOv5s-BPU | targets={','.join(sorted(self.target_classes))} "
            f"| fps={self.current_fps:.1f} | infer={self.last_infer_ms:.1f}ms"
        ]

    def _apply_nav_overlay(self, frame, target_override=None):
        return annotate_nav_frame(
            frame,
            self.overlay_ctx,
            target_override=target_override,
            header_lines=self._overlay_header(),
        )

    def load_model(self, args):
        runtime_dir = os.path.abspath(args.runtime_dir)
        zoo_root = os.path.abspath(args.zoo_root)

        if not os.path.exists(runtime_dir):
            raise FileNotFoundError(f"runtime_dir not found: {runtime_dir}")
        if not os.path.exists(args.model):
            raise FileNotFoundError(f"model not found: {args.model}")

        old_cwd = os.getcwd()
        os.chdir(runtime_dir)
        sys.path.insert(0, runtime_dir)
        sys.path.insert(0, zoo_root)

        from yolov5_det import YOLOv5Config, YOLOv5Detect

        cfg = YOLOv5Config(
            model_path=args.model,
            classes_num=80,
            score_thres=args.score_thres,
            nms_thres=args.nms_thres,
            resize_type=args.resize_type,
        )
        model = YOLOv5Detect(cfg)
        model.set_scheduling_params(priority=args.priority, bpu_cores=args.bpu_cores)

        os.chdir(old_cwd)
        return model

    def start_web_server(self, host, port):
        def run():
            server = HTTPServer((host, port), MJPEGHandler)
            server.serve_forever()

        th = threading.Thread(target=run, daemon=True)
        th.start()

    def on_target_words(self, msg):
        new_targets = parse_target_classes(msg.data)
        if new_targets:
            self.target_classes = new_targets
            self.get_logger().info(f"target_classes updated: {sorted(self.target_classes)}")

    def on_raw_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"raw image convert failed: {e}")
            return
        self.process_frame(frame)

    def on_compressed_image(self, msg):
        import numpy as np
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception as e:
            self.get_logger().warn(f"compressed image decode failed: {e}")
            return
        self.process_frame(frame)

    def publish_empty(self, image_w, image_h, annotated, reason, infer_ms=0.0):
        data = {
            "timestamp": time.time(),
            "visible": False,
            "found": False,
            "source": "yolov5s_bpu",
            "reason": reason,
            "image_width": image_w,
            "image_height": image_h,
            "target_classes": sorted(self.target_classes),
            "score": 0.0,
            "boxes": [],
            "infer_ms": infer_ms,
        }
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_json.publish(msg)
        annotated = self._apply_nav_overlay(
            annotated,
            target_override={"visible": False, "reason": reason},
        )
        self.publish_annotated(annotated)

    def process_frame(self, frame):
        now = time.time()
        if now - self.last_infer_time < self.min_interval:
            return
        self.last_infer_time = now

        image_h, image_w = frame.shape[:2]

        t0 = time.time()
        try:
            dets = self.model.predict(frame)
        except Exception as e:
            self.get_logger().error(f"model.predict failed: {e}")
            return
        infer_ms = (time.time() - t0) * 1000.0
        self.last_infer_ms = infer_ms

        annotated = frame.copy()
        all_boxes = []
        candidates = []

        for cls_id, score, x1, y1, x2, y2 in dets:
            if cls_id < 0 or cls_id >= len(COCO_NAMES):
                continue

            name = COCO_NAMES[cls_id]

            x1 = max(0, min(int(x1), image_w - 1))
            y1 = max(0, min(int(y1), image_h - 1))
            x2 = max(0, min(int(x2), image_w - 1))
            y2 = max(0, min(int(y2), image_h - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            w = x2 - x1
            h = y2 - y1
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            area_ratio = (w * h) / float(image_w * image_h)

            item = {
                "class_name": name,
                "class_id": int(cls_id),
                "score": float(score),
                "bbox": [x1, y1, x2, y2],
                "bbox_xyxy": [x1, y1, x2, y2],
                "bbox_xywh": [x1, y1, w, h],
                "cx": cx,
                "cy": cy,
                "center": [cx, cy],
                "area_ratio": area_ratio,
            }
            all_boxes.append(item)

            is_target = name in self.target_classes
            color = (0, 255, 0) if is_target else (160, 160, 160)
            thick = 2 if is_target else 1
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thick)
            cv2.putText(
                annotated,
                f"{name} {score:.2f}",
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )

            if is_target:
                candidates.append(item)

        self.frame_count += 1
        t = time.time()
        if t - self.fps_t0 >= 1.0:
            self.current_fps = self.frame_count / (t - self.fps_t0)
            self.frame_count = 0
            self.fps_t0 = t

        if not candidates:
            self.publish_empty(image_w, image_h, annotated, "no_target_class_detected", infer_ms)
            self.publish_debug(None, infer_ms)
            return

        best = max(candidates, key=lambda x: x["score"])

        result = {
            "timestamp": time.time(),
            "visible": True,
            "found": True,
            "source": "yolov5s_bpu",
            "class_name": best["class_name"],
            "class_id": best["class_id"],
            "score": best["score"],
            "bbox": best["bbox"],
            "bbox_xyxy": best["bbox_xyxy"],
            "bbox_xywh": best["bbox_xywh"],
            "cx": best["cx"],
            "cy": best["cy"],
            "u": best["cx"],
            "v": best["cy"],
            "center": best["center"],
            "area_ratio": best["area_ratio"],
            "image_width": image_w,
            "image_height": image_h,
            "target_classes": sorted(self.target_classes),
            "boxes": all_boxes,
            "infer_ms": infer_ms,
            "fps": self.current_fps,
        }

        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.pub_json.publish(msg)

        self.publish_debug(best, infer_ms)
        target_override = {
            "visible": True,
            "class_name": best["class_name"],
            "score": best["score"],
            "area_ratio": best["area_ratio"],
            "bbox": best["bbox"],
            "bbox_xyxy": best["bbox_xyxy"],
            "u": best["cx"],
            "v": best["cy"],
        }
        annotated = self._apply_nav_overlay(annotated, target_override=target_override)
        self.publish_annotated(annotated)

    def publish_debug(self, best, infer_ms):
        data = {
            "source": "yolov5s_bpu",
            "target_classes": sorted(self.target_classes),
            "fps": self.current_fps,
            "infer_ms": infer_ms,
            "best": best,
        }
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_debug.publish(msg)

    def publish_annotated(self, image):
        ok, enc = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.jpeg_quality)]
        )
        if not ok:
            return

        jpeg = enc.tobytes()

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = jpeg
        self.pub_img.publish(msg)

        with shared.lock:
            shared.jpeg = jpeg
            shared.status = {
                "fps": self.current_fps,
                "target": ",".join(sorted(self.target_classes)),
            }


def parse_args():
    ap = argparse.ArgumentParser()

    ap.add_argument("--model", default="/root/rdk_model_zoo/samples/vision/yolov5/model/yolov5s_tag_v7.0_detect_640x640_bayese_nv12.bin")
    ap.add_argument("--runtime-dir", default="/root/rdk_model_zoo/samples/vision/yolov5/runtime/python")
    ap.add_argument("--zoo-root", default="/root/rdk_model_zoo")

    ap.add_argument("--input-type", choices=["raw", "compressed"], default="raw")
    ap.add_argument("--image-topic", default="/image_raw")

    ap.add_argument("--out-topic", default="/target_bbox_json")
    ap.add_argument("--debug-topic", default="/yolov5s_bpu/debug")
    ap.add_argument("--annotated-topic", default="/yolov5s_bpu/annotated/compressed")
    ap.add_argument("--target-words-topic", default="/target_words")

    ap.add_argument("--target-classes", default="bottle,cup")
    ap.add_argument("--score-thres", type=float, default=0.25)
    ap.add_argument("--nms-thres", type=float, default=0.45)
    ap.add_argument("--resize-type", type=int, default=0)

    ap.add_argument("--max-hz", type=float, default=6.0)
    ap.add_argument("--jpeg-quality", type=int, default=45)

    ap.add_argument("--priority", type=int, default=0)
    ap.add_argument("--bpu-cores", nargs="+", type=int, default=[0])

    ap.add_argument("--web-host", default="0.0.0.0")
    ap.add_argument("--web-port", type=int, default=8088)
    ap.add_argument("--nav-state-topic", default="/nav_state")
    ap.add_argument("--cmd-vel-topic", default="/cmd_vel")
    ap.add_argument("--nav-point-topic", default="/nav_target_point")

    return ap.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Yolov5sBpuWebNode(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
