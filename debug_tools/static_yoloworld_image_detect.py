#!/usr/bin/env python3
"""Publish static images to hobot_yolo_world and save annotated results."""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import rclpy
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.perception.target_backend_yolo import list_yolo_candidates, parse_target_classes


class StaticYoloWorld(Node):
    def __init__(self, image_topic, words_topic, det_topic, target_words):
        super().__init__("static_yoloworld_image_detector")
        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, image_topic, 10)
        self.words_pub = self.create_publisher(String, words_topic, 10)
        self.det_sub = self.create_subscription(PerceptionTargets, det_topic, self.det_cb, 10)
        self.words_msg = String()
        self.words_msg.data = target_words
        self.pending_stamp = None
        self.latest_det = None

    def publish_words(self):
        self.words_pub.publish(self.words_msg)

    def publish_image(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "test_yoloworld"
        self.pending_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.latest_det = None
        self.img_pub.publish(msg)

    def det_cb(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.pending_stamp is not None and stamp == self.pending_stamp:
            self.latest_det = msg


def draw_text_panel(canvas, lines):
    h, w = canvas.shape[:2]
    line_h = 26
    panel_h = 14 + line_h * len(lines)
    overlay = canvas.copy()
    cv2.rectangle(overlay, (8, 8), (min(w - 1, 1180), 8 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, canvas, 0.4, 0, canvas)
    y = 32
    for line in lines:
        cv2.putText(
            canvas,
            line,
            (18, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y += line_h


def draw_result(frame, det_msg, out_path, image_name, target_words, target_classes, min_score):
    h, w = frame.shape[:2]
    canvas = frame.copy()
    if det_msg is None:
        candidates = []
    else:
        candidates = list_yolo_candidates(
            det_msg,
            target_classes=target_classes,
            image_width=w,
            image_height=h,
            min_score=0.0,
            frame=frame,
            require_red_verify=False,
            max_area_ratio=1.0,
            min_area_ratio=0.0,
        )

    kept = [d for d in candidates if d.get("score", 0.0) >= min_score]
    kept.sort(key=lambda d: d.get("score", 0.0), reverse=True)

    colors = [(0, 255, 0), (0, 255, 255), (255, 0, 0), (255, 0, 255), (0, 128, 255)]
    for idx, det in enumerate(kept):
        x, y, bw, bh = [int(v) for v in det["bbox"]]
        color = colors[idx % len(colors)]
        cv2.rectangle(canvas, (x, y), (x + bw, y + bh), color, 3 if idx == 0 else 2)
        label = (
            f"#{idx + 1} {det['class_name']} {det['score']:.3f} "
            f"area={det['area_ratio']:.3f}"
        )
        y_text = max(24, y - 8)
        cv2.rectangle(
            canvas,
            (x, y_text - 20),
            (min(w - 1, x + len(label) * 10 + 8), y_text + 4),
            (0, 0, 0),
            -1,
        )
        cv2.putText(
            canvas,
            label,
            (x + 4, y_text),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    summary = [
        f"YOLO-World static image: {image_name}",
        f"texts={target_words} threshold={min_score}",
        f"detections={len(kept)} raw_candidates={len(candidates)} size={w}x{h}",
    ]
    if kept:
        best = kept[0]
        summary.append(
            f"best={best['class_name']} score={best['score']:.4f} "
            f"bbox={best['bbox']} cx={best['cx']:.0f}"
        )
    else:
        summary.append("best=none")
    draw_text_panel(canvas, summary)

    cv2.imwrite(str(out_path), canvas)
    meta = {
        "image": image_name,
        "output": str(out_path),
        "texts": target_words,
        "threshold": min_score,
        "image_width": w,
        "image_height": h,
        "detections": [
            {
                key: det[key]
                for key in ("class_name", "score", "bbox", "cx", "cy", "area_ratio")
                if key in det
            }
            for det in kept
        ],
        "raw_candidate_count": len(candidates),
    }
    with open(str(out_path).replace(".jpg", ".json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    return meta


def run(args):
    target_classes = parse_target_classes(args.target_words)
    image_paths = [Path(p) for p in args.images]

    rclpy.init()
    node = StaticYoloWorld(
        args.image_topic,
        args.words_topic,
        args.det_topic,
        args.target_words,
    )
    try:
        settle_until = time.time() + args.settle_sec
        while time.time() < settle_until:
            node.publish_words()
            rclpy.spin_once(node, timeout_sec=0.05)

        for path in image_paths:
            frame = cv2.imread(str(path))
            if frame is None:
                raise RuntimeError(f"failed to read image: {path}")

            node.latest_det = None
            node.pending_stamp = None
            deadline = time.time() + args.timeout_sec
            last_sent = 0.0
            while time.time() < deadline and node.latest_det is None:
                now = time.time()
                if now - last_sent >= args.publish_interval_sec:
                    node.publish_words()
                    node.publish_image(frame)
                    last_sent = now
                rclpy.spin_once(node, timeout_sec=0.1)

            out_path = path.with_name(f"{path.stem}_yoloworld_result.jpg")
            meta = draw_result(
                frame,
                node.latest_det,
                out_path,
                path.name,
                args.target_words,
                target_classes,
                args.min_score,
            )
            print(f"SAVED {out_path} detections={len(meta['detections'])}")
            for det in meta["detections"][:8]:
                print(
                    f"  {det['class_name']} score={det['score']:.4f} "
                    f"bbox={det['bbox']} cx={det['cx']:.0f} area={det['area_ratio']:.3f}"
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+")
    parser.add_argument("--target-words", default="green bottle,bottle,cup")
    parser.add_argument("--min-score", type=float, default=0.002)
    parser.add_argument("--image-topic", default="/test_yoloworld_image")
    parser.add_argument("--words-topic", default="/test_yoloworld_words")
    parser.add_argument("--det-topic", default="/test_yoloworld_det")
    parser.add_argument("--settle-sec", type=float, default=2.0)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--publish-interval-sec", type=float, default=0.5)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
