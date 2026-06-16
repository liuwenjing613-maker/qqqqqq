#!/usr/bin/env python3
"""
等待 YOLO-World 检测到目标框后，保存当前图像与 bbox 到 check_bbox，然后退出。

需先启动相机与 hobot_yolo_world（并设置 /target_words 或 -p texts:=...）。

用法：
  source /opt/tros/humble/setup.bash
  python3 ~/rdk_x5_vln_robot/debug_tools/check_yolo_bbox_once.py

  # 绿色水杯
  python3 ~/rdk_x5_vln_robot/debug_tools/check_yolo_bbox_once.py \
    --target-classes "bottle,cup,wine glass" \
    --min-score 0.01 \
    --no-red-verify

  # 任意类别（不过滤 class）
  python3 ~/rdk_x5_vln_robot/debug_tools/check_yolo_bbox_once.py \
    --target-classes "" \
    --min-score 0.01 \
    --no-red-verify
"""

import argparse
import json
import os
import sys
import time

import cv2
import rclpy
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.config.mvp_tune import DEFAULT_TUNE_PATH, load_mvp_tune
from src.perception.stamp_sync import StampSyncBuffer
from src.perception.target_backend_yolo import (
    extract_yolo_target,
    list_all_yolo_detections,
    parse_target_classes,
)

DEFAULT_SAVE_DIR = os.path.join(PROJECT_ROOT, "check_bbox")


class CheckYoloBboxOnce(Node):
    def __init__(
        self,
        image_topic="/image_raw",
        det_topic="/hobot_yolo_world",
        save_dir=DEFAULT_SAVE_DIR,
        target_classes="",
        image_width=1280,
        image_height=720,
        min_score=0.01,
        max_area_ratio=0.15,
        min_red_ratio=0.06,
        require_red_verify=False,
        on_raw=False,
        timeout_sec=0.0,
        sync_max_delta_sec=0.12,
        sync_buffer_len=60,
    ):
        super().__init__("check_yolo_bbox_once")

        self.image_topic = image_topic
        self.det_topic = det_topic
        self.save_dir = os.path.expanduser(save_dir)
        self.target_classes = parse_target_classes(target_classes)
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.min_score = float(min_score)
        self.max_area_ratio = float(max_area_ratio)
        self.min_red_ratio = float(min_red_ratio)
        self.require_red_verify = bool(require_red_verify)
        self.on_raw = bool(on_raw)
        self.timeout_sec = float(timeout_sec)
        self.sync_max_delta_sec = float(sync_max_delta_sec)

        self.bridge = CvBridge()
        self.frame_buffer = StampSyncBuffer(
            max_len=sync_buffer_len,
            max_delta_sec=self.sync_max_delta_sec,
        )
        self.saved = False
        self.start_time = time.time()
        self.det_count = 0
        self.last_wait_log = 0.0
        self.last_sync_warn_time = 0.0

        self.sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        os.makedirs(self.save_dir, exist_ok=True)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            self.sensor_qos,
        )
        self.det_sub = self.create_subscription(
            PerceptionTargets,
            self.det_topic,
            self.det_callback,
            self.sensor_qos,
        )

        if self.timeout_sec > 0:
            self.create_timer(1.0, self.timeout_callback)

        filter_desc = self.target_classes if self.target_classes else "(all classes)"
        self.get_logger().info("===== check_yolo_bbox_once =====")
        self.get_logger().info(f"waiting YOLO detection on: {self.det_topic}")
        self.get_logger().info(f"image topic: {self.image_topic}")
        self.get_logger().info(f"target_classes: {filter_desc}")
        self.get_logger().info(
            f"min_score={self.min_score} max_area={self.max_area_ratio} "
            f"red_verify={self.require_red_verify} on_raw={self.on_raw}"
        )
        self.get_logger().info(f"save dir: {self.save_dir}")
        if self.timeout_sec > 0:
            self.get_logger().info(f"timeout: {self.timeout_sec:.1f}s")

    def timeout_callback(self):
        if self.saved:
            return
        if time.time() - self.start_time >= self.timeout_sec:
            self.get_logger().warn(
                f"timeout after {self.timeout_sec:.1f}s, no bbox saved (det_count={self.det_count})"
            )
            rclpy.shutdown()

    def image_callback(self, msg: Image):
        if self.saved:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")
            return
        self.frame_buffer.push(msg.header.stamp, frame)

    def _sync_frame(self, det_msg):
        det_stamp = det_msg.header.stamp
        frame, delta = self.frame_buffer.find_closest(det_stamp)
        if frame is None:
            now = time.time()
            if now - self.last_sync_warn_time >= 1.0:
                self.last_sync_warn_time = now
                delta_str = f"{delta:.3f}s" if delta is not None else "no_frame"
                self.get_logger().warn(
                    f"stamp sync skip: no frame for det stamp, delta {delta_str} "
                    f"> {self.sync_max_delta_sec}s"
                )
            return None
        return frame

    def _resolve_detection(self, det_msg, frame):
        if self.on_raw:
            dets = list_all_yolo_detections(
                det_msg,
                image_width=self.image_width,
                image_height=self.image_height,
                min_score=self.min_score,
            )
            if not dets:
                return None
            if self.target_classes:
                dets = [
                    d for d in dets
                    if any(
                        tc.lower() in d["class_name"].lower()
                        or d["class_name"].lower() in tc.lower()
                        for tc in self.target_classes
                    )
                ]
                if not dets:
                    return None
            best = max(dets, key=lambda item: item["score"])
            area_ratio = best["area_ratio"]
            if area_ratio > self.max_area_ratio:
                return {
                    "visible": False,
                    "reason": "area_too_large",
                    "class_name": best["class_name"],
                    "score": best["score"],
                    "bbox": best["bbox"],
                    "cx": best["cx"],
                    "cy": best["cy"],
                    "area_ratio": area_ratio,
                }
            return {
                "visible": True,
                "class_name": best["class_name"],
                "score": best["score"],
                "bbox": best["bbox"],
                "cx": best["cx"],
                "cy": best["cy"],
                "area_ratio": area_ratio,
                "source": "raw_yolo",
            }

        return extract_yolo_target(
            det_msg,
            target_classes=self.target_classes,
            image_width=self.image_width,
            image_height=self.image_height,
            min_score=self.min_score,
            max_area_ratio=self.max_area_ratio,
            frame=frame,
            min_red_ratio=self.min_red_ratio,
            require_red_verify=self.require_red_verify,
        )

    def det_callback(self, msg: PerceptionTargets):
        if self.saved:
            return

        self.det_count += 1
        now = time.time()
        if now - self.last_wait_log >= 2.0:
            self.last_wait_log = now
            self.get_logger().info(
                f"waiting... det_count={self.det_count} "
                f"elapsed={now - self.start_time:.1f}s"
            )

        frame = self._sync_frame(msg)
        if frame is None:
            return

        target = self._resolve_detection(msg, frame)
        if not target or not target.get("visible", False):
            return

        self._save_snapshot(frame, target)
        self.saved = True
        rclpy.shutdown()

    def _save_snapshot(self, frame, target):
        vis = frame.copy()
        h, w = frame.shape[:2]
        stamp = time.strftime("%Y%m%d_%H%M%S")
        class_name = str(target.get("class_name", "target")).replace(" ", "_")
        x, y, bw, bh = target["bbox"]
        cx = target.get("cx", x + bw / 2.0)
        cy = target.get("cy", y + bh / 2.0)
        score = float(target.get("score", 0.0))
        area_ratio = float(target.get("area_ratio", 0.0))

        cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.circle(vis, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        cv2.putText(
            vis,
            f"{target.get('class_name', '')} score={score:.3f} bbox=[{x},{y},{bw},{bh}]",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            vis,
            f"area={area_ratio:.3f} cx={cx:.1f} cy={cy:.1f}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            vis,
            f"{w}x{h} det={self.det_topic}",
            (20, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        base = f"yolo_{stamp}_{class_name}"
        bbox_path = os.path.join(self.save_dir, f"bbox_{base}.jpg")
        raw_path = os.path.join(self.save_dir, f"raw_{base}.jpg")
        meta_path = os.path.join(self.save_dir, f"meta_{base}.json")

        meta = {
            "timestamp": stamp,
            "image_topic": self.image_topic,
            "det_topic": self.det_topic,
            "target_classes": self.target_classes,
            "min_score": self.min_score,
            "max_area_ratio": self.max_area_ratio,
            "require_red_verify": self.require_red_verify,
            "on_raw": self.on_raw,
            "detection": target,
            "image_size": {"width": w, "height": h},
        }

        cv2.imwrite(bbox_path, vis)
        cv2.imwrite(raw_path, frame)
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        self.get_logger().info(
            f"YOLO target found: class={target.get('class_name')} "
            f"score={score:.3f} bbox={target['bbox']} area={area_ratio:.3f}"
        )
        self.get_logger().info(f"saved annotated image: {bbox_path}")
        self.get_logger().info(f"saved raw image: {raw_path}")
        self.get_logger().info(f"saved metadata: {meta_path}")


def main():
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--mvp-tune-config", default=DEFAULT_TUNE_PATH)
    pre_args, _ = pre_parser.parse_known_args()
    tune = load_mvp_tune(pre_args.mvp_tune_config)

    parser = argparse.ArgumentParser(
        description="Wait for YOLO-World bbox, save image + metadata once, then exit."
    )
    parser.add_argument("--mvp-tune-config", default=pre_args.mvp_tune_config)
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--det-topic", default="/hobot_yolo_world")
    parser.add_argument("--save-dir", default=DEFAULT_SAVE_DIR)
    parser.add_argument(
        "--target-classes",
        default="",
        help="Comma-separated class filter; empty means accept all YOLO classes",
    )
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--min-score", type=float, default=tune["min_score"])
    parser.add_argument("--max-area-ratio", type=float, default=tune["max_area_ratio"])
    parser.add_argument("--min-red-ratio", type=float, default=tune["min_red_ratio"])
    parser.add_argument("--no-red-verify", action="store_true")
    parser.add_argument(
        "--on-raw",
        action="store_true",
        help="Save on first raw YOLO roi (skip MVP post-filter except max_area)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="Exit without saving after N seconds (0 = wait forever)",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CheckYoloBboxOnce(
        image_topic=args.image_topic,
        det_topic=args.det_topic,
        save_dir=args.save_dir,
        target_classes=args.target_classes,
        image_width=args.image_width,
        image_height=args.image_height,
        min_score=args.min_score,
        max_area_ratio=args.max_area_ratio,
        min_red_ratio=args.min_red_ratio,
        require_red_verify=not args.no_red_verify,
        on_raw=args.on_raw,
        timeout_sec=args.timeout,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.saved:
            node.get_logger().warn("no YOLO bbox saved")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
