#!/usr/bin/env python3
"""
YOLO-World 检测后处理预览（非推理节点）。

本脚本只订阅 hobot_yolo_world 发布的 /hobot_yolo_world，做类别过滤与 MVP 后处理。
必须先单独启动推理节点 hobot_yolo_world，并通过 -p texts:=... 或 /target_words 设置
模型检测类别；本脚本的 --target-classes 仅影响后处理过滤，不会改变模型输出类别。

启动顺序见 yolo_world/README.md
"""
import argparse
import json
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.perception.target_backend_yolo import (
    extract_yolo_target,
    list_all_yolo_detections,
    parse_target_classes,
)


class YoloWorldBBoxPreview(Node):
    def __init__(
        self,
        det_topic="/hobot_yolo_world",
        out_topic="/target_bbox_json",
        target_classes="backpack,handbag,suitcase",
        image_width=1280,
        image_height=720,
        min_score=0.002,
        max_area_ratio=0.20,
        min_red_ratio=0.06,
        require_red_verify=False,
        debug_all=True,
        log_interval=1.0,
    ):
        super().__init__("yolo_world_bbox_preview")

        self.det_topic = det_topic
        self.out_topic = out_topic
        self.target_classes = parse_target_classes(target_classes)
        self.image_width = image_width
        self.image_height = image_height
        self.min_score = min_score
        self.max_area_ratio = max_area_ratio
        self.min_red_ratio = min_red_ratio
        self.require_red_verify = require_red_verify
        self.debug_all = debug_all
        self.log_interval = max(0.1, float(log_interval))
        self.det_count = 0
        self.last_log_time = 0.0

        self.sub = self.create_subscription(
            PerceptionTargets,
            self.det_topic,
            self.callback,
            10,
        )

        self.pub = self.create_publisher(String, self.out_topic, 10)

        self.get_logger().info("===== yolo_world_bbox_preview (post-process only) =====")
        self.get_logger().info(
            "NOTE: this is NOT hobot_yolo_world. Start inference node first; see yolo_world/README.md"
        )
        self.get_logger().info(f"det_topic={self.det_topic}")
        filter_desc = self.target_classes if self.target_classes else "(no filter / all classes)"
        self.get_logger().info(f"target_classes={filter_desc}")
        self.get_logger().info(f"min_score={self.min_score} debug_all={self.debug_all}")

    def _log_all_detections(self, msg: PerceptionTargets):
        all_dets = list_all_yolo_detections(
            msg,
            image_width=self.image_width,
            image_height=self.image_height,
            min_score=0.0,
        )
        stamp = msg.header.stamp
        stamp_str = f"{stamp.sec}.{stamp.nanosec:09d}" if stamp.sec or stamp.nanosec else "no_stamp"
        self.get_logger().info(
            f"det#{self.det_count} stamp={stamp_str} "
            f"target_count={len(msg.targets)} raw_roi_count={len(all_dets)}"
        )
        if not all_dets:
            self.get_logger().info("  (no ROI above min_score=0.0)")
            return

        for item in all_dets:
            raw = item["rect_raw"]
            self.get_logger().info(
                f"  [{item['target_index']}:{item['roi_index']}] "
                f"type={item['class_name']} conf={item['score']:.6f} "
                f"bbox={item['bbox']} area={item['area_ratio']:.3f} "
                f"rect_raw=(x={raw['x_offset']:.1f}, y={raw['y_offset']:.1f}, "
                f"w={raw['width']:.1f}, h={raw['height']:.1f}) "
                f"norm640={item['rect_norm_640']}"
            )

    def callback(self, msg: PerceptionTargets):
        import time

        self.det_count += 1
        now = time.time()
        should_log = (now - self.last_log_time) >= self.log_interval

        if self.debug_all and should_log:
            self.last_log_time = now
            self._log_all_detections(msg)

        target = extract_yolo_target(
            msg,
            target_classes=self.target_classes,
            image_width=self.image_width,
            image_height=self.image_height,
            min_score=self.min_score,
            max_area_ratio=self.max_area_ratio,
            min_red_ratio=self.min_red_ratio,
            require_red_verify=self.require_red_verify,
        )

        if not target.get("visible", False):
            if should_log:
                reason = target.get("reason", "not_visible")
                self.get_logger().info(f"MVP target LOST reason={reason}")
            return

        out = String()
        out.data = json.dumps(target, ensure_ascii=False)
        self.pub.publish(out)

        cx = target["cx"]
        ex = (cx - self.image_width / 2.0) / self.image_width

        if should_log:
            self.get_logger().info(
                f"MVP_FOUND class={target['class_name']} score={target['score']:.3f} "
                f"bbox={target['bbox']} ex={ex:+.3f} area={target['area_ratio']:.3f}"
            )


def main():
    parser = argparse.ArgumentParser(
        description="YOLO-World bbox post-process preview (requires hobot_yolo_world running)."
    )
    parser.add_argument("--det-topic", default="/hobot_yolo_world")
    parser.add_argument("--out-topic", default="/target_bbox_json")
    parser.add_argument(
        "--target-classes",
        default="backpack,handbag,suitcase",
        help="Post-process class filter; empty string = no filter (all classes)",
    )
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--min-score", type=float, default=0.002)
    parser.add_argument("--max-area-ratio", type=float, default=0.20)
    parser.add_argument("--min-red-ratio", type=float, default=0.06)
    parser.add_argument("--require-red-verify", action="store_true")
    parser.add_argument(
        "--no-debug-all",
        action="store_true",
        help="Disable printing all raw ROIs before class filter",
    )
    parser.add_argument("--log-interval", type=float, default=1.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = YoloWorldBBoxPreview(
        det_topic=args.det_topic,
        out_topic=args.out_topic,
        target_classes=args.target_classes,
        image_width=args.image_width,
        image_height=args.image_height,
        min_score=args.min_score,
        max_area_ratio=args.max_area_ratio,
        min_red_ratio=args.min_red_ratio,
        require_red_verify=args.require_red_verify,
        debug_all=not args.no_debug_all,
        log_interval=args.log_interval,
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
