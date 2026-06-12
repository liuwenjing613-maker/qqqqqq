#!/usr/bin/env python3
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

from src.perception.target_backend_yolo import extract_yolo_target


class YoloWorldBBoxPreview(Node):
    def __init__(
        self,
        det_topic="/hobot_yolo_world",
        out_topic="/target_bbox_json",
        target_classes="red backpack,backpack,bag,school bag",
        image_width=640,
        image_height=480,
        min_score=0.08,
    ):
        super().__init__("yolo_world_bbox_preview")

        self.det_topic = det_topic
        self.out_topic = out_topic
        self.target_classes = [x.strip() for x in target_classes.split(",") if x.strip()]
        self.image_width = image_width
        self.image_height = image_height
        self.min_score = min_score

        self.sub = self.create_subscription(
            PerceptionTargets,
            self.det_topic,
            self.callback,
            10
        )

        self.pub = self.create_publisher(String, self.out_topic, 10)

        self.get_logger().info("yolo_world_bbox_preview started.")
        self.get_logger().info(f"det_topic={self.det_topic}")
        self.get_logger().info(f"target_classes={self.target_classes}")
        self.get_logger().info(f"min_score={self.min_score}")

    def callback(self, msg):
        target = extract_yolo_target(
            msg,
            target_classes=self.target_classes,
            image_width=self.image_width,
            image_height=self.image_height,
            min_score=self.min_score,
        )

        if not target.get("visible", False):
            self.get_logger().info("target LOST")
            return

        out = String()
        out.data = json.dumps(target, ensure_ascii=False)
        self.pub.publish(out)

        cx = target["cx"]
        ex = (cx - self.image_width / 2.0) / self.image_width

        self.get_logger().info(
            f"FOUND class={target['class_name']} score={target['score']:.3f} "
            f"bbox={target['bbox']} ex={ex:+.3f} area={target['area_ratio']:.3f}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--det-topic", default="/hobot_yolo_world")
    parser.add_argument("--out-topic", default="/target_bbox_json")
    parser.add_argument("--target-classes", default="red backpack,backpack,bag,school bag")
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--image-height", type=int, default=480)
    parser.add_argument("--min-score", type=float, default=0.08)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = YoloWorldBBoxPreview(
        det_topic=args.det_topic,
        out_topic=args.out_topic,
        target_classes=args.target_classes,
        image_width=args.image_width,
        image_height=args.image_height,
        min_score=args.min_score,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
