#!/usr/bin/env python3
"""Publish fake bbox JSON for TARGET_TRACK testing without YOLO."""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FakeBBoxPub(Node):
    def __init__(self, topic: str, bbox, score: float, cls: str, rate: float):
        super().__init__("fake_bbox_pub")
        self.pub = self.create_publisher(String, topic, 10)
        self.bbox = bbox
        self.score = score
        self.cls = cls
        self.timer = self.create_timer(1.0 / rate, self.cb)

    def cb(self):
        x1, y1, x2, y2 = self.bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        data = {
            "visible": True,
            "bbox": list(self.bbox),
            "score": self.score,
            "class": self.cls,
            "cx": cx,
            "cy": cy,
        }
        self.pub.publish(String(data=json.dumps(data)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/target_bbox_json")
    parser.add_argument("--bbox", nargs=4, type=float, default=[250, 140, 390, 420])
    parser.add_argument("--score", type=float, default=0.8)
    parser.add_argument("--class-name", default="bottle")
    parser.add_argument("--rate", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = FakeBBoxPub(args.topic, args.bbox, args.score, args.class_name, args.rate)
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
