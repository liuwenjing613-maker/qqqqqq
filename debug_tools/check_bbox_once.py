#!/usr/bin/env python3
"""
导航过程中抓取一帧图像，检测红色目标并绘制 bbox，保存到 check_bbox 目录。

用法（导航运行时另开终端执行）：
  source /opt/tros/humble/setup.bash
  python3 ~/rdk_x5_vln_robot/debug_tools/check_bbox_once.py

  # 指定话题与输出目录
  python3 ~/rdk_x5_vln_robot/debug_tools/check_bbox_once.py \
    --image-topic /image_raw \
    --save-dir ~/rdk_x5_vln_robot/check_bbox
"""

import argparse
import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.perception.target_backend_red import find_red_target


DEFAULT_SAVE_DIR = os.path.join(PROJECT_ROOT, "check_bbox")


class CheckBboxOnce(Node):
    def __init__(self, image_topic="/image_raw", save_dir=DEFAULT_SAVE_DIR):
        super().__init__("check_bbox_once")
        self.image_topic = image_topic
        self.save_dir = os.path.expanduser(save_dir)
        self.bridge = CvBridge()
        self.saved = False

        os.makedirs(self.save_dir, exist_ok=True)

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.get_logger().info("===== check_bbox_once =====")
        self.get_logger().info(f"waiting image topic: {self.image_topic}")
        self.get_logger().info(f"save dir: {self.save_dir}")

    def image_callback(self, msg: Image):
        if self.saved:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")
            return

        target = find_red_target(frame)
        vis = frame.copy()
        h, w = frame.shape[:2]
        stamp = time.strftime("%Y%m%d_%H%M%S")

        if target.get("visible", False):
            x, y, bw, bh = target["bbox"]
            cx = target.get("cx", x + bw / 2.0)
            cy = target.get("cy", y + bh / 2.0)
            area_ratio = target.get("area_ratio", 0.0)

            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.circle(vis, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.putText(
                vis,
                f"bbox=[{x},{y},{bw},{bh}] cx={cx:.1f} cy={cy:.1f} area={area_ratio:.3f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            status = "FOUND"
            self.get_logger().info(
                f"target found: bbox={target['bbox']} cx={cx:.1f} cy={cy:.1f} area={area_ratio:.3f}"
            )
        else:
            reason = target.get("reason", "unknown")
            cv2.putText(
                vis,
                f"NO TARGET ({reason})",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2,
            )
            status = "NO_TARGET"
            self.get_logger().warn(f"no target in frame, reason={reason}")

        cv2.putText(
            vis,
            f"{w}x{h} topic={self.image_topic}",
            (20, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        save_path = os.path.join(self.save_dir, f"bbox_{stamp}_{status}.jpg")
        raw_path = os.path.join(self.save_dir, f"raw_{stamp}.jpg")

        cv2.imwrite(save_path, vis)
        cv2.imwrite(raw_path, frame)

        self.get_logger().info(f"saved annotated image: {save_path}")
        self.get_logger().info(f"saved raw image: {raw_path}")

        self.saved = True
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Grab one navigation frame and save bbox image.")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--save-dir", default=DEFAULT_SAVE_DIR)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CheckBboxOnce(
        image_topic=args.image_topic,
        save_dir=args.save_dir,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.saved:
            node.get_logger().warn("no image received, nothing saved")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
