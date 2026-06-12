#!/usr/bin/env python3
import os
import time
import argparse
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class RedBackpackDebug(Node):
    def __init__(self, image_topic="/image_raw", save_dir="/root/rdk_x5_vln_robot/data/images/red_backpack_debug"):
        super().__init__("red_backpack_debug")
        self.image_topic = image_topic
        self.save_dir = save_dir
        self.bridge = CvBridge()
        self.count = 0

        os.makedirs(self.save_dir, exist_ok=True)

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.callback,
            10
        )

        self.get_logger().info(f"subscribe: {self.image_topic}")
        self.get_logger().info(f"save_dir: {self.save_dir}")

    def callback(self, msg):
        self.count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")
            return

        H, W = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 调试阶段先放宽红色阈值：允许暗红、橙红
        lower_red1 = np.array([0, 50, 35])
        upper_red1 = np.array([18, 255, 255])

        lower_red2 = np.array([165, 50, 35])
        upper_red2 = np.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        kernel = np.ones((5, 5), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        vis = frame.copy()
        candidates = []

        for idx, c in enumerate(contours):
            area = cv2.contourArea(c)
            if area <= 0:
                continue

            x, y, w, h = cv2.boundingRect(c)
            cx = x + w / 2.0
            cy = y + h / 2.0

            area_ratio = area / float(W * H)
            aspect = w / float(h)

            reason = "PASS"

            if area_ratio < 0.004:
                reason = "small_area"
            elif w < 35 or h < 35:
                reason = "small_wh"
            elif aspect < 0.20 or aspect > 4.0:
                reason = "bad_aspect"
            elif cy > H * 0.92 and area_ratio < 0.025:
                reason = "bottom_noise"

            color = (0, 255, 0) if reason == "PASS" else (0, 0, 255)
            cv2.rectangle(vis, (x, y), (x + w, y + h), color, 2)
            cv2.putText(
                vis,
                f"{idx}:{reason} ar={area_ratio:.3f} wh={w}x{h}",
                (x, max(20, y - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )

            candidates.append((reason, x, y, w, h, area_ratio, aspect))

        candidates_sorted = sorted(candidates, key=lambda t: t[5], reverse=True)

        self.get_logger().info("========== frame debug ==========")
        self.get_logger().info(f"image shape: {W}x{H}, contours={len(contours)}")

        if not candidates_sorted:
            self.get_logger().info("NO red candidates")
        else:
            for item in candidates_sorted[:10]:
                reason, x, y, w, h, area_ratio, aspect = item
                self.get_logger().info(
                    f"{reason:12s} bbox=({x},{y},{w},{h}) area_ratio={area_ratio:.4f} aspect={aspect:.2f}"
                )

        # 每 20 帧保存一次
        if self.count % 20 == 0:
            raw_path = os.path.join(self.save_dir, f"raw_{self.count:04d}.jpg")
            mask_path = os.path.join(self.save_dir, f"mask_{self.count:04d}.jpg")
            vis_path = os.path.join(self.save_dir, f"vis_{self.count:04d}.jpg")

            cv2.imwrite(raw_path, frame)
            cv2.imwrite(mask_path, mask_clean)
            cv2.imwrite(vis_path, vis)

            self.get_logger().info(f"saved: {raw_path}")
            self.get_logger().info(f"saved: {mask_path}")
            self.get_logger().info(f"saved: {vis_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--save-dir", default="/root/rdk_x5_vln_robot/data/images/red_backpack_debug")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = RedBackpackDebug(args.image_topic, args.save_dir)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
