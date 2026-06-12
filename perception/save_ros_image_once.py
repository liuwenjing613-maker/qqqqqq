#!/usr/bin/env python3
import os
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class SaveRosImageOnce(Node):
    def __init__(self, image_topic="/image", save_path="../data/images/stage6_ros_image.jpg"):
        super().__init__("save_ros_image_once")
        self.image_topic = image_topic
        self.save_path = save_path
        self.bridge = CvBridge()
        self.saved = False

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Waiting for image topic: {self.image_topic}")

    def image_callback(self, msg: Image):
        if self.saved:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")
            return

        h, w = frame.shape[:2]
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        cv2.imwrite(self.save_path, frame)

        self.get_logger().info(f"Saved image: {self.save_path}, shape={w}x{h}")
        self.saved = True
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/image")
    parser.add_argument("--save-path", default="../data/images/stage6_ros_image.jpg")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = SaveRosImageOnce(
        image_topic=args.image_topic,
        save_path=args.save_path
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
