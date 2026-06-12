#!/usr/bin/env python3
import os
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class SaveRosCompressedImageOnce(Node):
    def __init__(self, image_topic="/image", save_path="../data/images/stage6_ros_compressed_image.jpg"):
        super().__init__("save_ros_compressed_image_once")

        self.image_topic = image_topic
        self.save_path = save_path
        self.saved = False

        self.sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Waiting for compressed image topic: {self.image_topic}")
        self.get_logger().info("Expected type: sensor_msgs/msg/CompressedImage")

    def image_callback(self, msg: CompressedImage):
        if self.saved:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().error("cv2.imdecode failed: frame is None")
                return

        except Exception as e:
            self.get_logger().error(f"decode compressed image failed: {repr(e)}")
            return

        h, w = frame.shape[:2]

        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        cv2.imwrite(self.save_path, frame)

        self.get_logger().info(f"Saved image: {self.save_path}, shape={w}x{h}, format={msg.format}")

        self.saved = True
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/image")
    parser.add_argument("--save-path", default="../data/images/stage6_ros_compressed_image.jpg")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = SaveRosCompressedImageOnce(
        image_topic=args.image_topic,
        save_path=args.save_path
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
