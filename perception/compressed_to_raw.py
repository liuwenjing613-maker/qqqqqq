#!/usr/bin/env python3
import argparse
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class CompressedToRaw(Node):
    def __init__(self, in_topic="/image", out_topic="/image_raw"):
        super().__init__("compressed_to_raw")

        self.in_topic = in_topic
        self.out_topic = out_topic
        self.bridge = CvBridge()

        self.pub = self.create_publisher(Image, self.out_topic, 10)

        self.sub = self.create_subscription(
            CompressedImage,
            self.in_topic,
            self.callback,
            10
        )

        self.get_logger().info("compressed_to_raw started")
        self.get_logger().info(f"subscribe: {self.in_topic} [CompressedImage]")
        self.get_logger().info(f"publish:   {self.out_topic} [Image/bgr8]")

    def callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warn("cv2.imdecode returned None")
                return

            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw_msg.header = msg.header
            self.pub.publish(raw_msg)

        except Exception as e:
            self.get_logger().error(f"decode failed: {repr(e)}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-topic", default="/image")
    parser.add_argument("--out-topic", default="/image_raw")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CompressedToRaw(
        in_topic=args.in_topic,
        out_topic=args.out_topic
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
