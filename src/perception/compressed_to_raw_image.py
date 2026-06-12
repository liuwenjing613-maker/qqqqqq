#!/usr/bin/env python3
import argparse
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class CompressedToRawImage(Node):
    """
    /image(sensor_msgs/msg/CompressedImage) -> /image_raw(sensor_msgs/msg/Image)

    约定：
    - /image：hobot_usb_cam 默认 MJPEG/压缩图像，给 websocket 网页看
    - /image_raw：算法真正使用的 BGR8 原图
    """

    def __init__(self, in_topic="/image", out_topic="/image_raw", frame_id="usb_camera"):
        super().__init__("compressed_to_raw_image")

        self.in_topic = in_topic
        self.out_topic = out_topic
        self.frame_id = frame_id
        self.bridge = CvBridge()

        self.pub = self.create_publisher(Image, self.out_topic, 10)
        self.sub = self.create_subscription(
            CompressedImage,
            self.in_topic,
            self.callback,
            10
        )

        self.count = 0
        self.last_time = time.time()
        self.get_logger().info(f"compressed_to_raw_image started")
        self.get_logger().info(f"subscribe: {self.in_topic}  type=sensor_msgs/msg/CompressedImage")
        self.get_logger().info(f"publish  : {self.out_topic} type=sensor_msgs/msg/Image")

    def callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warn("cv2.imdecode returned None")
                return

            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw_msg.header = msg.header
            if not raw_msg.header.frame_id:
                raw_msg.header.frame_id = self.frame_id

            self.pub.publish(raw_msg)

            self.count += 1
            now = time.time()
            if now - self.last_time >= 2.0:
                h, w = frame.shape[:2]
                fps = self.count / (now - self.last_time)
                self.get_logger().info(f"published /image_raw: {w}x{h}, approx_fps={fps:.2f}")
                self.count = 0
                self.last_time = now

        except Exception as e:
            self.get_logger().error(f"convert failed: {repr(e)}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-topic", default="/image")
    parser.add_argument("--out-topic", default="/image_raw")
    parser.add_argument("--frame-id", default="usb_camera")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CompressedToRawImage(
        in_topic=args.in_topic,
        out_topic=args.out_topic,
        frame_id=args.frame_id
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
