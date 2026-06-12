#!/usr/bin/env python3
import argparse

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class OpenCVImagePublisher(Node):
    def __init__(self, camera="/dev/video0", topic="/image_raw", width=640, height=480, fps=10):
        super().__init__("opencv_image_publisher")

        self.camera = camera
        self.topic = topic
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.topic, 10)

        self.cap = cv2.VideoCapture(self.camera)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {self.camera}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.get_logger().info("OpenCV image publisher started.")
        self.get_logger().info(
            f"camera={self.camera}, topic={self.topic}, size={self.width}x{self.height}, fps={self.fps}"
        )

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("camera read failed")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--topic", default="/image_raw")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=10)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = OpenCVImagePublisher(
        camera=args.camera,
        topic=args.topic,
        width=args.width,
        height=args.height,
        fps=args.fps,
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
