#!/usr/bin/env python3
import argparse
import os
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def find_red_bbox(frame):
    """
    输入 BGR 图像，返回最大红色区域。
    返回:
      result = {
        "bbox": (x, y, w, h),
        "area": area,
        "area_ratio": area_ratio,
        "cx": cx,
        "cy": cy
      }
      mask
    找不到则 result 为 None。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 80, 60])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 80, 60])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, mask

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)

    H, W = frame.shape[:2]
    area_ratio = area / float(W * H)

    if area < 500:
        return None, mask

    x, y, w, h = cv2.boundingRect(c)
    cx = x + w / 2.0
    cy = y + h / 2.0

    result = {
        "bbox": (x, y, w, h),
        "area": area,
        "area_ratio": area_ratio,
        "cx": cx,
        "cy": cy,
    }

    return result, mask


class RedTargetServo(Node):
    """
    红色目标视觉伺服节点。

    输入:
      /image: sensor_msgs/msg/Image

    输出:
      /cmd_vel: geometry_msgs/msg/Twist

    控制原则:
      1. 不使用横移 linear.y。
      2. 目标丢失立即停车。
      3. 目标偏左/偏右时先转正。
      4. 目标居中后低速前进。
      5. 目标面积足够大后停车。
    """

    def __init__(
        self,
        image_topic="/image",
        cmd_topic="/cmd_vel",
        kp_turn=1.2,
        max_vx=0.06,
        max_wz=0.35,
        center_threshold=0.15,
        arrive_area_ratio=0.12,
        min_area_ratio=0.002,
        save_debug=False,
        debug_dir="../data/images/red_servo_debug",
    ):
        super().__init__("red_target_servo")

        self.image_topic = image_topic
        self.cmd_topic = cmd_topic

        self.kp_turn = float(kp_turn)
        self.max_vx = float(max_vx)
        self.max_wz = float(max_wz)
        self.center_threshold = float(center_threshold)
        self.arrive_area_ratio = float(arrive_area_ratio)
        self.min_area_ratio = float(min_area_ratio)

        self.save_debug = save_debug
        self.debug_dir = debug_dir
        self.debug_save_count = 0

        if self.save_debug:
            os.makedirs(self.debug_dir, exist_ok=True)

        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.last_image_time = time.time()
        self.image_timeout = 0.8

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info("red_target_servo started.")
        self.get_logger().info(f"subscribe image_topic: {self.image_topic}")
        self.get_logger().info(f"publish cmd_topic: {self.cmd_topic}")
        self.get_logger().info("linear.y is always 0.0 in this stage.")
        self.get_logger().info(
            f"params: kp_turn={self.kp_turn}, max_vx={self.max_vx}, max_wz={self.max_wz}, "
            f"center_threshold={self.center_threshold}, arrive_area_ratio={self.arrive_area_ratio}"
        )

    def publish_cmd(self, vx, wz):
        msg = Twist()

        msg.linear.x = float(clamp(vx, -self.max_vx, self.max_vx))
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(clamp(wz, -self.max_wz, self.max_wz))

        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def watchdog_callback(self):
        """
        如果相机图像长时间没有更新，立即停车。
        """
        if time.time() - self.last_image_time > self.image_timeout:
            self.stop()

    def image_callback(self, msg: Image):
        self.last_image_time = time.time()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")
            self.stop()
            return

        H, W = frame.shape[:2]
        result, mask = find_red_bbox(frame)

        if result is None:
            self.stop()
            self.get_logger().info("target LOST -> STOP")
            return

        x, y, w, h = result["bbox"]
        cx = result["cx"]
        cy = result["cy"]
        area_ratio = result["area_ratio"]

        ex = (cx - W / 2.0) / W

        # 目标太小，可能是噪声或距离太远，先只转向不前进
        if area_ratio < self.min_area_ratio:
            vx = 0.0
            wz = -self.kp_turn * ex
            state = "SMALL_TURN_ONLY"

        # 目标足够大，认为已经靠近
        elif area_ratio >= self.arrive_area_ratio:
            vx = 0.0
            wz = 0.0
            state = "ARRIVED_STOP"

        # 偏差太大，先原地转正
        elif abs(ex) > self.center_threshold:
            vx = 0.0
            wz = -self.kp_turn * ex
            state = "TURN_ONLY"

        # 基本居中，低速前进，同时保留少量角速度修正
        else:
            vx = self.max_vx
            wz = -self.kp_turn * ex
            state = "FORWARD"

        wz = clamp(wz, -self.max_wz, self.max_wz)

        self.publish_cmd(vx, wz)

        self.get_logger().info(
            f"{state}: bbox=({x},{y},{w},{h}), ex={ex:+.3f}, "
            f"area_ratio={area_ratio:.3f}, cmd=(vx={vx:.3f}, wz={wz:.3f})"
        )

        if self.save_debug and self.debug_save_count < 100:
            vis = frame.copy()
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.line(vis, (W // 2, 0), (W // 2, H), (255, 255, 255), 1)
            cv2.circle(vis, (int(cx), int(cy)), 5, (255, 0, 0), -1)
            cv2.putText(
                vis,
                f"{state} ex={ex:+.2f} area={area_ratio:.3f} vx={vx:.2f} wz={wz:.2f}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            if self.debug_save_count % 10 == 0:
                save_path = os.path.join(
                    self.debug_dir,
                    f"servo_{self.debug_save_count:03d}.jpg"
                )
                cv2.imwrite(save_path, vis)

            self.debug_save_count += 1

    def destroy_node(self):
        self.get_logger().info("Stopping robot before shutdown...")
        self.stop()
        time.sleep(0.2)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/image")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--kp-turn", type=float, default=1.2)
    parser.add_argument("--max-vx", type=float, default=0.06)
    parser.add_argument("--max-wz", type=float, default=0.35)
    parser.add_argument("--center-threshold", type=float, default=0.15)
    parser.add_argument("--arrive-area-ratio", type=float, default=0.12)
    parser.add_argument("--min-area-ratio", type=float, default=0.002)
    parser.add_argument("--save-debug", action="store_true")
    args, _ = parser.parse_known_args()

    rclpy.init()

    node = RedTargetServo(
        image_topic=args.image_topic,
        cmd_topic=args.cmd_topic,
        kp_turn=args.kp_turn,
        max_vx=args.max_vx,
        max_wz=args.max_wz,
        center_threshold=args.center_threshold,
        arrive_area_ratio=args.arrive_area_ratio,
        min_area_ratio=args.min_area_ratio,
        save_debug=args.save_debug,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt.")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
