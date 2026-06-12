#!/usr/bin/env python3
import argparse
import os
import time
import subprocess

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def detect_red(frame, min_area=500):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 放宽后的红色阈值，适合比赛初期调试
    lower_red1 = np.array([0, 50, 40])
    upper_red1 = np.array([15, 255, 255])
    lower_red2 = np.array([165, 50, 40])
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

    if area < min_area:
        return None, mask

    H, W = frame.shape[:2]
    x, y, w, h = cv2.boundingRect(c)
    cx = x + w / 2.0
    cy = y + h / 2.0
    area_ratio = area / float(W * H)

    return {
        "bbox": (x, y, w, h),
        "cx": cx,
        "cy": cy,
        "area": area,
        "area_ratio": area_ratio,
        "image_w": W,
        "image_h": H,
    }, mask


def get_ros_topic_type(topic_name):
    try:
        out = subprocess.check_output(
            ["ros2", "topic", "type", topic_name],
            text=True,
            stderr=subprocess.STDOUT,
            timeout=3,
        ).strip()
        return out
    except Exception:
        return ""


class RedTargetServoAuto(Node):
    def __init__(
        self,
        image_topic="/image",
        cmd_topic="/cmd_vel",
        max_vx=0.06,
        max_wz=0.35,
        kp_turn=0.9,
        center_threshold=0.22,
        arrive_area_ratio=0.30,
        min_area=500,
        dry_run=False,
        save_debug=False,
        debug_dir="../data/images/red_servo_auto_debug",
    ):
        super().__init__("red_target_servo_auto")

        self.image_topic = image_topic
        self.cmd_topic = cmd_topic

        self.max_vx = float(max_vx)
        self.max_wz = float(max_wz)
        self.kp_turn = float(kp_turn)
        self.center_threshold = float(center_threshold)
        self.arrive_area_ratio = float(arrive_area_ratio)
        self.min_area = float(min_area)
        self.dry_run = dry_run

        self.save_debug = save_debug
        self.debug_dir = debug_dir
        self.debug_count = 0

        if self.save_debug:
            os.makedirs(self.debug_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.last_image_time = time.time()
        self.last_frame_count = 0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, "/red_servo_state", 10)

        self.topic_type = get_ros_topic_type(self.image_topic)

        self.get_logger().info("========== red_target_servo_auto ==========")
        self.get_logger().info(f"image_topic: {self.image_topic}")
        self.get_logger().info(f"detected topic type: {self.topic_type}")
        self.get_logger().info(f"cmd_topic: {self.cmd_topic}")
        self.get_logger().info(f"dry_run: {self.dry_run}")
        self.get_logger().info(
            f"params: max_vx={self.max_vx}, max_wz={self.max_wz}, "
            f"kp_turn={self.kp_turn}, center_threshold={self.center_threshold}, "
            f"arrive_area_ratio={self.arrive_area_ratio}, min_area={self.min_area}"
        )
        self.get_logger().info("linear.y is always 0.0")

        if self.topic_type == "sensor_msgs/msg/CompressedImage":
            self.get_logger().info("Subscribing as CompressedImage")
            self.sub = self.create_subscription(
                CompressedImage,
                self.image_topic,
                self.compressed_image_callback,
                10,
            )
        elif self.topic_type == "sensor_msgs/msg/Image":
            self.get_logger().info("Subscribing as Image")
            self.sub = self.create_subscription(
                Image,
                self.image_topic,
                self.raw_image_callback,
                10,
            )
        else:
            self.get_logger().error(
                f"Unknown or empty image topic type: {self.topic_type}. "
                f"Please check: ros2 topic list -t | grep image"
            )

        self.timer = self.create_timer(0.2, self.watchdog_callback)

    def publish_state(self, text):
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def publish_cmd(self, vx, wz, reason):
        msg = Twist()
        msg.linear.x = float(clamp(vx, -self.max_vx, self.max_vx))
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(clamp(wz, -self.max_wz, self.max_wz))

        if not self.dry_run:
            self.cmd_pub.publish(msg)

        self.get_logger().info(
            f"{reason}: publish cmd vx={msg.linear.x:.3f}, wz={msg.angular.z:.3f}, dry_run={self.dry_run}"
        )
        self.publish_state(
            f"{reason}, vx={msg.linear.x:.3f}, wz={msg.angular.z:.3f}, dry_run={self.dry_run}"
        )

    def stop(self, reason):
        self.publish_cmd(0.0, 0.0, reason)

    def watchdog_callback(self):
        dt = time.time() - self.last_image_time
        if dt > 1.0:
            self.stop(f"NO_IMAGE_TIMEOUT dt={dt:.2f}s")

    def raw_image_callback(self, msg):
        self.last_image_time = time.time()
        self.last_frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.stop(f"CV_BRIDGE_FAILED {repr(e)}")
            return

        self.process_frame(frame)

    def compressed_image_callback(self, msg):
        self.last_image_time = time.time()
        self.last_frame_count += 1

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.stop(f"COMPRESSED_DECODE_FAILED {repr(e)}")
            return

        if frame is None:
            self.stop("COMPRESSED_DECODE_NONE")
            return

        self.process_frame(frame)

    def process_frame(self, frame):
        H, W = frame.shape[:2]
        result, mask = detect_red(frame, min_area=self.min_area)

        if result is None:
            self.stop(f"LOST_STOP frame={self.last_frame_count}, image={W}x{H}")
            self.save_debug_image(frame, None, "LOST_STOP")
            return

        x, y, w, h = result["bbox"]
        cx = result["cx"]
        cy = result["cy"]
        area = result["area"]
        area_ratio = result["area_ratio"]

        ex = (cx - W / 2.0) / W

        if area_ratio >= self.arrive_area_ratio:
            vx = 0.0
            wz = 0.0
            state = "ARRIVED_STOP"
        elif abs(ex) > self.center_threshold:
            vx = 0.0
            wz = -self.kp_turn * ex
            state = "TURN_ONLY"
        else:
            vx = self.max_vx
            wz = -self.kp_turn * ex
            state = "FORWARD"

        wz = clamp(wz, -self.max_wz, self.max_wz)

        reason = (
            f"{state} frame={self.last_frame_count}, "
            f"bbox=({x},{y},{w},{h}), ex={ex:+.3f}, "
            f"area={area:.1f}, area_ratio={area_ratio:.3f}"
        )

        self.publish_cmd(vx, wz, reason)
        self.save_debug_image(frame, result, state)

    def save_debug_image(self, frame, result, state):
        if not self.save_debug:
            return

        # 前 200 帧里每 10 帧保存一次，避免写盘太多
        if self.debug_count > 200:
            return
        if self.debug_count % 10 != 0:
            self.debug_count += 1
            return

        vis = frame.copy()
        H, W = vis.shape[:2]

        cv2.line(vis, (W // 2, 0), (W // 2, H), (255, 255, 255), 1)

        if result is not None:
            x, y, w, h = result["bbox"]
            cx = int(result["cx"])
            cy = int(result["cy"])
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(vis, (cx, cy), 5, (255, 0, 0), -1)
            area_ratio = result["area_ratio"]
            ex = (result["cx"] - W / 2.0) / W
            text = f"{state} ex={ex:+.2f} area={area_ratio:.3f}"
        else:
            text = state

        cv2.putText(
            vis,
            text,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        save_path = os.path.join(self.debug_dir, f"auto_{self.debug_count:04d}_{state}.jpg")
        cv2.imwrite(save_path, vis)
        self.get_logger().info(f"saved debug image: {save_path}")

        self.debug_count += 1

    def destroy_node(self):
        self.stop("NODE_SHUTDOWN_STOP")
        time.sleep(0.2)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/image")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--max-vx", type=float, default=0.06)
    parser.add_argument("--max-wz", type=float, default=0.35)
    parser.add_argument("--kp-turn", type=float, default=0.9)
    parser.add_argument("--center-threshold", type=float, default=0.22)
    parser.add_argument("--arrive-area-ratio", type=float, default=0.30)
    parser.add_argument("--min-area", type=float, default=500)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--save-debug", action="store_true")
    args, _ = parser.parse_known_args()

    rclpy.init()

    node = RedTargetServoAuto(
        image_topic=args.image_topic,
        cmd_topic=args.cmd_topic,
        max_vx=args.max_vx,
        max_wz=args.max_wz,
        kp_turn=args.kp_turn,
        center_threshold=args.center_threshold,
        arrive_area_ratio=args.arrive_area_ratio,
        min_area=args.min_area,
        dry_run=args.dry_run,
        save_debug=args.save_debug,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop("KEYBOARD_INTERRUPT_STOP")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
