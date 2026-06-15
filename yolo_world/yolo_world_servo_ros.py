#!/usr/bin/env python3
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from ai_msgs.msg import PerceptionTargets
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.perception.target_backend_yolo import extract_yolo_target, parse_target_classes


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class YoloWorldServo(Node):
    """
    YOLO-World 检测框视觉伺服节点（后处理，非推理）。

    必须先启动 hobot_yolo_world 推理节点；详见 yolo_world/README.md

    输入:
      /hobot_yolo_world: ai_msgs/msg/PerceptionTargets

    输出:
      /cmd_vel: geometry_msgs/msg/Twist

    阶段 8 控制原则:
      1. 不使用横移 linear.y。
      2. 没有检测到目标 -> 停车。
      3. 目标偏左/偏右 -> 原地转正。
      4. 目标居中 -> 低速前进。
      5. 目标面积足够大 -> 停车。
    """

    def __init__(
        self,
        det_topic="/hobot_yolo_world",
        cmd_topic="/cmd_vel",
        image_topic="/image_raw",
        target_classes="backpack,handbag,suitcase",
        image_width=1280,
        image_height=720,
        min_score=0.002,
        min_red_ratio=0.06,
        max_area_ratio=0.20,
        require_red_verify=True,
        kp_turn=1.0,
        max_vx=0.05,
        max_wz=0.28,
        center_threshold=0.18,
        arrive_area_ratio=0.12,
        lost_timeout=0.5,
    ):
        super().__init__("yolo_world_servo")

        self.det_topic = det_topic
        self.cmd_topic = cmd_topic
        self.image_topic = image_topic
        self.target_classes = parse_target_classes(target_classes)
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.min_score = float(min_score)
        self.min_red_ratio = float(min_red_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.require_red_verify = bool(require_red_verify)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.kp_turn = float(kp_turn)
        self.max_vx = float(max_vx)
        self.max_wz = float(max_wz)
        self.center_threshold = float(center_threshold)
        self.arrive_area_ratio = float(arrive_area_ratio)
        self.lost_timeout = float(lost_timeout)

        self.last_target_time = 0.0
        self.last_state = "INIT"

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.det_sub = self.create_subscription(
            PerceptionTargets,
            self.det_topic,
            self.det_callback,
            self.sensor_qos,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            self.sensor_qos,
        )

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info("yolo_world_servo started.")
        self.get_logger().info(f"det_topic: {self.det_topic}")
        self.get_logger().info(f"cmd_topic: {self.cmd_topic}")
        self.get_logger().info(f"target_classes: {self.target_classes}")
        self.get_logger().info(f"image_topic: {self.image_topic}")
        self.get_logger().info(
            f"params: min_score={self.min_score}, min_red_ratio={self.min_red_ratio}, "
            f"max_area_ratio={self.max_area_ratio}, require_red_verify={self.require_red_verify}, "
            f"kp_turn={self.kp_turn}, "
            f"max_vx={self.max_vx}, max_wz={self.max_wz}, "
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

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")

    def extract_best_target(self, msg: PerceptionTargets):
        target = extract_yolo_target(
            msg,
            target_classes=self.target_classes,
            image_width=self.image_width,
            image_height=self.image_height,
            min_score=self.min_score,
            max_area_ratio=self.max_area_ratio,
            frame=self.latest_frame,
            min_red_ratio=self.min_red_ratio,
            require_red_verify=self.require_red_verify,
        )
        if not target.get("visible", False):
            return None

        cx = target["cx"]
        ex = (cx - self.image_width / 2.0) / self.image_width

        return {
            "type": target["class_name"],
            "score": target["score"],
            "bbox": target["bbox"],
            "cx": cx,
            "cy": target["cy"],
            "area": target["bbox"][2] * target["bbox"][3],
            "area_ratio": target["area_ratio"],
            "ex": ex,
        }

    def det_callback(self, msg: PerceptionTargets):
        best = self.extract_best_target(msg)

        if best is None:
            self.stop()
            self.last_state = "LOST_STOP"
            self.get_logger().info("target LOST -> STOP")
            return

        self.last_target_time = time.time()

        ex = best["ex"]
        area_ratio = best["area_ratio"]

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

        self.publish_cmd(vx, wz)
        self.last_state = state

        self.get_logger().info(
            f"{state}: type={best['type']} score={best['score']:.3f} "
            f"bbox={best['bbox']} ex={ex:+.3f} area_ratio={area_ratio:.3f} "
            f"cmd=(vx={vx:.3f}, wz={wz:.3f})"
        )

    def watchdog_callback(self):
        if self.last_target_time <= 0:
            self.stop()
            return

        if time.time() - self.last_target_time > self.lost_timeout:
            self.stop()
            if self.last_state != "WATCHDOG_STOP":
                self.get_logger().warn("target timeout -> STOP")
                self.last_state = "WATCHDOG_STOP"

    def destroy_node(self):
        self.get_logger().info("Stopping robot before shutdown...")
        self.stop()
        time.sleep(0.2)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--det-topic", default="/hobot_yolo_world")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument(
        "--target-classes",
        default="backpack,handbag,suitcase",
        help="Post-process filter only; empty = no filter. Model texts set via hobot_yolo_world.",
    )
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--min-score", type=float, default=0.002)
    parser.add_argument("--min-red-ratio", type=float, default=0.06)
    parser.add_argument("--max-area-ratio", type=float, default=0.20)
    parser.add_argument("--no-red-verify", action="store_true")
    parser.add_argument("--kp-turn", type=float, default=1.0)
    parser.add_argument("--max-vx", type=float, default=0.05)
    parser.add_argument("--max-wz", type=float, default=0.28)
    parser.add_argument("--center-threshold", type=float, default=0.18)
    parser.add_argument("--arrive-area-ratio", type=float, default=0.12)
    parser.add_argument("--lost-timeout", type=float, default=0.5)
    args, _ = parser.parse_known_args()

    rclpy.init()

    node = YoloWorldServo(
        det_topic=args.det_topic,
        cmd_topic=args.cmd_topic,
        image_topic=args.image_topic,
        target_classes=args.target_classes,
        image_width=args.image_width,
        image_height=args.image_height,
        min_score=args.min_score,
        min_red_ratio=args.min_red_ratio,
        max_area_ratio=args.max_area_ratio,
        require_red_verify=not args.no_red_verify,
        kp_turn=args.kp_turn,
        max_vx=args.max_vx,
        max_wz=args.max_wz,
        center_threshold=args.center_threshold,
        arrive_area_ratio=args.arrive_area_ratio,
        lost_timeout=args.lost_timeout,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
