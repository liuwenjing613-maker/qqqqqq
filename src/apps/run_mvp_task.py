#!/usr/bin/env python3
import argparse
import json
import os
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge

# 允许直接从项目根目录运行
PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.vlm.mock_qwen import mock_qwen_parse
from src.perception.target_backend_red import find_red_target
from src.perception.target_backend_yolo import extract_yolo_target
from src.control.mvp_visual_servo import MVPVisualServo
from src.fsm.mvp_state_machine import MVPStateMachine, MVPState


class RunMVPTask(Node):
    def __init__(
        self,
        instruction="find the red backpack",
        image_topic="/image_raw",
        cmd_topic="/cmd_vel",
        backend="red",
        image_width=1280,
        image_height=720,
        det_topic="/hobot_yolo_world",
        target_words_topic="/target_words",
        min_score=0.08,
        det_stale_sec=0.5,
        save_debug=False,
    ):
        super().__init__("run_mvp_task")

        self.instruction = instruction
        self.image_topic = image_topic
        self.cmd_topic = cmd_topic
        self.backend = backend
        self.image_width = image_width
        self.image_height = image_height
        self.det_topic = det_topic
        self.target_words_topic = target_words_topic
        self.min_score = float(min_score)
        self.det_stale_sec = float(det_stale_sec)
        self.save_debug = save_debug

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.qwen_result = mock_qwen_parse(self.instruction)
        self.yolo_prompts = list(self.qwen_result.get("possible_yolo_world_prompts", []))
        self.target_classes = list(self.qwen_result.get("target_classes", []))
        self.latest_yolo_target = {"visible": False}
        self.last_det_time = 0.0

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        if self.backend == "yolo_world":
            self.det_sub = self.create_subscription(
                PerceptionTargets,
                self.det_topic,
                self.det_callback,
                10,
            )
            self.target_words_pub = self.create_publisher(
                String,
                self.target_words_topic,
                10,
            )
            self.create_timer(1.0, self.publish_target_words)
            self.publish_target_words()

        self.servo = MVPVisualServo(
            image_width=self.image_width,
            kp_turn=0.08,
            max_vx=0.01,
            max_wz=0.16,
            center_threshold=0.28,
            arrive_area_ratio=0.20,
        )

        self.fsm = MVPStateMachine(
            stable_frames_required=5,
            lost_frames_limit=8
        )

        self.start_time = time.time()
        self.frame_count = 0
        self.success = False

        self.debug_dir = os.path.join(PROJECT_ROOT, "data/images/mvp_debug")
        if self.save_debug:
            os.makedirs(self.debug_dir, exist_ok=True)

        self.get_logger().info("===== MVP TASK START =====")
        self.get_logger().info(f"instruction: {self.instruction}")
        self.get_logger().info(f"backend: {self.backend}")
        if self.backend == "yolo_world":
            self.get_logger().info(f"det_topic: {self.det_topic}")
            self.get_logger().info(f"target_words_topic: {self.target_words_topic}")
            self.get_logger().info(f"yolo_prompts: {self.yolo_prompts}")
            self.get_logger().info(f"target_classes: {self.target_classes}")
            self.get_logger().info(f"min_score: {self.min_score}")
        self.get_logger().info("Mock Qwen output:")
        self.get_logger().info(json.dumps(self.qwen_result, ensure_ascii=False))

    def publish_target_words(self):
        if self.backend != "yolo_world" or not self.yolo_prompts:
            return
        msg = String()
        msg.data = ",".join(self.yolo_prompts)
        self.target_words_pub.publish(msg)

    def det_callback(self, msg: PerceptionTargets):
        self.last_det_time = time.time()
        self.latest_yolo_target = extract_yolo_target(
            msg,
            target_classes=self.target_classes,
            image_width=self.image_width,
            image_height=self.image_height,
            min_score=self.min_score,
        )

    def resolve_target(self, frame):
        if self.backend == "red":
            return find_red_target(frame)
        if time.time() - self.last_det_time > self.det_stale_sec:
            return {"visible": False, "reason": "det_stale"}
        return dict(self.latest_yolo_target)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_scan_cmd(self):
        """
        目标丢失后的简化恢复：原地慢速扫描。
        注意：只用 angular.z，不用横移。
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.02
        self.cmd_pub.publish(msg)

    def image_callback(self, msg):
        if self.success:
            self.publish_stop()
            return

        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")
            self.publish_stop()
            return

        target = self.resolve_target(frame)

        servo_state, cmd = self.servo.compute_cmd(target)

        target_visible = bool(target.get("visible", False))
        fsm_state = self.fsm.update(target_visible, servo_state)

        if fsm_state == MVPState.RECOVERY_SCAN:
            self.publish_scan_cmd()
            action = "RECOVERY_SCAN_CMD"
        elif fsm_state == MVPState.SUCCESS:
            self.publish_stop()
            self.success = True
            action = "SUCCESS_STOP"
        else:
            self.cmd_pub.publish(cmd)
            action = "SERVO_CMD"

        if target_visible:
            bbox = target.get("bbox", None)
            area_ratio = target.get("area_ratio", 0.0)
            cx = target.get("cx", 0.0)
            ex = (cx - self.image_width / 2.0) / self.image_width
            extra = ""
            if self.backend == "yolo_world":
                extra = (
                    f" class={target.get('class_name', '')}"
                    f" score={target.get('score', 0.0):.3f}"
                )
            self.get_logger().info(
                f"frame={self.frame_count} fsm={fsm_state} servo={servo_state} action={action} "
                f"bbox={bbox} ex={ex:+.3f} area={area_ratio:.3f}{extra} "
                f"cmd_vx={cmd.linear.x:.3f} cmd_wz={cmd.angular.z:.3f}"
            )
        else:
            reason = target.get("reason", "not_visible")
            self.get_logger().info(
                f"frame={self.frame_count} fsm={fsm_state} servo={servo_state} action={action} "
                f"target=LOST reason={reason}"
            )

        if self.save_debug and self.frame_count % 15 == 0:
            vis = frame.copy()
            if target_visible:
                x, y, w, h = target["bbox"]
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                label = f"{fsm_state} {servo_state}"
                if self.backend == "yolo_world":
                    label = (
                        f"{label} {target.get('class_name', '')}"
                        f" {target.get('score', 0.0):.2f}"
                    )
                cv2.putText(
                    vis,
                    label,
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
            save_path = os.path.join(self.debug_dir, f"frame_{self.frame_count:05d}.jpg")
            cv2.imwrite(save_path, vis)

        if self.success:
            elapsed = time.time() - self.start_time
            self.get_logger().info(f"===== TASK SUCCESS, elapsed={elapsed:.2f}s =====")
            self.publish_stop()

    def destroy_node(self):
        self.publish_stop()
        time.sleep(0.2)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--instruction", default="find the red backpack")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--backend", default="red", choices=["red", "yolo_world"])
    parser.add_argument("--det-topic", default="/hobot_yolo_world")
    parser.add_argument("--target-words-topic", default="/target_words")
    parser.add_argument("--min-score", type=float, default=0.08)
    parser.add_argument("--det-stale-sec", type=float, default=0.5)
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--save-debug", action="store_true")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = RunMVPTask(
        instruction=args.instruction,
        image_topic=args.image_topic,
        cmd_topic=args.cmd_topic,
        backend=args.backend,
        image_width=args.image_width,
        image_height=args.image_height,
        det_topic=args.det_topic,
        target_words_topic=args.target_words_topic,
        min_score=args.min_score,
        det_stale_sec=args.det_stale_sec,
        save_debug=args.save_debug,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
