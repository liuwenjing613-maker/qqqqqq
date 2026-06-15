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
from src.perception.stamp_sync import StampSyncBuffer
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
        min_score=0.002,
        det_stale_sec=1.0,
        save_debug=False,
        save_bbox_dir=None,
        save_bbox_interval=15,
        min_red_ratio=0.06,
        max_area_ratio=0.15,
        require_red_verify=True,
        sync_max_delta_sec=0.12,
        sync_buffer_len=60,
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
        self.save_bbox_interval = max(1, int(save_bbox_interval))
        self.save_bbox_dir = (
            os.path.expanduser(save_bbox_dir)
            if save_bbox_dir
            else os.path.join(PROJECT_ROOT, "check_bbox")
        )
        self.save_bbox = bool(save_bbox_dir) or save_debug
        self.found_bbox_count = 0
        self.min_red_ratio = float(min_red_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.require_red_verify = bool(require_red_verify)
        self.sync_max_delta_sec = float(sync_max_delta_sec)

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.qwen_result = mock_qwen_parse(self.instruction)
        self.yolo_prompts = list(self.qwen_result.get("possible_yolo_world_prompts", []))
        self.target_classes = list(self.qwen_result.get("target_classes", []))
        self.det_buffer = StampSyncBuffer(
            max_len=sync_buffer_len,
            max_delta_sec=self.sync_max_delta_sec,
        )
        self.frame_buffer = StampSyncBuffer(
            max_len=sync_buffer_len,
            max_delta_sec=self.sync_max_delta_sec,
        )
        self.last_det_time = 0.0
        self.last_sync_warn_time = 0.0

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
        if self.save_bbox:
            os.makedirs(self.save_bbox_dir, exist_ok=True)

        self.get_logger().info("===== MVP TASK START =====")
        self.get_logger().info(f"instruction: {self.instruction}")
        self.get_logger().info(f"backend: {self.backend}")
        if self.backend == "yolo_world":
            self.get_logger().info(f"det_topic: {self.det_topic}")
            self.get_logger().info(f"target_words_topic: {self.target_words_topic}")
            self.get_logger().info(f"yolo_prompts: {self.yolo_prompts}")
            self.get_logger().info(f"target_classes: {self.target_classes}")
            self.get_logger().info(f"min_score: {self.min_score}")
            self.get_logger().info(f"min_red_ratio: {self.min_red_ratio}")
            self.get_logger().info(f"max_area_ratio: {self.max_area_ratio}")
            self.get_logger().info(f"require_red_verify: {self.require_red_verify}")
            self.get_logger().info(
                f"stamp_sync max_delta={self.sync_max_delta_sec}s buffer={sync_buffer_len}"
            )
        if self.save_bbox:
            self.get_logger().info(
                f"save_bbox_dir={self.save_bbox_dir} interval={self.save_bbox_interval}"
            )
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
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self.det_buffer.push(stamp, msg)

    def resolve_target(self, frame, image_stamp=None):
        if self.backend == "red":
            return find_red_target(frame), frame
        det_msg = self.det_buffer.peek_latest()
        if det_msg is None:
            return {"visible": False, "reason": "no_det_msg"}, frame
        if time.time() - self.last_det_time > self.det_stale_sec:
            return {"visible": False, "reason": "det_stale"}, frame
        matched_frame, delta = self.frame_buffer.find_closest(det_msg.header.stamp)
        if matched_frame is None:
            now = time.time()
            if now - self.last_sync_warn_time >= 1.0:
                self.last_sync_warn_time = now
                delta_str = f"{delta:.3f}s" if delta is not None else "no_frame"
                self.get_logger().warn(
                    f"stamp sync skip: no frame for det stamp, delta {delta_str} "
                    f"> {self.sync_max_delta_sec}s (frame_buf={len(self.frame_buffer)})"
                )
            return {"visible": False, "reason": "stamp_sync_failed"}, frame
        target = extract_yolo_target(
            det_msg,
            target_classes=self.target_classes,
            image_width=self.image_width,
            image_height=self.image_height,
            min_score=self.min_score,
            max_area_ratio=self.max_area_ratio,
            frame=matched_frame,
            min_red_ratio=self.min_red_ratio,
            require_red_verify=self.require_red_verify,
        )
        return target, matched_frame

    def _draw_bbox_overlay(self, img, target, fsm_state, servo_state, trigger):
        h, w = img.shape[:2]
        status = "FOUND" if target.get("visible", False) else "NO_MVP"
        reject_hint = ""
        if status == "NO_MVP" and target.get("reason"):
            reject_hint = f" reject={target['reason']}"

        if target.get("visible", False):
            x, y, bw, bh = target["bbox"]
            cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
            red_ratio = target.get("red_ratio", 0.0)
            label = (
                f"MVP {target.get('class_name', '')} {target.get('score', 0.0):.3f} "
                f"r={red_ratio:.2f}"
            )
            cv2.putText(
                img,
                label,
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )

        lines = [
            f"MVP task frame={self.frame_count} found={self.found_bbox_count} fsm={fsm_state}",
            (
                f"backend={self.backend} min_score={self.min_score} "
                f"red_min={self.min_red_ratio} max_area={self.max_area_ratio}"
            ),
            f"status={status}{reject_hint} trigger={trigger} servo={servo_state}",
            f"{w}x{h} topic={self.image_topic}",
        ]
        if target.get("visible", False):
            lines.append(
                f"bbox={target.get('bbox')} area={target.get('area_ratio', 0.0):.3f}"
            )
        elif target.get("class_name"):
            lines.append(
                f"best={target.get('class_name')} score={target.get('score', 0.0):.4f} "
                f"bbox={target.get('bbox')}"
            )

        y0 = 12
        for line in lines:
            cv2.rectangle(img, (8, y0), (8 + len(line) * 9 + 10, y0 + 22), (0, 0, 0), -1)
            cv2.putText(
                img, line, (12, y0 + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
            )
            y0 += 24
        return status

    def save_bbox_snapshot(self, vis_frame, target, fsm_state, servo_state, trigger):
        if vis_frame is None:
            return
        vis = vis_frame.copy()
        status = self._draw_bbox_overlay(vis, target, fsm_state, servo_state, trigger)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        save_path = os.path.join(
            self.save_bbox_dir,
            f"mvp_{stamp}_f{self.frame_count}_{status}.jpg",
        )
        cv2.imwrite(save_path, vis)
        self.get_logger().info(f"saved bbox snapshot: {save_path}")

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

        self.frame_buffer.push(msg.header.stamp, frame)

        target, vis_frame = self.resolve_target(frame, msg.header.stamp)

        servo_state, cmd = self.servo.compute_cmd(target)

        target_visible = bool(target.get("visible", False))
        if target_visible:
            self.found_bbox_count += 1
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

        if self.save_bbox:
            if target_visible:
                self.save_bbox_snapshot(vis_frame, target, fsm_state, servo_state, "found")
            elif self.frame_count % self.save_bbox_interval == 0:
                self.save_bbox_snapshot(vis_frame, target, fsm_state, servo_state, "interval")

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
    parser.add_argument("--min-score", type=float, default=0.002)
    parser.add_argument("--min-red-ratio", type=float, default=0.06)
    parser.add_argument("--max-area-ratio", type=float, default=0.15)
    parser.add_argument("--no-red-verify", action="store_true")
    parser.add_argument("--det-stale-sec", type=float, default=1.0)
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--save-debug", action="store_true")
    parser.add_argument(
        "--save-bbox-dir",
        default=None,
        help="Save annotated snapshots to this dir (e.g. check_bbox); set to enable",
    )
    parser.add_argument("--save-bbox-interval", type=int, default=15)
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
        save_bbox_dir=args.save_bbox_dir,
        save_bbox_interval=args.save_bbox_interval,
        min_red_ratio=args.min_red_ratio,
        max_area_ratio=args.max_area_ratio,
        require_red_verify=not args.no_red_verify,
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
