#!/usr/bin/env python3
import argparse
import json
import os
import sys
import time
from typing import Any, Dict, Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.control.mvp_visual_servo import MVPVisualServo
from src.vlm.qwen_ollama_client import QwenOllamaClient


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class RunQwenPixelTask(Node):
    """
    Qwen-only Pixel Servo MVP

    流程：
    /image_raw -> Qwen2.5-VL JSON -> target_point/bbox -> MVPVisualServo -> /cmd_vel

    注意：
    1. Qwen 只低频看图，不连续控制底盘。
    2. 每次只执行短动作 burst，然后停车观察。
    3. target_locked 才允许前进。
    4. inferred_direction/searching 只允许转向搜索，不允许前进。
    """

    def __init__(
        self,
        instruction: str,
        image_topic: str,
        cmd_topic: str,
        json_topic: str,
        model: str,
        qwen_interval_sec: float,
        qwen_timeout_sec: float,
        qwen_resize_width: int,
        target_lock_conf: float,
        inferred_conf: float,
        max_vx: float,
        max_wz: float,
        kp_turn: float,
        wz_deadzone: float,
        turn_threshold: float,
        forward_turn_scale: float,
        cmd_wz_deadzone: float,
        arrive_area_ratio: float,
        default_area_ratio: float,
        servo_burst_sec: float,
        observe_stop_sec: float,
        scan_wz: float,
        scan_burst_sec: float,
        scan_observe_sec: float,
        max_steps: int,
        save_debug: bool,
    ):
        super().__init__("run_qwen_pixel_task")

        self.instruction = instruction
        self.image_topic = image_topic
        self.cmd_topic = cmd_topic
        self.json_topic = json_topic
        self.model = model

        self.qwen_interval_sec = float(qwen_interval_sec)
        self.target_lock_conf = float(target_lock_conf)
        self.inferred_conf = float(inferred_conf)

        self.default_area_ratio = float(default_area_ratio)
        self.servo_burst_sec = float(servo_burst_sec)
        self.observe_stop_sec = float(observe_stop_sec)
        self.scan_wz = float(scan_wz)
        self.scan_burst_sec = float(scan_burst_sec)
        self.scan_observe_sec = float(scan_observe_sec)
        self.max_steps = int(max_steps)
        self.save_debug = bool(save_debug)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_stamp = None

        self.frame_count = 0
        self.step_count = 0
        self.success = False

        self.query_busy = False
        self.next_query_time = 0.0
        self.action_stop_time = 0.0
        self.next_allowed_action_time = 0.0
        self.scan_direction = 1.0
        self.last_qwen_result = None
        self.last_cmd = Twist()

        self.debug_dir = os.path.join(PROJECT_ROOT, "data/images/qwen_pixel_debug")
        if self.save_debug:
            os.makedirs(self.debug_dir, exist_ok=True)

        self.qwen = QwenOllamaClient(
            model=self.model,
            timeout=qwen_timeout_sec,
            resize_width=qwen_resize_width,
            jpeg_quality=80,
        )

        self.servo = MVPVisualServo(
            image_width=1280,
            kp_turn=float(kp_turn),
            max_vx=float(max_vx),
            max_wz=float(max_wz),
            center_threshold=float(turn_threshold),
            arrive_area_ratio=float(arrive_area_ratio),
            turn_threshold=float(turn_threshold),
            wz_deadzone=float(wz_deadzone),
            forward_turn_scale=float(forward_turn_scale),
            cmd_wz_deadzone=float(cmd_wz_deadzone),
        )

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.json_pub = self.create_publisher(String, self.json_topic, 10)

        self.control_timer = self.create_timer(0.05, self.control_timer_cb)
        self.decision_timer = self.create_timer(0.20, self.decision_timer_cb)

        self.get_logger().info("===== QWEN PIXEL TASK START =====")
        self.get_logger().info(f"instruction={self.instruction}")
        self.get_logger().info(f"image_topic={self.image_topic}")
        self.get_logger().info(f"cmd_topic={self.cmd_topic}")
        self.get_logger().info(f"json_topic={self.json_topic}")
        self.get_logger().info(f"model={self.model}")
        self.get_logger().info(
            f"servo max_vx={max_vx} max_wz={max_wz} kp_turn={kp_turn} "
            f"wz_deadzone={wz_deadzone} turn_threshold={turn_threshold} "
            f"arrive_area={arrive_area_ratio}"
        )
        self.get_logger().info(
            f"qwen interval={qwen_interval_sec}s timeout={qwen_timeout_sec}s "
            f"target_lock_conf={target_lock_conf} inferred_conf={inferred_conf}"
        )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")
            return

        self.latest_frame = frame
        self.latest_stamp = msg.header.stamp
        self.frame_count += 1

    def publish_stop(self):
        self.last_cmd = Twist()
        self.cmd_pub.publish(Twist())

    def publish_cmd_burst(self, cmd: Twist, duration: float, observe_after: float):
        now = time.time()
        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd
        self.action_stop_time = now + float(duration)
        self.next_allowed_action_time = self.action_stop_time + float(observe_after)

    def control_timer_cb(self):
        now = time.time()
        if self.action_stop_time > 0.0 and now >= self.action_stop_time:
            self.publish_stop()
            self.action_stop_time = 0.0

    def _safe_float(self, x, default=0.0):
        try:
            return float(x)
        except Exception:
            return float(default)

    def _scale_point_to_orig(self, result: Dict[str, Any], point) -> Optional[Tuple[float, float]]:
        if point is None or not isinstance(point, list) or len(point) < 2:
            return None

        sx = self._safe_float(result.get("_scale_x_to_orig", 1.0), 1.0)
        sy = self._safe_float(result.get("_scale_y_to_orig", 1.0), 1.0)
        orig_w = int(result.get("_orig_image_width", 1280))
        orig_h = int(result.get("_orig_image_height", 720))

        x = self._safe_float(point[0]) * sx
        y = self._safe_float(point[1]) * sy

        x = clamp(x, 0, orig_w - 1)
        y = clamp(y, 0, orig_h - 1)
        return x, y

    def _scale_bbox_to_orig(self, result: Dict[str, Any], bbox):
        if bbox is None or not isinstance(bbox, list) or len(bbox) < 4:
            return None

        sx = self._safe_float(result.get("_scale_x_to_orig", 1.0), 1.0)
        sy = self._safe_float(result.get("_scale_y_to_orig", 1.0), 1.0)
        orig_w = int(result.get("_orig_image_width", 1280))
        orig_h = int(result.get("_orig_image_height", 720))

        x1 = clamp(self._safe_float(bbox[0]) * sx, 0, orig_w - 1)
        y1 = clamp(self._safe_float(bbox[1]) * sy, 0, orig_h - 1)
        x2 = clamp(self._safe_float(bbox[2]) * sx, 0, orig_w - 1)
        y2 = clamp(self._safe_float(bbox[3]) * sy, 0, orig_h - 1)

        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1

        return [int(x1), int(y1), int(x2), int(y2)]

    def _target_from_qwen(self, result: Dict[str, Any]) -> Dict[str, Any]:
        """
        把 Qwen JSON 转成 MVPVisualServo 需要的 target。
        """
        status = str(result.get("status", "searching"))
        confidence = self._safe_float(result.get("confidence", 0.0), 0.0)
        visible = bool(result.get("target_visible", False))

        point = self._scale_point_to_orig(result, result.get("target_point"))
        bbox = self._scale_bbox_to_orig(result, result.get("target_bbox"))

        orig_w = int(result.get("_orig_image_width", 1280))
        orig_h = int(result.get("_orig_image_height", 720))

        if point is None and bbox is not None:
            x1, y1, x2, y2 = bbox
            point = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

        area_ratio = self.default_area_ratio
        bbox_xywh = None
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            bw = max(1, x2 - x1)
            bh = max(1, y2 - y1)
            area_ratio = (bw * bh) / float(orig_w * orig_h)
            bbox_xywh = [int(x1), int(y1), int(bw), int(bh)]

        can_servo = (
            status in ("target_locked", "verified_success")
            and visible
            and point is not None
            and confidence >= self.target_lock_conf
        )

        if not can_servo:
            return {
                "visible": False,
                "reason": f"qwen_not_locked status={status} conf={confidence:.2f}",
                "status": status,
                "confidence": confidence,
                "point": point,
                "bbox": bbox_xywh,
                "area_ratio": area_ratio,
            }

        return {
            "visible": True,
            "cx": float(point[0]),
            "cy": float(point[1]),
            "bbox": bbox_xywh if bbox_xywh is not None else [int(point[0]) - 20, int(point[1]) - 20, 40, 40],
            "area_ratio": float(area_ratio),
            "status": status,
            "confidence": confidence,
            "class_name": result.get("target_category", "unknown"),
            "reason": result.get("reason", ""),
        }

    def _publish_json(self, result: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.json_pub.publish(msg)

    def _draw_debug(self, frame, result, target, servo_state, cmd, action):
        vis = frame.copy()
        h, w = vis.shape[:2]

        if target.get("visible", False):
            x, y, bw, bh = target.get("bbox", [0, 0, 0, 0])
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cx = int(target.get("cx", x + bw / 2))
            cy = int(target.get("cy", y + bh / 2))
            cv2.circle(vis, (cx, cy), 6, (0, 0, 255), -1)

        lines = [
            f"step={self.step_count} action={action} servo={servo_state}",
            f"status={result.get('status')} visible={result.get('target_visible')} conf={result.get('confidence')}",
            f"hint={result.get('action_hint')} search={result.get('search_direction')}",
            f"cmd vx={cmd.linear.x:.3f} wz={cmd.angular.z:+.3f}",
            f"reason={result.get('reason')}",
        ]

        y0 = 24
        for line in lines:
            cv2.rectangle(vis, (8, y0 - 18), (min(w - 1, 8 + len(line) * 9), y0 + 6), (0, 0, 0), -1)
            cv2.putText(vis, line, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y0 += 26

        path = os.path.join(self.debug_dir, f"qwen_step_{self.step_count:04d}_{action}.jpg")
        cv2.imwrite(path, vis)

    def _scan_cmd_from_direction(self, direction: str) -> Twist:
        msg = Twist()
        msg.linear.x = 0.0

        if direction in ("right", "front_right"):
            msg.angular.z = -abs(self.scan_wz)
            self.scan_direction = -1.0
        elif direction in ("left", "front_left"):
            msg.angular.z = +abs(self.scan_wz)
            self.scan_direction = +1.0
        else:
            msg.angular.z = self.scan_direction * abs(self.scan_wz)

        return msg

    def decision_timer_cb(self):
        if self.success:
            self.publish_stop()
            return

        now = time.time()

        if self.query_busy:
            return

        if self.latest_frame is None:
            self.get_logger().warn("no image yet")
            self.publish_stop()
            return

        if now < self.next_allowed_action_time:
            return

        if now < self.next_query_time:
            return

        if self.step_count >= self.max_steps:
            self.get_logger().warn("max_steps reached, stop")
            self.publish_stop()
            self.success = True
            return

        self.query_busy = True
        self.step_count += 1
        frame = self.latest_frame.copy()

        try:
            result = self.qwen.infer_navigation(frame, self.instruction)
        except Exception as e:
            self.get_logger().error(f"Qwen infer failed: {repr(e)}")
            self.publish_stop()
            self.next_query_time = time.time() + 2.0
            self.query_busy = False
            return

        self.last_qwen_result = result
        self._publish_json(result)

        status = str(result.get("status", "searching"))
        confidence = self._safe_float(result.get("confidence", 0.0), 0.0)
        action_hint = str(result.get("action_hint", "search"))
        search_direction = str(result.get("search_direction", "unknown"))

        target = self._target_from_qwen(result)
        servo_state, cmd = self.servo.compute_cmd(target)

        action = "STOP_OBSERVE"

        if status == "verified_success" or bool(result.get("stop", False)) and status == "verified_success":
            self.publish_stop()
            self.success = True
            action = "SUCCESS_STOP"

        elif target.get("visible", False):
            if servo_state == "ARRIVED_STOP":
                self.publish_stop()
                self.success = True
                action = "ARRIVED_STOP"
            else:
                # target_locked 才允许前进/转向
                self.publish_cmd_burst(cmd, self.servo_burst_sec, self.observe_stop_sec)
                action = "QWEN_PIXEL_SERVO"

        elif status == "inferred_direction" and confidence >= self.inferred_conf:
            # 推测方向只能转向，不能前进
            scan_cmd = self._scan_cmd_from_direction(search_direction)
            self.publish_cmd_burst(scan_cmd, self.scan_burst_sec, self.scan_observe_sec)
            action = "INFERRED_TURN_SEARCH"

        elif status == "searching" or action_hint == "search":
            scan_cmd = self._scan_cmd_from_direction(search_direction)
            self.publish_cmd_burst(scan_cmd, self.scan_burst_sec, self.scan_observe_sec)
            action = "SEARCH_SCAN"

        elif status == "unsafe":
            self.publish_stop()
            action = "UNSAFE_STOP"

        else:
            self.publish_stop()
            action = "LOW_CONF_STOP"

        self.next_query_time = time.time() + self.qwen_interval_sec

        point = result.get("target_point")
        bbox = result.get("target_bbox")
        latency = self._safe_float(result.get("_latency_sec", 0.0), 0.0)

        self.get_logger().info(
            f"step={self.step_count} action={action} status={status} "
            f"qwen_conf={confidence:.2f} latency={latency:.2f}s "
            f"point={point} bbox={bbox} servo={servo_state} "
            f"cmd_vx={cmd.linear.x:.3f} cmd_wz={cmd.angular.z:+.3f} "
            f"reason={result.get('reason')}"
        )

        if self.save_debug:
            self._draw_debug(frame, result, target, servo_state, cmd, action)

        self.query_busy = False

    def destroy_node(self):
        self.publish_stop()
        time.sleep(0.2)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--instruction", default="find the bottle")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--json-topic", default="/qwen_nav_json")
    parser.add_argument("--model", default="moondream:latest")

    parser.add_argument("--qwen-interval-sec", type=float, default=240.0)
    parser.add_argument("--qwen-timeout-sec", type=float, default=900.0)
    parser.add_argument("--qwen-resize-width", type=int, default=256)

    parser.add_argument("--target-lock-conf", type=float, default=0.45)
    parser.add_argument("--inferred-conf", type=float, default=0.30)

    parser.add_argument("--max-vx", type=float, default=0.035)
    parser.add_argument("--max-wz", type=float, default=0.045)
    parser.add_argument("--kp-turn", type=float, default=0.09)
    parser.add_argument("--wz-deadzone", type=float, default=0.08)
    parser.add_argument("--turn-threshold", type=float, default=0.28)
    parser.add_argument("--forward-turn-scale", type=float, default=0.45)
    parser.add_argument("--cmd-wz-deadzone", type=float, default=0.012)
    parser.add_argument("--arrive-area-ratio", type=float, default=0.20)
    parser.add_argument("--default-area-ratio", type=float, default=0.05)

    parser.add_argument("--servo-burst-sec", type=float, default=0.25)
    parser.add_argument("--observe-stop-sec", type=float, default=0.55)

    parser.add_argument("--scan-wz", type=float, default=0.040)
    parser.add_argument("--scan-burst-sec", type=float, default=0.18)
    parser.add_argument("--scan-observe-sec", type=float, default=0.55)

    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--save-debug", action="store_true")

    args = parser.parse_args()

    rclpy.init()
    node = RunQwenPixelTask(
        instruction=args.instruction,
        image_topic=args.image_topic,
        cmd_topic=args.cmd_topic,
        json_topic=args.json_topic,
        model=args.model,
        qwen_interval_sec=args.qwen_interval_sec,
        qwen_timeout_sec=args.qwen_timeout_sec,
        qwen_resize_width=args.qwen_resize_width,
        target_lock_conf=args.target_lock_conf,
        inferred_conf=args.inferred_conf,
        max_vx=args.max_vx,
        max_wz=args.max_wz,
        kp_turn=args.kp_turn,
        wz_deadzone=args.wz_deadzone,
        turn_threshold=args.turn_threshold,
        forward_turn_scale=args.forward_turn_scale,
        cmd_wz_deadzone=args.cmd_wz_deadzone,
        arrive_area_ratio=args.arrive_area_ratio,
        default_area_ratio=args.default_area_ratio,
        servo_burst_sec=args.servo_burst_sec,
        observe_stop_sec=args.observe_stop_sec,
        scan_wz=args.scan_wz,
        scan_burst_sec=args.scan_burst_sec,
        scan_observe_sec=args.scan_observe_sec,
        max_steps=args.max_steps,
        save_debug=args.save_debug,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()