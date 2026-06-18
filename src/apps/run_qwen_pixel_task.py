#!/usr/bin/env python3
import argparse
import json
import os
import sys
import time
from typing import Any, Dict, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.config.qwen_pixel_tune import DEFAULT_TUNE_PATH, load_qwen_pixel_tune
from src.control.pixel_point_servo import PixelPointServo
from src.vlm.qwen_ollama_client import QwenOllamaClient


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class RunQwenPixelTask(Node):
    """
    Qwen 点伺服 MVP

    /image_raw -> Qwen2.5-VL JSON(u,v) -> PixelPointServo -> /cmd_vel burst

    到达：verify 模式 + v/居中条件满足
    模型 JSON 只含 u/v
    """

    def __init__(
        self,
        instruction: str,
        tune: Dict[str, Any],
        max_steps: int,
        save_debug: bool,
    ):
        super().__init__("run_qwen_pixel_task")

        self.instruction = instruction
        self.tune = tune
        self.image_topic = tune["image_topic"]
        self.cmd_topic = tune["cmd_topic"]
        self.json_topic = tune["qwen_json_topic"]
        self.image_width = int(tune["image_width"])
        self.image_height = int(tune["image_height"])

        self.qwen_interval_sec = float(tune["qwen_interval_sec"])
        self.target_lock_conf = float(tune["target_lock_conf"])
        self.verify_v_min = float(tune["verify_v_min"])
        self.verify_u_center_px = float(tune["verify_u_center_px"])
        self.verify_min_forward_bursts = int(tune["verify_min_forward_bursts"])

        self.servo_burst_sec = float(tune["servo_burst_sec"])
        self.observe_stop_sec = float(tune["observe_stop_sec"])
        self.scan_wz = float(tune["scan_wz"])
        self.scan_burst_sec = float(tune["scan_burst_sec"])
        self.scan_observe_sec = float(tune["scan_observe_sec"])
        self.max_steps = int(max_steps)
        self.save_debug = bool(save_debug)

        self.bridge = CvBridge()
        self.latest_frame = None

        self.step_count = 0
        self.forward_burst_count = 0
        self.success = False
        self.verify_mode = False

        self.query_busy = False
        self.next_query_time = 0.0
        self.action_stop_time = 0.0
        self.next_allowed_action_time = 0.0
        self.scan_direction = 1.0
        self.last_cmd = Twist()

        self.debug_dir = os.path.join(PROJECT_ROOT, "data/images/qwen_pixel_debug")
        if self.save_debug:
            os.makedirs(self.debug_dir, exist_ok=True)

        self.qwen = QwenOllamaClient(
            model=tune["model"],
            timeout=tune["qwen_timeout_sec"],
            resize_width=tune["qwen_resize_width"],
            keep_alive=tune.get("qwen_keep_alive", "30m"),
            coord_mode=tune.get("qwen_coord_mode", "norm1000"),
        )

        self.servo = PixelPointServo(
            image_width=self.image_width,
            kp_turn=tune["kp_turn"],
            max_vx=tune["max_vx"],
            max_wz=tune["max_wz"],
            wz_deadzone=tune["wz_deadzone"],
            turn_threshold=tune["turn_threshold"],
            forward_turn_scale=tune["forward_turn_scale"],
            cmd_wz_deadzone=tune["cmd_wz_deadzone"],
        )

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.json_pub = self.create_publisher(String, self.json_topic, 10)

        self.create_timer(0.05, self.control_timer_cb)
        self.create_timer(0.20, self.decision_timer_cb)

        self.get_logger().info("===== QWEN POINT SERVO TASK START =====")
        self.get_logger().info(f"instruction={self.instruction}")
        self.get_logger().info(f"model={tune['model']}")
        self.get_logger().info(
            f"verify: v>={self.verify_v_min} "
            f"|u-center|<={self.verify_u_center_px} "
            f"forward_bursts>={self.verify_min_forward_bursts}"
        )

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")

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

    def _parse_qwen_point(self, result: Dict[str, Any]) -> Dict[str, Any]:
        usable = bool(result.get("usable", False))
        u = result.get("u")
        v = result.get("v")

        if usable and u is not None and v is not None:
            return {
                "visible": True,
                "u": float(u),
                "v": float(v),
                "cx": float(u),
            }

        return {
            "visible": False,
            "u": u,
            "v": v,
            "reason": "no_valid_uv",
        }

        # --- 旧版：依赖 status / target_visible / confidence ---
        # status = str(result.get("status", "searching"))
        # confidence = self._safe_float(result.get("confidence", 0.0), 0.0)
        # can_servo = (
        #     status == "target_locked"
        #     and bool(result.get("target_visible", False))
        #     and point_valid
        #     and confidence >= self.target_lock_conf
        # )

    def _should_enter_verify(self, target: Dict[str, Any]) -> bool:
        if not target.get("visible", False):
            return False
        u = target.get("u")
        v = target.get("v")
        if u is None or v is None:
            return False
        center = self.image_width / 2.0
        if float(v) < self.verify_v_min:
            return False
        if abs(float(u) - center) > self.verify_u_center_px:
            return False
        if self.forward_burst_count < self.verify_min_forward_bursts:
            return False
        return True

    def _publish_json(self, result: Dict[str, Any]):
        payload = {"u": result.get("u"), "v": result.get("v")}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.json_pub.publish(msg)

    def _draw_debug(self, frame, result, target, servo_state, cmd, action):
        vis = frame.copy()
        u = target.get("u")
        v = target.get("v")
        if u is not None and v is not None:
            cx, cy = int(u), int(v)
            cv2.drawMarker(vis, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.circle(vis, (cx, cy), 6, (0, 255, 0), -1)

        lines = [
            f"step={self.step_count} action={action} verify={self.verify_mode} fwd={self.forward_burst_count}",
            f"u={result.get('u')} v={result.get('v')}",
            f"servo={servo_state} cmd vx={cmd.linear.x:.3f} wz={cmd.angular.z:+.3f}",
        ]
        y0 = 24
        for line in lines:
            cv2.putText(vis, line, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y0 += 22

        path = os.path.join(self.debug_dir, f"qwen_step_{self.step_count:04d}_{action}.jpg")
        cv2.imwrite(path, vis)

    def _scan_cmd(self) -> Twist:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.scan_direction * abs(self.scan_wz)
        self.scan_direction *= -1.0
        return msg

    def decision_timer_cb(self):
        if self.success:
            self.publish_stop()
            return

        now = time.time()
        if self.query_busy or self.latest_frame is None:
            return
        if now < self.next_allowed_action_time or now < self.next_query_time:
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
            self.next_query_time = time.time() + 30.0
            self.query_busy = False
            return

        self._publish_json(result)
        target = self._parse_qwen_point(result)
        latency = self._safe_float(result.get("_latency_sec", 0.0), 0.0)

        action = "STOP_OBSERVE"
        servo_state = "LOST_STOP"
        cmd = Twist()

        if self.verify_mode and self._should_enter_verify(target):
            self.publish_stop()
            self.success = True
            action = "VERIFY_SUCCESS"

        elif self.verify_mode:
            if target.get("visible", False):
                servo_state, cmd = self.servo.compute_cmd(target)
                self.publish_cmd_burst(cmd, self.servo_burst_sec, self.observe_stop_sec)
                if servo_state == "FORWARD":
                    self.forward_burst_count += 1
                action = "VERIFY_TRACK"
            else:
                scan_cmd = self._scan_cmd()
                self.publish_cmd_burst(scan_cmd, self.scan_burst_sec, self.scan_observe_sec)
                action = "VERIFY_SEARCH"

        elif target.get("visible", False):
            if self._should_enter_verify(target):
                self.verify_mode = True
                self.get_logger().info(
                    f"enter VERIFY mode u={target.get('u')} v={target.get('v')} "
                    f"forward_bursts={self.forward_burst_count}"
                )

            servo_state, cmd = self.servo.compute_cmd(target)
            self.publish_cmd_burst(cmd, self.servo_burst_sec, self.observe_stop_sec)
            if servo_state == "FORWARD":
                self.forward_burst_count += 1
            action = "POINT_SERVO"

        else:
            scan_cmd = self._scan_cmd()
            self.publish_cmd_burst(scan_cmd, self.scan_burst_sec, self.scan_observe_sec)
            action = "SEARCH_SCAN"

        self.next_query_time = time.time() + self.qwen_interval_sec

        self.get_logger().info(
            f"step={self.step_count} action={action} "
            f"u={result.get('u')} v={result.get('v')} "
            f"latency={latency:.1f}s verify={self.verify_mode} servo={servo_state} "
            f"cmd_vx={cmd.linear.x:.3f} cmd_wz={cmd.angular.z:+.3f}"
        )

        if self.save_debug:
            self._draw_debug(frame, result, target, servo_state, cmd, action)

        self.query_busy = False

    def destroy_node(self):
        self.publish_stop()
        time.sleep(0.2)
        super().destroy_node()


def main():
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--qwen-pixel-tune-config", default=DEFAULT_TUNE_PATH)
    pre_args, _ = pre_parser.parse_known_args()
    tune = load_qwen_pixel_tune(pre_args.qwen_pixel_tune_config)

    parser = argparse.ArgumentParser()
    parser.add_argument("--instruction", default="find the bottle")
    parser.add_argument("--qwen-pixel-tune-config", default=pre_args.qwen_pixel_tune_config)
    parser.add_argument("--image-topic", default=tune["image_topic"])
    parser.add_argument("--cmd-topic", default=tune["cmd_topic"])
    parser.add_argument("--json-topic", default=tune["qwen_json_topic"])
    parser.add_argument("--model", default=tune["model"])
    parser.add_argument("--qwen-interval-sec", type=float, default=tune["qwen_interval_sec"])
    parser.add_argument("--qwen-timeout-sec", type=float, default=tune["qwen_timeout_sec"])
    parser.add_argument("--qwen-resize-width", type=int, default=tune["qwen_resize_width"])
    parser.add_argument("--qwen-coord-mode", default=tune.get("qwen_coord_mode", "norm1000"))
    parser.add_argument("--qwen-keep-alive", default=tune.get("qwen_keep_alive", "30m"))
    parser.add_argument("--target-lock-conf", type=float, default=tune["target_lock_conf"])
    parser.add_argument("--verify-v-min", type=float, default=tune["verify_v_min"])
    parser.add_argument("--verify-u-center-px", type=float, default=tune["verify_u_center_px"])
    parser.add_argument(
        "--verify-min-forward-bursts",
        type=int,
        default=tune["verify_min_forward_bursts"],
    )
    parser.add_argument("--max-vx", type=float, default=tune["max_vx"])
    parser.add_argument("--max-wz", type=float, default=tune["max_wz"])
    parser.add_argument("--kp-turn", type=float, default=tune["kp_turn"])
    parser.add_argument("--wz-deadzone", type=float, default=tune["wz_deadzone"])
    parser.add_argument("--turn-threshold", type=float, default=tune["turn_threshold"])
    parser.add_argument("--forward-turn-scale", type=float, default=tune["forward_turn_scale"])
    parser.add_argument("--cmd-wz-deadzone", type=float, default=tune["cmd_wz_deadzone"])
    parser.add_argument("--servo-burst-sec", type=float, default=tune["servo_burst_sec"])
    parser.add_argument("--observe-stop-sec", type=float, default=tune["observe_stop_sec"])
    parser.add_argument("--scan-wz", type=float, default=tune["scan_wz"])
    parser.add_argument("--scan-burst-sec", type=float, default=tune["scan_burst_sec"])
    parser.add_argument("--scan-observe-sec", type=float, default=tune["scan_observe_sec"])
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--save-debug", action="store_true")
    parser.add_argument("--prep", action="store_true", help="run ollama_prep_infer.sh before spin")
    parser.add_argument(
        "--warmup",
        action="store_true",
        default=True,
        help="warm up Qwen (text+vision) before first infer (default: on)",
    )
    parser.add_argument(
        "--no-warmup",
        action="store_false",
        dest="warmup",
        help="skip Qwen warmup",
    )

    args = parser.parse_args()

    tune.update(
        {
            "model": args.model,
            "image_topic": args.image_topic,
            "cmd_topic": args.cmd_topic,
            "qwen_json_topic": args.json_topic,
            "qwen_interval_sec": args.qwen_interval_sec,
            "qwen_timeout_sec": args.qwen_timeout_sec,
            "qwen_resize_width": args.qwen_resize_width,
            "qwen_coord_mode": args.qwen_coord_mode,
            "qwen_keep_alive": args.qwen_keep_alive,
            "target_lock_conf": args.target_lock_conf,
            "verify_v_min": args.verify_v_min,
            "verify_u_center_px": args.verify_u_center_px,
            "verify_min_forward_bursts": args.verify_min_forward_bursts,
            "max_vx": args.max_vx,
            "max_wz": args.max_wz,
            "kp_turn": args.kp_turn,
            "wz_deadzone": args.wz_deadzone,
            "turn_threshold": args.turn_threshold,
            "forward_turn_scale": args.forward_turn_scale,
            "cmd_wz_deadzone": args.cmd_wz_deadzone,
            "servo_burst_sec": args.servo_burst_sec,
            "observe_stop_sec": args.observe_stop_sec,
            "scan_wz": args.scan_wz,
            "scan_burst_sec": args.scan_burst_sec,
            "scan_observe_sec": args.scan_observe_sec,
        }
    )

    if args.prep:
        import subprocess

        prep = os.path.join(PROJECT_ROOT, "scripts/ollama_prep_infer.sh")
        subprocess.run(["bash", prep, tune["model"]], check=True)

    qwen_client = None
    if args.warmup and not args.prep:
        qwen_client = QwenOllamaClient(
            model=tune["model"],
            timeout=tune["qwen_timeout_sec"],
            resize_width=tune["qwen_resize_width"],
            keep_alive=tune.get("qwen_keep_alive", "30m"),
            coord_mode=tune.get("qwen_coord_mode", "norm1000"),
        )
        print("[run_qwen_pixel_task] warming up Qwen before spin...", flush=True)
        qwen_client.warmup_full(timeout=tune["qwen_timeout_sec"])

    rclpy.init()
    node = RunQwenPixelTask(
        instruction=args.instruction,
        tune=tune,
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
