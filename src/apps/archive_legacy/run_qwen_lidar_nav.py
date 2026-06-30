#!/usr/bin/env python3
"""
Qwen-only + LiDAR navigation node.

/image_raw + /scan -> Qwen u/v -> point servo -> /cmd_vel.
This branch does NOT use YOLO-World.
"""
import argparse
import json
import os
import sys
import time
import subprocess
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, Optional

import cv2
import yaml
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from src.vlm.qwen_ollama_client import QwenOllamaClient
from src.perception.lidar_depth import LidarDepthEstimator
from src.control.qwen_lidar_point_servo import QwenLidarPointServo


DEFAULT_CONFIG = os.path.join(PROJECT_ROOT, "configs/qwen_lidar_nav.yaml")


def load_yaml(path: str) -> Dict[str, Any]:
    path = os.path.expanduser(path)
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise RuntimeError(
            f"Config file did not parse into a dict: {path}. "
            "Your YAML may have been compressed into one commented line."
        )
    return cfg


class RunQwenLidarNav(Node):
    def __init__(self, instruction: str, cfg: Dict[str, Any]):
        super().__init__("run_qwen_lidar_nav")

        self.instruction = instruction
        self.cfg = cfg
        self.require_lidar = bool(cfg.get("require_lidar", True))
        self.recover_on_timeout = bool(cfg.get("recover_on_timeout", True))
        self.recover_script = os.path.expanduser(str(cfg.get(
            "recover_script",
            os.path.join(PROJECT_ROOT, "scripts/qwen/ollama_recover.sh"),
        )))

        self.image_topic = cfg["image_topic"]
        self.scan_topic = cfg["scan_topic"]
        self.cmd_topic = cfg["cmd_topic"]
        self.json_topic = cfg["qwen_json_topic"]
        self.state_topic = cfg["state_topic"]

        self.image_width = int(cfg["image_width"])
        self.image_height = int(cfg["image_height"])

        self.bridge = CvBridge()
        self.latest_frame: Optional[Any] = None
        self.latest_scan: Optional[LaserScan] = None
        self.last_scan_time: Optional[float] = None

        self.query_busy = False
        self.qwen_executor = ThreadPoolExecutor(max_workers=1)
        self.qwen_future = None
        self.qwen_future_frame = None
        self.step_count = 0
        self.forward_burst_count = 0
        self.arrive_count = 0
        self.lost_scan_count = 0
        self.scan_direction = 1.0

        self.success = False
        self.action_stop_time = 0.0
        self.next_allowed_action_time = 0.0
        self.next_query_time = 0.0
        self.last_cmd = Twist()

        self.qwen_interval_sec = float(cfg["qwen_interval_sec"])
        self.servo_burst_sec = float(cfg["servo_burst_sec"])
        self.observe_stop_sec = float(cfg["observe_stop_sec"])
        self.scan_wz = float(cfg["scan_wz"])
        self.scan_burst_sec = float(cfg["scan_burst_sec"])
        self.scan_observe_sec = float(cfg["scan_observe_sec"])
        self.max_steps = int(cfg["max_steps"])

        self.arrive_distance = float(cfg["arrive_distance"])
        self.center_arrive_px = float(cfg["center_arrive_px"])
        self.arrive_required_count = int(cfg["arrive_required_count"])
        self.min_forward_bursts_before_arrive = int(cfg["min_forward_bursts_before_arrive"])

        self.max_scan_age_sec = float(cfg.get("max_scan_age_sec", 1.5))
        self.timeout_backoff_sec = float(cfg.get("timeout_backoff_sec", 8.0))
        self.lidar_wait_backoff_sec = float(cfg.get("lidar_wait_backoff_sec", 1.0))

        debug_dir_cfg = cfg.get("debug_dir", "data/images/qwen_lidar_debug")
        self.debug_dir = debug_dir_cfg if os.path.isabs(debug_dir_cfg) else os.path.join(PROJECT_ROOT, debug_dir_cfg)
        self.save_debug = bool(cfg.get("save_debug", True))
        if self.save_debug:
            os.makedirs(self.debug_dir, exist_ok=True)

        self.qwen = QwenOllamaClient(
            model=cfg["model"],
            timeout=float(cfg["qwen_timeout_sec"]),
            resize_width=int(cfg["qwen_resize_width"]),
            jpeg_quality=int(cfg.get("qwen_jpeg_quality", 45)),
            num_predict=int(cfg.get("qwen_num_predict", 16)),
            num_ctx=int(cfg.get("qwen_num_ctx", 256)),
            keep_alive=cfg.get("qwen_keep_alive", -1),
            coord_mode=cfg.get("qwen_coord_mode", "norm1000"),
            debug_dir=self.debug_dir,
            save_debug=self.save_debug,
            min_confidence=float(cfg.get("min_confidence", 0.0)),
        )

        self.lidar = LidarDepthEstimator(
            min_range=float(cfg["lidar_min_range"]),
            max_range=float(cfg["lidar_max_range"]),
            front_deg=float(cfg["lidar_front_deg"]),
            target_window_deg=float(cfg["lidar_target_window_deg"]),
            camera_hfov_deg=float(cfg["camera_hfov_deg"]),
            camera_lidar_yaw_offset_deg=float(cfg["camera_lidar_yaw_offset_deg"]),
        )

        self.servo = QwenLidarPointServo(
            image_width=self.image_width,
            require_lidar=self.require_lidar,
            kp_turn=float(cfg["kp_turn"]),
            max_wz=float(cfg["max_wz"]),
            wz_deadzone=float(cfg["wz_deadzone"]),
            cmd_wz_deadzone=float(cfg["cmd_wz_deadzone"]),
            turn_threshold=float(cfg["turn_threshold"]),
            forward_turn_scale=float(cfg["forward_turn_scale"]),
            max_vx=float(cfg["max_vx"]),
            mid_vx=float(cfg["mid_vx"]),
            slow_vx=float(cfg["slow_vx"]),
            min_vx=float(cfg["min_vx"]),
            emergency_stop_distance=float(cfg["emergency_stop_distance"]),
            hard_stop_distance=float(cfg["hard_stop_distance"]),
            slow_distance=float(cfg["slow_distance"]),
            normal_distance=float(cfg["normal_distance"]),
        )

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.json_pub = self.create_publisher(String, self.json_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.create_timer(0.05, self.control_timer_cb)
        self.create_timer(0.20, self.decision_timer_cb)

        self.get_logger().info("===== QWEN-ONLY + LIDAR NAV START =====")
        self.get_logger().info(f"instruction={instruction}")
        self.get_logger().info(f"image_topic={self.image_topic} scan_topic={self.scan_topic} cmd_topic={self.cmd_topic}")
        self.get_logger().info(
            "qwen params: "
            f"w={cfg['qwen_resize_width']} q={cfg.get('qwen_jpeg_quality', 45)} "
            f"np={cfg.get('qwen_num_predict', 16)} ctx={cfg.get('qwen_num_ctx', 256)} "
            f"timeout={cfg['qwen_timeout_sec']}s keep_alive={cfg.get('qwen_keep_alive', -1)}"
        )

        self.get_logger().info("waiting for first camera frame; step=1 will be first infer (no discard warmup)")

    def _sync_image_geometry(self, frame) -> None:
        frame_h, frame_w = frame.shape[:2]
        if frame_w != self.image_width or frame_h != self.image_height:
            self.get_logger().warn(
                f"frame size mismatch current={self.image_width}x{self.image_height}, "
                f"actual={frame_w}x{frame_h}; using actual"
            )
            self.image_width = int(frame_w)
            self.image_height = int(frame_h)
            self.servo.update_image_width(frame_w)

    def _maybe_recover_ollama(self) -> None:
        if not self.recover_on_timeout:
            return
        if not os.path.exists(self.recover_script):
            self.get_logger().warn(f"recover script not found, skip: {self.recover_script}")
            return
        self.get_logger().warn(f"running Ollama recover script: {self.recover_script}")
        try:
            subprocess.run(["bash", self.recover_script], timeout=30, check=False)
        except Exception as e:
            self.get_logger().error(f"ollama recover failed: {repr(e)}")

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.last_scan_time = time.time()
        self.lidar.update_scan(msg)

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

    def _parse_qwen_point(self, result: Dict[str, Any]) -> Dict[str, Any]:
        usable = bool(result.get("usable", result.get("_point_valid", False)))
        u = result.get("u")
        v = result.get("v")
        if usable and u is not None and v is not None:
            return {"visible": True, "u": float(u), "v": float(v), "cx": float(u)}
        return {"visible": False, "u": u, "v": v, "reason": result.get("_coord_reason", "no_valid_uv")}

    def _scan_cmd(self) -> Twist:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.scan_direction * abs(self.scan_wz)
        self.scan_direction *= -1.0
        return msg

    def _scan_is_fresh(self) -> bool:
        if self.latest_scan is None or self.last_scan_time is None:
            return False
        return (time.time() - self.last_scan_time) <= self.max_scan_age_sec

    def _check_arrive(self, target: Dict[str, Any], front_distance: Optional[float], target_distance: Optional[float]) -> bool:
        if not target.get("visible", False):
            self.arrive_count = 0
            return False
        u = target.get("u")
        if u is None:
            self.arrive_count = 0
            return False
        center = self.image_width / 2.0
        centered = abs(float(u) - center) <= self.center_arrive_px
        depth_used = target_distance if target_distance is not None else front_distance
        close_enough = depth_used is not None and depth_used <= self.arrive_distance
        moved_enough = self.forward_burst_count >= self.min_forward_bursts_before_arrive
        if centered and close_enough and moved_enough:
            self.arrive_count += 1
        else:
            self.arrive_count = 0
        return self.arrive_count >= self.arrive_required_count

    def _publish_json(self, result: Dict[str, Any], state: str, lidar_state: Dict[str, Any]):
        payload = {
            "u": result.get("u"),
            "v": result.get("v"),
            "raw_u": result.get("_raw_u"),
            "raw_v": result.get("_raw_v"),
            "usable": bool(result.get("usable", result.get("_point_valid", False))),
            "state": state,
            "front_distance": lidar_state.get("front_distance"),
            "target_distance": lidar_state.get("target_distance"),
            "target_angle_deg": lidar_state.get("target_angle_deg"),
            "latency_sec": result.get("_latency_sec"),
            "ollama_total_ms": result.get("_ollama_total_ms"),
            "ollama_load_ms": result.get("_ollama_load_ms"),
            "ollama_prompt_eval_ms": result.get("_ollama_prompt_eval_ms"),
            "ollama_eval_ms": result.get("_ollama_eval_ms"),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.json_pub.publish(msg)

    def _publish_state(self, payload: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)

    def _draw_debug(self, frame, target: Dict[str, Any], servo_res, action: str):
        if not self.save_debug:
            return
        vis = frame.copy()
        u, v = target.get("u"), target.get("v")
        if u is not None and v is not None:
            cv2.drawMarker(vis, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
            cv2.circle(vis, (int(u), int(v)), 8, (0, 255, 0), 2)
        lines = [
            f"step={self.step_count} action={action}",
            f"u={u} v={v} state={servo_res.state}",
            f"vx={servo_res.cmd.linear.x:.3f} wz={servo_res.cmd.angular.z:+.3f}",
            f"front={servo_res.front_distance} target_d={servo_res.target_distance}",
            f"reason={servo_res.reason}",
        ]
        y = 24
        for line in lines:
            cv2.putText(vis, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            y += 24
        os.makedirs(self.debug_dir, exist_ok=True)
        safe_action = "".join(c if c.isalnum() or c in "_-" else "_" for c in action)
        path = os.path.join(self.debug_dir, f"qwen_lidar_{self.step_count:04d}_{safe_action}.jpg")
        cv2.imwrite(path, vis)

    def _submit_qwen_infer(self, frame) -> None:
        if self.qwen_future is not None:
            return
        self.query_busy = True
        self.qwen_future_frame = frame
        self.step_count += 1
        self.qwen_future = self.qwen_executor.submit(self.qwen.infer_navigation, frame, self.instruction)
        self.publish_stop()
        if self.step_count == 1:
            self.get_logger().info(
                f"submitted first infer as step=1 ({frame.shape[1]}x{frame.shape[0]}); "
                "result will drive navigation (no separate warmup)"
            )
        else:
            self.get_logger().info(f"submitted async Qwen infer step={self.step_count}")

    def _poll_qwen_future(self) -> bool:
        if self.qwen_future is None:
            return False

        if not self.qwen_future.done():
            return True

        frame = self.qwen_future_frame

        try:
            result = self.qwen_future.result()
        except Exception as e:
            self.get_logger().error(f"async Qwen infer failed: {repr(e)}")
            self.publish_stop()
            self.next_query_time = time.time() + self.timeout_backoff_sec
            self.qwen_future = None
            self.qwen_future_frame = None
            self.query_busy = False
            self._maybe_recover_ollama()
            return True

        self.qwen_future = None
        self.qwen_future_frame = None
        self.query_busy = False
        self._handle_qwen_result(result, frame)
        return True

    def _handle_qwen_result(self, result: Dict[str, Any], frame) -> None:
        target = self._parse_qwen_point(result)
        depth_state = self.lidar.estimate_for_point(target.get("u"), self.image_width)

        if self.require_lidar and depth_state.front_distance is None:
            self.publish_stop()
            action = "WAIT_LIDAR_VALID"
            servo_res = self.servo.stop_result(action, depth_state.reason, depth_state.front_distance, depth_state.target_distance)
            lidar_payload = {
                "front_distance": depth_state.front_distance,
                "target_distance": depth_state.target_distance,
                "target_angle_deg": depth_state.target_angle_deg,
                "lidar_valid": depth_state.valid,
                "lidar_reason": depth_state.reason,
            }
            self._publish_json(result, action, lidar_payload)
            self._publish_state({
                "step": self.step_count,
                "action": action,
                "success": self.success,
                "servo_state": servo_res.state,
                "cmd_vx": 0.0,
                "cmd_wz": 0.0,
                **lidar_payload,
            })
            self.next_query_time = time.time() + self.lidar_wait_backoff_sec
            return

        action = "STOP_OBSERVE"
        servo_res = self.servo.compute_cmd(target, depth_state.front_distance, depth_state.target_distance)

        if self._check_arrive(target, depth_state.front_distance, depth_state.target_distance):
            self.publish_stop()
            self.success = True
            action = "ARRIVE_SUCCESS"
        elif target.get("visible", False):
            self.publish_cmd_burst(servo_res.cmd, self.servo_burst_sec, self.observe_stop_sec)
            action = servo_res.state
            if servo_res.state in ("FORWARD", "FORWARD_STEER") and servo_res.cmd.linear.x > 0.0:
                self.forward_burst_count += 1
            self.lost_scan_count = 0
        else:
            self.arrive_count = 0
            self.lost_scan_count += 1
            if self.lost_scan_count > int(self.cfg.get("lost_scan_max", 8)):
                self.publish_stop()
                action = "LOST_STOP"
                servo_res = self.servo.stop_result(action, "lost_scan_max_reached", depth_state.front_distance, depth_state.target_distance)
            else:
                scan_cmd = self._scan_cmd()
                self.publish_cmd_burst(scan_cmd, self.scan_burst_sec, self.scan_observe_sec)
                action = "SEARCH_SCAN"
                servo_res = self.servo.scan_result(scan_cmd, depth_state.front_distance, depth_state.target_distance, "qwen_no_valid_point")

        lidar_payload = {
            "front_distance": depth_state.front_distance,
            "target_distance": depth_state.target_distance,
            "target_angle_deg": depth_state.target_angle_deg,
            "lidar_valid": depth_state.valid,
            "lidar_reason": depth_state.reason,
        }
        self._publish_json(result, action, lidar_payload)
        self._publish_state({
            "step": self.step_count,
            "action": action,
            "success": self.success,
            "forward_burst_count": self.forward_burst_count,
            "arrive_count": self.arrive_count,
            "lost_scan_count": self.lost_scan_count,
            "servo_state": servo_res.state,
            "cmd_vx": servo_res.cmd.linear.x,
            "cmd_wz": servo_res.cmd.angular.z,
            **lidar_payload,
        })
        self.get_logger().info(
            f"step={self.step_count} action={action} u={result.get('u')} v={result.get('v')} "
            f"front={depth_state.front_distance} target_d={depth_state.target_distance} "
            f"servo={servo_res.state} vx={servo_res.cmd.linear.x:.3f} wz={servo_res.cmd.angular.z:+.3f} "
            f"latency={result.get('_latency_sec')}"
        )
        if frame is not None:
            self._draw_debug(frame, target, servo_res, action)
        self.next_query_time = time.time() + self.qwen_interval_sec

    def decision_timer_cb(self):
        if self.success:
            self.publish_stop()
            return

        now = time.time()
        if self.latest_frame is None:
            return

        if self._poll_qwen_future():
            return

        if now < self.next_allowed_action_time or now < self.next_query_time:
            return

        if self.step_count >= self.max_steps:
            self.get_logger().warn("max_steps reached, stop")
            self.publish_stop()
            self.success = True
            return

        if self.require_lidar and not self._scan_is_fresh():
            self.publish_stop()
            self.get_logger().warn("waiting for fresh /scan, motion disabled")
            self._publish_state({
                "step": self.step_count,
                "action": "WAIT_LIDAR",
                "success": self.success,
                "lidar_valid": False,
                "lidar_reason": "no_fresh_scan",
            })
            self.next_query_time = time.time() + self.lidar_wait_backoff_sec
            return

        frame = self.latest_frame.copy()
        self._sync_image_geometry(frame)
        self._submit_qwen_infer(frame)

    def destroy_node(self):
        self.publish_stop()
        try:
            self.qwen_executor.shutdown(wait=False, cancel_futures=True)
        except TypeError:
            self.qwen_executor.shutdown(wait=False)
        time.sleep(0.2)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--instruction", default="find the bottle")
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    rclpy.init()
    node = RunQwenLidarNav(args.instruction, cfg)
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
