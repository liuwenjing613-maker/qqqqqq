#!/usr/bin/env python3
import argparse
import json
import os
import sys
import time
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
sys.path.append(PROJECT_ROOT)

from src.vlm.qwen_ollama_client import QwenOllamaClient
from src.perception.lidar_depth import LidarDepthEstimator
from src.control.qwen_lidar_point_servo import QwenLidarPointServo


DEFAULT_CONFIG = os.path.join(PROJECT_ROOT, "configs/qwen_lidar_nav.yaml")


def load_yaml(path: str) -> Dict[str, Any]:
    with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class RunQwenLidarNav(Node):
    def __init__(self, instruction: str, cfg: Dict[str, Any], *, skip_warmup: bool = False):
        super().__init__("run_qwen_lidar_nav")

        self.instruction = instruction
        self.cfg = cfg
        self.skip_warmup = skip_warmup
        self.qwen_warmup_pending = (
            not skip_warmup and bool(cfg.get("qwen_warmup_on_first_frame", True))
        )

        self.image_topic = cfg["image_topic"]
        self.scan_topic = cfg["scan_topic"]
        self.cmd_topic = cfg["cmd_topic"]
        self.json_topic = cfg["qwen_json_topic"]
        self.state_topic = cfg["state_topic"]

        self.image_width = int(cfg["image_width"])
        self.image_height = int(cfg["image_height"])

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_scan = None

        self.query_busy = False
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

        self.debug_dir = os.path.join(PROJECT_ROOT, cfg.get("debug_dir", "data/images/qwen_lidar_debug"))
        self.save_debug = bool(cfg.get("save_debug", True))
        if self.save_debug:
            os.makedirs(self.debug_dir, exist_ok=True)

        self.qwen = QwenOllamaClient(
            model=cfg["model"],
            timeout=cfg["qwen_timeout_sec"],
            resize_width=cfg["qwen_resize_width"],
            jpeg_quality=cfg.get("qwen_jpeg_quality", 60),
            num_predict=cfg.get("qwen_num_predict", 64),
            num_ctx=cfg.get("qwen_num_ctx", 768),
            keep_alive=cfg.get("qwen_keep_alive", "1h"),
            coord_mode=cfg.get("qwen_coord_mode", "norm1000"),
            debug_dir=self.debug_dir,
            save_debug=self.save_debug,
        )

        self.lidar = LidarDepthEstimator(
            min_range=cfg["lidar_min_range"],
            max_range=cfg["lidar_max_range"],
            front_deg=cfg["lidar_front_deg"],
            target_window_deg=cfg["lidar_target_window_deg"],
            camera_hfov_deg=cfg["camera_hfov_deg"],
            camera_lidar_yaw_offset_deg=cfg["camera_lidar_yaw_offset_deg"],
        )

        self.servo = QwenLidarPointServo(
            image_width=self.image_width,
            kp_turn=cfg["kp_turn"],
            max_wz=cfg["max_wz"],
            wz_deadzone=cfg["wz_deadzone"],
            cmd_wz_deadzone=cfg["cmd_wz_deadzone"],
            turn_threshold=cfg["turn_threshold"],
            forward_turn_scale=cfg["forward_turn_scale"],
            max_vx=cfg["max_vx"],
            mid_vx=cfg["mid_vx"],
            slow_vx=cfg["slow_vx"],
            min_vx=cfg["min_vx"],
            emergency_stop_distance=cfg["emergency_stop_distance"],
            hard_stop_distance=cfg["hard_stop_distance"],
            slow_distance=cfg["slow_distance"],
            normal_distance=cfg["normal_distance"],
        )

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.json_pub = self.create_publisher(String, self.json_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.create_timer(0.05, self.control_timer_cb)
        self.create_timer(0.20, self.decision_timer_cb)

        self.get_logger().info("===== QWEN + LIDAR NAV START =====")
        self.get_logger().info(f"instruction={instruction}")
        self.get_logger().info(f"image_topic={self.image_topic} scan_topic={self.scan_topic}")
        if self.qwen_warmup_pending:
            self.get_logger().info(
                "waiting for first camera frame, then aligned Qwen warmup (camera-on mode)..."
            )
        elif self.skip_warmup:
            self.get_logger().warn(
                "warmup skipped (--no-warmup); first infer may be very slow on 7GB boards"
            )

    def _sync_image_geometry(self, frame) -> None:
        frame_h, frame_w = frame.shape[:2]
        if frame_w != self.image_width or frame_h != self.image_height:
            self.get_logger().warn(
                f"frame size mismatch config={self.image_width}x{self.image_height}, "
                f"actual={frame_w}x{frame_h}; using actual"
            )
            self.image_width = frame_w
            self.image_height = frame_h
            self.servo.update_image_width(frame_w)

    def _run_camera_aligned_warmup(self, frame) -> bool:
        self.get_logger().info(
            f"camera frame ready ({frame.shape[1]}x{frame.shape[0]}), "
            "running aligned Qwen warmup with camera already on..."
        )
        try:
            dt = self.qwen.warmup_on_camera_frame(frame, timeout=self.cfg["qwen_timeout_sec"])
            self.get_logger().info(f"aligned Qwen warmup done in {dt:.1f}s")
            return True
        except Exception as e:
            self.get_logger().error(f"aligned Qwen warmup failed: {repr(e)}")
            return False

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
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
            "reason": result.get("_coord_reason", "no_valid_uv"),
        }

    def _scan_cmd(self) -> Twist:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.scan_direction * abs(self.scan_wz)
        self.scan_direction *= -1.0
        return msg

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
            "state": state,
            "front_distance": lidar_state.get("front_distance"),
            "target_distance": lidar_state.get("target_distance"),
            "target_angle_deg": lidar_state.get("target_angle_deg"),
            "latency_sec": result.get("_latency_sec"),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.json_pub.publish(msg)

    def _publish_state(self, payload: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)

    def _draw_debug(self, frame, target, servo_res, action: str):
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

        path = os.path.join(self.debug_dir, f"qwen_lidar_{self.step_count:04d}_{action}.jpg")
        cv2.imwrite(path, vis)

    def decision_timer_cb(self):
        if self.success:
            self.publish_stop()
            return

        now = time.time()
        if self.query_busy or self.latest_frame is None:
            return

        if self.qwen_warmup_pending:
            self.query_busy = True
            frame = self.latest_frame.copy()
            self._sync_image_geometry(frame)
            ok = self._run_camera_aligned_warmup(frame)
            self.qwen_warmup_pending = False
            self.query_busy = False
            if not ok:
                self.next_query_time = time.time() + 10.0
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
        self._sync_image_geometry(frame)

        try:
            result = self.qwen.infer_navigation(frame, self.instruction)
        except Exception as e:
            self.get_logger().error(f"Qwen infer failed: {repr(e)}")
            self.publish_stop()
            self.next_query_time = time.time() + 10.0
            self.query_busy = False
            return

        target = self._parse_qwen_point(result)
        depth_state = self.lidar.estimate_for_point(target.get("u"), self.image_width)

        action = "STOP_OBSERVE"
        servo_res = self.servo.compute_cmd(
            target,
            front_distance=depth_state.front_distance,
            target_distance=depth_state.target_distance,
        )

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
            # Qwen 没给点：小角度扫描
            self.arrive_count = 0
            self.lost_scan_count += 1
            scan_cmd = self._scan_cmd()
            self.publish_cmd_burst(scan_cmd, self.scan_burst_sec, self.scan_observe_sec)
            action = "SEARCH_SCAN"

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
            "servo_state": servo_res.state,
            "cmd_vx": servo_res.cmd.linear.x,
            "cmd_wz": servo_res.cmd.angular.z,
            **lidar_payload,
        })

        self.get_logger().info(
            f"step={self.step_count} action={action} "
            f"u={result.get('u')} v={result.get('v')} "
            f"front={depth_state.front_distance} target_d={depth_state.target_distance} "
            f"servo={servo_res.state} vx={servo_res.cmd.linear.x:.3f} wz={servo_res.cmd.angular.z:+.3f}"
        )

        self._draw_debug(frame, target, servo_res, action)

        self.next_query_time = time.time() + self.qwen_interval_sec
        self.query_busy = False

    def destroy_node(self):
        self.publish_stop()
        time.sleep(0.2)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--instruction", default="find the bottle")
    parser.add_argument("--no-warmup", action="store_true")
    args = parser.parse_args()

    cfg = load_yaml(args.config)

    rclpy.init()
    node = RunQwenLidarNav(args.instruction, cfg, skip_warmup=args.no_warmup)

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