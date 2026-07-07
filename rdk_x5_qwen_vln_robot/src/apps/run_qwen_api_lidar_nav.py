#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Qwen cloud API + LiDAR point-servo navigation node (Qwen-only project).

/image_raw + /scan -> Qwen u/v -> QwenLidarPointServo -> cmd_topic.
Default cmd_topic is /cmd_vel_test for safe first tests.
"""

import argparse
import json
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, Optional, Tuple

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_qwen_vln_robot")
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_dashscope_client import QwenDashScopeClient
from src.perception.lidar_depth import LidarDepthEstimator
from src.control.qwen_lidar_point_servo import QwenLidarPointServo, clamp, turn_dir_from_ex

DEFAULT_CONFIG = os.path.join(PROJECT_ROOT, "configs/qwen_api_lidar_nav.yaml")

_ZERO_HOLD_REASONS = frozenset({
    "stop",
    "EMERGENCY_STOP",
    "OBSTACLE_STOP",
    "DEPTH_STOP",
    "LOST_STOP",
    "LOST_HOLD",
    "WAIT_SCAN_STALE",
    "ARRIVED",
})

_TRACK_HOLD_REASONS = frozenset({
    "FORWARD",
    "FORWARD_STEER",
    "TURN_ONLY",
    "SEARCH_SCAN",
})


def load_yaml(path: str) -> Dict[str, Any]:
    with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise RuntimeError(f"Config did not parse into a dict: {path}")
    return cfg


def _nested_get(cfg: Dict[str, Any], block: str, key: str, default: Any) -> Any:
    section = cfg.get(block)
    if isinstance(section, dict) and key in section:
        return section[key]
    if key in cfg:
        return cfg[key]
    return default


class RunQwenApiLidarNav(Node):
    def __init__(self, instruction: str, cfg: Dict[str, Any]):
        super().__init__("run_qwen_api_lidar_nav")
        self.instruction = instruction
        self.cfg = cfg

        self.image_topic = str(cfg.get("image_topic", "/image_raw"))
        self.scan_topic = str(cfg.get("scan_topic", "/scan"))
        self.cmd_topic = str(cfg.get("cmd_topic", "/cmd_vel_test"))
        self.json_topic = str(cfg.get("qwen_json_topic", "/qwen_api_json"))
        self.state_topic = str(cfg.get("state_topic", "/qwen_api_state"))
        self.require_lidar = bool(cfg.get("require_lidar", True))

        self.image_width = int(cfg.get("image_width", 640))
        self.image_height = int(cfg.get("image_height", 480))
        self.max_scan_age_sec = float(cfg.get("max_scan_age_sec", 1.5))
        rates = cfg.get("rates") or {}
        self.control_hz = float(rates.get("control_hz", 20.0))
        self.decision_hz = float(rates.get("decision_hz", 5.0))
        self.qwen_interval_sec = float(_nested_get(cfg, "qwen", "qwen_interval_sec", 1.0))
        self.timeout_backoff_sec = float(_nested_get(cfg, "qwen", "timeout_backoff_sec", 8.0))
        self.lidar_wait_backoff_sec = float(cfg.get("lidar_wait_backoff_sec", 1.0))
        self.scan_wz = float(_nested_get(cfg, "search", "scan_wz", cfg.get("scan_wz", 0.05)))
        self.lost_stop_sec = float(_nested_get(cfg, "search", "lost_stop_sec", 1.2))
        self.scan_flip_interval_sec = float(_nested_get(cfg, "search", "scan_flip_interval_sec", 5.0))
        self.search_resume_sec = float(_nested_get(cfg, "search", "search_resume_sec", 3.0))
        self.max_steps = int(cfg.get("max_steps", 80))
        self.lost_scan_max = int(cfg.get("lost_scan_max", 8))
        self.save_debug = bool(cfg.get("save_debug", True))

        self.emergency_stop_distance = float(cfg.get("emergency_stop_distance", 0.28))
        self.hard_stop_distance = float(cfg.get("hard_stop_distance", 0.42))
        self.arrive_distance = float(
            _nested_get(cfg, "success", "lidar_target_arrive_distance", cfg.get("lidar_target_arrive_distance", 0.6))
        )
        self.arrive_frames_required = int(_nested_get(cfg, "success", "arrive_frames", cfg.get("arrive_frames", 2)))

        self.target_filter_enabled = bool(_nested_get(cfg, "target_filter", "enabled", False))
        self.target_smooth_alpha = float(_nested_get(cfg, "target_filter", "smooth_alpha", 0.25))
        self.max_u_jump_px = float(_nested_get(cfg, "target_filter", "max_u_jump_px", 280.0))
        self.hold_last_target_sec = float(_nested_get(cfg, "target_filter", "hold_last_target_sec", 1.2))

        self.active_hold_sec = float(_nested_get(cfg, "cmd_hold", "active_hold_sec", 0.35))
        self.stale_slow_sec = float(_nested_get(cfg, "cmd_hold", "stale_slow_sec", 0.9))
        self.stale_vx_scale = float(_nested_get(cfg, "cmd_hold", "stale_vx_scale", 0.55))
        self.stale_wz_scale = float(_nested_get(cfg, "cmd_hold", "stale_wz_scale", 0.25))
        self.stale_max_wz = float(_nested_get(cfg, "cmd_hold", "stale_max_wz", 0.010))
        self.stop_after_sec = float(_nested_get(cfg, "cmd_hold", "stop_after_sec", 0.9))

        debug_dir_cfg = cfg.get("debug_dir", "data/images/qwen_api_lidar_debug")
        self.debug_dir = debug_dir_cfg if os.path.isabs(str(debug_dir_cfg)) else os.path.join(PROJECT_ROOT, str(debug_dir_cfg))
        if self.save_debug:
            os.makedirs(self.debug_dir, exist_ok=True)
            self.debug_latest_path = os.path.join(self.debug_dir, "latest.jpg")
            self.debug_prev_path = os.path.join(self.debug_dir, "prev.jpg")

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_scan = None
        self.last_scan_time = None
        self.query_executor = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.future_frame = None
        self.step_count = 0
        self.lost_scan_count = 0
        self.scan_direction = 1.0
        self.last_scan_flip_time: Optional[float] = None
        self.lost_since_time: Optional[float] = None
        self.lost_stop_since: Optional[float] = None
        self.next_query_time = 0.0
        self.desired_cmd = Twist()
        self.last_desired_cmd = Twist()
        self.desired_reason = "boot_stop"
        self.last_fresh_cmd_time = 0.0
        self.success = False
        self.arrive_frame_count = 0

        self.filtered_u: Optional[float] = None
        self.last_raw_u: Optional[float] = None
        self.last_target_ex: Optional[float] = None

        self.qwen = QwenDashScopeClient(
            timeout=float(_nested_get(cfg, "qwen", "qwen_timeout_sec", 15.0)),
            resize_width=int(_nested_get(cfg, "qwen", "qwen_resize_width", 640)),
            jpeg_quality=int(_nested_get(cfg, "qwen", "qwen_jpeg_quality", 80)),
            min_confidence=float(_nested_get(cfg, "qwen", "min_confidence", 0.60)),
            max_tokens=int(_nested_get(cfg, "qwen", "qwen_max_tokens", 80)),
        )
        self.lidar = LidarDepthEstimator(
            min_range=float(cfg.get("lidar_min_range", 0.08)),
            max_range=float(cfg.get("lidar_max_range", 12.0)),
            front_deg=float(cfg.get("lidar_front_deg", 18.0)),
            target_window_deg=float(cfg.get("lidar_target_window_deg", 6.0)),
            camera_hfov_deg=float(cfg.get("camera_hfov_deg", 70.0)),
            camera_lidar_yaw_offset_deg=float(cfg.get("camera_lidar_yaw_offset_deg", 0.0)),
        )
        self.servo = QwenLidarPointServo(
            image_width=self.image_width,
            require_lidar=self.require_lidar,
            kp_turn=float(_nested_get(cfg, "servo", "kp_turn", 0.1)),
            max_wz=float(_nested_get(cfg, "servo", "max_wz", 0.05)),
            center_deadband=float(_nested_get(cfg, "servo", "center_deadband", 0.06)),
            straight_hysteresis=float(_nested_get(cfg, "servo", "straight_hysteresis", 0.02)),
            cmd_wz_deadband=float(
                _nested_get(cfg, "servo", "cmd_wz_deadband", _nested_get(cfg, "servo", "cmd_wz_deadzone", 0.006))
            ),
            turn_only_threshold=float(_nested_get(cfg, "servo", "turn_only_threshold", 0.4)),
            max_vx=float(_nested_get(cfg, "servo", "max_vx", 0.06)),
            steer_vx=float(_nested_get(cfg, "servo", "steer_vx", 0.05)),
            turn_only_vx=float(_nested_get(cfg, "servo", "turn_only_vx", 0.0)),
            creep_mode=bool(_nested_get(cfg, "servo", "creep_mode", False)),
            creep_vx=float(_nested_get(cfg, "servo", "creep_vx", 0.012)),
            creep_wz=float(_nested_get(cfg, "servo", "creep_wz", 0.012)),
            creep_turn_vx=float(_nested_get(cfg, "servo", "creep_turn_vx", 0.0)),
            allow_track_without_depth=bool(_nested_get(cfg, "servo", "allow_track_without_depth", True)),
            forward_vx_no_depth=float(_nested_get(cfg, "servo", "forward_vx_no_depth", 0.04)),
            emergency_stop_distance=self.emergency_stop_distance,
            hard_stop_distance=self.hard_stop_distance,
            slow_distance=float(cfg.get("slow_distance", 0.65)),
            arrive_distance=self.arrive_distance,
        )

        self.create_subscription(Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.json_pub = self.create_publisher(String, self.json_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.create_timer(1.0 / max(self.control_hz, 1e-3), self.control_timer_cb)
        self.create_timer(1.0 / max(self.decision_hz, 1e-3), self.decision_timer_cb)

        self.get_logger().info("===== QWEN CLOUD API + LIDAR NAV (qwen_vln_robot) =====")
        self.get_logger().info(
            f"instruction={instruction} cmd_topic={self.cmd_topic} "
            f"control_hz={self.control_hz} decision_hz={self.decision_hz} "
            f"qwen_interval_sec={self.qwen_interval_sec} scan_wz={self.scan_wz} "
            f"straight_band={self.servo.center_deadband}±{self.servo.straight_hysteresis} "
            f"arrive_dist={self.arrive_distance}m creep={self.servo.creep_mode}"
        )

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.last_scan_time = time.time()
        self.lidar.update_scan(msg)

    def _set_desired(self, cmd: Twist, reason: str, *, fresh: bool = True) -> None:
        self.desired_cmd = cmd
        self.desired_reason = reason
        if fresh:
            self.last_desired_cmd = Twist()
            self.last_desired_cmd.linear.x = float(cmd.linear.x)
            self.last_desired_cmd.angular.z = float(cmd.angular.z)
            self.last_fresh_cmd_time = time.time()

    def publish_stop(self) -> None:
        self._set_desired(Twist(), "stop", fresh=True)

    def _cmd_age(self) -> float:
        if self.last_fresh_cmd_time <= 0.0:
            return float("inf")
        return time.time() - self.last_fresh_cmd_time

    def _lidar_requires_stop(self) -> bool:
        front = self.lidar.front_distance()
        if front is not None and float(front) <= self.emergency_stop_distance:
            return True
        return False

    def _cmd_with_hold(self) -> Tuple[Twist, str]:
        if self.desired_reason in _ZERO_HOLD_REASONS:
            return Twist(), "zero_cmd"

        age = self._cmd_age()
        base = self.last_desired_cmd

        if age <= self.active_hold_sec:
            out = Twist()
            out.linear.x = float(base.linear.x)
            out.angular.z = float(base.angular.z)
            return out, "fresh_cmd"

        if age > self.stop_after_sec:
            return Twist(), "zero_cmd"

        if age <= self.stale_slow_sec:
            out = Twist()
            if self.desired_reason in _TRACK_HOLD_REASONS:
                out.linear.x = float(base.linear.x) * self.stale_vx_scale
                wz = clamp(
                    float(base.angular.z) * self.stale_wz_scale,
                    -self.stale_max_wz,
                    self.stale_max_wz,
                )
                out.angular.z = wz
                return out, "stale_cmd"
            out.linear.x = 0.0
            wz = clamp(float(base.angular.z) * self.stale_wz_scale, -self.stale_max_wz, self.stale_max_wz)
            out.angular.z = wz
            return out, "stale_cmd"

        return Twist(), "zero_cmd"

    def control_timer_cb(self) -> None:
        if self._lidar_requires_stop():
            self.cmd_pub.publish(Twist())
            return
        cmd, _mode = self._cmd_with_hold()
        self.cmd_pub.publish(cmd)

    def _scan_is_fresh(self) -> bool:
        if self.latest_scan is None or self.last_scan_time is None:
            return False
        return (time.time() - self.last_scan_time) <= self.max_scan_age_sec

    def _sync_image_geometry(self, frame):
        h, w = frame.shape[:2]
        if w != self.image_width or h != self.image_height:
            self.image_width = int(w)
            self.image_height = int(h)
            self.servo.update_image_width(w)

    def _save_latest_debug_frame(self, frame, target: Dict[str, Any]) -> None:
        """After Qwen result: rotate latest->prev, save new latest; drop older files."""
        vis = frame.copy()
        u, v = target.get("u"), target.get("v")
        if u is not None and v is not None:
            cv2.drawMarker(vis, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        raw_u = target.get("raw_u")
        if raw_u is not None and u is not None and abs(float(raw_u) - float(u)) > 1.0:
            cv2.drawMarker(vis, (int(raw_u), int(v)), (255, 0, 0), cv2.MARKER_CROSS, 16, 2)
        if os.path.isfile(self.debug_latest_path):
            try:
                os.replace(self.debug_latest_path, self.debug_prev_path)
            except OSError:
                pass
        for name in os.listdir(self.debug_dir):
            if name in ("latest.jpg", "prev.jpg"):
                continue
            if name.endswith((".jpg", ".jpeg", ".png")):
                try:
                    os.remove(os.path.join(self.debug_dir, name))
                except OSError:
                    pass
        cv2.imwrite(self.debug_latest_path, vis)

    def _parse_qwen_point(self, result: Dict[str, Any]) -> Dict[str, Any]:
        usable = bool(result.get("usable", result.get("_point_valid", False)))
        u, v = result.get("u"), result.get("v")
        if usable and u is not None and v is not None:
            return {"visible": True, "u": float(u), "v": float(v), "cx": float(u), "raw_u": float(u)}
        return {"visible": False, "u": u, "v": v, "reason": result.get("_coord_reason", "no_valid_uv")}

    def _filter_target_u(self, raw_u: float) -> Tuple[float, float]:
        if not self.target_filter_enabled:
            self.filtered_u = raw_u
            self.last_raw_u = raw_u
            return raw_u, raw_u

        if self.filtered_u is None:
            self.filtered_u = raw_u
            self.last_raw_u = raw_u
            return raw_u, raw_u

        delta = raw_u - self.filtered_u
        if abs(delta) > self.max_u_jump_px:
            sign = 1.0 if delta > 0.0 else -1.0
            raw_u_limited = self.filtered_u + sign * self.max_u_jump_px
        else:
            raw_u_limited = raw_u

        self.filtered_u = (
            self.target_smooth_alpha * raw_u_limited
            + (1.0 - self.target_smooth_alpha) * self.filtered_u
        )
        self.last_raw_u = raw_u
        return raw_u, self.filtered_u

    def _apply_target_filter(self, target: Dict[str, Any]) -> Dict[str, Any]:
        if not target.get("visible", False):
            return target
        raw_u = target.get("u")
        if raw_u is None:
            return target
        raw_u_val, filtered_u_val = self._filter_target_u(float(raw_u))
        out = dict(target)
        out["raw_u"] = raw_u_val
        out["u"] = filtered_u_val
        out["cx"] = filtered_u_val
        return out

    def _lost_duration(self) -> float:
        if self.lost_since_time is None:
            return 0.0
        return time.time() - self.lost_since_time

    def _set_scan_direction_from_last_target(self) -> None:
        if self.last_target_ex is not None:
            self.scan_direction = turn_dir_from_ex(self.last_target_ex)
            return
        if self.filtered_u is not None:
            ex = (float(self.filtered_u) - self.image_width / 2.0) / max(1.0, float(self.image_width))
            self.scan_direction = turn_dir_from_ex(ex)
            return
        self.scan_direction = 1.0

    def _maybe_flip_scan_direction(self, now: float) -> None:
        if self.last_scan_flip_time is None:
            self.last_scan_flip_time = now
            return
        if (now - self.last_scan_flip_time) >= self.scan_flip_interval_sec:
            self.scan_direction *= -1.0
            self.last_scan_flip_time = now

    def _scan_cmd(self) -> Twist:
        now = time.time()
        self._set_scan_direction_from_last_target()
        self._maybe_flip_scan_direction(now)
        msg = Twist()
        msg.angular.z = self.scan_direction * abs(self.scan_wz)
        return msg

    def _submit_infer(self, frame):
        if self.future is not None:
            return
        self.step_count += 1
        self.future_frame = frame
        self.future = self.query_executor.submit(self.qwen.infer_navigation, frame, self.instruction)
        # Interval is from submit time so a 1s setting ≈ 1Hz when API latency < 1s.
        self.next_query_time = time.time() + self.qwen_interval_sec
        self.get_logger().info(f"submitted cloud Qwen infer step={self.step_count}")

    def _poll_future(self) -> bool:
        if self.future is None:
            return False
        if not self.future.done():
            return True
        frame = self.future_frame
        try:
            result = self.future.result()
        except Exception as e:
            self.get_logger().error(f"cloud Qwen infer failed: {repr(e)}")
            self.publish_stop()
            self.future = None
            self.future_frame = None
            self.next_query_time = time.time() + self.timeout_backoff_sec
            return True
        self.future = None
        self.future_frame = None
        self._handle_result(result, frame)
        return True

    def _handle_result(self, result, frame):
        target = self._parse_qwen_point(result)
        raw_u_log = result.get("u")
        filtered_u_log: Optional[float] = None
        center_error: Optional[float] = None
        wz_out = 0.0
        search_direction = self.scan_direction
        lost_duration = self._lost_duration()

        depth_state = self.lidar.estimate_for_point(target.get("u"), self.image_width)
        action, cmd, servo_state, reason = "STOP_OBSERVE", Twist(), "STOP", ""

        if target.get("visible", False):
            target = self._apply_target_filter(target)
            raw_u_log = target.get("raw_u", raw_u_log)
            filtered_u_log = target.get("u")
            if filtered_u_log is not None:
                center_error = (float(filtered_u_log) - self.image_width / 2.0) / max(1.0, float(self.image_width))
                self.last_target_ex = float(center_error)

            self.lost_since_time = None
            self.lost_stop_since = None
            self.last_scan_flip_time = None
            self.last_target_seen_time = time.time()

            depth_state = self.lidar.estimate_for_point(target.get("u"), self.image_width)
            servo_res = self.servo.compute_cmd(target, depth_state.front_distance, depth_state.target_distance)
            cmd, servo_state, reason = servo_res.cmd, servo_res.state, servo_res.reason
            wz_out = servo_res.wz
            if servo_state == "ARRIVED":
                self.arrive_frame_count += 1
                self.publish_stop()
                self._set_desired(cmd, "ARRIVED", fresh=True)
                action = "ARRIVED"
                if self.arrive_frame_count >= self.arrive_frames_required:
                    self.success = True
                    self.get_logger().info(
                        f"ARRIVED: target_distance={depth_state.target_distance:.3f}m "
                        f"<={self.arrive_distance:.3f}m (frames={self.arrive_frame_count})"
                    )
            else:
                self.arrive_frame_count = 0
                self._set_desired(cmd, servo_state, fresh=True)
                action = servo_res.state
            self.lost_scan_count = 0
        elif self.require_lidar and not self._scan_is_fresh():
            self.publish_stop()
            action, reason = "WAIT_SCAN_STALE", "scan_stale_no_target"
            self.lost_scan_count += 1
            if self.lost_since_time is None:
                self.lost_since_time = time.time()
        else:
            if self.lost_since_time is None:
                self.lost_since_time = time.time()
            lost_duration = self._lost_duration()
            self.lost_scan_count += 1

            if lost_duration < self.lost_stop_sec:
                cmd = Twist()
                self._set_desired(cmd, "LOST_HOLD", fresh=True)
                action, servo_state, reason = "LOST_HOLD", "LOST_HOLD", "lost_wait_reacquire"
            elif self.lost_scan_count > self.lost_scan_max:
                now = time.time()
                if self.lost_stop_since is None:
                    self.lost_stop_since = now
                if (now - self.lost_stop_since) >= self.search_resume_sec:
                    self.lost_scan_count = 0
                    self.lost_stop_since = None
                    self.lost_since_time = now
                    lost_duration = 0.0
                    cmd = self._scan_cmd()
                    search_direction = self.scan_direction
                    wz_out = float(cmd.angular.z)
                    self._set_desired(cmd, "SEARCH_SCAN", fresh=True)
                    action, servo_state, reason = (
                        "SEARCH_SCAN",
                        "SEARCH_SCAN",
                        "lost_resume_scan",
                    )
                else:
                    self.publish_stop()
                    action, reason = "LOST_STOP", "lost_scan_max_reached"
            else:
                self.lost_stop_since = None
                cmd = self._scan_cmd()
                search_direction = self.scan_direction
                wz_out = float(cmd.angular.z)
                self._set_desired(cmd, "SEARCH_SCAN", fresh=True)
                action, servo_state, reason = "SEARCH_SCAN", "SEARCH_SCAN", target.get("reason", "qwen_no_valid_point")

        cmd_age = self._cmd_age()
        _, publish_mode = self._cmd_with_hold()

        lidar_payload = {
            "front_distance": depth_state.front_distance,
            "target_distance": depth_state.target_distance,
            "target_angle_deg": depth_state.target_angle_deg,
            "lidar_valid": depth_state.valid,
            "lidar_reason": depth_state.reason,
        }
        self.json_pub.publish(String(data=json.dumps({
            "u": filtered_u_log if filtered_u_log is not None else result.get("u"),
            "v": result.get("v"),
            "raw_u": raw_u_log,
            "filtered_u": filtered_u_log,
            "raw_v": result.get("_raw_v"),
            "usable": bool(result.get("usable", False)), "status": result.get("status"),
            "confidence": result.get("confidence"), "reason": result.get("reason"),
            "coord_reason": result.get("_coord_reason"), "state": action,
            "latency_sec": result.get("_latency_sec"), **lidar_payload,
        }, ensure_ascii=False)))
        self.state_pub.publish(String(data=json.dumps({
            "step": self.step_count, "action": action, "servo_state": servo_state,
            "reason": reason, "cmd_vx": float(cmd.linear.x), "cmd_wz": float(cmd.angular.z),
            "raw_u": raw_u_log, "filtered_u": filtered_u_log,
            "center_error": center_error, "cmd_age": cmd_age, "cmd_mode": publish_mode,
            "cmd_wz": wz_out, "last_target_ex": self.last_target_ex,
            "search_direction": search_direction, "lost_duration": lost_duration,
            "lost_scan_count": self.lost_scan_count, **lidar_payload,
        }, ensure_ascii=False)))
        self.get_logger().info(
            f"step={self.step_count} state={action} raw_u={raw_u_log} filtered_u={filtered_u_log} "
            f"ex={center_error if center_error is not None else 'n/a'} "
            f"vx={cmd.linear.x:.3f} cmd_wz={cmd.angular.z:+.3f} servo_wz={wz_out:+.3f} "
            f"cmd_age={cmd_age:.2f} {publish_mode} "
            f"search_dir={search_direction:+.0f} lost_dur={lost_duration:.2f} "
            f"front={depth_state.front_distance} target_dist={depth_state.target_distance} "
            f"conf={result.get('confidence')} "
            f"latency={result.get('_latency_sec')}"
        )
        if self.save_debug and frame is not None:
            self._save_latest_debug_frame(frame, target if target.get("visible") else {"u": None, "v": None})

    def decision_timer_cb(self):
        if self.success or self.latest_frame is None:
            return
        if self._poll_future():
            return
        now = time.time()
        if now < self.next_query_time:
            return
        if self.max_steps > 0 and self.step_count >= self.max_steps:
            self.publish_stop()
            self.success = True
            return
        if self.require_lidar and not self._scan_is_fresh():
            self.publish_stop()
            self.next_query_time = time.time() + self.lidar_wait_backoff_sec
            return
        frame = self.latest_frame.copy()
        self._sync_image_geometry(frame)
        self._submit_infer(frame)

    def destroy_node(self):
        self.publish_stop()
        try:
            self.query_executor.shutdown(wait=False, cancel_futures=True)
        except TypeError:
            self.query_executor.shutdown(wait=False)
        try:
            self.qwen.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--instruction", default="find bottle")
    args = parser.parse_args()
    cfg = load_yaml(args.config)
    rclpy.init()
    node = RunQwenApiLidarNav(args.instruction, cfg)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
