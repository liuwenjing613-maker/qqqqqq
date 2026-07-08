#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Qwen cloud API + LiDAR point-servo navigation node (Qwen-only project).

/image_raw + /scan -> Qwen u/v -> QwenLidarPointServo -> cmd_topic.
Default cmd_topic is /cmd_vel_test for safe first tests.
"""

import argparse
import json
import math
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

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_dashscope_client import QwenDashScopeClient
from src.perception.lidar_depth import LidarDepthEstimator, fuse_target_distance, resolve_arrive_distance
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
    "INFERRED_FORWARD",
    "INFERRED_FORWARD_STEER",
    "INFERRED_TURN_ONLY",
    "TARGET_SCAN_360",
    "EXPLORE_ALIGN",
    "EXPLORE_FORWARD",
})

_ANGLE_TRACK_REASONS = frozenset({
    "FORWARD_STEER",
    "TURN_ONLY",
    "INFERRED_FORWARD_STEER",
    "INFERRED_TURN_ONLY",
})

PHASE_TARGET_SCAN_360 = "TARGET_SCAN_360"
PHASE_TARGET_SERVO = "TARGET_SERVO"
PHASE_ASK_QWEN_PATH = "ASK_QWEN_PATH"
PHASE_EXPLORE_ALIGN = "EXPLORE_ALIGN"
PHASE_EXPLORE_FORWARD = "EXPLORE_FORWARD"
PHASE_SUCCESS = "SUCCESS"
PHASE_FAILED = "FAILED"

_EXPLORE_TRACK_REASONS = frozenset({
    "TARGET_SCAN_360",
    "EXPLORE_ALIGN",
    "EXPLORE_FORWARD",
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
        self.qwen_default_mode = str(_nested_get(cfg, "qwen", "default_mode", "track")).strip().lower()
        self.min_inferred_confidence = float(_nested_get(cfg, "qwen", "min_inferred_confidence", 0.20))
        self.inferred_vx_scale = float(_nested_get(cfg, "qwen", "inferred_vx_scale", 0.85))
        self.qwen_first_request_sent = False
        self.lidar_wait_backoff_sec = float(cfg.get("lidar_wait_backoff_sec", 1.0))
        self.scan_wz = float(_nested_get(cfg, "search", "scan_wz", cfg.get("scan_wz", 0.05)))
        self.lost_stop_sec = float(_nested_get(cfg, "search", "lost_stop_sec", 1.2))
        self.scan_flip_interval_sec = float(_nested_get(cfg, "search", "scan_flip_interval_sec", 5.0))
        self.search_resume_sec = float(_nested_get(cfg, "search", "search_resume_sec", 3.0))
        self.max_steps = int(cfg.get("max_steps", 80))
        self.lost_scan_max = int(cfg.get("lost_scan_max", 8))
        self.save_debug = bool(cfg.get("save_debug", True))

        explore_cfg = cfg.get("explore") or {}
        self.explore_enable = bool(_nested_get(cfg, "explore", "enable", False))
        self.full_scan_wz = float(_nested_get(cfg, "explore", "full_scan_wz", 0.22))
        self.full_scan_turns = float(_nested_get(cfg, "explore", "full_scan_turns", 1.0))
        self.full_scan_margin = float(_nested_get(cfg, "explore", "full_scan_margin", 1.08))
        _scan_query_iv = float(_nested_get(cfg, "explore", "full_scan_query_interval_sec", 0.0))
        self.full_scan_query_interval_sec = (
            _scan_query_iv if _scan_query_iv > 0.0 else self.qwen_interval_sec
        )
        self.path_confidence_threshold = float(
            _nested_get(cfg, "explore", "path_confidence_threshold", 0.50)
        )
        self.path_turn_threshold = float(_nested_get(cfg, "explore", "path_turn_threshold", 0.16))
        self.path_kp_turn = float(_nested_get(cfg, "explore", "path_kp_turn", 0.12))
        self.explore_distance_m = float(_nested_get(cfg, "explore", "explore_distance_m", 2.5))
        self.explore_vx = float(_nested_get(cfg, "explore", "explore_vx", 0.08))
        self.explore_max_sec = float(_nested_get(cfg, "explore", "explore_max_sec", 40.0))
        self.explore_hard_stop_distance = float(
            _nested_get(cfg, "explore", "explore_hard_stop_distance", 0.45)
        )
        self.chassis_max_wz = float(_nested_get(cfg, "chassis", "max_wz", 0.02))
        self.chassis_max_vx = float(_nested_get(cfg, "chassis", "max_vx", 0.06))
        self.full_scan_yaw_target = (
            2.0 * math.pi * max(self.full_scan_turns, 0.1) * max(self.full_scan_margin, 1.0)
        )
        self.full_scan_max_sec = float(_nested_get(cfg, "explore", "full_scan_max_sec", 45.0))
        # Legacy log hint only (2π/wz); do not use for phase transition.
        self.full_scan_sec = 2.0 * math.pi / max(abs(self.full_scan_wz), 1e-3)
        self.full_scan_sec *= self.full_scan_margin

        self.phase = PHASE_TARGET_SCAN_360 if self.explore_enable else PHASE_TARGET_SERVO
        self.scan_start_time: Optional[float] = None
        self.scan_yaw_integrated = 0.0
        self._last_scan_integrate_time: Optional[float] = None
        self.scan_last_query_time = 0.0
        self.path_point: Optional[Tuple[float, float]] = None
        self.path_confidence = 0.0
        self.explore_start_time: Optional[float] = None
        self.explore_end_time: Optional[float] = None
        self.explore_phase_reason = ""

        self.emergency_stop_distance = float(cfg.get("emergency_stop_distance", 0.28))
        self.hard_stop_distance = float(cfg.get("hard_stop_distance", 0.42))
        self.arrive_distance = float(
            _nested_get(cfg, "success", "lidar_target_arrive_distance", cfg.get("lidar_target_arrive_distance", 0.6))
        )
        self.arrive_frames_required = int(_nested_get(cfg, "success", "arrive_frames", cfg.get("arrive_frames", 2)))
        # TEMP: when false, LiDAR does not trigger ARRIVED; resolve_arrive_distance still logged for viz.
        self.lidar_arrive_enable = bool(
            _nested_get(cfg, "success", "lidar_arrive_enable", cfg.get("lidar_arrive_enable", True))
        )

        self.target_filter_enabled = bool(_nested_get(cfg, "target_filter", "enabled", False))
        self.target_smooth_alpha = float(_nested_get(cfg, "target_filter", "smooth_alpha", 0.25))
        self.max_u_jump_px = float(_nested_get(cfg, "target_filter", "max_u_jump_px", 280.0))
        self.hold_last_target_sec = float(_nested_get(cfg, "target_filter", "hold_last_target_sec", 3.0))
        self.hold_max_miss_frames = int(_nested_get(cfg, "target_filter", "hold_max_miss_frames", 3))

        self.target_dist_filter_enabled = bool(
            _nested_get(cfg, "target_distance_filter", "enabled", True)
        )
        self.target_dist_smooth_alpha = float(
            _nested_get(cfg, "target_distance_filter", "smooth_alpha", 0.35)
        )
        self.max_target_dist_jump_m = float(
            _nested_get(cfg, "target_distance_filter", "max_jump_m", 1.5)
        )
        self.arrive_fuse_front_when_centered = bool(
            _nested_get(cfg, "target_distance_filter", "fuse_front_when_centered", True)
        )
        self.arrive_fuse_min_always = bool(
            _nested_get(cfg, "target_distance_filter", "fuse_min_always", True)
        )
        self.arrive_front_margin_m = float(
            _nested_get(cfg, "target_distance_filter", "arrive_front_margin_m", 0.3)
        )
        self.arrive_ray_far_invalid_m = float(
            _nested_get(cfg, "target_distance_filter", "arrive_ray_far_invalid_m", 3.0)
        )
        self.arrive_center_deadband = float(
            _nested_get(cfg, "target_distance_filter", "fuse_center_deadband", _nested_get(cfg, "servo", "center_deadband", 0.10))
        )

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
        self.arrived_locked = False

        self.filtered_u: Optional[float] = None
        self.last_raw_u: Optional[float] = None
        self.last_target_ex: Optional[float] = None
        self.last_target_seen_time: Optional[float] = None
        self.last_held_target: Optional[Dict[str, Any]] = None
        self.consecutive_miss_count = 0
        self.filtered_target_distance: Optional[float] = None

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
            target_distance_method=str(cfg.get("lidar_target_distance_method", "min")),
            front_distance_method=str(cfg.get("lidar_front_distance_method", "min")),
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
            angle_servo_enabled=bool(_nested_get(cfg, "servo", "angle_servo_enabled", _nested_get(cfg, "angle_servo", "enabled", False))),
            angle_gain=float(_nested_get(cfg, "angle_servo", "gain", 120.0)),
            angle_power=float(_nested_get(cfg, "angle_servo", "power", 0.85)),
            angle_max_turn_deg=float(_nested_get(cfg, "angle_servo", "max_turn_deg", 30.0)),
            angle_turn_wz=float(_nested_get(cfg, "angle_servo", "turn_wz", 0.04)),
            angle_complete_tol_deg=float(_nested_get(cfg, "angle_servo", "complete_tol_deg", 0.5)),
            angle_max_pending_deg=float(_nested_get(cfg, "angle_servo", "max_pending_deg", 45.0)),
            angle_skip_while_turning=bool(_nested_get(cfg, "angle_servo", "skip_while_turning", True)),
            angle_wait_turn_complete=bool(_nested_get(cfg, "angle_servo", "wait_turn_complete", True)),
            angle_absolute_cap_deg=float(_nested_get(cfg, "angle_servo", "absolute_cap_deg", 1.0)),
        )
        self._last_control_time: Optional[float] = None
        self._turn_was_busy = False

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
            f"arrive_dist={self.arrive_distance}m lidar_arrive={self.lidar_arrive_enable} creep={self.servo.creep_mode} "
            f"angle_servo={self.servo.angle_servo_enabled} "
            f"explore={self.explore_enable} phase={self.phase} "
            f"scan_yaw_target={math.degrees(self.full_scan_yaw_target):.0f}deg "
            f"scan_query_iv={self.full_scan_query_interval_sec}s"
        )
        if self.explore_enable:
            self._start_target_scan_360()

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
        self.servo.clear_remaining()
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
        if self.arrived_locked or self.success:
            self.cmd_pub.publish(Twist())
            return
        if self._lidar_requires_stop():
            self.servo.clear_remaining()
            self.cmd_pub.publish(Twist())
            return
        cmd, _mode = self._cmd_with_hold()
        turn_busy = self.servo.angle_servo_enabled and self.servo.is_turn_busy()
        if turn_busy and self.desired_reason == "TURN_ONLY":
            cmd.linear.x = 0.0
        if self.servo.angle_servo_enabled and (
            self.desired_reason in _ANGLE_TRACK_REASONS or turn_busy
        ):
            now = time.time()
            if self._last_control_time is None:
                dt = 1.0 / max(self.control_hz, 1e-3)
            else:
                dt = clamp(now - self._last_control_time, 0.001, 0.2)
            self._last_control_time = now
            ex_for_step = None if self.servo.angle_wait_turn_complete else self.last_target_ex
            cmd.angular.z = self.servo.step_remaining(dt, ex=ex_for_step)
        if self._turn_was_busy and not turn_busy:
            self.next_query_time = 0.0
        self._turn_was_busy = turn_busy
        if self.explore_enable and self.phase == PHASE_TARGET_SCAN_360:
            now = time.time()
            if self._last_scan_integrate_time is None:
                self._last_scan_integrate_time = now
            else:
                dt = clamp(now - self._last_scan_integrate_time, 0.001, 0.2)
                self._last_scan_integrate_time = now
                effective_wz = min(abs(float(cmd.angular.z)), self.chassis_max_wz)
                self.scan_yaw_integrated += effective_wz * dt
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

    def _save_latest_debug_frame(self, frame, target: Dict[str, Any], result: Optional[Dict[str, Any]] = None) -> None:
        """After Qwen result: rotate latest->prev, save new latest; drop older files."""
        vis = frame.copy()
        u, v = target.get("u"), target.get("v")
        if u is not None and v is not None:
            point_kind = target.get("point_kind")
            if point_kind is None and result is not None:
                point_kind = "locked" if result.get("usable") else (
                    "inferred" if result.get("direction_valid") and result.get("status") == "inferred" else "none"
                )
            color = (0, 165, 255) if point_kind == "inferred" else (0, 0, 255)
            cv2.drawMarker(vis, (int(u), int(v)), color, cv2.MARKER_CROSS, 24, 2)
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

    def _resolve_qwen_mode(self) -> str:
        """Escalate prompt mode when target stays lost: track -> search -> scan."""
        if self.lost_since_time is None:
            return self.qwen_default_mode if self.qwen_default_mode in ("track", "search", "scan") else "track"
        lost_dur = self._lost_duration()
        if lost_dur < self.lost_stop_sec:
            return "track"
        if self.lost_scan_count <= self.lost_scan_max:
            return "search"
        return "scan"

    def _parse_qwen_point(self, result: Dict[str, Any]) -> Dict[str, Any]:
        qwen_mode = str(result.get("mode", "")).upper()
        confidence = float(result.get("confidence", 0.0) or 0.0)

        if qwen_mode == "TARGET" and bool(result.get("usable", False)):
            u, v = result.get("u"), result.get("v")
            if u is not None and v is not None:
                return {
                    "visible": True,
                    "point_kind": "locked",
                    "u": float(u),
                    "v": float(v),
                    "cx": float(u),
                    "raw_u": float(u),
                    "confidence": confidence,
                }

        if qwen_mode == "PATH" and bool(result.get("usable", False)):
            wu, wv = result.get("waypoint_u"), result.get("waypoint_v")
            if wu is not None and wv is not None:
                return {
                    "visible": True,
                    "point_kind": "path",
                    "u": float(wu),
                    "v": float(wv),
                    "cx": float(wu),
                    "raw_u": float(wu),
                    "confidence": confidence,
                    "reason": result.get("reason", "path_waypoint"),
                }

        # Legacy locked/inferred fallback when explore is disabled.
        status = str(result.get("status", "searching")).strip().lower()
        u, v = result.get("u"), result.get("v")

        if bool(result.get("usable", result.get("_point_valid", False))) and u is not None and v is not None:
            return {
                "visible": True,
                "point_kind": "locked",
                "u": float(u),
                "v": float(v),
                "cx": float(u),
                "raw_u": float(u),
                "confidence": confidence,
            }

        direction_valid = bool(result.get("direction_valid", False))
        if (
            direction_valid
            and status == "inferred"
            and u is not None
            and v is not None
            and confidence >= self.min_inferred_confidence
        ):
            return {
                "visible": True,
                "point_kind": "inferred",
                "u": float(u),
                "v": float(v),
                "cx": float(u),
                "raw_u": float(u),
                "confidence": confidence,
                "inferred_confidence": confidence,
                "reason": result.get("_coord_reason", "inferred_waypoint"),
            }

        return {
            "visible": False,
            "point_kind": "none",
            "u": u,
            "v": v,
            "reason": result.get("_coord_reason", "no_valid_uv"),
        }

    def _hold_window_sec(self) -> float:
        return self.hold_last_target_sec

    def _maybe_hold_last_target(self, target: Dict[str, Any]) -> Tuple[Dict[str, Any], bool]:
        """Brief Qwen dropout: keep tracking with last locked u (never hold inferred)."""
        if target.get("point_kind") == "inferred":
            return target, False
        if target.get("visible", False):
            return target, False
        if self.hold_last_target_sec <= 0.0 or self.last_held_target is None:
            return target, False
        if self.last_held_target.get("point_kind") != "locked":
            return target, False
        if self.last_target_seen_time is None:
            return target, False
        age = time.time() - self.last_target_seen_time
        within_time = age <= self._hold_window_sec()
        within_miss_budget = self.consecutive_miss_count < self.hold_max_miss_frames
        if not (within_time or within_miss_budget):
            return target, False
        u = self.filtered_u if self.filtered_u is not None else self.last_held_target.get("u")
        if u is None:
            return target, False
        held = {
            "visible": True,
            "point_kind": "locked",
            "u": float(u),
            "v": self.last_held_target.get("v"),
            "cx": float(u),
            "raw_u": float(u),
            "held": True,
            "reason": "hold_last_target",
        }
        return held, True

    def _filter_target_u(self, raw_u: float, *, snap: bool = False) -> Tuple[float, float]:
        if not self.target_filter_enabled or snap:
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

    def _apply_target_filter(self, target: Dict[str, Any], *, snap: bool = False) -> Dict[str, Any]:
        if not target.get("visible", False):
            return target
        raw_u = target.get("u")
        if raw_u is None:
            return target
        raw_u_val, filtered_u_val = self._filter_target_u(float(raw_u), snap=snap)
        out = dict(target)
        out["raw_u"] = raw_u_val
        out["u"] = filtered_u_val
        out["cx"] = filtered_u_val
        return out

    def _filter_target_distance(self, raw_dist: Optional[float]) -> Optional[float]:
        if raw_dist is None:
            return self.filtered_target_distance
        raw = float(raw_dist)
        if not self.target_dist_filter_enabled:
            self.filtered_target_distance = raw
            return raw
        if self.filtered_target_distance is None:
            self.filtered_target_distance = raw
            return raw
        if abs(raw - self.filtered_target_distance) > self.max_target_dist_jump_m:
            return self.filtered_target_distance
        alpha = self.target_dist_smooth_alpha
        self.filtered_target_distance = alpha * raw + (1.0 - alpha) * self.filtered_target_distance
        return self.filtered_target_distance

    def _resolve_target_distance(
        self,
        depth_state,
        center_error: Optional[float],
    ) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        raw_ray = depth_state.target_distance_raw
        if raw_ray is None:
            raw_ray = depth_state.target_distance
        filtered_ray = self._filter_target_distance(raw_ray)
        safety_fused = fuse_target_distance(
            depth_state.front_distance,
            filtered_ray,
            center_error,
            center_deadband=self.arrive_center_deadband,
            fuse_when_centered=self.arrive_fuse_front_when_centered,
            fuse_min_always=self.arrive_fuse_min_always,
        )
        arrive_dist = resolve_arrive_distance(
            depth_state.front_distance,
            filtered_ray,
            arrive_threshold=self.arrive_distance,
            front_margin_m=self.arrive_front_margin_m,
            ray_far_invalid_m=self.arrive_ray_far_invalid_m,
        )
        return raw_ray, filtered_ray, safety_fused, arrive_dist

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

    def _start_target_scan_360(self) -> None:
        self.phase = PHASE_TARGET_SCAN_360
        now = time.time()
        self.scan_start_time = now
        self.scan_yaw_integrated = 0.0
        self._last_scan_integrate_time = None
        self.scan_last_query_time = 0.0
        self.path_point = None
        self.path_confidence = 0.0
        self.explore_start_time = None
        self.explore_end_time = None
        self.explore_phase_reason = "target_scan_start"
        self.publish_stop()
        self.get_logger().info(
            f"[explore] phase=TARGET_SCAN_360 start yaw_target="
            f"{math.degrees(self.full_scan_yaw_target):.0f}deg wz={self.full_scan_wz:+.3f}"
        )

    def _scan_turn_complete(self, now: float) -> bool:
        if self.scan_yaw_integrated >= self.full_scan_yaw_target:
            return True
        if self.scan_start_time is not None and (now - self.scan_start_time) >= self.full_scan_max_sec:
            self.get_logger().warn(
                f"[explore] scan yaw timeout integrated={math.degrees(self.scan_yaw_integrated):.0f}deg "
                f"target={math.degrees(self.full_scan_yaw_target):.0f}deg"
            )
            return True
        return False

    def _full_scan_cmd(self) -> Twist:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.full_scan_wz
        return msg

    def _compute_path_align_cmd(self, waypoint_u: float) -> Tuple[str, Twist]:
        ex = (float(waypoint_u) - self.image_width * 0.5) / max(float(self.image_width), 1.0)
        cmd = Twist()
        if abs(ex) > self.path_turn_threshold:
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(
                -self.path_kp_turn * ex,
                -abs(self.full_scan_wz),
                abs(self.full_scan_wz),
            )
            return "EXPLORE_ALIGN", cmd
        cmd.linear.x = self.explore_vx
        cmd.angular.z = clamp(-self.path_kp_turn * ex * 0.5, -0.08, 0.08)
        return "EXPLORE_FORWARD", cmd

    def _explore_motion_tick(self, now: float) -> None:
        if self.require_lidar and not self._scan_is_fresh():
            self.publish_stop()
            self.explore_phase_reason = "wait_scan_stale"
            return

        if self.path_point is None:
            self._start_target_scan_360()
            return

        waypoint_u = self.path_point[0]
        sub_phase, cmd = self._compute_path_align_cmd(waypoint_u)

        if sub_phase == "EXPLORE_ALIGN":
            self.phase = PHASE_EXPLORE_ALIGN
            self.explore_phase_reason = "align_to_path"
            self._set_desired(cmd, "EXPLORE_ALIGN", fresh=True)
            self._publish_explore_state(cmd)
            return

        if self.phase != PHASE_EXPLORE_FORWARD or self.explore_end_time is None:
            duration = self.explore_distance_m / max(self.explore_vx, 1e-3)
            duration = min(duration, self.explore_max_sec)
            self.explore_start_time = now
            self.explore_end_time = now + duration
            self.phase = PHASE_EXPLORE_FORWARD
            self.explore_phase_reason = "explore_forward_start"
            self.get_logger().info(
                f"[explore] explore forward start, duration={duration:.1f}s vx={self.explore_vx:.3f}"
            )

        if self.explore_end_time is not None and now >= self.explore_end_time:
            self.publish_stop()
            self.get_logger().info("[explore] explore forward done, rescan target")
            self._start_target_scan_360()
            return

        front_distance = self.lidar.front_distance()
        if front_distance is not None and float(front_distance) <= self.explore_hard_stop_distance:
            self.publish_stop()
            self.explore_phase_reason = "OBSTACLE_STOP"
            self.get_logger().info(
                f"[explore] obstacle stop, front_distance={float(front_distance):.3f}"
            )
            self._start_target_scan_360()
            return

        cmd.linear.x = self.explore_vx
        self._set_desired(cmd, "EXPLORE_FORWARD", fresh=True)
        self._publish_explore_state(cmd)

    def _publish_explore_state(self, cmd: Twist) -> None:
        payload: Dict[str, Any] = {
            "step": self.step_count,
            "action": self.explore_phase_reason or self.phase,
            "phase": self.phase,
            "point_kind": "path" if self.path_point else "none",
            "cmd_vx": float(cmd.linear.x),
            "cmd_wz": float(cmd.angular.z),
            "path_confidence": self.path_confidence,
            "path_point": list(self.path_point) if self.path_point else None,
            "front_distance": self.lidar.front_distance(),
        }
        if self.path_point is not None:
            payload["waypoint_u"] = self.path_point[0]
            payload["waypoint_v"] = self.path_point[1]
            payload["u"] = self.path_point[0]
            payload["v"] = self.path_point[1]
        self.state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _handle_path_result(self, result: Dict[str, Any]) -> None:
        mode = str(result.get("mode", "")).upper()
        conf = float(result.get("confidence", 0.0) or 0.0)
        wu = result.get("waypoint_u")
        wv = result.get("waypoint_v")

        if (
            mode == "PATH"
            and bool(result.get("usable", False))
            and bool(result.get("waypoint_visible", False))
            and wu is not None
            and wv is not None
            and conf >= self.path_confidence_threshold
        ):
            self.path_point = (float(wu), float(wv))
            self.path_confidence = conf
            self.phase = PHASE_EXPLORE_ALIGN
            self.explore_phase_reason = "path_accepted"
            self.publish_stop()
            self.get_logger().info(
                f"[explore] path waypoint=({wu:.0f},{wv:.0f}), confidence={conf:.2f}"
            )
            return

        self.explore_phase_reason = "NO_SAFE_PATH"
        self.get_logger().info(
            f"[explore] NO_SAFE_PATH mode={mode} conf={conf:.2f} usable={result.get('usable')}"
        )
        self._start_target_scan_360()

    def _resolve_infer_mode(self) -> str:
        if not self.explore_enable:
            return self._resolve_qwen_mode()
        if self.phase in (PHASE_ASK_QWEN_PATH,):
            return "path"
        return "target"

    def _submit_infer(self, frame):
        if self.future is not None:
            return
        self.step_count += 1
        self.future_frame = frame
        qwen_mode = self._resolve_infer_mode()
        first_request = not self.qwen_first_request_sent
        self.qwen_first_request_sent = True
        self.future = self.query_executor.submit(
            self.qwen.infer_navigation,
            frame,
            self.instruction,
            mode=qwen_mode,
            first_request=first_request,
        )
        # Interval is from submit time so a 1s setting ≈ 1Hz when API latency < 1s.
        self.next_query_time = time.time() + self.qwen_interval_sec
        self.get_logger().info(
            f"submitted cloud Qwen infer step={self.step_count} mode={qwen_mode} first={first_request}"
        )

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
        if self.arrived_locked:
            self.publish_stop()
            depth_state = self.lidar.estimate_for_point(self.filtered_u, self.image_width)
            self.get_logger().info(
                f"step={self.step_count} state=ARRIVED locked "
                f"front={depth_state.front_distance} target_dist={self.filtered_target_distance}"
            )
            return

        qwen_mode = str(result.get("mode", "")).upper()

        if self.explore_enable and self.phase == PHASE_ASK_QWEN_PATH:
            self._handle_path_result(result)
            self.state_pub.publish(String(data=json.dumps({
                "step": self.step_count,
                "action": self.explore_phase_reason or "ASK_QWEN_PATH",
                "phase": self.phase,
                "qwen_mode": qwen_mode,
                "path_confidence": self.path_confidence,
                "path_point": list(self.path_point) if self.path_point else None,
            }, ensure_ascii=False)))
            if self.save_debug and frame is not None:
                debug_pt = (
                    {"u": self.path_point[0], "v": self.path_point[1], "point_kind": "path"}
                    if self.path_point
                    else {"u": None, "v": None, "point_kind": "none"}
                )
                self._save_latest_debug_frame(frame, debug_pt, result)
            return

        if self.explore_enable and self.phase == PHASE_TARGET_SCAN_360:
            if qwen_mode == "TARGET" and bool(result.get("usable", False)):
                self.publish_stop()
                self.phase = PHASE_TARGET_SERVO
                self.get_logger().info("[explore] target found, switch to TARGET_SERVO")
            else:
                elapsed = 0.0
                if self.scan_start_time is not None:
                    elapsed = time.time() - self.scan_start_time
                self.get_logger().info(
                    f"[explore] phase=TARGET_SCAN_360 elapsed={elapsed:.1f}s "
                    f"yaw={math.degrees(self.scan_yaw_integrated):.0f}/"
                    f"{math.degrees(self.full_scan_yaw_target):.0f}deg mode={qwen_mode}"
                )
                if self.save_debug and frame is not None:
                    self._save_latest_debug_frame(frame, {"u": None, "v": None, "point_kind": "none"}, result)
                return

        if self.explore_enable and qwen_mode == "PATH" and self.phase != PHASE_ASK_QWEN_PATH:
            if self.save_debug and frame is not None:
                self._save_latest_debug_frame(frame, {"u": None, "v": None, "point_kind": "none"}, result)
            return

        target = self._parse_qwen_point(result)
        target, held_target = self._maybe_hold_last_target(target)
        raw_u_log = result.get("u")
        filtered_u_log: Optional[float] = None
        center_error: Optional[float] = None
        wz_out = 0.0
        turn_angle_deg = 0.0
        remaining_yaw_deg = 0.0
        target_dist_raw_log: Optional[float] = None
        target_dist_fused_log: Optional[float] = None
        target_dist_arrive_log: Optional[float] = None
        search_direction = self.scan_direction
        lost_duration = self._lost_duration()
        hold_reason = "hold_last_target" if held_target else ""

        point_kind = target.get("point_kind", "none")
        depth_state = self.lidar.estimate_for_point(target.get("u"), self.image_width)
        action, cmd, servo_state, reason = "STOP_OBSERVE", Twist(), "STOP", ""

        if target.get("visible", False) and point_kind == "locked":
            if not held_target:
                snap_u = self.servo.angle_servo_enabled and self.servo.angle_wait_turn_complete
                target = self._apply_target_filter(target, snap=snap_u)
            raw_u_log = target.get("raw_u", raw_u_log)
            filtered_u_log = target.get("u")
            if filtered_u_log is not None:
                center_error = (float(filtered_u_log) - self.image_width / 2.0) / max(1.0, float(self.image_width))
                self.last_target_ex = float(center_error)

            self.lost_since_time = None
            self.lost_stop_since = None
            self.last_scan_flip_time = None
            if not held_target:
                self.last_target_seen_time = time.time()
                self.last_held_target = dict(target)
                self.consecutive_miss_count = 0
            else:
                self.consecutive_miss_count += 1

            depth_state = self.lidar.estimate_for_point(target.get("u"), self.image_width)
            target_dist_raw_log, target_dist_filtered, target_dist_fused, target_dist_arrive = (
                self._resolve_target_distance(depth_state, center_error)
            )
            target_dist_fused_log = target_dist_fused
            target_dist_arrive_log = target_dist_arrive
            # TEMP: lidar_arrive_enable=false → pass arrive_distance=None so servo skips ARRIVED.
            arrive_for_servo = target_dist_arrive if self.lidar_arrive_enable else None
            servo_res = self.servo.compute_cmd(
                target,
                depth_state.front_distance,
                target_dist_fused,
                arrive_distance=arrive_for_servo,
            )
            cmd, servo_state, reason = servo_res.cmd, servo_res.state, servo_res.reason
            if held_target:
                reason = hold_reason
            wz_out = servo_res.wz
            turn_angle_deg = servo_res.turn_angle_deg
            remaining_yaw_deg = servo_res.remaining_yaw_deg
            if servo_state == "ARRIVED" and self.lidar_arrive_enable:
                self.arrived_locked = True
                self.arrive_frame_count += 1
                self.publish_stop()
                self._set_desired(cmd, "ARRIVED", fresh=True)
                action = "ARRIVED"
                self.success = True
                if self.explore_enable:
                    self.phase = PHASE_SUCCESS
                self.get_logger().info(
                    f"ARRIVED: arrive_dist={target_dist_arrive_log:.3f}m "
                    f"front={depth_state.front_distance:.3f}m "
                    f"<={self.arrive_distance:.3f}m (locked)"
                )
            else:
                self.arrive_frame_count = 0
                self._set_desired(cmd, servo_state, fresh=True)
                action = "HOLD_TRACK" if held_target else servo_res.state
            self.lost_scan_count = 0
            if self.explore_enable:
                self.phase = PHASE_TARGET_SERVO
        elif target.get("visible", False) and point_kind == "inferred":
            if self.explore_enable:
                self.consecutive_miss_count += 1
                if self.lost_since_time is None:
                    self.lost_since_time = time.time()
                if self.phase != PHASE_TARGET_SCAN_360:
                    self._start_target_scan_360()
                cmd = self._full_scan_cmd()
                wz_out = float(cmd.angular.z)
                self._set_desired(cmd, "TARGET_SCAN_360", fresh=True)
                action, servo_state, reason = "TARGET_SCAN_360", "TARGET_SCAN_360", "ignore_inferred_use_scan"
            else:
                snap_u = self.servo.angle_servo_enabled and self.servo.angle_wait_turn_complete
                target = self._apply_target_filter(target, snap=snap_u)
                raw_u_log = target.get("raw_u", result.get("u"))
                filtered_u_log = target.get("u")
                if filtered_u_log is not None:
                    center_error = (float(filtered_u_log) - self.image_width / 2.0) / max(1.0, float(self.image_width))
                    self.last_target_ex = float(center_error)

                self.lost_since_time = None
                self.lost_stop_since = None
                self.last_scan_flip_time = None
                self.last_target_seen_time = None
                self.last_held_target = None
                self.consecutive_miss_count = 0
                self.filtered_target_distance = None
                self.lost_scan_count = 0

                depth_state = self.lidar.estimate_for_point(target.get("u"), self.image_width)
                servo_res = self.servo.compute_cmd(
                    target,
                    depth_state.front_distance,
                    None,
                    arrive_distance=None,
                    point_kind="inferred",
                    inferred_confidence=target.get("inferred_confidence"),
                    inferred_vx_scale=self.inferred_vx_scale,
                )
                cmd, servo_state, reason = servo_res.cmd, servo_res.state, servo_res.reason
                wz_out = servo_res.wz
                turn_angle_deg = servo_res.turn_angle_deg
                remaining_yaw_deg = servo_res.remaining_yaw_deg
                self.arrive_frame_count = 0
                self._set_desired(cmd, servo_state, fresh=True)
                action = "INFERRED_NAV"
        elif self.require_lidar and not self._scan_is_fresh():
            self.filtered_target_distance = None
            self.publish_stop()
            action, reason = "WAIT_SCAN_STALE", "scan_stale_no_target"
            self.lost_scan_count += 1
            if self.lost_since_time is None:
                self.lost_since_time = time.time()
        else:
            self.consecutive_miss_count += 1
            self.filtered_target_distance = None
            if self.lost_since_time is None:
                self.lost_since_time = time.time()
            lost_duration = self._lost_duration()
            self.lost_scan_count += 1

            if self.explore_enable:
                if self.phase == PHASE_TARGET_SERVO and lost_duration < self.lost_stop_sec:
                    self.servo.clear_remaining()
                    cmd = Twist()
                    self._set_desired(cmd, "LOST_HOLD", fresh=True)
                    action, servo_state, reason = "LOST_HOLD", "LOST_HOLD", "lost_wait_reacquire"
                else:
                    if self.phase != PHASE_TARGET_SCAN_360:
                        self._start_target_scan_360()
                    action, servo_state, reason = "TARGET_SCAN_360", "TARGET_SCAN_360", "target_lost_rescan"
                    cmd = self._full_scan_cmd()
                    wz_out = float(cmd.angular.z)
                    self._set_desired(cmd, "TARGET_SCAN_360", fresh=True)
            elif lost_duration < self.lost_stop_sec:
                self.servo.clear_remaining()
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
                    self.servo.clear_remaining()
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
                self.servo.clear_remaining()
                search_direction = self.scan_direction
                wz_out = float(cmd.angular.z)
                self._set_desired(cmd, "SEARCH_SCAN", fresh=True)
                action, servo_state, reason = "SEARCH_SCAN", "SEARCH_SCAN", target.get("reason", "qwen_no_valid_point")

        cmd_age = self._cmd_age()
        _, publish_mode = self._cmd_with_hold()

        lidar_payload = {
            "front_distance": depth_state.front_distance,
            "target_distance": target_dist_fused_log if target_dist_fused_log is not None else depth_state.target_distance,
            "target_distance_raw": target_dist_raw_log if target_dist_raw_log is not None else depth_state.target_distance_raw,
            "target_distance_fused": target_dist_fused_log,
            "target_distance_arrive": target_dist_arrive_log,
            "target_angle_deg": depth_state.target_angle_deg,
            "lidar_valid": depth_state.valid,
            "lidar_reason": depth_state.reason,
        }
        self.json_pub.publish(String(data=json.dumps({
            "u": filtered_u_log if filtered_u_log is not None else result.get("u"),
            "v": result.get("v"),
            "waypoint_u": result.get("waypoint_u"),
            "waypoint_v": result.get("waypoint_v"),
            "raw_u": raw_u_log,
            "filtered_u": filtered_u_log,
            "raw_v": result.get("_raw_v"),
            "mode": result.get("mode"),
            "usable": bool(result.get("usable", False)),
            "direction_valid": bool(result.get("direction_valid", False)),
            "point_kind": point_kind,
            "status": result.get("status"),
            "phase": self.phase,
            "qwen_mode": result.get("_qwen_mode"),
            "first_request": bool(result.get("_first_request", False)),
            "confidence": result.get("confidence"), "reason": result.get("reason"),
            "coord_reason": result.get("_coord_reason"), "state": action,
            "latency_sec": result.get("_latency_sec"), **lidar_payload,
        }, ensure_ascii=False)))
        self.state_pub.publish(String(data=json.dumps({
            "step": self.step_count, "action": action, "servo_state": servo_state,
            "phase": self.phase,
            "point_kind": point_kind,
            "reason": reason, "cmd_vx": float(cmd.linear.x), "cmd_wz": float(cmd.angular.z),
            "raw_u": raw_u_log, "filtered_u": filtered_u_log,
            "center_error": center_error, "cmd_age": cmd_age, "cmd_mode": publish_mode,
            "cmd_wz": wz_out, "last_target_ex": self.last_target_ex,
            "turn_angle_deg": turn_angle_deg, "remaining_yaw_deg": remaining_yaw_deg,
            "search_direction": search_direction, "lost_duration": lost_duration,
            "lost_scan_count": self.lost_scan_count, **lidar_payload,
        }, ensure_ascii=False)))
        self.get_logger().info(
            f"step={self.step_count} state={action} kind={point_kind} raw_u={raw_u_log} filtered_u={filtered_u_log} "
            f"ex={center_error if center_error is not None else 'n/a'} "
            f"vx={cmd.linear.x:.3f} cmd_wz={cmd.angular.z:+.3f} servo_wz={wz_out:+.3f} "
            f"turn_deg={turn_angle_deg:.1f} rem_yaw_deg={remaining_yaw_deg:.1f} "
            f"cmd_age={cmd_age:.2f} {publish_mode} "
            f"search_dir={search_direction:+.0f} lost_dur={lost_duration:.2f} "
            f"front={depth_state.front_distance} target_dist={target_dist_fused_log if target_dist_fused_log is not None else depth_state.target_distance} "
            f"arrive_dist={target_dist_arrive_log} "
            f"target_raw={target_dist_raw_log} "
            f"conf={result.get('confidence')} "
            f"latency={result.get('_latency_sec')}"
        )
        if self.save_debug and frame is not None:
            debug_target = target if target.get("visible") else {"u": None, "v": None, "point_kind": point_kind}
            self._save_latest_debug_frame(frame, debug_target, result)

    def decision_timer_cb(self):
        if self.success or self.arrived_locked or self.latest_frame is None:
            return
        if self._poll_future():
            return
        if self.servo.angle_servo_enabled and self.servo.angle_wait_turn_complete:
            if self.servo.is_turn_busy() and self.phase == PHASE_TARGET_SERVO:
                return

        now = time.time()
        frame = self.latest_frame.copy()
        self._sync_image_geometry(frame)

        if self.explore_enable:
            if self.phase in (PHASE_EXPLORE_ALIGN, PHASE_EXPLORE_FORWARD):
                self._explore_motion_tick(now)
                return

            if self.phase == PHASE_TARGET_SCAN_360:
                if self._scan_turn_complete(now):
                    self.publish_stop()
                    self.phase = PHASE_ASK_QWEN_PATH
                    self.explore_phase_reason = "ASK_QWEN_PATH"
                    self.get_logger().info(
                        f"[explore] full scan done yaw={math.degrees(self.scan_yaw_integrated):.0f}deg, "
                        f"ask qwen path"
                    )
                    if self.require_lidar and not self._scan_is_fresh():
                        self.publish_stop()
                        self.next_query_time = now + self.lidar_wait_backoff_sec
                        return
                    self._submit_infer(frame)
                    return

                self._set_desired(self._full_scan_cmd(), "TARGET_SCAN_360", fresh=True)
                if (now - self.scan_last_query_time) >= self.full_scan_query_interval_sec:
                    if self.require_lidar and not self._scan_is_fresh():
                        self.publish_stop()
                        self.next_query_time = now + self.lidar_wait_backoff_sec
                    else:
                        self._submit_infer(frame)
                        self.scan_last_query_time = now
                return

            if self.phase == PHASE_ASK_QWEN_PATH:
                self.publish_stop()
                if now < self.next_query_time:
                    return
                if self.require_lidar and not self._scan_is_fresh():
                    self.publish_stop()
                    self.next_query_time = now + self.lidar_wait_backoff_sec
                    return
                self._submit_infer(frame)
                return

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
