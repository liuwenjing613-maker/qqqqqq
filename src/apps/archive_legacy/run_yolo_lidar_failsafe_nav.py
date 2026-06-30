#!/usr/bin/env python3
"""
YOLO-World + LiDAR failsafe navigation — layered architecture.

Sensor: /scan, /image_raw, /target_bbox_json
Behavior + Local Planner @ decision_rate_hz (default 5Hz)
Safety + Controller @ control_rate_hz (default 20Hz)
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.perception.free_space_waypoint import FreeSpaceConfig, FreeSpaceWaypointProvider
from src.perception.target_bbox_parser import TargetBBoxParser, TargetBBoxParserConfig


DEFAULT_CONFIG = str(ROOT / "configs" / "yolo_lidar_failsafe_nav.yaml")


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def load_yaml(path: str) -> Dict[str, Any]:
    path = os.path.expanduser(path)
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise RuntimeError(f"Config file did not parse into a dict: {path}")
    return cfg


def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return str(value)


class YoloLidarFailsafeNav(Node):
    def __init__(self, cfg: Dict[str, Any], instruction: str):
        super().__init__("yolo_lidar_failsafe_nav")
        self.cfg = cfg
        self.instruction = instruction or cfg.get("instruction", "bottle")

        self.image_width = int(cfg.get("image_width", cfg.get("camera_width", 640)))
        self.image_height = int(cfg.get("image_height", cfg.get("camera_height", 480)))

        self.step_count = 0
        self.success = False
        self.arrive_count = 0
        self.forward_burst_count = 0
        self.last_cmd = Twist()
        self.last_image_time = 0.0

        self.emergency_stop_distance = float(cfg.get("emergency_stop_distance", 0.50))
        self.hard_stop_distance = float(cfg.get("hard_stop_distance", 0.65))
        self.slow_distance = float(cfg.get("slow_distance", 0.90))
        self.arrive_distance = float(cfg.get("success_distance", cfg.get("arrive_distance", 0.6)))
        self.arrive_required_count = max(1, int(cfg.get("arrive_required_count", 2)))
        self.exit_on_success = bool(cfg.get("exit_on_success", True))
        self.success_shutdown_delay_sec = float(cfg.get("success_shutdown_delay_sec", 0.5))
        self.explore_vx = float(cfg.get("explore_vx", 0.035))
        self.explore_slow_vx = float(cfg.get("explore_slow_vx", 0.025))
        self.explore_kp_turn = float(cfg.get("explore_kp_turn", 0.10))
        self.explore_max_wz = float(cfg.get("explore_max_wz", 0.15))
        self.explore_turn_only_threshold = float(cfg.get("explore_turn_only_threshold", 0.18))
        self.explore_forward_turn_scale = float(cfg.get("explore_forward_turn_scale", 0.45))

        self.recovery_wz = float(cfg.get("scan_wz", cfg.get("blocked_rotate_wz", 0.16)))
        self.recovery_side_deg = float(cfg.get("recovery_side_deg", 45.0))

        self.center_arrive_px = float(cfg.get("center_arrive_px", 80))
        self.min_forward_bursts_before_arrive = int(cfg.get("min_forward_bursts_before_arrive", 2))
        self.max_steps = int(cfg.get("max_steps", 600))
        self.min_state_frames = int(cfg.get("min_state_frames", 2))
        self.bad_wp_grace_frames = int(cfg.get("bad_wp_grace_frames", 2))
        self.no_target_grace_frames = int(cfg.get("no_target_grace_frames", 2))
        self.bad_wp_limit = int(cfg.get("bad_wp_limit", 3))

        self.target_reacquire_wz = float(cfg.get("target_reacquire_wz", 0.16))
        self.max_scan_age_sec = float(cfg.get("max_scan_age_sec", 0.35))

        self.control_rate_hz = float(cfg.get("control_rate_hz", 20.0))
        self.max_cmd_vx = float(cfg.get("max_cmd_vx", cfg.get("target_max_vx", 0.05)))
        self.max_cmd_wz = float(cfg.get("max_cmd_wz", 0.24))
        self.wz_slow_vx_threshold = float(cfg.get("wz_slow_vx_threshold", 0.10))
        self.wz_zero_vx_threshold = float(cfg.get("wz_zero_vx_threshold", 0.16))
        self.wz_slow_vx_scale = float(cfg.get("wz_slow_vx_scale", 0.5))

        self.min_drive_vx = float(cfg.get("min_drive_vx", 0.01))
        self.min_turn_wz = float(cfg.get("min_turn_wz", 0.12))
        self.vx_deadband = float(cfg.get("vx_deadband", 0.005))
        self.wz_deadband = float(cfg.get("wz_deadband", 0.015))

        self.fsm_mode = "INIT"
        self.fsm_mode_frames = 0
        self._last_blocked_reason = "no_free_space"

        self.desired_cmd = Twist()
        self.desired_mode = "INIT"
        self.desired_reason = "init"
        self.recovery_turn_side = ""
        self.recovery_turn_dir = 1.0

        self.last_safety_info: Dict[str, Any] = {}
        self.bad_wp_count = 0
        self.no_target_count = 0
        self._control_tick = 0
        self._state_publish_div = max(1, int(round(self.control_rate_hz / max(float(cfg.get("decision_rate_hz", 5.0)), 1.0))))
        self._decision_timer = None
        self._success_shutdown_timer = None

        fs_cfg = FreeSpaceConfig(
            lidar_min_range=float(cfg.get("lidar_min_range", 0.08)),
            lidar_max_range=float(cfg.get("lidar_max_range", 6.0)),
            lidar_front_deg=float(cfg.get("lidar_front_deg", 25.0)),
            camera_hfov_deg=float(cfg.get("camera_hfov_deg", 70.0)),
            camera_lidar_yaw_offset_deg=float(cfg.get("camera_lidar_yaw_offset_deg", 0.0)),
            sector_deg=float(cfg.get("free_space_sector_deg", 70.0)),
            step_deg=float(cfg.get("free_space_step_deg", 5.0)),
            window_deg=float(cfg.get("free_space_window_deg", 10.0)),
            min_clearance=float(cfg.get("free_space_min_clearance", 0.45)),
            good_clearance=float(cfg.get("free_space_good_clearance", 1.20)),
            smooth_alpha=float(cfg.get("free_space_smooth_alpha", 0.22)),
            waypoint_v_ratio=float(cfg.get("free_space_waypoint_v_ratio", 0.62)),
            min_u_ratio=float(cfg.get("free_space_min_u_ratio", 0.20)),
            max_u_ratio=float(cfg.get("free_space_max_u_ratio", 0.80)),
            clearance_weight=float(cfg.get("free_space_clearance_weight", 0.60)),
            center_weight=float(cfg.get("free_space_center_weight", 0.30)),
            consistency_weight=float(cfg.get("free_space_consistency_weight", 0.10)),
            target_window_deg=float(cfg.get("lidar_target_window_deg", 8.0)),
        )
        self.free_space = FreeSpaceWaypointProvider(fs_cfg)

        parser_cfg = TargetBBoxParserConfig(
            target_words=list(cfg.get("target_words", [])),
            target_min_score=float(cfg.get("target_min_score", 0.002)),
            target_stable_frames=int(cfg.get("target_stable_frames", 1)),
            target_lost_timeout_sec=float(cfg.get("target_lost_timeout_sec", 0.8)),
            target_memory_sec=float(cfg.get("target_memory_sec", 1.5)),
            accept_unknown_class=bool(cfg.get("accept_unknown_class", True)),
            bbox_min_area_ratio=float(cfg.get("bbox_min_area_ratio", 0.0005)),
            bbox_max_area_ratio=float(cfg.get("bbox_max_area_ratio", 0.24)),
            bbox_edge_margin_px=int(cfg.get("bbox_edge_margin_px", 4)),
        )
        self.target_parser = TargetBBoxParser(parser_cfg)

        self.image_topic = cfg.get("image_topic", "/image_raw")
        self.scan_topic = cfg.get("scan_topic", "/scan")
        self.cmd_topic = cfg.get("cmd_topic", "/cmd_vel")
        self.target_bbox_topic = cfg.get("target_bbox_topic", "/target_bbox_json")
        self.target_words_topic = cfg.get("target_words_topic", "/target_words")
        self.state_topic = cfg.get("state_topic", "/failsafe_nav_state")
        self.point_topic = cfg.get("point_topic", "/failsafe_nav_point")

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.point_pub = self.create_publisher(String, self.point_topic, 10)
        self.words_pub = self.create_publisher(String, self.target_words_topic, 10)

        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, self.target_bbox_topic, self.bbox_cb, 10)

        decision_rate = float(cfg.get("decision_rate_hz", 5.0))
        self._decision_timer = self.create_timer(1.0 / max(decision_rate, 1e-3), self.decision_timer_cb)
        self.create_timer(1.0 / max(self.control_rate_hz, 1e-3), self.control_timer_cb)

        if bool(cfg.get("publish_target_words", True)):
            period = float(cfg.get("target_words_publish_period_sec", 3.0))
            self.create_timer(max(period, 0.5), self.publish_target_words)

        self.get_logger().info("===== YOLO + LiDAR FAILSAFE NAV (layered) =====")
        self.get_logger().info(
            f"control={self.control_rate_hz}Hz decision={decision_rate}Hz "
            f"emergency={self.emergency_stop_distance}m hard={self.hard_stop_distance}m"
        )

    def has_fresh_scan(self) -> bool:
        age = self.free_space.scan_age()
        return age is not None and age <= self.max_scan_age_sec

    def get_front_min(self) -> Optional[float]:
        return self.free_space.front_min_distance()

    def apply_motion_deadband(self, cmd: Twist) -> Twist:
        out = Twist()
        vx = float(cmd.linear.x)
        wz = float(cmd.angular.z)

        if abs(vx) < self.vx_deadband:
            out.linear.x = 0.0
        elif abs(vx) < self.min_drive_vx:
            out.linear.x = math.copysign(self.min_drive_vx, vx)
        else:
            out.linear.x = vx

        if abs(wz) < self.wz_deadband:
            out.angular.z = 0.0
        elif abs(wz) < self.min_turn_wz and abs(out.linear.x) < 1e-6:
            out.angular.z = math.copysign(self.min_turn_wz, wz)
        else:
            out.angular.z = wz

        return out

    def apply_safety_layer(self, raw_cmd: Twist) -> Tuple[Twist, Dict[str, Any]]:
        now = time.time()
        scan_age = self.free_space.scan_age(now)
        front_min = self.get_front_min()
        front_safe = front_min

        info: Dict[str, Any] = {
            "scan_age": scan_age,
            "front_min_distance": front_min,
            "front_safe_distance": front_safe,
            "safety_limited": False,
            "raw_cmd_vx": float(raw_cmd.linear.x),
            "raw_cmd_wz": float(raw_cmd.angular.z),
            "recovery_turn_side": self.recovery_turn_side,
        }

        safe = Twist()

        if scan_age is None or scan_age > self.max_scan_age_sec:
            info["safe_cmd_vx"] = 0.0
            info["safe_cmd_wz"] = 0.0
            info["safety_reason"] = "stale_scan"
            return safe, info

        if front_min is not None and front_min <= self.emergency_stop_distance:
            info["safe_cmd_vx"] = 0.0
            info["safe_cmd_wz"] = 0.0
            info["safety_reason"] = "emergency_stop"
            info["control_mode"] = "EMERGENCY_STOP"
            return safe, info

        vx = float(raw_cmd.linear.x)
        wz = float(raw_cmd.angular.z)
        safety_reason = "pass_through"

        if front_min is not None and front_min <= self.hard_stop_distance:
            vx = 0.0
            safety_reason = "hard_stop"
            info["safety_limited"] = True
            if self.desired_mode != "BLOCKED_RECOVERY" and abs(wz) < 1e-6:
                wz = 0.0

        elif front_min is not None and front_min < self.slow_distance:
            span = max(self.slow_distance - self.hard_stop_distance, 1e-6)
            scale = clamp((front_min - self.hard_stop_distance) / span, 0.0, 1.0)
            if vx > 0:
                vx *= scale
                info["safety_limited"] = True
                safety_reason = "slow_zone_scale"

        if abs(wz) > self.wz_zero_vx_threshold:
            vx = 0.0
            info["safety_limited"] = True
            safety_reason = "turn_zero_vx"
        elif abs(wz) > self.wz_slow_vx_threshold and vx > 0:
            vx *= self.wz_slow_vx_scale
            info["safety_limited"] = True
            safety_reason = "turn_slow_vx"

        vx = clamp(vx, -self.max_cmd_vx, self.max_cmd_vx)
        wz = clamp(wz, -self.max_cmd_wz, self.max_cmd_wz)

        safe.linear.x = vx
        safe.angular.z = wz
        safe = self.apply_motion_deadband(safe)

        info["safe_cmd_vx"] = float(safe.linear.x)
        info["safe_cmd_wz"] = float(safe.angular.z)
        info["safety_reason"] = safety_reason
        return safe, info

    def control_timer_cb(self) -> None:
        safe_cmd, safety = self.apply_safety_layer(self.desired_cmd)
        self.last_safety_info = safety
        self.last_cmd = safe_cmd
        self.cmd_pub.publish(safe_cmd)

        self._control_tick += 1
        if self._control_tick % self._state_publish_div == 0:
            mode = str(safety.get("control_mode", self.fsm_mode))
            self.publish_state(mode, reason=self.desired_reason, cmd=safe_cmd, from_control=True)

    def _resolve_effective_mode(self, desired_mode: str, immediate: bool = False) -> str:
        if immediate or self.fsm_mode in ("INIT",):
            self.fsm_mode = desired_mode
            self.fsm_mode_frames = 1
            return self.fsm_mode

        if desired_mode == self.fsm_mode:
            self.fsm_mode_frames += 1
            return self.fsm_mode

        if self.fsm_mode_frames < self.min_state_frames:
            self.fsm_mode_frames += 1
            return self.fsm_mode

        self.fsm_mode = desired_mode
        self.fsm_mode_frames = 1
        return self.fsm_mode

    def image_cb(self, msg: Image) -> None:
        self.last_image_time = time.time()
        if msg.width and msg.height:
            self.image_width = int(msg.width)
            self.image_height = int(msg.height)

    def scan_cb(self, msg: LaserScan) -> None:
        self.free_space.update_scan(msg)

    def bbox_cb(self, msg: String) -> None:
        self.target_parser.update_json(msg.data, self.image_width, self.image_height)

    def publish_target_words(self) -> None:
        words = list(self.cfg.get("target_words", []))
        self.words_pub.publish(String(data=",".join(words)))

    def pick_recovery_direction(self) -> Tuple[float, str]:
        left = self.free_space.left_clearance(self.recovery_side_deg)
        right = self.free_space.right_clearance(-self.recovery_side_deg)

        if left is None and right is None:
            return self.recovery_turn_dir, self.recovery_turn_side or "unknown"
        if left is None:
            self.recovery_turn_dir = -1.0
            side = "right"
        elif right is None:
            self.recovery_turn_dir = 1.0
            side = "left"
        elif left >= right:
            self.recovery_turn_dir = 1.0
            side = "left"
        else:
            self.recovery_turn_dir = -1.0
            side = "right"

        self.recovery_turn_side = side
        return self.recovery_turn_dir, side

    def set_desired_cmd(self, cmd: Twist, mode: str, reason: str) -> None:
        self.desired_cmd = cmd
        self.desired_mode = mode
        self.desired_reason = reason

    def get_target_distance(self, target: Dict[str, Any]) -> Optional[float]:
        if not target.get("visible", False):
            return None
        u = target.get("u")
        if u is None:
            return self.get_front_min()
        return self.free_space.target_distance_at_u(float(u), self.image_width)

    def _enter_success_state(
        self,
        target: Dict[str, Any],
        target_distance: float,
        front_min: Optional[float],
    ) -> None:
        if self.success:
            return
        self.success = True
        self.fsm_mode = "ARRIVED"
        self.set_desired_cmd(Twist(), "ARRIVED", "success_target_visible_lidar_close")
        self.get_logger().info(
            "SUCCESS: target visible, "
            f"target_distance={target_distance:.2f}m <= success_distance={self.arrive_distance:.2f}m"
        )
        self.publish_state(
            "ARRIVED",
            target=target,
            front_distance=front_min,
            target_distance=target_distance,
            success=True,
            reason="success_target_visible_lidar_close",
        )
        if self._decision_timer is not None:
            self._decision_timer.cancel()
            self._decision_timer = None
        if self.exit_on_success:
            delay = max(0.2, self.success_shutdown_delay_sec)
            self._success_shutdown_timer = self.create_timer(delay, self._shutdown_after_success)

    def _shutdown_after_success(self) -> None:
        if self._success_shutdown_timer is not None:
            self._success_shutdown_timer.cancel()
            self._success_shutdown_timer = None
        self.set_desired_cmd(Twist(), "ARRIVED", "nav_stopped_after_success")
        self.cmd_pub.publish(Twist())
        self.publish_state(
            "ARRIVED",
            success=True,
            nav_active=False,
            reason="nav_stopped_after_success",
        )
        self.get_logger().info(
            "Navigation stopped after success (nav node exiting). "
            "Camera/LiDAR/YOLO/chassis/Foxglove keep running."
        )
        rclpy.shutdown()

    def decision_timer_cb(self) -> None:
        if self.success:
            return

        self.step_count += 1
        if self.step_count > self.max_steps:
            self.fsm_mode = "STOPPED"
            self.set_desired_cmd(Twist(), "STOPPED", "max_steps")
            self.publish_state("STOPPED", reason="max_steps")
            return

        front_min = self.get_front_min()

        if not self.has_fresh_scan():
            self._resolve_effective_mode("WAIT_SCAN", immediate=True)
            self.set_desired_cmd(Twist(), "WAIT_SCAN", "stale_or_no_scan")
            self.publish_state("WAIT_SCAN", front_distance=front_min, reason="stale_or_no_scan")
            return

        target = self.target_parser.get_target(time.time(), self.image_width, self.image_height)

        if front_min is not None and front_min <= self.emergency_stop_distance:
            self._resolve_effective_mode("EMERGENCY_STOP", immediate=True)
            self.set_desired_cmd(Twist(), "EMERGENCY_STOP", "emergency_front_min")
            self.publish_state("EMERGENCY_STOP", front_distance=front_min, reason="emergency_front_min")
            return

        if target.get("visible", False):
            if self.check_arrive(target, front_min):
                desired_mode = "ARRIVED"
            else:
                desired_mode = "TARGET_TRACK"
        else:
            wp = self.free_space.get_waypoint(self.image_width, self.image_height)
            if not wp.get("usable", False):
                desired_mode = "BLOCKED_RECOVERY"
                self._last_blocked_reason = str(wp.get("reason", "no_free_space"))
            else:
                desired_mode = "FRONTIER_EXPLORE"

        immediate = desired_mode in ("WAIT_SCAN", "EMERGENCY_STOP", "STOPPED")
        effective_mode = self._resolve_effective_mode(desired_mode, immediate=immediate)

        if effective_mode == "ARRIVED":
            target_dist = self.get_target_distance(target)
            if target_dist is not None:
                self._enter_success_state(target, target_dist, front_min)
            return

        if effective_mode == "TARGET_TRACK":
            if not target.get("visible", False):
                self.no_target_count += 1
                if self.no_target_count <= self.no_target_grace_frames:
                    cmd = Twist()
                    last_u = target.get("u", self.image_width / 2.0)
                    ex = (float(last_u) - self.image_width / 2.0) / max(float(self.image_width), 1.0)
                    direction = -1.0 if ex > 0 else 1.0
                    cmd.angular.z = direction * self.target_reacquire_wz
                    self.set_desired_cmd(cmd, "OBSERVE", "target_lost_reacquire")
                    self.publish_state("OBSERVE", target=target, front_distance=front_min, cmd=cmd)
                    return
                self.set_desired_cmd(Twist(), "OBSERVE", "target_lost_hold")
                self.publish_state("OBSERVE", target=target, front_distance=front_min, reason="target_lost_hold")
                return

            self.no_target_count = 0
            cmd, servo_state = self.compute_target_cmd(target, front_min)
            if self.check_arrive(target, front_min):
                target_dist = self.get_target_distance(target)
                if target_dist is not None:
                    self._enter_success_state(target, target_dist, front_min)
                return

            self.set_desired_cmd(cmd, "TARGET_TRACK", servo_state)
            self.publish_point(target.get("u"), target.get("v"), "target", "TARGET_TRACK")
            self.publish_state(
                "TARGET_TRACK",
                target=target,
                front_distance=front_min,
                cmd=cmd,
                reason=servo_state,
                fsm_frames=self.fsm_mode_frames,
            )
            return

        if effective_mode == "FRONTIER_EXPLORE":
            target_reason = target.get("reason", "target_not_visible")
            wp = self.free_space.get_waypoint(self.image_width, self.image_height)
            if not wp.get("usable", False):
                self.bad_wp_count += 1
                if self.bad_wp_count < self.bad_wp_limit:
                    self.publish_state(
                        "FRONTIER_EXPLORE",
                        front_distance=front_min,
                        reason=f"bad_wp_grace_{self.bad_wp_count}",
                        fsm_frames=self.fsm_mode_frames,
                    )
                    return
                self.run_blocked_recovery(wp.get("reason", "bad_wp_limit"))
                return

            self.bad_wp_count = 0
            cmd, mode = self.compute_explore_cmd(wp, front_min)
            self.set_desired_cmd(cmd, "FRONTIER_EXPLORE", f"{mode}; target={target_reason}")
            self.publish_point(wp.get("u"), wp.get("v"), "free_space", "FRONTIER_EXPLORE")
            self.publish_state(
                "FRONTIER_EXPLORE",
                waypoint=wp,
                front_distance=front_min,
                cmd=cmd,
                reason=f"{mode}; target={target_reason}",
                fsm_frames=self.fsm_mode_frames,
            )
            return

        if effective_mode == "BLOCKED_RECOVERY":
            self.run_blocked_recovery(self._last_blocked_reason)
            return

        self.set_desired_cmd(Twist(), "OBSERVE", f"unhandled_mode={effective_mode}")
        self.publish_state("OBSERVE", reason=f"unhandled_mode={effective_mode}")

    def run_blocked_recovery(self, reason: str) -> None:
        turn_dir, side = self.pick_recovery_direction()
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = turn_dir * abs(self.recovery_wz)
        self.set_desired_cmd(cmd, "BLOCKED_RECOVERY", reason)
        self.publish_state(
            "BLOCKED_RECOVERY",
            front_distance=self.get_front_min(),
            cmd=cmd,
            reason=reason,
            recovery_turn_side=side,
            fsm_frames=self.fsm_mode_frames,
        )

    def compute_target_cmd(self, target: Dict[str, Any], front: Optional[float]):
        cmd = Twist()
        u = float(target.get("u", self.image_width / 2.0))
        ex = (u - self.image_width / 2.0) / max(float(self.image_width), 1.0)

        kp = float(self.cfg.get("target_kp_turn", 0.10))
        max_wz = float(self.cfg.get("target_max_wz", 0.20))
        turn_threshold = float(self.cfg.get("target_turn_threshold", 0.18))
        forward_turn_scale = float(self.cfg.get("target_forward_turn_scale", 0.45))

        if front is not None and front <= self.hard_stop_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd, "front_hard_stop"

        if abs(ex) > turn_threshold:
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(-kp * ex, -max_wz, max_wz)
            return cmd, "target_turn_only"

        if front is not None and front <= self.slow_distance:
            vx = float(self.cfg.get("target_slow_vx", 0.020))
        else:
            vx = float(self.cfg.get("target_mid_vx", 0.035))

        cmd.linear.x = vx
        cmd.angular.z = clamp(-kp * ex * forward_turn_scale, -max_wz, max_wz)

        if cmd.linear.x > 0:
            self.forward_burst_count += 1

        return cmd, "target_forward"

    def compute_explore_cmd(self, wp: Dict[str, Any], front: Optional[float]):
        cmd = Twist()

        if front is not None and front <= self.hard_stop_distance:
            turn_dir, _ = self.pick_recovery_direction()
            cmd.linear.x = 0.0
            cmd.angular.z = turn_dir * abs(self.recovery_wz)
            return cmd, "blocked_recovery"

        u = float(wp.get("u", self.image_width / 2.0))
        ex = (u - self.image_width / 2.0) / max(float(self.image_width), 1.0)

        if abs(ex) >= self.explore_turn_only_threshold:
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(-self.explore_kp_turn * ex, -self.explore_max_wz, self.explore_max_wz)
            return cmd, "explore_turn"

        if front is not None and front <= self.slow_distance:
            cmd.linear.x = self.explore_slow_vx
        else:
            cmd.linear.x = self.explore_vx

        cmd.angular.z = clamp(
            -self.explore_kp_turn * ex * self.explore_forward_turn_scale,
            -self.explore_max_wz,
            self.explore_max_wz,
        )
        return cmd, "explore_forward"

    def check_arrive(self, target: Dict[str, Any], front: Optional[float]) -> bool:
        del front
        if not target.get("visible", False):
            self.arrive_count = 0
            return False

        target_dist = self.get_target_distance(target)
        if target_dist is None:
            self.arrive_count = 0
            return False
        if target_dist > self.arrive_distance:
            self.arrive_count = 0
            return False

        self.arrive_count += 1
        return self.arrive_count >= self.arrive_required_count

    def publish_stop(self) -> None:
        self.set_desired_cmd(Twist(), "STOPPED", "manual_stop")
        self.cmd_pub.publish(Twist())

    def publish_point(self, u: Any, v: Any, source: str, mode: str) -> None:
        data = {
            "u": u,
            "v": v,
            "source": source,
            "mode": mode,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "time": time.time(),
        }
        self.point_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def publish_state(self, mode: str, from_control: bool = False, **kwargs: Any) -> None:
        cmd = kwargs.pop("cmd", None)
        safety = self.last_safety_info if self.last_safety_info else {}

        raw_vx = float(cmd.linear.x) if cmd is not None else float(safety.get("raw_cmd_vx", self.desired_cmd.linear.x))
        raw_wz = float(cmd.angular.z) if cmd is not None else float(safety.get("raw_cmd_wz", self.desired_cmd.angular.z))

        data = {
            "step": self.step_count,
            "mode": mode,
            "fsm_mode": self.fsm_mode,
            "desired_mode": self.desired_mode,
            "fsm_frames": self.fsm_mode_frames,
            "min_state_frames": self.min_state_frames,
            "instruction": self.instruction,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "front_distance": kwargs.pop("front_distance", self.get_front_min()),
            "front_min_distance": safety.get("front_min_distance", self.get_front_min()),
            "front_safe_distance": safety.get("front_safe_distance", self.get_front_min()),
            "scan_age": safety.get("scan_age", self.free_space.scan_age()),
            "safety_limited": safety.get("safety_limited", False),
            "raw_cmd_vx": raw_vx,
            "raw_cmd_wz": raw_wz,
            "safe_cmd_vx": float(safety.get("safe_cmd_vx", self.last_cmd.linear.x)),
            "safe_cmd_wz": float(safety.get("safe_cmd_wz", self.last_cmd.angular.z)),
            "recovery_turn_side": kwargs.pop("recovery_turn_side", self.recovery_turn_side),
            "cmd_vx": float(safety.get("safe_cmd_vx", self.last_cmd.linear.x)),
            "cmd_wz": float(safety.get("safe_cmd_wz", self.last_cmd.angular.z)),
            "desired_reason": self.desired_reason,
            "success": self.success,
            "success_distance": self.arrive_distance,
            "target_distance": kwargs.pop("target_distance", None),
            "time": time.time(),
        }
        data.update(_json_safe(kwargs))
        payload = json.dumps(data, ensure_ascii=False)
        self.state_pub.publish(String(data=payload))
        if not from_control:
            self.get_logger().info(payload)


def main():
    parser = argparse.ArgumentParser(description="YOLO + LiDAR failsafe navigation (layered)")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--instruction", default=None)
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    instruction = args.instruction or cfg.get("instruction", "bottle")

    rclpy.init()
    node = YoloLidarFailsafeNav(cfg, instruction)
    try:
        rclpy.spin(node)
    finally:
        if node.success:
            node.cmd_pub.publish(Twist())
        else:
            node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
