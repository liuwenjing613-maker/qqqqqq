#!/usr/bin/env python3
"""
YOLO-World + LiDAR failsafe navigation (P0).

/image_raw + /scan + /target_bbox_json -> visual servo / free-space explore -> /cmd_vel.
Does NOT use Qwen/Ollama.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

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
        self.latest_front_distance: Optional[float] = None

        self.emergency_stop_distance = float(cfg.get("emergency_stop_distance", 0.22))
        self.hard_stop_distance = float(cfg.get("hard_stop_distance", 0.32))
        self.slow_distance = float(cfg.get("slow_distance", 0.55))
        self.arrive_distance = float(cfg.get("arrive_distance", 0.38))

        self.explore_vx = float(cfg.get("explore_vx", 0.04))
        self.explore_slow_vx = float(cfg.get("explore_slow_vx", 0.02))
        self.explore_kp_turn = float(cfg.get("explore_kp_turn", 0.20))
        self.explore_max_wz = float(cfg.get("explore_max_wz", 0.12))
        self.explore_turn_only_threshold = float(cfg.get("explore_turn_only_threshold", 0.22))
        self.explore_forward_turn_scale = float(cfg.get("explore_forward_turn_scale", 0.55))

        self.scan_wz = float(cfg.get("scan_wz", 0.12))
        self.blocked_rotate_wz = float(cfg.get("blocked_rotate_wz", cfg.get("scan_wz", 0.24)))
        self.blocked_rotate_hold_sec = float(cfg.get("blocked_rotate_hold_sec", 1.2))
        self.blocked_rotate_min_switch_sec = float(cfg.get("blocked_rotate_min_switch_sec", 1.2))
        self.blocked_rotate_alternate = bool(cfg.get("blocked_rotate_alternate", True))
        self.blocked_rotate_until = 0.0
        self.blocked_rotate_dir = 1.0
        self.last_blocked_switch_time = 0.0

        self.center_arrive_px = float(cfg.get("center_arrive_px", 80))
        self.arrive_required_count = int(cfg.get("arrive_required_count", 3))
        self.min_forward_bursts_before_arrive = int(cfg.get("min_forward_bursts_before_arrive", 2))
        self.max_steps = int(cfg.get("max_steps", 600))
        self.min_state_frames = int(cfg.get("min_state_frames", 10))
        self.bad_wp_grace_frames = int(cfg.get("bad_wp_grace_frames", 3))
        self.no_target_grace_frames = int(cfg.get("no_target_grace_frames", 3))
        self.bad_wp_limit = int(cfg.get("bad_wp_limit", 3))

        self.target_reacquire_wz = float(cfg.get("target_reacquire_wz", 0.18))
        self.target_reacquire_hold_sec = float(cfg.get("target_reacquire_hold_sec", 0.6))

        self.last_scan_time = 0.0
        self.last_valid_front_distance: Optional[float] = None
        self.last_valid_front_time = 0.0
        self.max_scan_age_sec = float(cfg.get("max_scan_age_sec", 0.8))
        self.front_distance_hold_sec = float(cfg.get("front_distance_hold_sec", 0.5))
        self.scan_stale_nav_grace_sec = float(cfg.get("scan_stale_nav_grace_sec", 1.2))

        self.active_cmd = Twist()
        self.active_cmd_until = 0.0
        self.active_cmd_reason = "init"
        self.last_nonzero_cmd_time = 0.0
        self.last_decision_time = 0.0
        self.bad_wp_count = 0
        self.no_target_count = 0

        self.control_rate_hz = float(cfg.get("control_rate_hz", 20.0))
        self.cmd_hold_sec = float(cfg.get("cmd_hold_sec", 0.75))
        self.stop_on_cmd_expire = bool(cfg.get("stop_on_cmd_expire", True))
        self.cmd_publish_immediate = bool(cfg.get("cmd_publish_immediate", True))
        self.min_drive_vx = float(cfg.get("min_drive_vx", 0.055))
        self.min_turn_wz = float(cfg.get("min_turn_wz", 0.18))
        self.wz_deadband = float(cfg.get("wz_deadband", 0.02))
        self.vx_deadband = float(cfg.get("vx_deadband", 0.01))

        self.fsm_mode = "INIT"
        self.fsm_mode_frames = 0
        self._last_blocked_reason = "no_free_space"

        fs_cfg = FreeSpaceConfig(
            lidar_min_range=float(cfg.get("lidar_min_range", 0.08)),
            lidar_max_range=float(cfg.get("lidar_max_range", 6.0)),
            lidar_front_deg=float(cfg.get("lidar_front_deg", 18.0)),
            camera_hfov_deg=float(cfg.get("camera_hfov_deg", 70.0)),
            camera_lidar_yaw_offset_deg=float(cfg.get("camera_lidar_yaw_offset_deg", 0.0)),
            sector_deg=float(cfg.get("free_space_sector_deg", 70.0)),
            step_deg=float(cfg.get("free_space_step_deg", 5.0)),
            window_deg=float(cfg.get("free_space_window_deg", 10.0)),
            min_clearance=float(cfg.get("free_space_min_clearance", 0.45)),
            good_clearance=float(cfg.get("free_space_good_clearance", 1.20)),
            smooth_alpha=float(cfg.get("free_space_smooth_alpha", 0.35)),
            waypoint_v_ratio=float(cfg.get("free_space_waypoint_v_ratio", 0.62)),
            min_u_ratio=float(cfg.get("free_space_min_u_ratio", 0.20)),
            max_u_ratio=float(cfg.get("free_space_max_u_ratio", 0.80)),
            clearance_weight=float(cfg.get("free_space_clearance_weight", 0.60)),
            center_weight=float(cfg.get("free_space_center_weight", 0.30)),
            consistency_weight=float(cfg.get("free_space_consistency_weight", 0.10)),
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
        self.timer = self.create_timer(1.0 / max(decision_rate, 1e-3), self.decision_timer_cb)
        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1e-3),
            self.control_timer_cb,
        )

        if bool(cfg.get("publish_target_words", True)):
            period = float(cfg.get("target_words_publish_period_sec", 3.0))
            self.words_timer = self.create_timer(max(period, 0.5), self.publish_target_words)

        self.get_logger().info("===== YOLO + LiDAR FAILSAFE NAV START =====")
        self.get_logger().info(f"instruction={self.instruction}")
        self.get_logger().info(
            f"topics image={self.image_topic} scan={self.scan_topic} "
            f"bbox={self.target_bbox_topic} cmd={self.cmd_topic}"
        )
        self.get_logger().info(
            f"control={self.control_rate_hz}Hz decision={decision_rate}Hz "
            f"cmd_hold={self.cmd_hold_sec}s min_vx={self.min_drive_vx} min_wz={self.min_turn_wz}"
        )
        self.get_logger().info(
            f"min_state_frames={self.min_state_frames} emergency_stop={self.emergency_stop_distance}m"
        )
        self.get_logger().info("Qwen/Ollama is NOT used in this P0 control loop.")

    def _is_emergency_front(self, front: Optional[float]) -> bool:
        return front is not None and front <= self.emergency_stop_distance

    def has_fresh_scan(self) -> bool:
        return self.free_space.has_scan() and (time.time() - self.last_scan_time <= self.max_scan_age_sec)

    def can_navigate(self) -> bool:
        """Fresh scan, or recent front distance still trustworthy enough to keep moving."""
        if self.has_fresh_scan():
            return True
        if self.last_valid_front_distance is None:
            return False
        return (time.time() - self.last_valid_front_time) <= self.scan_stale_nav_grace_sec

    def get_effective_front_distance(self) -> Optional[float]:
        now = time.time()
        if self.latest_front_distance is not None:
            return self.latest_front_distance
        if (
            self.last_valid_front_distance is not None
            and now - self.last_valid_front_time <= self.front_distance_hold_sec
        ):
            return self.last_valid_front_distance
        return None

    def apply_motion_deadband(self, cmd: Twist) -> Twist:
        out = Twist()
        vx = float(cmd.linear.x)
        wz = float(cmd.angular.z)

        # 严格小于 deadband 才置零；等于 deadband 仍视为有运动意图
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

    def set_active_cmd(self, cmd: Twist, hold_sec: float, reason: str = "") -> None:
        cmd = self.apply_motion_deadband(cmd)
        self.active_cmd = cmd
        self.active_cmd_until = time.time() + max(hold_sec, 0.05)
        self.active_cmd_reason = reason
        if abs(cmd.linear.x) > 1e-6 or abs(cmd.angular.z) > 1e-6:
            self.last_nonzero_cmd_time = time.time()

        if not self.cmd_publish_immediate:
            return

        front = self.get_effective_front_distance()
        if front is not None and front <= self.emergency_stop_distance:
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.last_cmd = stop
            return

        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

    def control_timer_cb(self) -> None:
        now = time.time()
        cmd = Twist()

        front = self.get_effective_front_distance()
        if front is not None and front <= self.emergency_stop_distance:
            self.cmd_pub.publish(cmd)
            self.last_cmd = cmd
            return

        if now <= self.active_cmd_until:
            cmd = self.active_cmd
        elif self.stop_on_cmd_expire:
            cmd = Twist()
        else:
            cmd = self.active_cmd

        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

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
        now = time.time()
        self.last_scan_time = now
        self.free_space.update_scan(msg)
        front = self.free_space.front_distance()
        if front is not None:
            self.latest_front_distance = front
            self.last_valid_front_distance = front
            self.last_valid_front_time = now
        elif (
            self.last_valid_front_distance is not None
            and now - self.last_valid_front_time <= self.front_distance_hold_sec
        ):
            self.latest_front_distance = self.last_valid_front_distance
        else:
            self.latest_front_distance = None

    def bbox_cb(self, msg: String) -> None:
        self.target_parser.update_json(msg.data, self.image_width, self.image_height)

    def publish_target_words(self) -> None:
        words = list(self.cfg.get("target_words", []))
        self.words_pub.publish(String(data=",".join(words)))

    def decision_timer_cb(self) -> None:
        self.last_decision_time = time.time()

        if self.success:
            self.fsm_mode = "ARRIVED"
            self.publish_stop()
            self.publish_state("ARRIVED", reason="success_hold")
            return

        self.step_count += 1
        if self.step_count > self.max_steps:
            self.fsm_mode = "STOPPED"
            self.publish_stop()
            self.publish_state("STOPPED", reason="max_steps")
            return

        front = self.get_effective_front_distance()

        if self._is_emergency_front(front):
            if self.fsm_mode == "EMERGENCY_STOP":
                self.fsm_mode_frames += 1
            else:
                self._resolve_effective_mode("EMERGENCY_STOP", immediate=True)
            self.publish_stop()
            self.publish_state(
                "EMERGENCY_STOP",
                front_distance=front,
                reason="emergency_front_too_close",
            )
            return

        if not self.can_navigate():
            desired_mode = "WAIT_SCAN"
        else:
            target = self.target_parser.get_target(time.time(), self.image_width, self.image_height)
            if target.get("visible", False):
                if self.check_arrive(target, front):
                    desired_mode = "ARRIVED"
                else:
                    desired_mode = "TARGET_TRACK"
            else:
                wp = self.free_space.get_waypoint(self.image_width, self.image_height)
                if not wp.get("usable", False):
                    desired_mode = "BLOCKED_ROTATE"
                    self._last_blocked_reason = str(wp.get("reason", "no_free_space"))
                else:
                    desired_mode = "FREE_SPACE_EXPLORE"

        immediate = desired_mode in ("WAIT_SCAN", "STOPPED")
        effective_mode = self._resolve_effective_mode(desired_mode, immediate=immediate)

        if effective_mode == "WAIT_SCAN":
            self.publish_stop()
            stale = not self.has_fresh_scan()
            self.publish_state(
                "WAIT_SCAN",
                reason="stale_scan" if stale else "no_scan",
                fsm_frames=self.fsm_mode_frames,
            )
            return

        if not self.can_navigate():
            self.publish_stop()
            self.publish_state("WAIT_SCAN", reason="nav_grace_expired", fsm_frames=self.fsm_mode_frames)
            return

        target = self.target_parser.get_target(time.time(), self.image_width, self.image_height)
        front = self.get_effective_front_distance()

        if effective_mode == "ARRIVED":
            self.success = True
            self.publish_stop()
            self.publish_state(
                "ARRIVED",
                target=target,
                front_distance=front,
                reason="target_center_and_close",
                fsm_frames=self.fsm_mode_frames,
            )
            return

        if effective_mode == "TARGET_TRACK":
            if not target.get("visible", False):
                self.no_target_count += 1
                if self.no_target_count <= self.no_target_grace_frames:
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    last_u = target.get("u", self.image_width / 2.0)
                    ex = (float(last_u) - self.image_width / 2.0) / max(float(self.image_width), 1.0)
                    direction = -1.0 if ex > 0 else 1.0
                    cmd.angular.z = direction * self.target_reacquire_wz
                    self.set_active_cmd(cmd, self.target_reacquire_hold_sec, reason="target_reacquire")
                    self.publish_state(
                        "TARGET_REACQUIRE",
                        target=target,
                        front_distance=front,
                        cmd=cmd,
                        reason="target_lost_reacquire",
                        fsm_frames=self.fsm_mode_frames,
                    )
                    return
                self.publish_stop()
                self.publish_state(
                    "TARGET_TRACK",
                    target=target,
                    front_distance=front,
                    reason="state_hold_no_target",
                    fsm_frames=self.fsm_mode_frames,
                )
                return
            self.no_target_count = 0
            self.handle_target_track(
                target,
                front,
                allow_arrive=(desired_mode == "ARRIVED" and self.fsm_mode_frames >= self.min_state_frames),
            )
            return

        if effective_mode == "FREE_SPACE_EXPLORE":
            target_reason = target.get("reason", "target_not_visible")
            wp = self.free_space.get_waypoint(self.image_width, self.image_height)
            if not wp.get("usable", False):
                self.bad_wp_count += 1
                if self.bad_wp_count < self.bad_wp_limit:
                    self.publish_state(
                        "FREE_SPACE_EXPLORE",
                        front_distance=front,
                        reason=f"bad_wp_grace_{self.bad_wp_count}; target={target_reason}",
                        fsm_frames=self.fsm_mode_frames,
                    )
                    return
                self.do_blocked_rotate(wp.get("reason", "bad_wp_limit"))
                return
            self.bad_wp_count = 0
            self.handle_free_space_explore(front, target_reason)
            return

        if effective_mode == "BLOCKED_ROTATE":
            self.do_blocked_rotate(self._last_blocked_reason)
            return

        if effective_mode == "EMERGENCY_STOP":
            self.publish_stop()
            self.publish_state(
                "EMERGENCY_STOP",
                front_distance=front,
                reason="emergency_front_too_close",
                fsm_frames=self.fsm_mode_frames,
            )
            return

        self.publish_stop()
        self.publish_state("UNKNOWN", reason=f"unhandled_mode={effective_mode}")

    def handle_target_track(
        self,
        target: Dict[str, Any],
        front: Optional[float],
        allow_arrive: bool = True,
    ) -> None:
        cmd, servo_state = self.compute_target_cmd(target, front)

        if allow_arrive and self.check_arrive(target, front):
            self.success = True
            self.fsm_mode = "ARRIVED"
            self.publish_stop()
            self.publish_state(
                "ARRIVED",
                target=target,
                front_distance=front,
                reason="target_center_and_close",
                fsm_frames=self.fsm_mode_frames,
            )
            return

        raw_vx = float(cmd.linear.x)
        raw_wz = float(cmd.angular.z)
        self.set_active_cmd(cmd, self.cmd_hold_sec, reason="target_track")
        self.publish_point(target.get("u"), target.get("v"), "target", "TARGET_TRACK")
        applied = self.active_cmd
        self.publish_state(
            "TARGET_TRACK",
            target=target,
            front_distance=front,
            cmd=applied,
            cmd_raw_vx=raw_vx,
            cmd_raw_wz=raw_wz,
            reason=servo_state,
            fsm_frames=self.fsm_mode_frames,
            desired_hold=(self.fsm_mode_frames < self.min_state_frames),
            active_cmd_reason=self.active_cmd_reason,
        )

    def compute_target_cmd(self, target: Dict[str, Any], front: Optional[float]):
        cmd = Twist()
        u = float(target.get("u", self.image_width / 2.0))
        ex = (u - self.image_width / 2.0) / max(float(self.image_width), 1.0)

        kp = float(self.cfg.get("target_kp_turn", 0.22))
        max_wz = float(self.cfg.get("target_max_wz", 0.14))
        turn_threshold = float(self.cfg.get("target_turn_threshold", 0.20))
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

    def handle_free_space_explore(self, front: Optional[float], target_reason: str) -> None:
        wp = self.free_space.get_waypoint(self.image_width, self.image_height)

        if not wp.get("usable", False):
            self.do_blocked_rotate(wp.get("reason", "no_free_space"))
            return

        cmd, mode = self.compute_explore_cmd(wp, front)
        self.set_active_cmd(cmd, self.cmd_hold_sec, reason="free_space_explore")
        applied = self.active_cmd
        self.publish_point(wp.get("u"), wp.get("v"), "free_space", "FREE_SPACE_EXPLORE")
        self.publish_state(
            "FREE_SPACE_EXPLORE",
            waypoint=wp,
            front_distance=front,
            heading_deg=wp.get("heading_deg"),
            clearance=wp.get("clearance"),
            cmd=applied,
            cmd_raw_vx=float(cmd.linear.x),
            cmd_raw_wz=float(cmd.angular.z),
            reason=f"{mode}; target={target_reason}",
            fsm_frames=self.fsm_mode_frames,
            desired_hold=(self.fsm_mode_frames < self.min_state_frames),
            active_cmd_reason=self.active_cmd_reason,
        )

    def compute_explore_cmd(self, wp: Dict[str, Any], front: Optional[float]):
        cmd = Twist()

        if front is not None and front <= self.hard_stop_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.blocked_rotate_dir * abs(self.blocked_rotate_wz)
            return cmd, "blocked_rotate"

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

    def do_blocked_rotate(self, reason: str) -> None:
        now = time.time()
        cmd = Twist()
        cmd.linear.x = 0.0

        if now >= self.blocked_rotate_until:
            if (
                self.blocked_rotate_alternate
                and now - self.last_blocked_switch_time >= self.blocked_rotate_min_switch_sec
            ):
                self.blocked_rotate_dir *= -1.0
                self.last_blocked_switch_time = now
            self.blocked_rotate_until = now + self.blocked_rotate_hold_sec

        cmd.angular.z = self.blocked_rotate_dir * abs(self.blocked_rotate_wz)
        self.set_active_cmd(cmd, self.blocked_rotate_hold_sec, reason="blocked_rotate")

        self.publish_state(
            "BLOCKED_ROTATE",
            front_distance=self.get_effective_front_distance(),
            cmd=cmd,
            reason=reason,
            fsm_frames=self.fsm_mode_frames,
            desired_hold=(self.fsm_mode_frames < self.min_state_frames),
            blocked_rotate_dir=self.blocked_rotate_dir,
            blocked_rotate_until=self.blocked_rotate_until,
        )

    def check_arrive(self, target: Dict[str, Any], front: Optional[float]) -> bool:
        if front is None:
            return False
        if front > self.arrive_distance:
            self.arrive_count = 0
            return False

        u = float(target.get("u", self.image_width / 2.0))
        center_err_px = abs(u - self.image_width / 2.0)
        if center_err_px > self.center_arrive_px:
            self.arrive_count = 0
            return False

        if self.forward_burst_count < self.min_forward_bursts_before_arrive:
            return False

        self.arrive_count += 1
        return self.arrive_count >= self.arrive_required_count

    def publish_stop(self) -> None:
        self.active_cmd = Twist()
        self.active_cmd_until = 0.0
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

    def publish_state(self, mode: str, **kwargs: Any) -> None:
        cmd = kwargs.pop("cmd", None)
        data = {
            "step": self.step_count,
            "mode": mode,
            "fsm_mode": self.fsm_mode,
            "fsm_frames": self.fsm_mode_frames,
            "min_state_frames": self.min_state_frames,
            "instruction": self.instruction,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "front_distance": kwargs.pop("front_distance", self.get_effective_front_distance()),
            "cmd_vx": float(cmd.linear.x) if cmd is not None else float(self.last_cmd.linear.x),
            "cmd_wz": float(cmd.angular.z) if cmd is not None else float(self.last_cmd.angular.z),
            "active_cmd_reason": self.active_cmd_reason,
            "time": time.time(),
        }
        data.update(_json_safe(kwargs))
        payload = json.dumps(data, ensure_ascii=False)
        self.state_pub.publish(String(data=payload))
        self.get_logger().info(payload)


def main():
    parser = argparse.ArgumentParser(description="YOLO + LiDAR failsafe navigation (P0)")
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
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
