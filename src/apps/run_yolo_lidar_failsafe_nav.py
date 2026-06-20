#!/usr/bin/env python3
"""
YOLO-World + LiDAR failsafe navigation (P0).

/image_raw + /scan + /target_bbox_json -> visual servo / free-space explore -> /cmd_vel.
Does NOT use Qwen/Ollama.
"""
import argparse
import json
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
        self.scan_direction = 1.0
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
        self.blocked_rotate_alternate = bool(cfg.get("blocked_rotate_alternate", True))

        self.center_arrive_px = float(cfg.get("center_arrive_px", 80))
        self.arrive_required_count = int(cfg.get("arrive_required_count", 3))
        self.min_forward_bursts_before_arrive = int(cfg.get("min_forward_bursts_before_arrive", 2))
        self.max_steps = int(cfg.get("max_steps", 600))

        self.target_burst_sec = float(cfg.get("target_burst_sec", 0.25))
        self.target_observe_sec = float(cfg.get("target_observe_sec", 0.20))
        self.explore_burst_sec = float(cfg.get("explore_burst_sec", 0.30))
        self.explore_observe_sec = float(cfg.get("explore_observe_sec", 0.10))
        self.scan_burst_sec = float(cfg.get("scan_burst_sec", 0.25))
        self.scan_observe_sec = float(cfg.get("scan_observe_sec", 0.20))

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

        if bool(cfg.get("publish_target_words", True)):
            period = float(cfg.get("target_words_publish_period_sec", 3.0))
            self.words_timer = self.create_timer(max(period, 0.5), self.publish_target_words)

        self.get_logger().info("===== YOLO + LiDAR FAILSAFE NAV START =====")
        self.get_logger().info(f"instruction={self.instruction}")
        self.get_logger().info(
            f"topics image={self.image_topic} scan={self.scan_topic} "
            f"bbox={self.target_bbox_topic} cmd={self.cmd_topic}"
        )
        self.get_logger().info("Qwen/Ollama is NOT used in this P0 control loop.")

    def image_cb(self, msg: Image) -> None:
        self.last_image_time = time.time()
        if msg.width and msg.height:
            self.image_width = int(msg.width)
            self.image_height = int(msg.height)

    def scan_cb(self, msg: LaserScan) -> None:
        self.free_space.update_scan(msg)
        self.latest_front_distance = self.free_space.front_distance()

    def bbox_cb(self, msg: String) -> None:
        self.target_parser.update_json(msg.data, self.image_width, self.image_height)

    def publish_target_words(self) -> None:
        words = list(self.cfg.get("target_words", []))
        self.words_pub.publish(String(data=",".join(words)))

    def decision_timer_cb(self) -> None:
        if self.success:
            self.publish_stop()
            self.publish_state("ARRIVED", reason="success_hold")
            return

        self.step_count += 1
        if self.step_count > self.max_steps:
            self.publish_stop()
            self.publish_state("STOPPED", reason="max_steps")
            return

        if not self.free_space.has_scan():
            self.publish_stop()
            self.publish_state("WAIT_SCAN", reason="no_scan")
            return

        front = self.latest_front_distance
        if front is not None and front <= self.emergency_stop_distance:
            self.do_blocked_rotate("emergency_front_too_close")
            return

        target = self.target_parser.get_target(time.time(), self.image_width, self.image_height)

        if target.get("visible", False):
            self.handle_target_track(target, front)
        else:
            self.handle_free_space_explore(front, target.get("reason", "target_not_visible"))

    def handle_target_track(self, target: Dict[str, Any], front: Optional[float]) -> None:
        cmd, servo_state = self.compute_target_cmd(target, front)

        if self.check_arrive(target, front):
            self.success = True
            self.publish_stop()
            self.publish_state(
                "ARRIVED",
                target=target,
                front_distance=front,
                reason="target_center_and_close",
            )
            return

        self.publish_cmd_burst(cmd, self.target_burst_sec, self.target_observe_sec)
        self.publish_point(target.get("u"), target.get("v"), "target", "TARGET_TRACK")
        self.publish_state(
            "TARGET_TRACK",
            target=target,
            front_distance=front,
            cmd=cmd,
            reason=servo_state,
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
        self.publish_cmd_burst(cmd, self.explore_burst_sec, self.explore_observe_sec)
        self.publish_point(wp.get("u"), wp.get("v"), "free_space", "FREE_SPACE_EXPLORE")
        self.publish_state(
            "FREE_SPACE_EXPLORE",
            waypoint=wp,
            front_distance=front,
            heading_deg=wp.get("heading_deg"),
            clearance=wp.get("clearance"),
            cmd=cmd,
            reason=f"{mode}; target={target_reason}",
        )

    def compute_explore_cmd(self, wp: Dict[str, Any], front: Optional[float]):
        cmd = Twist()

        if front is not None and front <= self.hard_stop_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.scan_direction * abs(self.scan_wz)
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
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.scan_direction * abs(self.scan_wz)

        self.publish_cmd_burst(cmd, self.scan_burst_sec, self.scan_observe_sec)

        if self.blocked_rotate_alternate:
            self.scan_direction *= -1.0

        self.publish_state(
            "BLOCKED_ROTATE",
            front_distance=self.latest_front_distance,
            cmd=cmd,
            reason=reason,
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

    def publish_cmd_burst(self, cmd: Twist, burst_sec: float, observe_sec: float) -> None:
        front = self.latest_front_distance
        if front is not None and front <= self.emergency_stop_distance:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

        time.sleep(max(0.0, burst_sec))
        self.publish_stop()
        time.sleep(max(0.0, observe_sec))

    def publish_stop(self) -> None:
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
            "instruction": self.instruction,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "front_distance": kwargs.pop("front_distance", self.latest_front_distance),
            "cmd_vx": float(cmd.linear.x) if cmd is not None else 0.0,
            "cmd_wz": float(cmd.angular.z) if cmd is not None else 0.0,
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
