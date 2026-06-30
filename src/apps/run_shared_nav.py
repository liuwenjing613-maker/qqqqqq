#!/usr/bin/env python3
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
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.control.point_servo import PointServo, PointServoConfig, ServoCommand, clamp
from src.fsm.nav_state_machine import NavFSMConfig, NavObservation, NavState, NavStateMachine
from src.perception.free_space_waypoint import FreeSpaceConfig, FreeSpaceWaypointProvider
from src.perception.target_adapter import NavTarget, TargetAdapter


DEFAULT_CONFIG = str(ROOT / "configs" / "nav_yolo_lidar.yaml")


def load_yaml(path: str) -> Dict[str, Any]:
    with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    if not isinstance(cfg, dict):
        raise RuntimeError(f"Config file did not parse into a dict: {path}")
    return cfg


def section(cfg: Dict[str, Any], key: str) -> Dict[str, Any]:
    value = cfg.get(key, {})
    return value if isinstance(value, dict) else {}


def topic(cfg: Dict[str, Any], key: str, flat_key: str, default: str) -> str:
    return str(section(cfg, "topics").get(key, cfg.get(flat_key, default)))


def json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return str(value)


class SharedNav(Node):
    def __init__(self, cfg: Dict[str, Any], instruction: str):
        super().__init__("shared_nav")
        self.cfg = cfg
        self.mode = str(cfg.get("mode", "yolo_lidar_nav"))
        self.instruction = instruction or str(cfg.get("instruction", "find the target"))

        camera = section(cfg, "camera")
        target_cfg = section(cfg, "target")
        freshness = section(cfg, "freshness")
        rates = section(cfg, "rates")
        safety = section(cfg, "safety")
        arrive = section(cfg, "arrive")
        search = section(cfg, "search")

        self.image_width = int(camera.get("width", cfg.get("image_width", cfg.get("camera_width", 640))))
        self.image_height = int(camera.get("height", cfg.get("image_height", cfg.get("camera_height", 480))))
        self.image_stale_sec = float(freshness.get("image_stale_sec", 0.40))
        self.scan_stale_sec = float(freshness.get("scan_stale_sec", 0.30))
        self.bbox_stale_sec = float(freshness.get("bbox_stale_sec", 0.45))

        self.require_lidar = bool(section(cfg, "fsm").get("require_lidar", safety.get("require_lidar", False)))
        self.target_source = str(target_cfg.get("source", "color" if self.mode == "color_nav" else "yolo_bbox"))
        self.target_color = str(target_cfg.get("color", "red"))
        self.target_min_score = float(target_cfg.get("min_score", 0.0))
        self.arrive_center_px = float(arrive.get("center_px", 70))

        fsm_cfg = section(cfg, "fsm")
        self.fsm = NavStateMachine(
            NavFSMConfig(
                stable_frames_required=int(fsm_cfg.get("stable_frames_required", 3)),
                lost_frames_limit=int(fsm_cfg.get("lost_frames_limit", 5)),
                arrive_required_frames=int(fsm_cfg.get("arrive_required_frames", arrive.get("arrive_required_frames", 4))),
                centered_required_frames=int(fsm_cfg.get("centered_required_frames", 3)),
                max_search_sec=float(fsm_cfg.get("max_search_sec", 30.0)),
                max_task_sec=float(fsm_cfg.get("max_task_sec", 180.0)),
                min_state_frames=int(fsm_cfg.get("min_state_frames", 2)),
                qwen_verify_required=bool(fsm_cfg.get("qwen_verify_required", False)),
                qwen_verify_timeout_sec=float(fsm_cfg.get("qwen_verify_timeout_sec", section(cfg, "qwen").get("timeout_sec", 12.0))),
                qwen_verify_fail_policy=str(fsm_cfg.get("qwen_verify_fail_policy", "search")),
                recovery_max_sec=float(fsm_cfg.get("recovery_max_sec", 4.0)),
                arrive_min_distance=float(arrive.get("arrive_min_distance", 0.55)),
                arrive_max_distance=float(arrive.get("arrive_max_distance", 0.75)),
                arrive_area_ratio=float(arrive.get("arrive_area_ratio", 0.16)),
                center_only_arrive_enabled=bool(arrive.get("center_only_arrive_enabled", False)),
            )
        )

        servo_cfg = section(cfg, "servo")
        self.servo = PointServo(
            PointServoConfig(
                image_width=self.image_width,
                image_height=self.image_height,
                max_vx=float(servo_cfg.get("max_vx", 0.06)),
                steer_vx=float(servo_cfg.get("steer_vx", 0.04)),
                max_wz=float(servo_cfg.get("max_wz", 0.06)),
                kp_turn=float(servo_cfg.get("kp_turn", 0.12)),
                center_deadband=float(servo_cfg.get("center_deadband", 0.06)),
                turn_only_threshold=float(servo_cfg.get("turn_only_threshold", 0.20)),
                cmd_wz_deadband=float(servo_cfg.get("cmd_wz_deadband", 0.006)),
            )
        )

        words = target_cfg.get("words", cfg.get("target_words", []))
        self.target_adapter = TargetAdapter(
            image_width=self.image_width,
            image_height=self.image_height,
            target_words=list(words or []),
            min_score=self.target_min_score,
            min_area_ratio=float(target_cfg.get("min_area_ratio", 0.0)),
            max_area_ratio=float(target_cfg.get("max_area_ratio", 1.0)),
            accept_unknown_class=bool(target_cfg.get("accept_unknown_class", True)),
            bbox_stale_sec=self.bbox_stale_sec,
        )

        self.free_space = FreeSpaceWaypointProvider(
            FreeSpaceConfig(
                lidar_min_range=float(cfg.get("lidar_min_range", 0.08)),
                lidar_max_range=float(cfg.get("lidar_max_range", 6.0)),
                lidar_front_deg=float(cfg.get("lidar_front_deg", 18.0)),
                camera_hfov_deg=float(cfg.get("camera_hfov_deg", 70.0)),
                camera_lidar_yaw_offset_deg=float(cfg.get("camera_lidar_yaw_offset_deg", 0.0)),
                min_clearance=float(search.get("free_space_min_clearance", 1.0)),
            )
        )

        self.scan_wz = float(search.get("scan_wz", 0.04))
        self.pulse_sec = float(search.get("pulse_sec", 0.20))
        self.observe_sec = float(search.get("observe_sec", 0.60))
        self.free_space_enabled = bool(search.get("free_space_enabled", False))
        self.free_space_enable_after_sec = float(search.get("free_space_enable_after_sec", 10.0))
        self.free_space_vx = float(search.get("free_space_vx", 0.015))

        self.emergency_stop_distance = float(safety.get("emergency_stop_distance", 0.45))
        self.hard_stop_distance = float(safety.get("hard_stop_distance", 0.55))
        self.slow_distance = float(safety.get("slow_distance", 0.90))
        self.max_cmd_vx = float(safety.get("max_cmd_vx", 0.06))
        self.max_cmd_wz = float(safety.get("max_cmd_wz", 0.06))
        self.turn_zero_vx_wz = float(safety.get("turn_zero_vx_wz", 0.05))
        self.turn_slow_vx_wz = float(safety.get("turn_slow_vx_wz", 0.035))
        self.turn_slow_vx_scale = float(safety.get("turn_slow_vx_scale", 0.5))

        self.bridge = CvBridge()
        self.last_frame = None
        self.last_image_time = 0.0
        self.last_scan_time = 0.0
        self.last_target = NavTarget(False, None, None, reason="init")
        self.last_fsm_result = None
        self.desired_cmd = Twist()
        self.last_cmd = Twist()
        self.last_safety: Dict[str, Any] = {}
        self.desired_reason = "init"
        self.step_count = 0
        self.qwen_verified: Optional[bool] = None

        self.image_topic = topic(cfg, "image_raw", "image_topic", "/image_raw")
        self.scan_topic = topic(cfg, "scan", "scan_topic", "/scan")
        self.cmd_topic = topic(cfg, "cmd_vel", "cmd_topic", "/cmd_vel")
        self.bbox_topic = topic(cfg, "target_bbox_json", "target_bbox_topic", "/target_bbox_json")
        self.words_topic = topic(cfg, "target_words", "target_words_topic", "/target_words")
        self.state_topic = topic(cfg, "nav_state", "state_topic", "/nav_state")
        self.point_topic = topic(cfg, "nav_target_point", "point_topic", "/nav_target_point")

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.point_pub = self.create_publisher(String, self.point_topic, 10)
        self.words_pub = self.create_publisher(String, self.words_topic, 10)

        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        if self.require_lidar:
            self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        if self.target_source == "yolo_bbox":
            self.create_subscription(String, self.bbox_topic, self.bbox_cb, 10)

        decision_hz = float(rates.get("decision_hz", 10.0))
        control_hz = float(rates.get("control_hz", 20.0))
        state_pub_hz = float(rates.get("state_pub_hz", 5.0))
        self._state_publish_div = max(1, int(round(control_hz / max(state_pub_hz, 1e-3))))
        self._control_tick = 0
        self.create_timer(1.0 / max(decision_hz, 1e-3), self.decision_timer_cb)
        self.create_timer(1.0 / max(control_hz, 1e-3), self.control_timer_cb)
        self.create_timer(3.0, self.publish_target_words)

        self.get_logger().info(f"===== shared_nav mode={self.mode} =====")
        self.get_logger().info(f"topics image={self.image_topic} scan={self.scan_topic} cmd={self.cmd_topic}")

    def image_cb(self, msg: Image) -> None:
        self.last_image_time = time.time()
        if msg.width and msg.height:
            self.image_width = int(msg.width)
            self.image_height = int(msg.height)
            self.servo.cfg.image_width = self.image_width
            self.servo.cfg.image_height = self.image_height
            self.target_adapter.update_image_geometry(self.image_width, self.image_height)
        if self.target_source == "color":
            try:
                self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as exc:
                self.get_logger().warn(f"cv_bridge image failed: {repr(exc)}")

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan_time = time.time()
        self.free_space.update_scan(msg)

    def bbox_cb(self, msg: String) -> None:
        self.last_target = self.target_adapter.update_yolo_bbox_json(msg.data)

    def publish_target_words(self) -> None:
        target_cfg = section(self.cfg, "target")
        words = target_cfg.get("words", self.cfg.get("target_words", []))
        if words:
            self.words_pub.publish(String(data=",".join(str(w) for w in words)))

    def decision_timer_cb(self) -> None:
        now = time.time()
        self.step_count += 1
        target = self.resolve_target(now)
        front = self.front_distance()
        obs = self.make_observation(now, target, front)
        result = self.fsm.update(obs)
        self.last_fsm_result = result
        self.last_target = target

        cmd, reason = self.command_for_state(result.state, target, now)
        self.desired_cmd = self.to_twist(cmd)
        self.desired_reason = reason
        self.publish_point(target, result.state.value)
        self.publish_state(result.state.value, reason=reason, target=target.to_dict(), from_control=False)

    def control_timer_cb(self) -> None:
        safe_cmd, safety = self.apply_safety_layer(self.desired_cmd)
        self.last_safety = safety
        self.last_cmd = safe_cmd
        self.cmd_pub.publish(safe_cmd)
        self._control_tick += 1
        if self._control_tick % self._state_publish_div == 0:
            state = self.fsm.state.value
            if safety.get("control_mode"):
                state = str(safety["control_mode"])
            self.publish_state(state, reason=self.desired_reason, from_control=True)

    def resolve_target(self, now: float) -> NavTarget:
        if self.target_source == "color":
            if self.last_frame is None:
                return NavTarget(False, None, None, source="color", reason="no_frame", stamp_time=now)
            return self.target_adapter.from_color(self.last_frame, self.target_color)
        return self.target_adapter.current_yolo_target(now)

    def make_observation(self, now: float, target: NavTarget, front: Optional[float]) -> NavObservation:
        image_fresh = self.last_image_time > 0 and now - self.last_image_time <= self.image_stale_sec
        scan_fresh = (not self.require_lidar) or (
            self.last_scan_time > 0 and now - self.last_scan_time <= self.scan_stale_sec
        )
        target_centered = False
        if target.u is not None:
            target_centered = abs(float(target.u) - self.image_width / 2.0) <= self.arrive_center_px
        emergency = bool(front is not None and front <= self.emergency_stop_distance)
        blocked = bool(front is not None and front <= self.hard_stop_distance)
        score_ok = bool(target.visible and not target.stale and target.score >= self.target_min_score)
        return NavObservation(
            now=now,
            image_fresh=image_fresh,
            scan_fresh=scan_fresh,
            require_lidar=self.require_lidar,
            target_visible=target.visible,
            target_stale=target.stale,
            target_score=target.score,
            target_score_ok=score_ok,
            target_u=target.u,
            target_v=target.v,
            target_centered=target_centered,
            target_area_ratio=target.area_ratio,
            front_distance=front,
            emergency=emergency,
            blocked=blocked,
            qwen_verified=self.mock_qwen_verify(now),
        )

    def mock_qwen_verify(self, now: float) -> Optional[bool]:
        if self.mode != "qwen_yolo_nav" or self.fsm.state != NavState.ARRIVE_VERIFY:
            return None
        elapsed = now - (self.fsm.state_enter_time or now)
        return True if elapsed >= 0.5 else None

    def command_for_state(self, state: NavState, target: NavTarget, now: float) -> Tuple[ServoCommand, str]:
        if state in (NavState.BOOT, NavState.WAIT_SENSORS, NavState.CANDIDATE_LOCK, NavState.ARRIVE_VERIFY, NavState.SUCCESS, NavState.FAILED):
            return ServoCommand(), f"{state.value.lower()}_stop"
        if state == NavState.TRACK:
            result = self.servo.compute_cmd(target.to_dict())
            return result.cmd, result.state
        if state == NavState.SEARCH:
            if self.free_space_enabled and self.fsm.state_enter_time is not None:
                if now - self.fsm.state_enter_time >= self.free_space_enable_after_sec:
                    wp = self.free_space.get_waypoint(self.image_width, self.image_height)
                    if wp.get("usable", False):
                        cmd = self.servo.compute_cmd({"visible": True, "u": wp.get("u"), "v": wp.get("v")}).cmd
                        cmd.vx = min(cmd.vx, self.free_space_vx)
                        return cmd, "free_space_search"
            return ServoCommand(vx=0.0, wz=self.scan_wz), "search_scan"
        if state == NavState.LOST_RECOVERY:
            return self.recovery_pulse_cmd(now)
        if state == NavState.BLOCKED:
            if self.last_safety.get("safety_reason") == "emergency_stop":
                return ServoCommand(), "blocked_emergency_stop"
            turn_dir, side = self.pick_clearance_turn()
            return ServoCommand(vx=0.0, wz=turn_dir * abs(self.scan_wz)), f"blocked_turn_{side}"
        return ServoCommand(), "unhandled_stop"

    def recovery_pulse_cmd(self, now: float) -> Tuple[ServoCommand, str]:
        start = self.fsm.state_enter_time or now
        cycle = max(self.pulse_sec + self.observe_sec, 1e-3)
        phase = (now - start) % cycle
        if phase < self.pulse_sec:
            return ServoCommand(vx=0.0, wz=self.scan_wz), "lost_recovery_pulse"
        return ServoCommand(), "lost_recovery_observe"

    def apply_safety_layer(self, raw_cmd: Twist) -> Tuple[Twist, Dict[str, Any]]:
        front = self.front_distance()
        scan_age = self.free_space.scan_age()
        info: Dict[str, Any] = {
            "front_distance": front,
            "scan_age": scan_age,
            "raw_cmd_vx": float(raw_cmd.linear.x),
            "raw_cmd_wz": float(raw_cmd.angular.z),
            "safety_limited": False,
        }
        safe = Twist()
        if self.require_lidar and (scan_age is None or scan_age > self.scan_stale_sec):
            info.update({"safe_cmd_vx": 0.0, "safe_cmd_wz": 0.0, "safety_reason": "stale_scan"})
            return safe, info
        if front is not None and front <= self.emergency_stop_distance:
            info.update(
                {
                    "safe_cmd_vx": 0.0,
                    "safe_cmd_wz": 0.0,
                    "safety_reason": "emergency_stop",
                    "control_mode": NavState.BLOCKED.value,
                }
            )
            return safe, info

        vx = float(raw_cmd.linear.x)
        wz = float(raw_cmd.angular.z)
        reason = "pass_through"

        if front is not None and front <= self.hard_stop_distance:
            vx = 0.0
            info["safety_limited"] = True
            reason = "hard_stop"
        elif front is not None and front < self.slow_distance and vx > 0.0:
            span = max(self.slow_distance - self.hard_stop_distance, 1e-6)
            vx *= clamp((front - self.hard_stop_distance) / span, 0.0, 1.0)
            info["safety_limited"] = True
            reason = "slow_zone_scale"

        if abs(wz) > self.turn_zero_vx_wz:
            vx = 0.0
            info["safety_limited"] = True
            reason = "turn_zero_vx"
        elif abs(wz) > self.turn_slow_vx_wz and vx > 0.0:
            vx *= self.turn_slow_vx_scale
            info["safety_limited"] = True
            reason = "turn_slow_vx"

        safe.linear.x = clamp(vx, -self.max_cmd_vx, self.max_cmd_vx)
        safe.angular.z = clamp(wz, -self.max_cmd_wz, self.max_cmd_wz)
        info.update({"safe_cmd_vx": float(safe.linear.x), "safe_cmd_wz": float(safe.angular.z), "safety_reason": reason})
        return safe, info

    def pick_clearance_turn(self) -> Tuple[float, str]:
        left = self.free_space.left_clearance()
        right = self.free_space.right_clearance()
        if left is None and right is None:
            return 1.0, "unknown"
        if right is None or (left is not None and left >= right):
            return 1.0, "left"
        return -1.0, "right"

    def front_distance(self) -> Optional[float]:
        if not self.require_lidar:
            return None
        return self.free_space.front_min_distance()

    @staticmethod
    def to_twist(cmd: ServoCommand) -> Twist:
        out = Twist()
        out.linear.x = float(cmd.vx)
        out.angular.z = float(cmd.wz)
        return out

    def publish_point(self, target: NavTarget, mode: str) -> None:
        data = {
            "u": target.u,
            "v": target.v,
            "source": target.source,
            "mode": mode,
            "visible": target.visible,
            "stale": target.stale,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "time": time.time(),
        }
        self.point_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def publish_state(self, mode: str, from_control: bool = False, **kwargs: Any) -> None:
        result = self.last_fsm_result
        data = {
            "step": self.step_count,
            "mode": mode,
            "fsm_mode": self.fsm.state.value,
            "fsm_reason": result.reason if result else "init",
            "instruction": self.instruction,
            "nav_mode": self.mode,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "front_distance": self.front_distance(),
            "desired_reason": self.desired_reason,
            "raw_cmd_vx": float(self.desired_cmd.linear.x),
            "raw_cmd_wz": float(self.desired_cmd.angular.z),
            "safe_cmd_vx": float(self.last_safety.get("safe_cmd_vx", self.last_cmd.linear.x)),
            "safe_cmd_wz": float(self.last_safety.get("safe_cmd_wz", self.last_cmd.angular.z)),
            "safety": self.last_safety,
            "time": time.time(),
        }
        data.update(json_safe(kwargs))
        payload = json.dumps(data, ensure_ascii=False)
        self.state_pub.publish(String(data=payload))
        if not from_control:
            self.get_logger().info(payload)

    def publish_stop(self) -> None:
        self.desired_cmd = Twist()
        self.cmd_pub.publish(Twist())


def main() -> None:
    parser = argparse.ArgumentParser(description="Shared color / YOLO LiDAR / Qwen YOLO navigation")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--instruction", default=None)
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    instruction = args.instruction or str(cfg.get("instruction", "find the target"))

    rclpy.init()
    node = SharedNav(cfg, instruction)
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
