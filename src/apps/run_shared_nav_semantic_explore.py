#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.config.nav_birth_scan import load_birth_scan_config
from src.config.nav_success import load_success_config
from src.config.nav_voter import load_voter_config
from src.control.point_servo import PointServo, PointServoConfig, ServoCommand, clamp
from src.fsm.nav_state_machine import NavFSMConfig, NavObservation, NavState, NavStateMachine
from src.nav.frame_transform_2d import (
    Transform2D,
    hint_goal_frame,
    transform2d_from_tf_message,
)
from src.nav.lidar_distance import combine_lidar_distances
from src.nav.search_strategy import (
    TargetSearchMemory,
    compute_loss_age,
    pick_clearance_turn_dir,
    should_use_free_space,
    turn_dir_from_ex,
)
from src.planning.path_follower import follow_path
from src.perception.free_space_waypoint import FreeSpaceConfig, FreeSpaceWaypointProvider
from src.perception.target_adapter import NavTarget, TargetAdapter


DEFAULT_CONFIG = str(ROOT / "configs" / "nav_yolo_lidar_semantic_explore.yaml")


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


class SharedNavSemanticExplore(Node):
    def __init__(self, cfg: Dict[str, Any], instruction: str):
        super().__init__("shared_nav_semantic_explore")
        self.cfg = cfg
        self.mode = str(cfg.get("mode", "yolo_lidar_nav"))
        self.instruction = instruction or str(cfg.get("instruction", "find the target"))

        camera = section(cfg, "camera")
        target_cfg = section(cfg, "target")
        freshness = section(cfg, "freshness")
        rates = section(cfg, "rates")
        safety = section(cfg, "safety")
        success_cfg = load_success_config(cfg)
        voter_cfg = load_voter_config(cfg)
        birth_scan_cfg = load_birth_scan_config(cfg)
        search = section(cfg, "search")
        track_cfg = section(cfg, "track")
        chassis_cfg = section(cfg, "chassis")

        self.image_width = int(camera.get("width", cfg.get("image_width", cfg.get("camera_width", 640))))
        self.image_height = int(camera.get("height", cfg.get("image_height", cfg.get("camera_height", 480))))
        self.image_stale_sec = float(freshness.get("image_stale_sec", 0.40))
        self.scan_stale_sec = float(freshness.get("scan_stale_sec", 0.30))
        self.bbox_stale_sec = float(freshness.get("bbox_stale_sec", 0.45))

        self.require_lidar = bool(success_cfg["require_lidar"])
        self.target_source = str(target_cfg.get("source", "color" if self.mode == "color_nav" else "yolo_bbox"))
        self.target_color = str(target_cfg.get("color", "red"))
        self.target_min_score = float(target_cfg.get("min_score", 0.0))
        self.arrive_center_px = float(success_cfg["center_px"])
        self.min_safe_distance = float(success_cfg["min_safe_distance"])
        self.stop_distance = float(success_cfg["stop_distance"])
        self.verify_distance_max = float(success_cfg["verify_distance_max"])
        self.success_min_area_ratio = float(success_cfg["min_area_ratio"])
        self.lost_target_servo_sec = float(success_cfg["lost_target_servo_sec"])
        self.lost_target_vx_scale = float(success_cfg["lost_target_vx_scale"])

        fsm_cfg = section(cfg, "fsm")
        self.fsm = NavStateMachine(
            NavFSMConfig(
                stable_frames_required=int(fsm_cfg.get("stable_frames_required", 3)),
                lost_frames_limit=int(fsm_cfg.get("lost_frames_limit", 5)),
                arrive_required_frames=int(success_cfg["arrive_frames"]),
                verify_required_frames=int(success_cfg["verify_frames"]),
                centered_required_frames=int(fsm_cfg.get("centered_required_frames", 3)),
                max_search_sec=float(fsm_cfg.get("max_search_sec", 30.0)),
                max_task_sec=float(fsm_cfg.get("max_task_sec", 180.0)),
                min_state_frames=int(fsm_cfg.get("min_state_frames", 2)),
                qwen_verify_required=bool(success_cfg["qwen_verify_required"]),
                qwen_verify_timeout_sec=float(success_cfg["qwen_verify_timeout_sec"]),
                qwen_verify_fail_policy=str(success_cfg["qwen_verify_fail_policy"]),
                recovery_max_sec=float(fsm_cfg.get("recovery_max_sec", 4.0)),
                min_safe_distance=self.min_safe_distance,
                stop_distance=self.stop_distance,
                verify_distance_max=self.verify_distance_max,
                emergency_stop_distance=float(success_cfg["emergency_stop_distance"]),
                arrive_area_ratio=self.success_min_area_ratio,
                arrive_height_ratio=float(success_cfg["min_height_ratio"]),
                success_min_score=float(success_cfg["min_score"]),
                success_center_px=float(success_cfg["center_px"]),
                success_target_recent_sec=float(success_cfg["target_recent_sec"]),
                lidar_success_distance=float(success_cfg["lidar_success_distance"]),
                stop_verify_sec=float(success_cfg["stop_verify_sec"]),
                emergency_target_recent_sec=float(success_cfg["emergency_target_recent_sec"]),
                lost_target_servo_sec=self.lost_target_servo_sec,
                center_only_arrive_enabled=bool(success_cfg["center_only_enabled"]),
                birth_scan_enabled=bool(birth_scan_cfg["enabled"]),
                birth_scan_wait_sec=float(birth_scan_cfg["wait_sec"]),
                birth_scan_wz=float(birth_scan_cfg["scan_wz"]),
                birth_scan_effective_wz=float(birth_scan_cfg["effective_scan_wz"]),
                birth_scan_deg=float(birth_scan_cfg["scan_deg"]),
                birth_scan_max_rotations=float(birth_scan_cfg["max_rotations"]),
                birth_scan_max_wall_timeout_sec=float(birth_scan_cfg["max_wall_timeout_sec"]),
                birth_scan_use_odom_yaw=True,
                birth_scan_turn_dir=float(birth_scan_cfg["turn_dir"]),
            )
        )

        self.birth_scan_wz = float(birth_scan_cfg["scan_wz"])
        self.birth_scan_effective_wz = float(birth_scan_cfg["effective_scan_wz"])
        self.birth_scan_turn_dir = float(birth_scan_cfg["turn_dir"])
        self.birth_scan_target_rad = math.radians(float(birth_scan_cfg["max_total_scan_deg"]))
        self.chassis_max_wz = float(chassis_cfg.get("max_wz", 0.06))

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
                turn_only_vx=float(servo_cfg.get("turn_only_vx", servo_cfg.get("steer_vx", 0.04))),
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
            voter_enabled=bool(voter_cfg["enabled"]),
            voter_window_size=int(voter_cfg["window_size"]),
            voter_min_votes=int(voter_cfg["min_votes"]),
            voter_lost_hold_frames=int(voter_cfg["lost_hold_frames"]),
            voter_iou_threshold=float(voter_cfg["iou_threshold"]),
            voter_center_dist_threshold=float(voter_cfg["center_dist_threshold"]),
            voter_smooth_alpha=float(voter_cfg["smooth_alpha"]),
            voter_accept_held_target=bool(voter_cfg["accept_held_target"]),
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
        self.search_arc_vx = float(search.get("search_arc_vx", 0.02))
        self.pulse_sec = float(search.get("pulse_sec", 0.20))
        self.observe_sec = float(search.get("observe_sec", 0.60))
        self.free_space_enabled = bool(search.get("free_space_enabled", False))
        self.free_space_enable_after_sec = float(search.get("free_space_enable_after_sec", 10.0))
        self.free_space_vx = float(search.get("free_space_vx", 0.015))
        self.loss_memory_sec = float(search.get("loss_memory_sec", 4.0))
        self.free_space_after_loss_sec = float(
            search.get("free_space_after_loss_sec", self.free_space_enable_after_sec)
        )
        self.turn_lock_sec = float(search.get("turn_lock_sec", 6.0))
        self.lidar_turn_min_delta = float(search.get("lidar_turn_min_delta", 0.15))
        self.lidar_turn_side_deg = float(search.get("lidar_turn_side_deg", 45.0))
        self.lidar_fallback_on_no_memory = bool(search.get("lidar_fallback_on_no_memory", True))
        self.track_blocked_hold_on_target = bool(track_cfg.get("blocked_hold_on_target", True))

        self.search_mem = TargetSearchMemory()

        self.emergency_stop_distance = float(safety.get("emergency_stop_distance", 0.45))
        self.safety_stop_distance = float(
            safety.get("stop_distance", safety.get("hard_stop_distance", 0.55))
        )
        self.hard_stop_distance = self.safety_stop_distance
        self.slow_distance = float(safety.get("slow_distance", 0.90))
        self.max_cmd_vx = float(safety.get("max_cmd_vx_explore", safety.get("max_cmd_vx", 0.06)))
        self.max_cmd_wz = float(safety.get("max_cmd_wz_explore", safety.get("max_cmd_wz", 0.06)))
        self.slow_vx = float(safety.get("slow_vx", min(self.max_cmd_vx, 0.02)))
        self.safe_turn_wz = float(safety.get("safe_turn_wz", min(self.max_cmd_wz, 0.04)))
        self.turn_zero_vx_wz = float(safety.get("turn_zero_vx_wz", 0.05))
        self.turn_slow_vx_wz = float(safety.get("turn_slow_vx_wz", 0.035))
        self.turn_slow_vx_scale = float(safety.get("turn_slow_vx_scale", 0.5))
        retreat_cfg = section(safety, "blocked_retreat")
        self.blocked_retreat_enabled = bool(retreat_cfg.get("enabled", True))
        self.blocked_retreat_margin_m = float(retreat_cfg.get("clearance_margin_m", 0.10))
        self.blocked_retreat_vx = float(retreat_cfg.get("reverse_vx", 0.03))
        self.blocked_retreat_max_sec = float(retreat_cfg.get("max_duration_sec", 5.0))
        self.blocked_retreat_fsm_buffer_m = float(retreat_cfg.get("fsm_unblock_buffer_m", 0.02))

        self.bridge = CvBridge()
        self.last_frame = None
        self.last_image_time = 0.0
        self.last_bbox_time = 0.0
        self.last_scan_time = 0.0
        self.last_target = NavTarget(False, None, None, reason="init")
        self.last_good_target = self.last_target
        self.last_fsm_result = None
        self.desired_cmd = Twist()
        self.last_cmd = Twist()
        self.last_safety: Dict[str, Any] = {}
        self.desired_reason = "init"
        self.step_count = 0
        self.qwen_verified: Optional[bool] = None

        explore_cfg = section(cfg, "semantic_explore")
        planner_cfg = section(cfg, "planner")
        bearing_cfg = section(planner_cfg, "bearing_first")
        birth_raw = section(cfg, "birth_scan")
        self.semantic_explore_enabled = bool(explore_cfg.get("enabled", False))
        self.explore_hint_topic = str(explore_cfg.get("hint_topic", "/explore_goal_hint"))
        self.explore_max_hint_age_sec = float(explore_cfg.get("max_hint_age_sec", 1.5))
        self.explore_min_hint_score = float(explore_cfg.get("min_hint_score", 0.42))
        self.explore_goal_hold_sec = float(explore_cfg.get("goal_hold_sec", 8.0))
        self.explore_goal_switch_max_distance_m = float(
            explore_cfg.get("goal_switch_max_distance_m", 0.6)
        )
        self.explore_goal_switch_min_dist_m = float(explore_cfg.get("goal_switch_min_dist_m", 0.45))
        blocked_reasons = explore_cfg.get(
            "blocked_reject_reasons", ["blocked", "unsafe_front_clearance", "emergency_stop"]
        )
        self.explore_blocked_reject_reasons = {str(r) for r in blocked_reasons}
        self.explore_use_in_search_only = bool(explore_cfg.get("use_in_search_only", True))
        self.explore_goal_reached_radius_m = float(explore_cfg.get("goal_reached_radius_m", 0.35))
        self.explore_goal_timeout_sec = float(explore_cfg.get("goal_timeout_sec", 12.0))
        self.explore_step_distance_m = float(
            explore_cfg.get("step_goal_distance_m", bearing_cfg.get("step_distance_m", 0.45))
        )
        self.explore_observe_after_reach_sec = float(explore_cfg.get("observe_after_reach_sec", 1.2))
        self.explore_observe_scan_deg = float(explore_cfg.get("observe_scan_deg", 90.0))
        self.explore_observe_scan_wz = float(explore_cfg.get("observe_scan_wz", 0.05))
        self.explore_blacklist_ttl_sec = float(explore_cfg.get("blacklist_ttl_sec", 180.0))
        self.explore_target_visible_interrupt = bool(explore_cfg.get("target_visible_interrupt", True))
        self.planner_mode = str(planner_cfg.get("mode", "bearing_first"))
        self.require_astar_path = bool(planner_cfg.get("require_astar_path", True))
        self.astar_fallback_bearing = bool(
            planner_cfg.get(
                "astar_fallback_bearing",
                planner_cfg.get("require_astar_path", True),
            )
        )
        self.path_follow_lookahead_m = float(planner_cfg.get("path_follow_lookahead_m", 0.35))
        self.bearing_max_vx = float(bearing_cfg.get("max_vx", 0.025))
        self.bearing_max_wz = float(bearing_cfg.get("max_wz", 0.05))
        self.bearing_turn_threshold = float(bearing_cfg.get("turn_in_place_threshold_rad", 0.35))
        self.bearing_forward_threshold = float(bearing_cfg.get("forward_bearing_threshold_rad", 0.22))
        self.explore_align_max_sec = float(bearing_cfg.get("align_max_sec", 2.5))
        self.explore_step_mode = str(bearing_cfg.get("step_mode", "distance")).lower()
        self.explore_step_sec = float(bearing_cfg.get("pulse_sec", 0.35))
        self.explore_inter_burst_pause_sec = float(
            bearing_cfg.get("inter_burst_pause_sec", bearing_cfg.get("observe_sec", 0.12))
        )
        self.explore_observe_hold_sec = float(bearing_cfg.get("observe_sec", 0.12))
        self.birth_record_sector = bool(birth_raw.get("record_sector", False))
        self.birth_views = int(birth_raw.get("views", 8))

        self.latest_explore_hint: Optional[Dict[str, Any]] = None
        self.latest_explore_hint_time: Optional[float] = None
        self.active_explore_goal: Optional[Dict[str, Any]] = None
        self.explore_goal_start_time: Optional[float] = None
        self.failed_explore_goals: list = []
        self.rejected_candidate_ids: Dict[str, float] = {}
        self.observe_update_active = False
        self.observe_update_start: Optional[float] = None
        self.observe_scan_accum_rad = 0.0
        self.explore_phase = "EXPLORE_SELECT"
        self.explore_phase_start = 0.0
        self.explore_burst_start_xy: Optional[Tuple[float, float]] = None
        self.explore_goal_traveled_m = 0.0
        self.explore_last_reject_reason: Optional[str] = None
        self.explore_last_reject_goal_pose: Optional[list] = None
        self.blocked_retreat_active = False
        self.blocked_retreat_start_time = 0.0
        self.blocked_retreat_clearance_target = 0.0
        self.active_planned_path: list = []
        self.active_planned_path_frame: Optional[str] = None
        self.active_path_waypoint_idx = -1
        mapping_frames = section(section(cfg, "semantic_mapping"), "frames")
        self.explore_map_frame = str(mapping_frames.get("fixed_frame", "map"))
        self.explore_odom_frame = str(mapping_frames.get("odom_frame", "odom"))
        self.explore_control_frame = self.explore_odom_frame
        self.explore_tf_cache_ttl_sec = float(
            explore_cfg.get("tf_cache_ttl_sec", 0.25)
        )
        self.explore_tf_lookup_timeout_sec = float(
            explore_cfg.get("tf_lookup_timeout_sec", 0.12)
        )
        self.tf_buffer = None
        self.tf_listener = None
        self._explore_tf_cache: Dict[Tuple[str, str], Tuple[float, Transform2D]] = {}
        self._explore_tf_last_error = ""
        self._explore_geometry_trusted = False
        self._explore_geometry_error = ""
        self.latest_odom_frame: Optional[str] = None
        try:
            import tf2_ros

            self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        except Exception as exc:
            self.get_logger().warn(f"explore TF unavailable: {exc!r}")
        self.birth_sectors: list = []
        self._birth_last_wz = 0.0
        self.latest_odom_yaw: Optional[float] = None
        self.latest_odom_xy: Optional[Tuple[float, float]] = None
        self.latest_odom_time: Optional[float] = None
        self._birth_odom_start_yaw: Optional[float] = None
        self._birth_odom_last_yaw: Optional[float] = None
        self._birth_odom_accum_rad = 0.0
        self._birth_odom_fallback_time: Optional[float] = None
        self._fsm_prev_state = NavState.BOOT
        self.last_explore_candidate_id: Optional[str] = None

        self.image_topic = topic(cfg, "image_raw", "image_topic", "/image_raw")
        _topics = section(cfg, "topics")
        self.scan_topic = str(_topics.get("scan_filtered", _topics.get("scan", "/scan_filtered")))
        self.odom_topic = topic(cfg, "odom", "odom_topic", "/odom")
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
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos_profile_sensor_data)
        if self.require_lidar:
            self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        if self.target_source == "yolo_bbox":
            self.create_subscription(String, self.bbox_topic, self.bbox_cb, 10)
        if self.semantic_explore_enabled:
            self.create_subscription(String, self.explore_hint_topic, self.on_explore_goal_hint, 10)

        decision_hz = float(rates.get("decision_hz", 10.0))
        control_hz = float(rates.get("control_hz", 20.0))
        state_pub_hz = float(rates.get("state_pub_hz", 5.0))
        self._state_publish_div = max(1, int(round(control_hz / max(state_pub_hz, 1e-3))))
        self._control_tick = 0
        self.create_timer(1.0 / max(decision_hz, 1e-3), self.decision_timer_cb)
        self.create_timer(1.0 / max(control_hz, 1e-3), self.control_timer_cb)
        self.create_timer(3.0, self.publish_target_words)

        self.get_logger().info(f"===== shared_nav_semantic_explore mode={self.mode} =====")
        if self.semantic_explore_enabled:
            self.get_logger().info(
                f"semantic_explore enabled hint={self.explore_hint_topic} planner={self.planner_mode} "
                f"goal_frame={self.explore_map_frame} control_frame={self.explore_control_frame} "
                f"tf_buffer={'ok' if self.tf_buffer is not None else 'missing'}"
            )
        self.get_logger().info(f"topics image={self.image_topic} scan={self.scan_topic} cmd={self.cmd_topic}")
        if voter_cfg["enabled"]:
            self.get_logger().info(
                "voter enabled "
                f"window={voter_cfg['window_size']} min_votes={voter_cfg['min_votes']} "
                f"lost_hold={voter_cfg['lost_hold_frames']}"
            )
        if birth_scan_cfg["enabled"]:
            self.get_logger().info(
                "birth_scan enabled "
                f"wait={birth_scan_cfg['wait_sec']}s scan_wz={birth_scan_cfg['scan_wz']} "
                f"scan_deg={birth_scan_cfg['scan_deg']} max_rotations={birth_scan_cfg['max_rotations']} "
                f"max_total_deg={birth_scan_cfg['max_total_scan_deg']:.0f} "
                f"max_total_duration={birth_scan_cfg['max_total_duration_sec']:.1f}s"
            )
            if float(birth_scan_cfg["scan_wz"]) > self.chassis_max_wz + 1e-6:
                self.get_logger().warn(
                    "birth_scan.scan_wz exceeds chassis.max_wz: "
                    f"{birth_scan_cfg['scan_wz']} > {self.chassis_max_wz}; "
                    "cmd_vel bridge will clip angular speed unless chassis.max_wz is raised"
                )

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

    @staticmethod
    def _yaw_from_odom(msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_yaw_delta(prev_yaw: float, current_yaw: float) -> float:
        return math.atan2(math.sin(current_yaw - prev_yaw), math.cos(current_yaw - prev_yaw))

    def odom_cb(self, msg: Odometry) -> None:
        self.latest_odom_yaw = self._yaw_from_odom(msg)
        self.latest_odom_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        self.latest_odom_frame = str(msg.header.frame_id or self.explore_odom_frame)
        self.latest_odom_time = time.time()

    def _sync_birth_scan_odom_yaw(self, prev_state: NavState) -> None:
        if self.fsm.state != NavState.SCANNING:
            if prev_state == NavState.SCANNING:
                self._birth_odom_last_yaw = None
            return
        if self.latest_odom_yaw is None:
            now = time.time()
            if self._birth_odom_fallback_time is not None:
                dt = max(0.0, now - self._birth_odom_fallback_time)
                wz = max(abs(float(self.fsm.cfg.birth_scan_effective_wz)), 0.0)
                self._birth_odom_accum_rad += wz * dt
                self.fsm.birth_scan_yaw_accumulated_rad = self._birth_odom_accum_rad
            self._birth_odom_fallback_time = now
            return

        self._birth_odom_fallback_time = None

        just_entered = prev_state != NavState.SCANNING
        if just_entered or self._birth_odom_last_yaw is None:
            if self.fsm.birth_scan_yaw_accumulated_rad > 1e-6:
                self._birth_odom_accum_rad = float(self.fsm.birth_scan_yaw_accumulated_rad)
            else:
                self._birth_odom_start_yaw = self.latest_odom_yaw
                self._birth_odom_accum_rad = 0.0
            self._birth_odom_last_yaw = self.latest_odom_yaw
            self.fsm.birth_scan_yaw_accumulated_rad = self._birth_odom_accum_rad
            return

        delta = abs(self._normalize_yaw_delta(self._birth_odom_last_yaw, self.latest_odom_yaw))
        self._birth_odom_accum_rad += delta
        self._birth_odom_last_yaw = self.latest_odom_yaw
        self.fsm.birth_scan_yaw_accumulated_rad = self._birth_odom_accum_rad

    def bbox_cb(self, msg: String) -> None:
        self.last_bbox_time = time.time()
        self.target_adapter.ingest_yolo_bbox_json(msg.data)

    def on_explore_goal_hint(self, msg: String) -> None:
        try:
            self.latest_explore_hint = json.loads(msg.data)
            self.latest_explore_hint_time = time.time()
        except json.JSONDecodeError:
            self.latest_explore_hint = None

    def _prune_rejected_candidates(self, now: float) -> None:
        expired = [k for k, t in self.rejected_candidate_ids.items() if t <= now]
        for k in expired:
            del self.rejected_candidate_ids[k]

    def _hint_has_planned_path(self, hint: Dict[str, Any]) -> bool:
        planned = hint.get("planned_path")
        return isinstance(planned, list) and len(planned) >= 2

    def _hint_uses_bearing_fallback(self, hint: Dict[str, Any]) -> bool:
        if not self.astar_fallback_bearing:
            return False
        if self._hint_has_planned_path(hint):
            return False
        if str(hint.get("nav_planner", "")) == "bearing_first":
            return True
        return bool(hint.get("astar_fallback"))

    def _hint_is_actionable(self, hint: Dict[str, Any], now: float) -> bool:
        if str(hint.get("mode", "")) == "none":
            return False
        if self.require_astar_path and not self._hint_has_planned_path(hint):
            if not self._hint_uses_bearing_fallback(hint):
                return False
            if self._goal_pose_xy(hint) is None:
                return False
        age = now - float(self.latest_explore_hint_time or 0.0)
        if age > self.explore_max_hint_age_sec:
            return False
        if float(hint.get("score", 0.0)) < self.explore_min_hint_score:
            return False
        cand_id = str(hint.get("candidate_id", ""))
        if cand_id and cand_id in self.rejected_candidate_ids:
            if self.rejected_candidate_ids[cand_id] > now:
                return False
        return True

    def valid_explore_hint(self, now: float) -> bool:
        if not self.semantic_explore_enabled or not self.latest_explore_hint:
            return False
        return self._hint_is_actionable(self.latest_explore_hint, now)

    def _locked_explore_hint(self, now: float) -> Optional[Dict[str, Any]]:
        if self.valid_explore_hint(now):
            return self.latest_explore_hint
        if not self.active_explore_goal:
            return None
        cand_id = str(self.active_explore_goal.get("candidate_id", ""))
        if cand_id and cand_id in self.rejected_candidate_ids:
            if self.rejected_candidate_ids[cand_id] > now:
                return None
        if self.explore_goal_start_time is None:
            return None
        if now - self.explore_goal_start_time > self.explore_goal_timeout_sec:
            return None
        hint_time = float(self.latest_explore_hint_time or 0.0)
        if hint_time > 0.0 and now - hint_time <= self.explore_goal_hold_sec:
            return self.active_explore_goal
        return None

    @staticmethod
    def _goal_pose_xy(hint: Dict[str, Any]) -> Optional[Tuple[float, float]]:
        goal_pose = hint.get("goal_pose")
        if isinstance(goal_pose, (list, tuple)) and len(goal_pose) >= 2:
            return float(goal_pose[0]), float(goal_pose[1])
        return None

    def _is_same_goal_pose(self, prev: Dict[str, Any], fresh: Dict[str, Any]) -> bool:
        if prev.get("candidate_id") == fresh.get("candidate_id"):
            return True
        prev_xy = self._goal_pose_xy(prev)
        next_xy = self._goal_pose_xy(fresh)
        if prev_xy is None or next_xy is None:
            return False
        return (
            math.hypot(next_xy[0] - prev_xy[0], next_xy[1] - prev_xy[1])
            < self.explore_goal_switch_min_dist_m
        )

    def _explore_goal_switch_allowed(
        self,
        prev: Optional[Dict[str, Any]],
        fresh: Dict[str, Any],
        live_distance: float,
    ) -> bool:
        if prev is None:
            return True
        if self._is_same_goal_pose(prev, fresh):
            return False
        return live_distance <= self.explore_goal_switch_max_distance_m

    def _odom_travel_since(self, start_xy: Optional[Tuple[float, float]]) -> float:
        if start_xy is None or self.latest_odom_xy is None:
            return 0.0
        dx = self.latest_odom_xy[0] - start_xy[0]
        dy = self.latest_odom_xy[1] - start_xy[1]
        return math.hypot(dx, dy)

    def _begin_explore_goal(self, now: float) -> None:
        self.explore_goal_start_time = now
        self.explore_phase = "EXPLORE_ALIGN"
        self.explore_phase_start = now
        self.explore_burst_start_xy = None
        self.explore_goal_traveled_m = 0.0
        self.observe_update_active = False
        self._clear_active_planned_path()

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _planned_path_points(self, hint: Dict[str, Any]) -> List[Tuple[float, float]]:
        planned = hint.get("planned_path")
        if not isinstance(planned, list):
            return []
        out: List[Tuple[float, float]] = []
        for pt in planned:
            if isinstance(pt, (list, tuple)) and len(pt) >= 2:
                out.append((float(pt[0]), float(pt[1])))
        return out

    def _set_active_planned_path(self, hint: Dict[str, Any]) -> None:
        self.active_planned_path = self._planned_path_points(hint)
        self.active_planned_path_frame = self._hint_goal_frame(hint)

    def _clear_active_planned_path(self) -> None:
        self.active_planned_path = []
        self.active_planned_path_frame = None
        self.active_path_waypoint_idx = -1

    def _hint_goal_frame(self, hint: Dict[str, Any]) -> str:
        return hint_goal_frame(hint, self.explore_map_frame)

    def _path_source_frame(self, hint: Dict[str, Any]) -> str:
        if self.active_planned_path and self.active_planned_path_frame:
            return self.active_planned_path_frame
        return self._hint_goal_frame(hint)

    def _lookup_frame_transform(
        self, target_frame: str, source_frame: str
    ) -> Optional[Transform2D]:
        if not target_frame or not source_frame:
            self._explore_tf_last_error = f"{target_frame}<-{source_frame}: empty frame"
            return None
        if target_frame == source_frame:
            return Transform2D.identity()

        cache_key = (target_frame, source_frame)
        now = time.time()
        cached = self._explore_tf_cache.get(cache_key)
        if cached and now - cached[0] <= self.explore_tf_cache_ttl_sec:
            return cached[1]

        if self.tf_buffer is None:
            self._explore_tf_last_error = f"{target_frame}<-{source_frame}: no tf_buffer"
            return cached[1] if cached else None

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.explore_tf_lookup_timeout_sec),
            )
            transform = transform2d_from_tf_message(tf_msg)
            self._explore_tf_cache[cache_key] = (now, transform)
            self._explore_tf_last_error = ""
            return transform
        except Exception as exc:
            self._explore_tf_last_error = f"{target_frame}<-{source_frame}: {exc}"
            return cached[1] if cached else None

    def _transform_xy_to_control(
        self, x: float, y: float, source_frame: str
    ) -> Optional[Tuple[float, float]]:
        transform = self._lookup_frame_transform(self.explore_control_frame, source_frame)
        if transform is None:
            return None
        return transform.apply(x, y)

    def _path_in_control_frame(
        self, hint: Dict[str, Any], path: List[Tuple[float, float]]
    ) -> Optional[List[Tuple[float, float]]]:
        if not path:
            return None
        source_frame = self._path_source_frame(hint)
        if source_frame == self.explore_control_frame:
            return list(path)
        transform = self._lookup_frame_transform(self.explore_control_frame, source_frame)
        if transform is None:
            return None
        return transform.apply_path(path)

    def _goal_pose_in_control(self, hint: Dict[str, Any]) -> Optional[Tuple[float, float]]:
        goal_xy = self._goal_pose_xy(hint)
        if goal_xy is None:
            return None
        source_frame = self._hint_goal_frame(hint)
        if source_frame == self.explore_control_frame:
            return goal_xy
        return self._transform_xy_to_control(goal_xy[0], goal_xy[1], source_frame)

    def _robot_control_pose(self) -> Optional[Tuple[float, float, float]]:
        if self.latest_odom_xy is None or self.latest_odom_yaw is None:
            return None
        if (
            self.latest_odom_frame
            and self.latest_odom_frame != self.explore_control_frame
        ):
            self._explore_geometry_error = (
                f"odom_frame_mismatch:{self.latest_odom_frame}!={self.explore_control_frame}"
            )
            return None
        return self.latest_odom_xy[0], self.latest_odom_xy[1], self.latest_odom_yaw

    def _final_goal_distance(self, hint: Dict[str, Any]) -> float:
        goal_xy = self._goal_pose_in_control(hint)
        robot = self._robot_control_pose()
        if goal_xy is not None and robot is not None:
            return math.hypot(goal_xy[0] - robot[0], goal_xy[1] - robot[1])
        return float(hint.get("goal_distance_m", 999.0))

    def _explore_goal_geometry(self, hint: Dict[str, Any]) -> Tuple[float, float]:
        self._explore_geometry_trusted = False
        self._explore_geometry_error = ""

        final_dist = self._final_goal_distance(hint)
        raw_path = self.active_planned_path or self._planned_path_points(hint)
        use_path = bool(raw_path) and (
            self._hint_has_planned_path(hint) or bool(self.active_planned_path)
        )
        robot = self._robot_control_pose()
        if robot is None:
            self._explore_geometry_error = "robot_pose_unavailable"
            return float(hint.get("goal_bearing_rad", 0.0)), final_dist

        rx, ry, ryaw = robot
        if use_path:
            path_control = self._path_in_control_frame(hint, raw_path)
            if path_control:
                bearing, _, wp_idx = follow_path(
                    path_control,
                    (rx, ry),
                    ryaw,
                    self.path_follow_lookahead_m,
                )
                if bearing is not None:
                    self.active_path_waypoint_idx = wp_idx
                    self._explore_geometry_trusted = True
                    return bearing, final_dist
            self._explore_geometry_error = (
                self._explore_geometry_error or "path_transform_failed"
            )

        goal_xy = self._goal_pose_in_control(hint)
        if goal_xy is not None:
            gx, gy = goal_xy
            bearing = self._normalize_angle(math.atan2(gy - ry, gx - rx) - ryaw)
            self._explore_geometry_trusted = True
            return bearing, final_dist

        self._explore_geometry_error = self._explore_tf_last_error or "goal_transform_failed"
        return float(hint.get("goal_bearing_rad", 0.0)), 999.0

    def _reject_candidate(self, hint: Dict[str, Any], now: float, reason: str) -> None:
        self.explore_last_reject_reason = reason
        goal_pose = hint.get("goal_pose")
        if isinstance(goal_pose, (list, tuple)):
            self.explore_last_reject_goal_pose = list(goal_pose)
        cand_id = str(hint.get("candidate_id", ""))
        if cand_id:
            self.rejected_candidate_ids[cand_id] = now + self.explore_blacklist_ttl_sec
            self.last_explore_candidate_id = cand_id
        self.failed_explore_goals.append(
            {
                "candidate_id": cand_id,
                "target_class": hint.get("target_class", ""),
                "goal_pose": hint.get("goal_pose"),
                "reason": reason,
                "created_time": now,
                "expire_time": now + self.explore_blacklist_ttl_sec,
            }
        )
        self.active_explore_goal = None
        self.observe_update_active = False
        self.explore_phase = "EXPLORE_SELECT"
        self.explore_phase_start = now
        self.explore_burst_start_xy = None
        self.explore_goal_traveled_m = 0.0
        self._clear_active_planned_path()

    def command_from_explore_hint(self, now: float) -> Optional[Tuple[ServoCommand, str]]:
        hint = self._locked_explore_hint(now)
        if hint is None:
            return None
        if self.valid_explore_hint(now):
            fresh = self.latest_explore_hint or hint
            prev = self.active_explore_goal
            prev_id = (prev or {}).get("candidate_id")
            next_id = fresh.get("candidate_id")
            switch_ids = prev_id != next_id
            if prev:
                _, live_distance = self._explore_goal_geometry(prev)
            else:
                live_distance = 999.0
            switch_allowed = (
                prev is None
                or (
                    switch_ids
                    and self._explore_goal_switch_allowed(prev, fresh, live_distance)
                )
            )
            if switch_allowed:
                self._begin_explore_goal(now)
                self.active_explore_goal = dict(fresh)
                if self._hint_has_planned_path(fresh):
                    self._set_active_planned_path(fresh)
            else:
                locked = dict(self.active_explore_goal or fresh)
                locked["goal_pose"] = fresh.get("goal_pose", locked.get("goal_pose"))
                locked["look_at"] = fresh.get("look_at", locked.get("look_at"))
                locked["score"] = fresh.get("score", locked.get("score"))
                locked["planned_path"] = fresh.get("planned_path", locked.get("planned_path"))
                locked["goal_frame"] = fresh.get("goal_frame", locked.get("goal_frame"))
                locked["nav_planner"] = fresh.get("nav_planner", locked.get("nav_planner"))
                locked["astar_fallback"] = fresh.get("astar_fallback", locked.get("astar_fallback"))
                locked["selection_explanation"] = fresh.get(
                    "selection_explanation", locked.get("selection_explanation")
                )
                self.active_explore_goal = locked
                if self._hint_has_planned_path(fresh):
                    self._set_active_planned_path(fresh)
                elif str(fresh.get("nav_planner", "")) == "bearing_first" or fresh.get(
                    "astar_fallback"
                ):
                    self._clear_active_planned_path()
        elif self.active_explore_goal is None:
            return None

        active = self.active_explore_goal or hint
        distance = float(active.get("goal_distance_m", 999.0))
        bearing = float(active.get("goal_bearing_rad", 0.0))
        live_bearing, live_distance = self._explore_goal_geometry(active)
        bearing = live_bearing
        distance = live_distance
        active["goal_bearing_rad"] = bearing
        active["goal_distance_m"] = distance

        if self.explore_goal_start_time and now - self.explore_goal_start_time > self.explore_goal_timeout_sec:
            self._reject_candidate(active, now, "goal_timeout")
            return None

        if not self._explore_geometry_trusted:
            return ServoCommand(vx=0.0, wz=0.0), "semantic_explore_tf_wait"

        if distance < self.explore_goal_reached_radius_m:
            if not self.observe_update_active:
                self.observe_update_active = True
                self.observe_update_start = now
                self.observe_scan_accum_rad = 0.0
            elapsed = now - (self.observe_update_start or now)
            if elapsed < 0.3:
                return ServoCommand(), "observe_update_stop"
            scan_target = math.radians(self.explore_observe_scan_deg)
            if self.observe_scan_accum_rad < scan_target:
                wz = self.explore_observe_scan_wz
                self.observe_scan_accum_rad += abs(wz) * (1.0 / max(float(section(self.cfg, "rates").get("decision_hz", 10)), 1.0))
                return ServoCommand(vx=0.0, wz=wz), "observe_update"
            if elapsed < self.explore_observe_after_reach_sec:
                return ServoCommand(), "observe_update_wait"
            if self.target_ok(self.last_target):
                self.active_explore_goal = None
                self.observe_update_active = False
                return None
            self._reject_candidate(active, now, "arrived_but_no_target")
            return None

        front = self.front_min_distance()
        align_threshold = self.bearing_turn_threshold
        kp = 0.9
        align_elapsed = now - self.explore_phase_start

        if self.explore_phase not in (
            "EXPLORE_ALIGN",
            "EXPLORE_STEP",
            "EXPLORE_BURST_PAUSE",
        ):
            self.explore_phase = "EXPLORE_ALIGN"
            self.explore_phase_start = now

        if self.explore_phase == "EXPLORE_ALIGN":
            if abs(bearing) > align_threshold and align_elapsed < self.explore_align_max_sec:
                wz = clamp(kp * bearing, -self.bearing_max_wz, self.bearing_max_wz)
                return ServoCommand(vx=0.0, wz=wz), "semantic_explore_align"
            self.explore_phase = "EXPLORE_STEP"
            self.explore_phase_start = now
            if self.latest_odom_xy is not None:
                self.explore_burst_start_xy = self.latest_odom_xy

        if self.explore_phase == "EXPLORE_STEP":
            if abs(bearing) > align_threshold * 1.35:
                self.explore_phase = "EXPLORE_ALIGN"
                self.explore_phase_start = now
                self.explore_burst_start_xy = None
                wz = clamp(kp * bearing, -self.bearing_max_wz, self.bearing_max_wz)
                return ServoCommand(vx=0.0, wz=wz), "semantic_explore_align"
            if front is not None and front < self.safety_stop_distance:
                self._reject_candidate(active, now, "unsafe_front_clearance")
                return None
            if self.explore_burst_start_xy is None and self.latest_odom_xy is not None:
                self.explore_burst_start_xy = self.latest_odom_xy
            burst_traveled = self._odom_travel_since(self.explore_burst_start_xy)
            step_done = (
                burst_traveled >= self.explore_step_distance_m
                if self.explore_step_mode == "distance"
                else (now - self.explore_phase_start) >= self.explore_step_sec
            )
            if not step_done:
                vx = clamp(self.bearing_max_vx, 0.0, self.max_cmd_vx)
                return ServoCommand(vx=vx, wz=0.0), "semantic_explore_step"
            self.explore_goal_traveled_m += burst_traveled
            self.explore_burst_start_xy = None
            self.explore_phase = "EXPLORE_BURST_PAUSE"
            self.explore_phase_start = now
            return ServoCommand(vx=0.0, wz=0.0), "semantic_explore_burst_pause"

        if self.explore_phase == "EXPLORE_BURST_PAUSE":
            if now - self.explore_phase_start < self.explore_inter_burst_pause_sec:
                return ServoCommand(vx=0.0, wz=0.0), "semantic_explore_burst_pause"
            self.explore_phase = "EXPLORE_ALIGN"
            self.explore_phase_start = now
            return ServoCommand(vx=0.0, wz=0.0), "semantic_explore_select"

        return ServoCommand(vx=0.0, wz=0.0), "semantic_explore_select"

    def _blocked_retreat_clearance_target(self) -> float:
        """Back until front clearance reaches emergency+margin, and above stop_distance for FSM unblock."""
        primary = self.emergency_stop_distance + self.blocked_retreat_margin_m
        fsm_min = self.safety_stop_distance + self.blocked_retreat_fsm_buffer_m
        return max(primary, fsm_min)

    def _start_blocked_retreat(self, now: float) -> None:
        self.blocked_retreat_active = True
        self.blocked_retreat_start_time = now
        self.blocked_retreat_clearance_target = self._blocked_retreat_clearance_target()

    def _stop_blocked_retreat(self) -> None:
        self.blocked_retreat_active = False
        self.blocked_retreat_start_time = 0.0

    def _blocked_retreat_cmd(self, now: float) -> Tuple[ServoCommand, str]:
        front = self.front_min_distance()
        target = self.blocked_retreat_clearance_target or self._blocked_retreat_clearance_target()
        if front is not None and front >= target:
            self._stop_blocked_retreat()
            return ServoCommand(vx=0.0, wz=0.0), "blocked_retreat_complete"
        if now - self.blocked_retreat_start_time > self.blocked_retreat_max_sec:
            self._stop_blocked_retreat()
            return ServoCommand(vx=0.0, wz=0.0), "blocked_retreat_timeout"
        vx = -abs(self.blocked_retreat_vx)
        return ServoCommand(vx=vx, wz=0.0), "blocked_retreat_reverse"

    def _record_birth_sector(self, now: float) -> None:
        if not self.birth_record_sector or self.fsm.state != NavState.SCANNING:
            return
        views = max(self.birth_views, 1)
        sector_span = 2.0 * math.pi / views
        yaw_accum = self.fsm.birth_scan_yaw_accumulated_rad
        idx = int(yaw_accum / sector_span)
        if idx >= views:
            return
        sector_id = f"spawn_sector_{idx:02d}"
        if any(s.get("sector_id") == sector_id for s in self.birth_sectors):
            return
        front = self.front_min_distance()
        self.birth_sectors.append(
            {
                "sector_id": sector_id,
                "yaw_robot": yaw_accum,
                "front_clearance_m": front if front is not None else 0.0,
                "target_seen": self.target_ok(self.last_target),
                "visited": True,
                "time": now,
            }
        )

    def publish_target_words(self) -> None:
        target_cfg = section(self.cfg, "target")
        words = target_cfg.get("words", self.cfg.get("target_words", []))
        if words:
            self.words_pub.publish(String(data=",".join(str(w) for w in words)))

    def decision_timer_cb(self) -> None:
        now = time.time()
        self._prune_rejected_candidates(now)
        self.step_count += 1
        target = self.resolve_target(now)
        self.update_target_memory(target, now)
        if (
            self.semantic_explore_enabled
            and self.explore_target_visible_interrupt
            and self.target_ok(target)
            and self.fsm.state in (NavState.SEARCH, NavState.LOST_RECOVERY)
        ):
            self.active_explore_goal = None
            self.observe_update_active = False
            self.explore_phase = "EXPLORE_SELECT"
            self.explore_phase_start = now
        front_min = self.front_min_distance()
        lidar_dist = self.effective_lidar_distance(target)
        obs = self.make_observation(now, target, lidar_dist, front_min=front_min)
        prev_state = self.fsm.state
        result = self.fsm.update(obs)
        self._sync_birth_scan_odom_yaw(prev_state)
        self._fsm_prev_state = result.state
        self.last_fsm_result = result
        self.last_target = target

        if prev_state == NavState.BLOCKED and result.state != NavState.BLOCKED:
            self._stop_blocked_retreat()

        if result.changed and result.state == NavState.BLOCKED:
            self.search_mem.search_turn_locked_until = 0.0
            if self.blocked_retreat_enabled:
                self._start_blocked_retreat(now)
            reject_on_blocked = (
                self.semantic_explore_enabled
                and self.active_explore_goal
                and result.reason in ("blocked", "emergency")
            )
            # In-place align should not blacklist the explore goal when front
            # clearance is near stop_distance (~0.42m indoors).
            if reject_on_blocked and result.reason == "blocked":
                if self.explore_phase in ("EXPLORE_ALIGN", "EXPLORE_BURST_PAUSE"):
                    reject_on_blocked = False
            if reject_on_blocked:
                self._reject_candidate(self.active_explore_goal, now, "blocked")
        if self.target_ok(target):
            self.search_mem.search_mode = "visual_handoff"

        if result.state in (NavState.ARRIVE_VERIFY, NavState.SUCCESS, NavState.FAILED):
            self.desired_cmd = Twist()
            self.desired_reason = f"{result.state.value.lower()}_stop"
            self.publish_point(target, result.state.value)
            self.publish_state(result.state.value, reason=self.desired_reason, target=target.to_dict(), from_control=False)
            return

        cmd, reason = self.command_for_state(result.state, target, now)
        self._record_birth_sector(now)
        self.desired_cmd = self.to_twist(cmd)
        self.desired_reason = reason
        self.publish_point(target, result.state.value)
        self.publish_state(result.state.value, reason=reason, target=target.to_dict(), from_control=False)

    def control_timer_cb(self) -> None:
        if self.fsm.state in (NavState.ARRIVE_VERIFY, NavState.SUCCESS, NavState.FAILED):
            self.publish_zero_cmd()
            self._control_tick += 1
            if self._control_tick % self._state_publish_div == 0:
                self.publish_state(self.fsm.state.value, reason=self.desired_reason, from_control=True)
            return

        self._sync_birth_scan_cmd(time.time())
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

    def _vision_pipeline_fresh(self, now: float) -> bool:
        """Treat YOLO bbox stream as vision-alive when raw image gaps are brief."""
        stamps = [t for t in (self.last_image_time, self.last_bbox_time) if t > 0.0]
        if not stamps:
            return False
        last_seen = max(stamps)
        return now - last_seen <= self.image_stale_sec

    def make_observation(
        self,
        now: float,
        target: NavTarget,
        lidar_distance: Optional[float],
        front_min: Optional[float] = None,
    ) -> NavObservation:
        image_fresh = self._vision_pipeline_fresh(now)
        scan_fresh = (not self.require_lidar) or (
            self.last_scan_time > 0 and now - self.last_scan_time <= self.scan_stale_sec
        )
        target_centered = False
        if target.u is not None:
            center_error_px = float(target.u) - self.image_width / 2.0
            target_centered = abs(center_error_px) <= self.arrive_center_px
        else:
            center_error_px = None
        target_height_ratio = self.target_height_ratio(target)
        safety_dist = lidar_distance if lidar_distance is not None else front_min
        emergency = bool(safety_dist is not None and safety_dist <= self.emergency_stop_distance)
        blocked = bool(safety_dist is not None and safety_dist <= self.hard_stop_distance)
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
            target_center_error_px=center_error_px,
            target_area_ratio=target.area_ratio,
            target_height_ratio=target_height_ratio,
            front_distance=lidar_distance,
            emergency=emergency,
            blocked=blocked,
            qwen_verified=self.mock_qwen_verify(now),
        )

    def target_height_ratio(self, target: NavTarget) -> Optional[float]:
        bbox = target.bbox
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return None
        x, y, a, b = [float(v) for v in bbox]
        del x
        height = b - y if b > y else b
        if height <= 0:
            return None
        return height / max(float(self.image_height), 1.0)

    def mock_qwen_verify(self, now: float) -> Optional[bool]:
        if self.mode != "qwen_yolo_nav" or self.fsm.state != NavState.ARRIVE_VERIFY:
            return None
        elapsed = now - (self.fsm.state_enter_time or now)
        return True if elapsed >= 0.5 else None

    def target_ok(self, target: NavTarget) -> bool:
        return bool(
            target.visible
            and not target.stale
            and target.score >= self.target_min_score
        )

    def update_target_memory(self, target: NavTarget, now: float) -> None:
        if not self.target_ok(target):
            return
        self.last_good_target = target
        self.search_mem.last_visible_time = now
        if target.u is not None:
            self.search_mem.last_target_u = float(target.u)
            self.search_mem.last_target_ex = (
                float(target.u) - self.image_width / 2.0
            ) / max(float(self.image_width), 1.0)

    def loss_age(self, now: float) -> float:
        return compute_loss_age(now, self.search_mem.last_visible_time)

    def lock_search_turn(self, turn_dir: float, now: float, mode: str) -> None:
        self.search_mem.search_turn_dir = float(turn_dir)
        self.search_mem.search_turn_locked_until = now + self.turn_lock_sec
        self.search_mem.search_mode = mode

    def pick_clearance_turn(self) -> Tuple[float, str]:
        left = self.free_space.left_clearance(self.lidar_turn_side_deg)
        right = self.free_space.right_clearance(-self.lidar_turn_side_deg)
        turn_dir, side = pick_clearance_turn_dir(
            left,
            right,
            self.lidar_turn_min_delta,
            self.search_mem.search_turn_dir,
        )
        return turn_dir, side

    def pick_memory_turn(self) -> Tuple[float, str]:
        if self.search_mem.last_target_ex is not None:
            return turn_dir_from_ex(self.search_mem.last_target_ex), "vision_memory"
        if self.lidar_fallback_on_no_memory:
            turn_dir, side = self.pick_clearance_turn()
            return turn_dir, f"lidar_clearance_{side}"
        return self.search_mem.search_turn_dir, "vision_memory_default"

    def resolve_locked_turn(self, now: float, pick_fn) -> Tuple[float, str]:
        if now < self.search_mem.search_turn_locked_until:
            return self.search_mem.search_turn_dir, self.search_mem.search_mode
        turn_dir, mode = pick_fn()
        self.lock_search_turn(turn_dir, now, mode)
        return turn_dir, mode

    def resolve_search_spin_cmd(self, now: float) -> Tuple[ServoCommand, str]:
        def pick() -> Tuple[float, str]:
            if self.search_mem.last_target_ex is not None:
                return self.pick_memory_turn()
            turn_dir, side = self.pick_clearance_turn()
            return turn_dir, f"lidar_clearance_{side}"

        turn_dir, mode = self.resolve_locked_turn(now, pick)
        vx = self._search_arc_vx()
        return ServoCommand(vx=vx, wz=turn_dir * abs(self.scan_wz)), f"{mode}_scan"

    def _search_arc_vx(self) -> float:
        """Slow forward while turning during search; zero when too close to obstacles."""
        front = self.front_min_distance()
        if front is not None and front <= self.hard_stop_distance + 0.05:
            return 0.0
        return self.search_arc_vx

    def resolve_search_cmd(self, now: float) -> Tuple[ServoCommand, str]:
        age = self.loss_age(now)
        if should_use_free_space(age, self.free_space_enabled, self.free_space_after_loss_sec):
            wp = self.free_space.get_waypoint(self.image_width, self.image_height)
            if wp.get("usable", False):
                self.search_mem.search_mode = "lidar_free_space"
                cmd = self.servo.compute_cmd(
                    {"visible": True, "u": wp.get("u"), "v": wp.get("v")}
                ).cmd
                cmd.vx = min(cmd.vx, self.free_space_vx)
                return cmd, "lidar_free_space"
        return self.resolve_search_spin_cmd(now)

    def blocked_recovery_cmd(self, target: NavTarget, now: float) -> Tuple[ServoCommand, str]:
        if self.target_ok(target) and self.track_blocked_hold_on_target:
            return ServoCommand(), "blocked_hold_target"
        wp = self.free_space.get_waypoint(self.image_width, self.image_height)
        if wp.get("usable", False):
            cmd = self.servo.compute_cmd(
                {"visible": True, "u": wp.get("u"), "v": wp.get("v")}
            ).cmd
            cmd.vx = min(cmd.vx, self.free_space_vx)
            return cmd, "blocked_free_space"
        turn_dir, side = self.pick_clearance_turn()
        return ServoCommand(vx=0.0, wz=turn_dir * abs(self.scan_wz)), f"blocked_turn_{side}"

    def _apply_front_speed_scale(self, cmd: ServoCommand, front: Optional[float]) -> ServoCommand:
        if front is not None and front < self.slow_distance and cmd.vx > 0.0:
            span = max(self.slow_distance - self.stop_distance, 1e-6)
            cmd.vx = self.servo.cfg.max_vx * (front - self.stop_distance) / span
        return cmd

    def command_for_state(self, state: NavState, target: NavTarget, now: float) -> Tuple[ServoCommand, str]:
        if state in (NavState.BOOT, NavState.WAIT_SENSORS):
            return ServoCommand(), f"{state.value.lower()}_stop"
        if state == NavState.BIRTH_WAIT:
            return ServoCommand(), "birth_wait_stop"
        if state == NavState.SCANNING:
            wz = self.birth_scan_turn_dir * abs(self.birth_scan_wz)
            return ServoCommand(vx=0.0, wz=wz), "birth_scanning"
        if state == NavState.SEARCH:
            if self.semantic_explore_enabled:
                explore_cmd = self.command_from_explore_hint(now)
                if explore_cmd is not None:
                    return explore_cmd
                hint = self.latest_explore_hint or {}
                hint_age = now - float(self.latest_explore_hint_time or 0.0)
                if hint_age < 4.0 and str(hint.get("mode", "")) == "none":
                    return ServoCommand(), "semantic_explore_waiting_candidate"
            return self.resolve_search_cmd(now)
        if state == NavState.CANDIDATE_LOCK:
            if self.target_ok(target):
                lidar_dist = self.effective_lidar_distance(target)
                result = self.servo.compute_cmd(target.to_dict())
                cmd = self._apply_front_speed_scale(result.cmd, lidar_dist)
                return cmd, f"candidate_{result.state.lower()}"
            return ServoCommand(), "candidate_lock_stop"
        if state == NavState.TRACK:
            lidar_dist = self.effective_lidar_distance(target)
            if not self.target_ok(target):
                if self.loss_age(now) <= self.lost_target_servo_sec and self.target_ok(self.last_good_target):
                    result = self.servo.compute_cmd(self.last_good_target.to_dict())
                    cmd = self._apply_front_speed_scale(result.cmd, lidar_dist)
                    cmd.vx *= self.lost_target_vx_scale
                    return cmd, "track_recent_target_servo"
                return ServoCommand(), "track_lost_wait"
            if (
                self.track_blocked_hold_on_target
                and lidar_dist is not None
                and lidar_dist <= self.hard_stop_distance
            ):
                return ServoCommand(), "track_blocked_hold"
            result = self.servo.compute_cmd(target.to_dict())
            cmd = self._apply_front_speed_scale(result.cmd, lidar_dist)
            return cmd, result.state
        if state == NavState.LOST_RECOVERY:
            if self.semantic_explore_enabled:
                explore_cmd = self.command_from_explore_hint(now)
                if explore_cmd is not None:
                    cmd, reason = explore_cmd
                    return cmd, f"lost_recovery_{reason}"
            cmd, reason = self.resolve_search_cmd(now)
            return cmd, f"lost_recovery_{reason}"
        if state == NavState.BLOCKED:
            if self.blocked_retreat_active:
                return self._blocked_retreat_cmd(now)
            if self.last_safety.get("safety_reason") == "emergency_stop":
                if self.semantic_explore_enabled and self.active_explore_goal:
                    self._reject_candidate(self.active_explore_goal, now, "emergency_stop")
                if self.blocked_retreat_enabled:
                    self._start_blocked_retreat(now)
                    return self._blocked_retreat_cmd(now)
                return ServoCommand(), "blocked_emergency_stop"
            if self.semantic_explore_enabled:
                explore_cmd = self.command_from_explore_hint(now)
                if explore_cmd is not None:
                    return explore_cmd
            return self.blocked_recovery_cmd(target, now)
        return ServoCommand(), "unhandled_stop"

    def _sync_birth_scan_cmd(self, now: float) -> None:
        """Keep birth wait/scan cmd_vel fresh at control rate (decision timer is slower)."""
        if self.fsm.state not in (NavState.BIRTH_WAIT, NavState.SCANNING):
            return
        cmd, reason = self.command_for_state(self.fsm.state, self.last_target, now)
        self.desired_cmd = self.to_twist(cmd)
        self.desired_reason = reason

    def _birth_scan_max_wz(self) -> float:
        return max(self.max_cmd_wz, abs(self.birth_scan_wz))

    def apply_safety_layer(self, raw_cmd: Twist) -> Tuple[Twist, Dict[str, Any]]:
        front_min = self.front_min_distance()
        target_dist = self.target_lidar_distance(self.last_target)
        safety_dist = self.safety_lidar_distance(self.last_target)
        scan_age = self.free_space.scan_age()
        info: Dict[str, Any] = {
            "front_distance": safety_dist,
            "front_min_distance": front_min,
            "target_distance": target_dist,
            "scan_age": scan_age,
            "raw_cmd_vx": float(raw_cmd.linear.x),
            "raw_cmd_wz": float(raw_cmd.angular.z),
            "safety_limited": False,
        }
        safe = Twist()
        birth_scanning = self.desired_reason == "birth_scanning"
        is_retreat = self.blocked_retreat_active or str(self.desired_reason).startswith(
            "blocked_retreat"
        )
        if (
            self.require_lidar
            and (scan_age is None or scan_age > self.scan_stale_sec)
            and not birth_scanning
            and not is_retreat
        ):
            info.update({"safe_cmd_vx": 0.0, "safe_cmd_wz": 0.0, "safety_reason": "stale_scan"})
            return safe, info

        vx = float(raw_cmd.linear.x)
        wz = float(raw_cmd.angular.z)

        if is_retreat:
            vx = clamp(vx, -abs(self.blocked_retreat_vx), 0.0)
            wz = 0.0
            safe.linear.x = vx
            safe.angular.z = wz
            info.update(
                {
                    "safe_cmd_vx": float(safe.linear.x),
                    "safe_cmd_wz": float(safe.angular.z),
                    "safety_reason": "blocked_retreat_pass",
                    "blocked_retreat_target_m": self.blocked_retreat_clearance_target,
                    "safety_limited": True,
                }
            )
            return safe, info

        if (
            not birth_scanning
            and front_min is not None
            and front_min < self.emergency_stop_distance
        ):
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

        if birth_scanning:
            vx = 0.0
            info["safety_limited"] = True
            reason = "birth_scan_vx_zero"
        elif front_min is not None and front_min < self.safety_stop_distance:
            vx = 0.0
            wz = clamp(wz, -self.safe_turn_wz, self.safe_turn_wz)
            info["safety_limited"] = True
            reason = "front_stop_turn_only"
        elif front_min is not None and front_min < self.slow_distance and vx > 0.0:
            vx = min(vx, self.slow_vx)
            info["safety_limited"] = True
            reason = "front_slow_vx"

        if not birth_scanning and abs(wz) > self.turn_zero_vx_wz:
            explore_forward = self.desired_reason in (
                "semantic_explore_step",
                "semantic_explore_burst_pause",
            )
            if not explore_forward:
                vx = 0.0
                info["safety_limited"] = True
                reason = "turn_zero_vx"
        elif abs(wz) > self.turn_slow_vx_wz and vx > 0.0:
            vx *= self.turn_slow_vx_scale
            info["safety_limited"] = True
            reason = "turn_slow_vx"

        safe.linear.x = clamp(vx, -self.max_cmd_vx, self.max_cmd_vx)
        max_wz = self._birth_scan_max_wz() if self.desired_reason == "birth_scanning" else self.max_cmd_wz
        safe.angular.z = clamp(wz, -max_wz, max_wz)
        info.update({"safe_cmd_vx": float(safe.linear.x), "safe_cmd_wz": float(safe.angular.z), "safety_reason": reason})
        return safe, info

    def front_min_distance(self) -> Optional[float]:
        if not self.require_lidar:
            return None
        return self.free_space.front_min_distance()

    def target_lidar_distance(self, target: NavTarget) -> Optional[float]:
        if not self.require_lidar or target.u is None:
            return None
        return self.free_space.target_distance_at_u(float(target.u), self.image_width)

    def target_lidar_distance_at_u(self, u: Optional[float]) -> Optional[float]:
        if not self.require_lidar or u is None:
            return None
        return self.free_space.target_distance_at_u(float(u), self.image_width)

    def resolve_target_u_for_lidar(self, target: NavTarget, kwargs: Optional[Dict[str, Any]] = None) -> Optional[float]:
        kwargs = kwargs or {}
        if isinstance(kwargs.get("target"), dict):
            u = kwargs["target"].get("u")
            if u is not None:
                return float(u)
        if target.u is not None:
            return float(target.u)
        if self.target_ok(self.last_good_target) and self.last_good_target.u is not None:
            return float(self.last_good_target.u)
        if self.search_mem.last_target_u is not None:
            return float(self.search_mem.last_target_u)
        return None

    def lidar_distance_bundle(
        self, target: NavTarget, kwargs: Optional[Dict[str, Any]] = None
    ) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        front_min = self.front_min_distance()
        target_dist = self.target_lidar_distance_at_u(self.resolve_target_u_for_lidar(target, kwargs))
        combined = combine_lidar_distances(front_min, target_dist)
        return front_min, target_dist, combined

    def effective_lidar_distance(self, target: NavTarget) -> Optional[float]:
        """Nav distance: min(front-sector, target-column); stable during brief vote gaps."""
        _, _, combined = self.lidar_distance_bundle(target)
        return combined

    def safety_lidar_distance(self, target: NavTarget) -> Optional[float]:
        """Conservative distance for safety: min(front sector, target column)."""
        return self.effective_lidar_distance(target)

    def front_distance(self) -> Optional[float]:
        return self.front_min_distance()

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
        target = self.last_target
        front_min, target_dist, lidar_dist = self.lidar_distance_bundle(target, kwargs)
        safety = dict(self.last_safety)
        if front_min is not None:
            safety["front_min_distance"] = front_min
        if target_dist is not None:
            safety["target_distance"] = target_dist
        if lidar_dist is not None:
            safety["front_distance"] = lidar_dist
        data = {
            "step": self.step_count,
            "mode": mode,
            "fsm_mode": self.fsm.state.value,
            "fsm_reason": result.reason if result else "init",
            "instruction": self.instruction,
            "nav_mode": self.mode,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "front_min_distance": front_min,
            "target_distance": target_dist,
            "front_distance": lidar_dist,
            "desired_reason": self.desired_reason,
            "raw_cmd_vx": float(self.desired_cmd.linear.x),
            "raw_cmd_wz": float(self.desired_cmd.angular.z),
            "safe_cmd_vx": float(self.last_safety.get("safe_cmd_vx", self.last_cmd.linear.x)),
            "safe_cmd_wz": float(self.last_safety.get("safe_cmd_wz", self.last_cmd.angular.z)),
            "safety": safety,
            "search_mode": self.search_mem.search_mode,
            "search_turn_dir": self.search_mem.search_turn_dir,
            "loss_age_sec": self.loss_age(time.time()),
            "last_target_u": self.search_mem.last_target_u,
            "last_target_ex": self.search_mem.last_target_ex,
            "birth_phase_completed": self.fsm.birth_phase_completed,
            "birth_scan_yaw_deg": math.degrees(self.fsm.birth_scan_yaw_accumulated_rad),
            "birth_scan_budget_deg": math.degrees(self.fsm._birth_scan_budget_rad()),
            "birth_scan_odom_yaw_deg": math.degrees(self._birth_odom_accum_rad),
            "birth_scan_target_deg": math.degrees(self.birth_scan_target_rad),
            "explore_phase": self.explore_phase,
            "explore_goal_traveled_m": round(self.explore_goal_traveled_m, 3),
            "explore_step_distance_m": self.explore_step_distance_m,
            "explore_goal_switch_max_distance_m": self.explore_goal_switch_max_distance_m,
            "explore_path_waypoint_idx": self.active_path_waypoint_idx,
            "explore_planned_path_len": len(self.active_planned_path),
            "explore_nav_planner": (self.active_explore_goal or {}).get("nav_planner"),
            "explore_astar_fallback": (self.active_explore_goal or {}).get("astar_fallback"),
            "explore_last_reject_reason": self.explore_last_reject_reason,
            "explore_last_reject_goal_pose": self.explore_last_reject_goal_pose,
            "blocked_retreat_active": self.blocked_retreat_active,
            "blocked_retreat_target_m": round(self.blocked_retreat_clearance_target, 3),
            "blocked_retreat_margin_m": self.blocked_retreat_margin_m,
            "time": time.time(),
        }
        if self.active_explore_goal:
            data["explore_candidate_id"] = self.active_explore_goal.get("candidate_id")
            data["explore_mode"] = self.active_explore_goal.get("mode")
            bearing, goal_dist = self._explore_goal_geometry(self.active_explore_goal)
            data["explore_goal_distance_m"] = round(goal_dist, 3)
            data["explore_goal_bearing_rad"] = round(bearing, 4)
            data["explore_geometry_trusted"] = self._explore_geometry_trusted
            data["explore_geometry_error"] = self._explore_geometry_error or None
            data["explore_goal_frame"] = self._hint_goal_frame(self.active_explore_goal)
            data["explore_control_frame"] = self.explore_control_frame
            data["explore_planned_path_frame"] = self.active_planned_path_frame
            data["explore_tf_error"] = self._explore_tf_last_error or None
        elif self.last_explore_candidate_id:
            data["explore_candidate_id"] = self.last_explore_candidate_id
        if self.failed_explore_goals:
            data["explore_failed_count"] = len(self.failed_explore_goals)
        data.update(json_safe(kwargs))
        payload = json.dumps(data, ensure_ascii=False)
        self.state_pub.publish(String(data=payload))
        if not from_control:
            self.get_logger().info(payload)

    def publish_zero_cmd(self) -> None:
        zero = Twist()
        self.desired_cmd = zero
        self.last_cmd = zero
        self.last_safety = {
            "safe_cmd_vx": 0.0,
            "safe_cmd_wz": 0.0,
            "safety_reason": "zero_cmd",
        }
        self.cmd_pub.publish(zero)

    def publish_stop(self) -> None:
        self.publish_zero_cmd()


def main() -> None:
    parser = argparse.ArgumentParser(description="Shared nav with semantic explore")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--instruction", default=None)
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    instruction = args.instruction or str(cfg.get("instruction", "find the target"))

    rclpy.init()
    node = SharedNavSemanticExplore(cfg, instruction)
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
