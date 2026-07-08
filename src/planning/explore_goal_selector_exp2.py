#!/usr/bin/env python3
"""Explore goal selector: semantic map + frontier -> /explore_goal_hint."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import textwrap
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path as PathLib
from typing import Any, Dict, List, Optional, Set, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

ROOT = PathLib(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.planning.frontier_extractor import extract_frontiers
from src.planning.grid_astar import plan_path
from src.planning.map_goal_validity import (
    collect_scanned_free_goals,
    goal_on_scanned_map,
    is_known_free,
    is_unknown_cell,
    world_to_map,
)
from src.planning.path_follower import follow_path
from src.planning.semantic_priors import (
    context_score,
    is_target_match,
    load_semantic_priors_config,
    parse_instruction_by_rules,
)
from src.vlm.qwen_text_reasoner import QwenTextReasoner

try:
    import tf2_ros
except ImportError:
    tf2_ros = None

try:
    from tf_transformations import euler_from_quaternion
except ImportError:
    euler_from_quaternion = None


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    if euler_from_quaternion is not None:
        _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
        return float(yaw)
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _section(raw: Dict[str, Any], key: str) -> Dict[str, Any]:
    block = raw.get(key, {})
    return block if isinstance(block, dict) else {}


def make_candidate_id(mode: str, x: float, y: float) -> str:
    qx = round(float(x) / 0.5)
    qy = round(float(y) / 0.5)
    return f"{mode}:{qx}:{qy}"


def load_config(path: str) -> Dict[str, Any]:
    with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


@dataclass
class ExploreCandidate:
    candidate_id: str
    mode: str
    goal_xy: Tuple[float, float]
    goal_yaw: float
    look_at: Tuple[float, float]
    semantic_score: float = 0.0
    information_gain: float = 0.0
    reachability: float = 0.7
    novelty: float = 0.5
    safety_margin: float = 0.7
    qwen_text_score: float = 0.0
    blacklist_penalty: float = 0.0
    repeated_observation_penalty: float = 0.0
    travel_cost_penalty: float = 0.0
    known_map_bonus: float = 0.0
    unknown_goal_penalty: float = 0.0
    reason: str = ""
    source: Dict[str, Any] = field(default_factory=dict)
    target_class: str = ""
    raw_goal_xy: Optional[Tuple[float, float]] = None
    projection_status: str = ""
    reject_reason: str = ""
    validation: Dict[str, Any] = field(default_factory=dict)
    sector_id: str = ""
    forced_pick: bool = False  # legacy: scan bypass force (to be phased)
    forced_below_threshold: bool = False  # 当 direction_lock 强制选低于 min_hint_score 的 active sector 最高分点时标记

    @property
    def total_score(self) -> float:
        return max(
            0.0,
            self.semantic_score
            + self.information_gain
            + self.reachability
            + self.novelty
            + self.safety_margin
            + self.qwen_text_score
            + self.known_map_bonus
            - self.blacklist_penalty
            - self.repeated_observation_penalty
            - self.travel_cost_penalty
            - self.unknown_goal_penalty,
        )


@dataclass
class ProjectionResult:
    ok: bool
    goal_xy: Optional[Tuple[float, float]] = None
    status: str = ""
    validation: Dict[str, Any] = field(default_factory=dict)
    raw_validation: Dict[str, Any] = field(default_factory=dict)
    sampled_count: int = 0
    valid_sample_count: int = 0


@dataclass
class BlacklistRegion:
    candidate_id: str
    target_class: str
    x: float
    y: float
    radius: float
    reason: str
    created_time: float
    expire_time: float


@dataclass
class TrajectoryPoint:
    x: float
    y: float
    yaw: float
    created_time: float


@dataclass
class VisitedGoal:
    candidate_id: str
    sector_id: str
    x: float
    y: float
    radius: float
    reason: str
    created_time: float
    expire_time: float


@dataclass
class SectorState:
    sector_id: str
    status: str = "pending"  # pending / active / exhausted
    no_candidate_cycles: int = 0
    visited_goals: int = 0
    failed_goals: int = 0
    last_update_time: float = 0.0

    # exp2 direction lifecycle
    active_started_time: float = 0.0
    exhausted_time: float = 0.0
    exhaust_reason: str = ""
    switch_count: int = 0


class ExploreGoalSelector(Node):
    def __init__(self, cfg: Dict[str, Any], instruction: str):
        super().__init__("explore_goal_selector")
        self.cfg = cfg
        self.instruction = instruction or str(cfg.get("instruction", "find the target"))

        explore = _section(cfg, "semantic_explore")
        frontier_cfg = _section(cfg, "frontier")
        planner_cfg = _section(cfg, "planner")
        goal_val_cfg = _section(cfg, "goal_validation")
        projection_cfg = _section(cfg, "safe_goal_projection")
        scoring = _section(cfg, "candidate_scoring")
        memory_cfg = _section(cfg, "explore_memory")
        direction_cfg = _section(cfg, "direction_lock")
        rates = _section(cfg, "rates")

        self.memory_cfg = memory_cfg
        self.explore_memory_enabled = bool(memory_cfg.get("enabled", False))
        self.explore_memory_mode = str(memory_cfg.get("mode", "visual_only"))

        self.trajectory_min_dist_m = float(memory_cfg.get("trajectory_min_dist_m", 0.08))
        self.trajectory_min_yaw_rad = math.radians(float(memory_cfg.get("trajectory_min_yaw_deg", 12.0)))
        self.max_trajectory_points = int(memory_cfg.get("max_trajectory_points", 1200))

        self.trajectory_points: List[TrajectoryPoint] = []

        self.visited_goals: List[VisitedGoal] = []

        # Direction lock configuration (plan section 8.3)
        self.direction_cfg = direction_cfg
        self.direction_lock_enabled = bool(direction_cfg.get("enabled", False))
        self.force_best_goal_in_active_sector = bool(
            direction_cfg.get(
                "force_best_goal_in_active_sector",
                direction_cfg.get("force_active_sector_goal", True),
            )
        )
        self.direction_sector_count = max(4, int(direction_cfg.get("sector_count", 8)))

        self.exhaust_no_candidate_cycles = int(direction_cfg.get("exhaust_no_candidate_cycles", 3))
        self.exhaust_failed_goals = int(direction_cfg.get("exhaust_failed_goals", 2))
        self.exhaust_visited_goals = int(direction_cfg.get("exhaust_visited_goals", 3))
        self.reset_when_all_exhausted = bool(direction_cfg.get("reset_when_all_exhausted", True))

        # exp2 strict sector selection policy
        self.sector_select_by_count = bool(direction_cfg.get("sector_select_by_count", True))
        self.sector_count_pool = str(direction_cfg.get("sector_count_pool", "raw_projectable"))
        # 耗尽/超时切换是否尊重 exhausted cooldown；默认 false，始终按候选点数量选
        self.sector_switch_respect_cooldown = bool(
            direction_cfg.get("sector_switch_respect_cooldown", False)
        )
        self.init_sector_respect_cooldown = bool(
            direction_cfg.get("init_sector_respect_cooldown", True)
        )
        self.force_active_sector_goal = bool(direction_cfg.get("force_active_sector_goal", True))
        self.allow_cross_sector_before_exhausted = bool(
            direction_cfg.get("allow_cross_sector_before_exhausted", False)
        )

        self.far_goal_priority_enabled = bool(direction_cfg.get("far_goal_priority_enabled", True))
        self.far_goal_bonus_max = float(direction_cfg.get("far_goal_bonus_max", 0.65))
        self.far_goal_bonus_norm_m = float(direction_cfg.get("far_goal_bonus_norm_m", 3.5))
        self.force_farthest_when_no_best = bool(
            direction_cfg.get(
                "force_farthest_when_no_best",
                direction_cfg.get("force_active_sector_goal", True),
            )
        )

        self.active_recovery_marker_enabled = bool(
            direction_cfg.get("active_recovery_marker_enabled", True)
        )
        self._last_sector_counts: Dict[str, Dict[str, Any]] = {}
        self._last_recovery_debug: List[Dict[str, Any]] = []
        self._last_recovery_candidates: List[ExploreCandidate] = []
        # active_sector_recovery config (under direction_lock, per corrected plan)
        recovery = direction_cfg.get("active_sector_recovery", {}) or {}
        self.active_sector_recovery_enabled = bool(recovery.get("enabled", True))
        self.expand_after_no_candidate_cycles = int(recovery.get("expand_after_no_candidate_cycles", 0))
        self.projection_radius_schedule_m = recovery.get("projection_radius_schedule_m", [0.8, 1.2, 1.6])
        self.allow_relax_unknown_gain = bool(recovery.get("allow_relax_unknown_gain", True))
        self.relaxed_min_unknown_gain_cells = int(recovery.get("relaxed_min_unknown_gain_cells", 0))
        self.max_expanded_candidates_per_tick = int(recovery.get("max_expanded_candidates_per_tick", 8))

        self.sector_states: Dict[str, SectorState] = {
            f"sector_{i:02d}": SectorState(sector_id=f"sector_{i:02d}")
            for i in range(self.direction_sector_count)
        }
        self._last_direction_reason: str = "initializing"
        self._last_valid_candidates: List[ExploreCandidate] = []
        self._logged_waiting_for_spawn = False

        self.visited_goal_reject_radius_m = float(memory_cfg.get("visited_goal_reject_radius_m", 0.45))
        self.visited_goal_penalty_radius_m = float(memory_cfg.get("visited_goal_penalty_radius_m", 0.90))
        self.visited_goal_penalty_weight = float(memory_cfg.get("visited_goal_penalty_weight", 0.45))

        self.no_target_radius_m = float(memory_cfg.get("no_target_radius_m", 0.65))
        self.no_target_ttl_sec = float(memory_cfg.get("no_target_ttl_sec", 240.0))

        topics = _section(cfg, "topics")
        target_cfg = _section(cfg, "target")
        mapping_cfg = _section(cfg, "semantic_mapping")
        frames_cfg = _section(mapping_cfg, "frames")
        mapping_topics = _section(mapping_cfg, "topics")

        self._frame_fixed = str(frames_cfg.get("fixed_frame", "map"))
        self._frame_fallback = str(frames_cfg.get("fallback_frame", "odom"))
        self._base_frame = str(frames_cfg.get("base_frame", "base_link"))
        self._pose_frame = self._frame_fixed

        self.explore_enabled = bool(explore.get("enabled", True))
        self.hint_topic = str(explore.get("hint_topic", topics.get("explore_goal_hint", "/explore_goal_hint")))
        self.state_topic = str(explore.get("state_topic", topics.get("explore_state", "/explore_state_json")))
        self.hud_topic = str(explore.get("hud_topic", topics.get("explore_hud", "/explore_hud")))
        self.hud_wrap_width = max(24, int(explore.get("hud_wrap_width", 46)))
        self.valid_sec = float(explore.get("max_hint_age_sec", 1.5))
        self.min_hint_score = float(explore.get("min_hint_score", 0.42))
        self.blacklist_radius_m = float(explore.get("blacklist_radius_m", 0.60))
        self.blacklist_ttl_sec = float(explore.get("blacklist_ttl_sec", 180.0))
        self.reselect_interval_sec = float(explore.get("reselect_interval_sec", 2.0))
        self.reselect_interval_fast_sec = float(
            explore.get("reselect_interval_fast_sec", min(1.5, self.reselect_interval_sec))
        )
        self.min_valid_candidates_for_slow_reselect = max(
            1, int(explore.get("min_valid_candidates_for_slow_reselect", 8))
        )
        self.valid_candidate_cache_ttl_sec = float(
            explore.get("valid_candidate_cache_ttl_sec", 12.0)
        )
        self.keep_goal_min_sec = float(explore.get("keep_goal_min_sec", 3.0))
        self.goal_switch_max_distance_m = float(explore.get("goal_switch_max_distance_m", 0.6))
        self.max_goal_select_distance_m = float(explore.get("max_goal_select_distance_m", 1.5))
        self.min_goal_select_distance_m = float(explore.get("min_goal_select_distance_m", 0.35))
        self.require_astar_path = bool(
            _section(cfg, "planner").get("require_astar_path", True)
        )
        self.astar_fallback_bearing = bool(
            planner_cfg.get(
                "astar_fallback_bearing",
                planner_cfg.get("require_astar_path", True),
            )
        )
        blocked_reasons = explore.get(
            "blocked_reject_reasons", ["blocked", "unsafe_front_clearance", "emergency_stop"]
        )
        self.blocked_reject_reasons = {str(r) for r in blocked_reasons}

        # exp2 direction lifecycle config (plan §4)
        self.exhaust_logic = str(direction_cfg.get("exhaust_logic", "any")).lower().strip()
        if self.exhaust_logic not in ("any", "or"):
            self.exhaust_logic = "any"

        self.max_active_sector_sec = float(direction_cfg.get("max_active_sector_sec", 180.0))
        self.exhausted_cooldown_sec = float(direction_cfg.get("exhausted_cooldown_sec", 90.0))

        self.sector_non_exhaust_reasons = {
            str(r) for r in direction_cfg.get(
                "sector_non_exhaust_reasons",
                ["blocked", "unsafe_front_clearance", "emergency_stop"],
            )
        }

        # 兼容原有 blocked_reject_reasons：只要原配置里认为是 blocked，也不计入 sector failed_goals
        self.sector_non_exhaust_reasons.update(self.blocked_reject_reasons)

        self.sector_visited_reasons = {
            str(r) for r in direction_cfg.get(
                "sector_visited_reasons",
                ["arrived_but_no_target"],
            )
        }

        self.count_unknown_reject_as_failed = bool(
            direction_cfg.get("count_unknown_reject_as_failed", True)
        )

        # active sector lifecycle state
        self.active_area_started_at: float = 0.0
        self.active_area_switch_count: int = 0
        self._last_counted_nav_event_key: str = ""

        self.switch_score_margin = float(explore.get("switch_score_margin", 0.20))
        self.switch_confirm_count = max(1, int(explore.get("switch_confirm_count", 2)))
        self.standoff_m = float(
            projection_cfg.get("frontier_standoff_m", frontier_cfg.get("observation_standoff_m", 0.65))
        )
        self.require_goal_on_known_free = bool(frontier_cfg.get("require_goal_on_known_free", True))
        self.scanned_free_candidates = bool(frontier_cfg.get("scanned_free_candidates", True))
        self.scanned_free_max_candidates = max(
            0, int(frontier_cfg.get("scanned_free_max_candidates", 10))
        )
        self.max_candidates_validate_per_tick = max(
            1, int(explore.get("max_candidates_validate_per_tick", 12))
        )
        self.max_active_sector_validate_per_tick = max(
            4,
            int(
                explore.get(
                    "max_active_sector_validate_per_tick",
                    self.max_candidates_validate_per_tick,
                )
            ),
        )
        self.max_validate_sec_per_tick = float(explore.get("max_validate_sec_per_tick", 0.4))
        viz_cfg = explore.get("visualization", {}) or {}
        self.max_raw_candidates_total = max(
            16,
            int(explore.get("max_raw_candidates_total", viz_cfg.get("max_raw_candidates_total", 64))),
        )
        self.max_raw_candidate_markers = max(
            12,
            int(viz_cfg.get("max_raw_candidate_markers", self.max_raw_candidates_total)),
        )
        self.max_candidate_goal_markers = max(
            12,
            int(viz_cfg.get("max_candidate_goal_markers", min(48, self.max_raw_candidate_markers))),
        )
        self.max_projection_debug_markers = max(
            8,
            int(viz_cfg.get("max_projection_debug_markers", 24)),
        )
        self.known_map_bonus_weight = float(scoring.get("known_map_bonus", 0.12))
        self.unknown_goal_penalty_weight = float(scoring.get("unknown_goal_penalty", 1.0))
        self.scoring_weights = scoring
        self.frontier_cfg = frontier_cfg
        self.planner_cfg = planner_cfg
        self.goal_validation_cfg = goal_val_cfg
        self.projection_cfg = projection_cfg
        self.goal_validation_enabled = bool(goal_val_cfg.get("enabled", True))
        self.goal_validation_strict = bool(goal_val_cfg.get("strict", True))
        self.require_inside_map = bool(goal_val_cfg.get("require_inside_map", True))
        self.require_known_free = bool(goal_val_cfg.get("require_known_free", True))
        self.unknown_as_obstacle = bool(goal_val_cfg.get("unknown_as_obstacle", True))
        self.validation_occupied_threshold = int(goal_val_cfg.get("occupied_threshold", 50))
        self.robot_radius_m = float(goal_val_cfg.get("robot_radius_m", 0.18))
        self.safety_margin_m = float(goal_val_cfg.get("safety_margin_m", 0.12))
        self.min_goal_clearance_m = float(goal_val_cfg.get("min_clearance_m", 0.30))
        self.min_astar_path_points = int(goal_val_cfg.get("min_astar_path_points", 2))
        self.projection_enabled = bool(projection_cfg.get("enabled", True))
        self.projection_max_radius_m = float(projection_cfg.get("max_projection_radius_m", 0.9))
        self.projection_step_m = float(projection_cfg.get("projection_step_m", 0.1))
        self.projection_prefer_nearest = bool(projection_cfg.get("prefer_nearest", True))
        self.projection_max_samples = max(
            4, int(projection_cfg.get("projection_max_samples", 24))
        )
        self.require_unknown_gain = bool(projection_cfg.get("require_unknown_gain", True))
        self.min_unknown_gain_cells = int(projection_cfg.get("min_unknown_gain_cells", 6))
        self.landmark_view_min_radius_m = float(projection_cfg.get("landmark_view_min_radius_m", 0.6))
        self.landmark_view_max_radius_m = float(projection_cfg.get("landmark_view_max_radius_m", 1.2))
        self.min_unknown_gain = float(
            explore.get("min_unknown_gain", frontier_cfg.get("min_unknown_gain", 0.08))
        )
        self.selector_hz = float(rates.get("selector_hz", 2.0))
        self.state_hz = float(rates.get("state_pub_hz", 5.0))
        self.selector_log_hz = float(rates.get("selector_log_hz", 1.0))

        self.priors_cfg = load_semantic_priors_config(cfg)
        self.parsed = parse_instruction_by_rules(
            self.instruction,
            self.priors_cfg,
            fallback_classes=[str(x) for x in target_cfg.get("classes", [])],
            fallback_words=[str(x) for x in target_cfg.get("words", [])],
        )
        self.qwen = QwenTextReasoner(_section(cfg, "qwen_text"))
        if self.qwen.enabled and self.qwen.use_for_instruction_parse:
            qwen_parsed = self.qwen.parse_instruction(self.instruction)
            if qwen_parsed.get("enabled"):
                aliases = qwen_parsed.get("target_aliases") or self.parsed.target_aliases
                self.parsed.target_aliases = [str(x) for x in aliases]
                ctx_list = qwen_parsed.get("context_objects") or []
                if isinstance(ctx_list, list):
                    self.parsed.context_objects = {str(c): 0.8 for c in ctx_list}

        self.semantic_map: Dict[str, Any] = {}
        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_bbox: Dict[str, Any] = {}
        self.latest_odom: Optional[Odometry] = None
        self.latest_pose: Optional[PoseWithCovarianceStamped] = None
        self.robot_pose: Optional[Tuple[float, float, float]] = None
        self.blacklist_regions: List[BlacklistRegion] = []
        self.observed_sectors: List[Dict[str, Any]] = []
        self._cand_counter = 0
        self._last_select_time = 0.0
        self._selected: Optional[ExploreCandidate] = None
        self._last_frontiers: List[Any] = []
        self._last_path: List[Tuple[float, float]] = []
        self._fixed_frame = self._frame_fixed
        self._last_nav_failed_count = 0
        self._last_candidates: List[ExploreCandidate] = []
        self._status_message = "initializing"
        self._selected_since = 0.0
        self._pending_switch_id: Optional[str] = None
        self._pending_switch_count = 0
        self._last_candidate_summaries: List[Dict[str, Any]] = []
        self._last_best_candidate_id: Optional[str] = None
        self._last_selection_explanation = "initializing"
        self._nav_abort_reselect = False
        self._last_nav_reject_reason = ""
        self._last_pick_stats: Dict[str, Any] = {}
        self.spawn_pose: Optional[Tuple[float, float, float]] = None
        self.active_area_id: Optional[str] = None
        self._last_valid_candidates: List[ExploreCandidate] = []
        self._last_candidate_debug: List[Dict[str, Any]] = []
        self._last_raw_count = 0
        self._last_valid_count = 0
        self._tf_debug: Dict[str, Any] = {}
        self._last_tf_error = ""
        self._pose_source = ""
        self._logged_sensors: set = set()
        self._last_logged_status = ""
        self._last_logged_hint_id: Optional[str] = None
        self._select_tick_count = 0
        self._select_busy = False
        self._sector_timeout_lock = threading.Lock()
        self._validate_budget_offset = 0
        self._valid_candidates_cache: Dict[str, Tuple[float, ExploreCandidate]] = {}

        self.pub_hint = self.create_publisher(String, self.hint_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_hud = self.create_publisher(String, self.hud_topic, 10)
        self.pub_path = self.create_publisher(
            NavPath, topics.get("explore_path", "/explore_path"), 10
        )
        self.pub_qwen = self.create_publisher(String, "/qwen_text_decision", 10)

        self.pub_candidate_markers = self.create_publisher(MarkerArray, "/explore_candidate_goals", 10)
        self.pub_selected_markers = self.create_publisher(MarkerArray, "/explore_selected_goal", 10)
        self.pub_frontier_markers = self.create_publisher(MarkerArray, "/explore_frontiers", 10)
        self.pub_selection_markers = self.create_publisher(MarkerArray, "/explore_selection_process", 10)
        self.pub_astar_markers = self.create_publisher(MarkerArray, "/explore_astar_markers", 10)
        self.pub_projection_markers = self.create_publisher(
            MarkerArray, "/explore_projection_markers", 10
        )

        self.pub_explore_trajectory = self.create_publisher(
            NavPath,
            str(memory_cfg.get("trajectory_topic", "/explore_trajectory_exp2")),
            10,
        )

        self.pub_memory_markers = self.create_publisher(
            MarkerArray,
            str(memory_cfg.get("marker_topic", "/explore_memory_markers_exp2")),
            10,
        )

        self._sensor_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = ReentrantCallbackGroup()
        # Heavy select tick must not overlap: reentrant timers caused piled-up
        # projection/A* work, 100% CPU, and frozen selector logs.
        self._select_cb_group = MutuallyExclusiveCallbackGroup()
        map_topic = str(frontier_cfg.get("map_topic", "/map"))
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "/semantic_map_json", self._on_semantic_map, 10,
            callback_group=self._sensor_cb_group,
        )
        self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, map_qos,
            callback_group=self._sensor_cb_group,
        )
        map_qos_live = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, map_qos_live,
            callback_group=self._sensor_cb_group,
        )
        self.create_subscription(
            LaserScan, "/scan_filtered", self._on_scan, qos_profile_sensor_data,
            callback_group=self._sensor_cb_group,
        )
        self.create_subscription(
            String, "/target_bbox_json", self._on_bbox, 10,
            callback_group=self._sensor_cb_group,
        )
        self.create_subscription(
            String, "/nav_state", self._on_nav_state, 10,
            callback_group=self._sensor_cb_group,
        )
        self.create_subscription(
            Odometry, mapping_topics.get("odom", "/odom"), self._on_odom, 10,
            callback_group=self._sensor_cb_group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self._on_pose, 10,
            callback_group=self._sensor_cb_group,
        )

        self.tf_buffer = None
        self.tf_listener = None
        if tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(
            1.0 / max(self.selector_hz, 0.1), self._select_tick,
            callback_group=self._select_cb_group,
        )
        self.create_timer(
            1.0 / max(self.state_hz, 0.1), self._publish_state_tick,
            callback_group=self._timer_cb_group,
        )
        self.create_timer(
            1.0 / max(self.selector_log_hz, 0.1), self._diagnostic_log_tick,
            callback_group=self._timer_cb_group,
        )
        self.get_logger().info(f"explore_goal_selector ready instruction={self.instruction!r}")
        self._log_startup_config()
        # 每次启动都保证是干净状态：轨迹、visited goal、黑名单全部清空
        self.get_logger().info(
            "EXPLORE MEMORY RESET: trajectory_points, visited_goals, blacklist_regions cleared on fresh start"
        )

    def _log_startup_config(self) -> None:
        self.get_logger().info(
            "selector config: "
            f"selector_hz={self.selector_hz} state_hz={self.state_hz} log_hz={self.selector_log_hz} "
            f"hint_topic={self.hint_topic} state_topic={self.state_topic} "
            f"tf_buffer_ok={self.tf_buffer is not None} map_frame={self._frame_fixed} "
            f"require_astar={self.require_astar_path} projection={self.projection_enabled} "
            f"min_clearance_m={self.min_goal_clearance_m} min_hint_score={self.min_hint_score}"
        )
        self.get_logger().info(
            f"active_sector_recovery enabled={self.active_sector_recovery_enabled} "
            f"expand_after={self.expand_after_no_candidate_cycles} "
            f"radii={self.projection_radius_schedule_m}"
        )

    def _log_sensor_milestones(self) -> None:
        checks = [
            ("map", self.latest_map is not None),
            ("scan", self.latest_scan is not None),
            ("odom", self.latest_odom is not None),
            ("pose", self.latest_pose is not None),
            ("semantic_map", bool(self.semantic_map)),
        ]
        for name, ready in checks:
            if not ready or name in self._logged_sensors:
                continue
            self._logged_sensors.add(name)
            extra = ""
            if name == "map" and self.latest_map is not None:
                m = self.latest_map
                extra = (
                    f" frame={m.header.frame_id}"
                    f" size={m.info.width}x{m.info.height}"
                    f" res={m.info.resolution:.3f}"
                )
            elif name == "semantic_map":
                extra = (
                    f" landmarks={len(self._confirmed_landmarks())}"
                    f" viewpoints={len(self.observed_sectors)}"
                )
            self.get_logger().info(f"sensor ready: {name}{extra}")

    def _selected_summary_str(self) -> str:
        sel = self._selected
        if sel is None:
            return "none"
        flags = ""
        if getattr(sel, "forced_below_threshold", False):
            flags += ",forced"
        if isinstance(sel.source, dict) and sel.source.get("recovery_layer") is not None:
            flags += ",inflate"
        return f"{sel.candidate_id}(s={sel.total_score:.2f}{flags})"

    def _top_reject_str(self) -> str:
        reject_stats = self._last_pick_stats.get("reject_stats", {})
        if not reject_stats:
            return ""
        top = sorted(reject_stats.items(), key=lambda kv: kv[1], reverse=True)[:2]
        return " ".join(f"{k}:{v}" for k, v in top)

    def _format_compact_log(self) -> str:
        reject = self._top_reject_str()
        line = (
            f"[SEL] t={self._select_tick_count} {self._status_message} "
            f"active={self.active_area_id or '-'}{self._active_sector_progress()} "
            f"valid={self._last_valid_count}/{self._last_raw_count} "
            f"sel={self._selected_summary_str()} path={len(self._last_path)}"
        )
        if reject:
            line += f" reject[{reject}]"
        if self._selected is None:
            # 仅在没有目标时补充位姿/地图就绪信息，便于排查
            line += (
                f" pose={self._pose_source or 'n/a'}"
                f" map={self.latest_map is not None} scan={self.latest_scan is not None}"
                f" tf={self._last_tf_error or 'ok'}"
            )
        return line

    def _format_detail_log(self) -> str:
        payload = self._build_state_payload()
        slim: Dict[str, Any] = {
            k: payload[k]
            for k in (
                "status",
                "selection_explanation",
                "map_coords_trusted",
                "tf_debug",
                "candidate_pipeline",
                "selected_candidate",
                "selected_mode",
                "selected_score",
                "num_frontiers",
                "has_map",
                "has_scan",
                "has_robot_pose",
                "pose_frame",
                "pose_source",
                "astar_path_points",
            )
            if k in payload
        }
        if self._last_candidate_debug:
            slim["candidate_debug"] = self._last_candidate_debug[:5]
        return json.dumps(slim, ensure_ascii=False)

    def _diagnostic_log_tick(self) -> None:
        try:
            self._log_sensor_milestones()
            compact = self._format_compact_log()
            status_changed = self._status_message != self._last_logged_status
            if status_changed:
                self._last_logged_status = self._status_message
                detail = self._format_detail_log()
                problem_statuses = {
                    "no_valid_safe_candidates",
                    "no_candidates_generated",
                    "no_navigable_candidate",
                    "waiting_map",
                    "waiting_robot_pose",
                    "current_goal_unsafe_cancel",
                }
                if self._status_message in problem_statuses:
                    self.get_logger().warning(f"STATUS_CHANGE {compact} detail={detail}")
                else:
                    self.get_logger().info(f"STATUS_CHANGE {compact} detail={detail}")
            else:
                self.get_logger().info(compact)
        except Exception as exc:
            self.get_logger().error(f"diagnostic_log_tick failed: {exc!r}")

    def _on_semantic_map(self, msg: String) -> None:
        try:
            self.semantic_map = json.loads(msg.data)
            self._fixed_frame = str(
                self.semantic_map.get("frame_id")
                or self.semantic_map.get("fixed_frame")
                or self._frame_fixed
            )
            vps = self.semantic_map.get("viewpoints") or []
            self.observed_sectors = []
            for vp in vps:
                self.observed_sectors.append(
                    {
                        "sector_id": vp.get("node_id", ""),
                        "yaw_map": vp.get("yaw", 0.0),
                        "seen_objects": vp.get("observed_classes", []),
                        "visited": vp.get("visited", True),
                    }
                )
        except json.JSONDecodeError:
            pass

    def _on_map(self, msg: OccupancyGrid) -> None:
        first_map = self.latest_map is None
        self.latest_map = msg
        if first_map:
            self.get_logger().info(
                f"map received frame={msg.header.frame_id} "
                f"size={msg.info.width}x{msg.info.height} res={msg.info.resolution:.3f}"
            )

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _on_bbox(self, msg: String) -> None:
        try:
            self.latest_bbox = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_bbox = {}

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_pose = msg

    def _blacklist_nav_failure(
        self,
        cand_id: str,
        goal_xy: Optional[Tuple[float, float]],
        reason: str,
        now: float,
    ) -> None:
        if goal_xy is None:
            return
        self.blacklist_regions.append(
            BlacklistRegion(
                candidate_id=cand_id,
                target_class=str(self.parsed.target_category),
                x=goal_xy[0],
                y=goal_xy[1],
                radius=self.blacklist_radius_m,
                reason=reason,
                created_time=now,
                expire_time=now + self.blacklist_ttl_sec,
            )
        )

    def _on_nav_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        reject_reason = str(state.get("explore_last_reject_reason", "") or "")
        if reject_reason and reject_reason != self._last_nav_reject_reason:
            self._last_nav_reject_reason = reject_reason
            cand_id = str(state.get("explore_candidate_id", ""))
            goal_xy = self._selected.goal_xy if self._selected else None
            if goal_xy is None:
                gp = state.get("explore_last_reject_goal_pose")
                if isinstance(gp, (list, tuple)) and len(gp) >= 2:
                    goal_xy = (float(gp[0]), float(gp[1]))
            self._blacklist_nav_failure(cand_id, goal_xy, reject_reason, now)
            if reject_reason in self.blocked_reject_reasons:
                self._nav_abort_reselect = True
                self._selected = None
                self._pending_switch_id = None
                self._pending_switch_count = 0
                self._selected_since = 0.0
                self._status_message = "nav_abort_reselect"
                self._last_selection_explanation = (
                    f"nav abort ({reject_reason}): clear current and reselect"
                )
        failed_count = int(state.get("explore_failed_count", 0) or 0)
        if failed_count <= self._last_nav_failed_count:
            return
        self._last_nav_failed_count = failed_count
        if reject_reason in self.blocked_reject_reasons:
            return
        cand_id = str(state.get("explore_candidate_id", ""))
        goal_xy = self._selected.goal_xy if self._selected else None
        if goal_xy is None:
            gp = state.get("explore_last_reject_goal_pose")
            if isinstance(gp, (list, tuple)) and len(gp) >= 2:
                goal_xy = (float(gp[0]), float(gp[1]))
        # Update sector stats for direction lock (plan 8.8)
        if goal_xy is not None:
            sector_id = self._sector_id_for_goal(goal_xy)
            st = self.sector_states.get(sector_id)
            if st is not None:
                st.last_update_time = now
                if reject_reason in self.sector_visited_reasons:
                    st.visited_goals += 1
                elif reject_reason not in self.sector_non_exhaust_reasons:
                    # blocked / unsafe / emergency 等不计入 failed_goals
                    st.failed_goals += 1
        self._blacklist_nav_failure(cand_id, goal_xy, reject_reason or "nav_observe_failed", now)

    def _note_robot_pose(self, pose: Tuple[float, float, float]) -> None:
        self.robot_pose = pose
        if self.spawn_pose is None:
            self.spawn_pose = pose
        self._update_explore_trajectory(pose)

    def _update_explore_trajectory(self, pose: Tuple[float, float, float]) -> None:
        if not self.explore_memory_enabled:
            return

        now = time.time()
        x, y, yaw = pose

        if not self.trajectory_points:
            self.trajectory_points.append(TrajectoryPoint(x=x, y=y, yaw=yaw, created_time=now))
            return

        last = self.trajectory_points[-1]
        dist = math.hypot(x - last.x, y - last.y)
        dyaw = abs(self._normalize_angle(yaw - last.yaw))

        if dist < self.trajectory_min_dist_m and dyaw < self.trajectory_min_yaw_rad:
            return

        self.trajectory_points.append(TrajectoryPoint(x=x, y=y, yaw=yaw, created_time=now))

        if len(self.trajectory_points) > self.max_trajectory_points:
            self.trajectory_points = self.trajectory_points[-self.max_trajectory_points:]

    def _publish_explore_trajectory(self) -> None:
        if not self.explore_memory_enabled:
            return

        msg = NavPath()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._fixed_frame

        for p in self.trajectory_points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(p.x)
            ps.pose.position.y = float(p.y)
            ps.pose.position.z = 0.03

            # 这里只显示 path，orientation 不关键，给单位四元数即可
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.pub_explore_trajectory.publish(msg)

    def _travel_cost_penalty(self, dist: float) -> float:
        """Direction lock 开启时把距离做成正向 bonus（负 penalty），权重加大，距离越远得分越高（用户需求：探索点距离权重增大，远更好）"""
        if getattr(self, "direction_lock_enabled", False):
            # 8m 达 0.30 bonus，鼓励优先选更远的探索点；非锁定时仍用惩罚避免过远
            return -min(0.30, dist / 8.0)
        return min(0.5, dist / 4.0)

    def _publish_memory_markers(self) -> None:
        if not self.explore_memory_enabled:
            return
        if self.spawn_pose is None:
            if not self._logged_waiting_for_spawn:
                self.get_logger().info(
                    "等待首次稳定位姿以设置 spawn_pose，方向锁扇区可视化及区域选择暂不启动"
                )
                self._logged_waiting_for_spawn = True
            return

        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        sx, sy, syaw = self.spawn_pose
        r = 4.0  # visualization radius for sectors

        # Draw 8 sector boundaries and wedges
        for i in range(self.direction_sector_count):
            angle_start = syaw + (i - 0.5) * (2 * math.pi / self.direction_sector_count)
            angle_end = syaw + (i + 0.5) * (2 * math.pi / self.direction_sector_count)
            sector_id = f"sector_{i:02d}"
            is_active = (sector_id == self.active_area_id)

            # Boundary lines (two rays from spawn)
            for ang, ns_suffix in [(angle_start, "start"), (angle_end, "end")]:
                m = Marker()
                m.header.stamp = stamp
                m.header.frame_id = self._fixed_frame
                m.ns = f"sector_bounds_{i}"
                m.id = 0 if ns_suffix == "start" else 1
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.08 if is_active else 0.03
                m.color.r = 1.0 if is_active else 0.2
                m.color.g = 0.8 if is_active else 0.6
                m.color.b = 0.0
                m.color.a = 0.9
                p0 = Point(x=sx, y=sy, z=0.05)
                p1 = Point(x=sx + r * math.cos(ang), y=sy + r * math.sin(ang), z=0.05)
                m.points = [p0, p1]
                arr.markers.append(m)

            # Text label at sector center
            mid_ang = (angle_start + angle_end) / 2
            label = Marker()
            label.header.stamp = stamp
            label.header.frame_id = self._fixed_frame
            label.ns = "sector_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.scale.z = 0.35
            label.color.r = 1.0 if is_active else 0.9
            label.color.g = 1.0 if is_active else 0.9
            label.color.b = 0.0 if is_active else 0.9
            label.color.a = 1.0
            label.pose.position.x = sx + (r * 0.65) * math.cos(mid_ang)
            label.pose.position.y = sy + (r * 0.65) * math.sin(mid_ang)
            label.pose.position.z = 0.3
            label.text = f"{sector_id}\n{'ACTIVE' if is_active else ''}"
            arr.markers.append(label)

        # Draw visited goals as small red spheres
        for j, vg in enumerate(self.visited_goals[-20:]):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self._fixed_frame
            m.ns = "visited_goals"
            m.id = j
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 0.8
            m.pose.position.x = vg.x
            m.pose.position.y = vg.y
            m.pose.position.z = 0.1
            arr.markers.append(m)

        # Decision reason: moved to /explore_hud (Foxglove fixed panel), skip 3D floating text.
        if False and self.active_area_id:
            reason_marker = Marker()
            reason_marker.header.stamp = stamp
            reason_marker.header.frame_id = self._fixed_frame
            reason_marker.ns = "direction_decision"
            reason_marker.id = 0
            reason_marker.type = Marker.TEXT_VIEW_FACING
            reason_marker.action = Marker.ADD
            reason_marker.scale.z = 0.4
            reason_marker.color.r = 1.0
            reason_marker.color.g = 1.0
            reason_marker.color.b = 0.0
            reason_marker.color.a = 1.0
            reason_marker.pose.position.x = sx
            reason_marker.pose.position.y = sy + 5.5
            reason_marker.pose.position.z = 1.2

            active_st = self.sector_states.get(self.active_area_id)
            exhaust_info = ""
            if active_st:
                exhaust_info = (
                    f"no_cand={active_st.no_candidate_cycles}/{self.exhaust_no_candidate_cycles} "
                    f"failed={active_st.failed_goals}/{self.exhaust_failed_goals} "
                    f"visited={active_st.visited_goals}/{self.exhaust_visited_goals}"
                )
            # 新增：显示 elapsed / exhaust_reason / cooldown
            elapsed = self._active_sector_elapsed_sec()
            exhaust_reason = active_st.exhaust_reason if active_st else ""
            cooldown_left = ""
            if self.active_area_id and self._is_sector_in_cooldown(self.active_area_id):
                st = self.sector_states.get(self.active_area_id)
                if st:
                    left = max(0.0, self.exhausted_cooldown_sec - (time.time() - st.exhausted_time))
                    cooldown_left = f" cooldown={left:.0f}s"
            # 显示最终选中点的扇区和距离，便于立即发现 active 与实际选择不一致的问题
            sel_sector = self._selected.sector_id if self._selected else "none"
            sel_dist = ""
            if self._selected and self.robot_pose:
                d = math.hypot(
                    self._selected.goal_xy[0] - self.robot_pose[0],
                    self._selected.goal_xy[1] - self.robot_pose[1],
                )
                sel_dist = f" | sel_dist={d:.1f}m"
            reason_marker.text = (
                f"ACTIVE: {self.active_area_id}\n"
                f"REASON: {self._last_direction_reason}\n"
                f"EXHAUST: {exhaust_info}\n"
                f"TIME: {elapsed:.0f}/{self.max_active_sector_sec:.0f}s {exhaust_reason}{cooldown_left}\n"
                f"SELECTED: {sel_sector}{sel_dist}"
            )
            arr.markers.append(reason_marker)

        self.pub_memory_markers.publish(arr)

    def _update_robot_pose(self) -> bool:
        if self.latest_map is not None:
            map_frame = self._map_frame_id()
            base_frame = self._base_frame or "base_link"
            tf_msg = self._lookup_latest_transform(map_frame, base_frame, timeout_sec=0.1)
            if tf_msg is not None:
                x = float(tf_msg.transform.translation.x)
                y = float(tf_msg.transform.translation.y)
                q = tf_msg.transform.rotation
                yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
                self._note_robot_pose((x, y, yaw))
                self._pose_frame = map_frame
                self._pose_source = "tf_map_base"
                return True

        if self.latest_odom is not None:
            msg = self.latest_odom
            q = msg.pose.pose.orientation
            yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
            self._note_robot_pose((
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                yaw,
            ))
            self._pose_frame = str(msg.header.frame_id or "odom")
            self._pose_source = "odom_topic"
            return True

        return False

    def _lookup_latest_transform(
        self, target_frame: str, source_frame: str, timeout_sec: float = 0.2
    ):
        if not target_frame or not source_frame:
            return None
        if self.tf_buffer is None:
            self._last_tf_error = f"{target_frame}<-{source_frame}: no tf_buffer"
            return None
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
        except Exception as e:
            self._last_tf_error = f"{target_frame}<-{source_frame}: {e}"
            return None

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _sector_id_from_relative_angle(self, rel_angle: float) -> str:
        """sector_00 centered at rel_angle=0, sector index increases CCW.
        This matches the marker visualization (sector_00 = forward).
        """
        sector_count = max(4, int(getattr(self, "direction_sector_count", 8)))
        width = 2.0 * math.pi / sector_count
        a = self._normalize_angle(rel_angle)
        # 加半个扇区宽度，使 rel=0 落到 sector_00
        idx = int(math.floor((a + 0.5 * width) / width)) % sector_count
        return f"sector_{idx:02d}"

    def _sector_id_for_goal(self, goal_xy: Tuple[float, float]) -> str:
        if self.spawn_pose is None:
            return "sector_unknown"
        sx, sy, syaw = self.spawn_pose
        dx = goal_xy[0] - sx
        dy = goal_xy[1] - sy
        rel = self._normalize_angle(math.atan2(dy, dx) - syaw)
        return self._sector_id_from_relative_angle(rel)

    def _sector_id_for_count(self, candidate: ExploreCandidate) -> str:
        """扇区候选点数量统计用 raw 黄点方位，与投影后 goal 无关。"""
        raw_xy = candidate.raw_goal_xy or candidate.goal_xy
        return self._sector_id_for_goal(raw_xy)

    def _sector_ids_with_raw_candidates(
        self, candidates: List[ExploreCandidate]
    ) -> List[str]:
        """有 raw 候选点的扇区列表（用于按数量切换，不按 exhausted 过滤）。"""
        return sorted({self._sector_id_for_count(c) for c in candidates})

    def _sector_id_for_candidate(self, candidate: ExploreCandidate) -> str:
        if candidate.sector_id:
            return candidate.sector_id
        return self._sector_id_for_goal(candidate.goal_xy)

    def _assign_sector_ids(self, candidates: List[ExploreCandidate]) -> None:
        for candidate in candidates:
            candidate.sector_id = self._sector_id_for_goal(candidate.goal_xy)

    def _sector_count_pool(self) -> List[ExploreCandidate]:
        """Foxglove 黄点对应的 raw 候选，用于 sector 数量统计。"""
        pool = list(self._last_candidates or [])
        if pool:
            self._assign_sector_ids(pool)
            return pool
        fallback = list(self._last_valid_candidates or [])
        if fallback:
            for candidate in fallback:
                if not candidate.sector_id:
                    candidate.sector_id = self._sector_id_for_goal(candidate.goal_xy)
        return fallback

    # --- Direction lock helpers (plan sections 8.5-8.8) ---

    def _sector_index(self, sector_id: str) -> int:
        try:
            return int(str(sector_id).split("_")[-1])
        except Exception:
            return 0

    # ========== exp2 direction lifecycle helpers (plan §5) ==========

    def _clear_current_selection_for_sector_switch(self, reason: str) -> None:
        """Clear selected candidate when active sector changes/exhausts."""
        self._selected = None
        self._pending_switch_id = None
        self._pending_switch_count = 0
        self._selected_since = 0.0
        self._last_selection_explanation = reason

    def _activate_sector(self, sector_id: str, reason: str) -> None:
        """Activate one sector and start its timer.

        All code that switches active sector should call this function instead of
        directly assigning self.active_area_id, otherwise time-limit exhaustion will
        be unreliable.
        """
        if not sector_id:
            return

        now = time.time()

        # Mark old active sector back to pending if it was not exhausted.
        if self.active_area_id and self.active_area_id in self.sector_states:
            old_st = self.sector_states[self.active_area_id]
            if old_st.status == "active":
                old_st.status = "pending"
                old_st.last_update_time = now

        self.active_area_id = sector_id
        self.active_area_started_at = now
        self.active_area_switch_count += 1
        # 切换 sector 后清掉旧 sector 的 valid 缓存，避免 valid=1 但 active 内无点
        self._valid_candidates_cache.clear()
        self._validate_budget_offset = 0
        self._last_valid_candidates = []
        self._last_valid_count = 0

        st = self.sector_states.get(sector_id)
        if st is not None:
            st.status = "active"
            st.active_started_time = now
            st.last_update_time = now
            st.switch_count += 1
            # 进入区域时，连续无候选计数清零；visited/failed 不清零，代表本轮该区域的累计状态。
            st.no_candidate_cycles = 0

        self._last_direction_reason = reason

    def _active_sector_elapsed_sec(self, now: Optional[float] = None) -> float:
        if now is None:
            now = time.time()
        if self.active_area_started_at <= 0.0:
            return 0.0
        return max(0.0, now - self.active_area_started_at)

    def _active_sector_timed_out(self, now: Optional[float] = None) -> bool:
        """纯时间判定：elapsed >= max_active_sector_sec，不依赖其它耗尽条件。"""
        if not self.direction_lock_enabled or not self.active_area_id:
            return False
        if self.max_active_sector_sec <= 0.0:
            return False
        if self.active_area_started_at <= 0.0:
            return False
        return self._active_sector_elapsed_sec(now) >= self.max_active_sector_sec

    def _force_switch_to_next_sector(
        self,
        old_sector: str,
        reason: str,
        *,
        ignore_cooldown: bool = False,
    ) -> Optional[str]:
        """轮询下一个 sector；超时强制切换时 ignore_cooldown=True 保证一定能换区。"""
        n = max(1, self.direction_sector_count)
        old_idx = self._sector_index(old_sector)
        now = time.time()
        for k in range(1, n + 1):
            next_id = f"sector_{(old_idx + k) % n:02d}"
            if next_id == old_sector:
                continue
            st = self.sector_states.get(next_id)
            if (
                not ignore_cooldown
                and st is not None
                and st.status == "exhausted"
                and self.exhausted_cooldown_sec > 0.0
                and (now - st.exhausted_time) < self.exhausted_cooldown_sec
            ):
                continue
            self._mark_sector_exhausted(old_sector, reason)
            self._activate_sector(next_id, f"force_rotate from {old_sector}: {reason}")
            self._clear_current_selection_for_sector_switch(
                f"force rotate to {next_id}: {reason}"
            )
            self._last_direction_reason = f"force_rotate to {next_id}"
            return next_id
        return None

    def _execute_active_sector_timeout_switch(self) -> bool:
        """执行强制超时切换（调用方需已持有 _sector_timeout_lock）。"""
        old_sector = self.active_area_id
        if not old_sector:
            return False

        elapsed = self._active_sector_elapsed_sec()
        reason = f"time_limit {elapsed:.1f}/{self.max_active_sector_sec:.1f}s (forced)"

        self._selected = None
        self._pending_switch_id = None
        self._pending_switch_count = 0
        self._selected_since = 0.0
        self._last_select_time = 0.0

        pool = list(self._last_candidates or [])
        switched: Optional[str] = None
        if pool and self.sector_select_by_count:
            switched = self._switch_active_sector_by_candidate_count(
                pool,
                old_sector,
                self._sector_ids_with_raw_candidates(pool),
                reason,
                respect_cooldown=False,
            )

        if not switched:
            switched = self._force_switch_to_next_sector(
                old_sector, reason, ignore_cooldown=True,
            )

        if switched:
            self._status_message = "sector_timeout_switch"
            self._last_selection_explanation = (
                f"forced sector timeout: {old_sector} -> {switched}; {reason}"
            )
            self.get_logger().info(
                f"SECTOR_TIMEOUT forced switch {old_sector} -> {switched} ({reason})"
            )
        else:
            self._status_message = "sector_timeout_no_target_sector"
            self._mark_sector_exhausted(old_sector, reason)
            self._last_selection_explanation = (
                f"forced timeout but no target sector from {old_sector}: {reason}"
            )
            self.get_logger().warn(
                f"SECTOR_TIMEOUT no target after {old_sector} ({reason})"
            )

        return bool(switched)

    def _maybe_force_active_sector_timeout(self) -> bool:
        """独立强制超时：不阻塞在 generate/validate，select_busy 时也会执行。"""
        if not self._active_sector_timed_out():
            return False
        with self._sector_timeout_lock:
            if not self._active_sector_timed_out():
                return False
            return self._execute_active_sector_timeout_switch()

    def _active_sector_progress(self) -> str:
        """active sector 的耗尽进度短串: nc/failed/visited/elapsed 相对各自阈值。"""
        st = self.sector_states.get(self.active_area_id) if self.active_area_id else None
        if st is None:
            return "-"
        parts = [
            f"nc{st.no_candidate_cycles}/{self.exhaust_no_candidate_cycles}",
            f"f{st.failed_goals}/{self.exhaust_failed_goals}",
            f"v{st.visited_goals}/{self.exhaust_visited_goals}",
        ]
        if self.max_active_sector_sec > 0:
            parts.append(f"t{self._active_sector_elapsed_sec():.0f}/{self.max_active_sector_sec:.0f}s")
        return "[" + " ".join(parts) + "]"

    def _sector_exhaust_reason(self, sector_id: str) -> str:
        """返回第一个满足的耗尽原因（OR 关系），空字符串表示未耗尽。"""
        st = self.sector_states.get(sector_id)
        if st is None:
            return ""

        reasons: List[str] = []

        if self.exhaust_no_candidate_cycles > 0 and st.no_candidate_cycles >= self.exhaust_no_candidate_cycles:
            reasons.append(
                f"no_candidate_cycles {st.no_candidate_cycles}/{self.exhaust_no_candidate_cycles}"
            )

        if self.exhaust_failed_goals > 0 and st.failed_goals >= self.exhaust_failed_goals:
            reasons.append(
                f"failed_goals {st.failed_goals}/{self.exhaust_failed_goals}"
            )

        if self.exhaust_visited_goals > 0 and st.visited_goals >= self.exhaust_visited_goals:
            reasons.append(
                f"visited_goals {st.visited_goals}/{self.exhaust_visited_goals}"
            )

        if (
            self.max_active_sector_sec > 0.0
            and sector_id == self.active_area_id
            and self.active_area_started_at > 0.0
        ):
            elapsed = self._active_sector_elapsed_sec()
            if elapsed >= self.max_active_sector_sec:
                reasons.append(
                    f"time_limit {elapsed:.1f}/{self.max_active_sector_sec:.1f}s"
                )

        if reasons:
            return " OR ".join(reasons)

        return ""

    def _mark_sector_exhausted(self, sector_id: str, reason: str, now: Optional[float] = None) -> None:
        if now is None:
            now = time.time()
        st = self.sector_states.get(sector_id)
        if st is None:
            return

        st.status = "exhausted"
        st.exhausted_time = now
        st.exhaust_reason = reason
        st.last_update_time = now

        self._last_direction_reason = f"sector exhausted: {sector_id}, reason={reason}"

    def _is_sector_in_cooldown(self, sector_id: str, now: Optional[float] = None) -> bool:
        if now is None:
            now = time.time()
        st = self.sector_states.get(sector_id)
        if st is None or st.status != "exhausted":
            return False
        if self.exhausted_cooldown_sec <= 0:
            return False
        return (now - st.exhausted_time) < self.exhausted_cooldown_sec

    def _group_candidates_by_sector(
        self, candidates: List[ExploreCandidate]
    ) -> Dict[str, List[ExploreCandidate]]:
        grouped: Dict[str, List[ExploreCandidate]] = {}
        for c in candidates:
            grouped.setdefault(self._sector_id_for_candidate(c), []).append(c)
        return grouped

    def _distance_to_robot(self, goal_xy: Tuple[float, float]) -> float:
        if self.robot_pose is None:
            return 0.0
        return math.hypot(goal_xy[0] - self.robot_pose[0], goal_xy[1] - self.robot_pose[1])

    def _candidate_pick_key(self, c: ExploreCandidate) -> Tuple[float, float, float, float]:
        dist = self._distance_to_robot(c.goal_xy)
        if self.far_goal_priority_enabled:
            # 距离远优先：主键为 dist，同距离再比 score
            return (dist, c.total_score, c.information_gain, c.safety_margin)
        return (c.total_score, dist, c.information_gain, c.safety_margin)

    def _sector_balanced_downsample_candidates(
        self,
        candidates: List[ExploreCandidate],
        max_total: int,
    ) -> List[ExploreCandidate]:
        if not candidates or max_total <= 0:
            return candidates

        for c in candidates:
            if not c.sector_id:
                c.sector_id = self._sector_id_for_goal(c.goal_xy)

        grouped = self._group_candidates_by_sector(candidates)
        per_sector_min = max(2, max_total // max(1, self.direction_sector_count))
        selected: List[ExploreCandidate] = []

        for sid in sorted(grouped.keys()):
            group = grouped[sid]
            group.sort(
                key=lambda c: (
                    self._distance_to_robot(c.goal_xy),
                    c.information_gain,
                    c.total_score,
                ),
                reverse=True,
            )
            selected.extend(group[:per_sector_min])

        if len(selected) < max_total:
            used = {c.candidate_id for c in selected}
            rest = [c for c in candidates if c.candidate_id not in used]
            rest.sort(
                key=lambda c: (
                    c.total_score,
                    self._distance_to_robot(c.goal_xy),
                    c.information_gain,
                ),
                reverse=True,
            )
            selected.extend(rest[: max_total - len(selected)])

        return selected[:max_total]

    def _build_sector_count_stats(
        self,
        candidates: List[ExploreCandidate],
    ) -> Dict[str, Dict[str, Any]]:
        stats: Dict[str, Dict[str, Any]] = {
            f"sector_{i:02d}": {
                "count": 0,
                "max_dist": 0.0,
                "avg_score": 0.0,
                "ids": [],
            }
            for i in range(self.direction_sector_count)
        }

        for c in candidates:
            sid = self._sector_id_for_count(c)

            if sid not in stats:
                continue

            d = self._distance_to_robot(c.goal_xy)
            stats[sid]["count"] += 1
            stats[sid]["max_dist"] = max(stats[sid]["max_dist"], d)
            stats[sid]["avg_score"] += float(c.total_score)
            stats[sid]["ids"].append(c.candidate_id)

        for sid, st in stats.items():
            if st["count"] > 0:
                st["avg_score"] /= float(st["count"])

        self._last_sector_counts = stats
        return stats

    def _format_sector_count_summary(self, stats: Optional[Dict[str, Dict[str, Any]]] = None) -> str:
        stats = stats or self._last_sector_counts
        parts = [
            f"{sid}:{int(st['count'])}"
            for sid, st in sorted(stats.items())
            if int(st.get("count", 0)) > 0
        ]
        return "{" + ",".join(parts) + "}" if parts else "{}"

    def _choose_sector_by_candidate_count(
        self,
        candidates: List[ExploreCandidate],
        exclude_sector: Optional[str] = None,
        allowed_sector_ids: Optional[Set[str]] = None,
        respect_cooldown: bool = True,
        now: Optional[float] = None,
    ) -> Optional[str]:
        if now is None:
            now = time.time()

        if not candidates:
            self._last_direction_reason = "candidate_count_select failed: no candidates"
            return None

        stats = self._build_sector_count_stats(candidates)

        best_sid = None
        best_key = None

        for sid, st in stats.items():
            if st["count"] <= 0:
                continue
            if exclude_sector and sid == exclude_sector:
                continue
            if allowed_sector_ids is not None and sid not in allowed_sector_ids:
                continue

            sec_state = self.sector_states.get(sid)
            if respect_cooldown and sec_state is not None and sec_state.status == "exhausted":
                if self.exhausted_cooldown_sec > 0:
                    if now - sec_state.exhausted_time < self.exhausted_cooldown_sec:
                        continue

            key = (
                int(st["count"]),
                float(st["max_dist"]),
                float(st["avg_score"]),
                -self._sector_index(sid),
            )

            if best_key is None or key > best_key:
                best_key = key
                best_sid = sid

        if best_sid:
            st = stats[best_sid]
            self._last_direction_reason = (
                f"sector_count_select {best_sid}: "
                f"count={st['count']} max_dist={st['max_dist']:.2f} avg_score={st['avg_score']:.2f}"
            )
        else:
            self._last_direction_reason = "sector_count_select failed: no eligible sector"

        return best_sid

    def _switch_active_sector_by_candidate_count(
        self,
        candidates: List[ExploreCandidate],
        old_sector: str,
        allowed_sector_ids: Optional[List[str]],
        reason: str,
        *,
        respect_cooldown: Optional[bool] = None,
    ) -> Optional[str]:
        """耗尽/超时后切换：只按候选点数量选择 sector（排除 old_sector）。"""
        if not self.sector_select_by_count:
            return None
        if respect_cooldown is None:
            respect_cooldown = self.sector_switch_respect_cooldown

        if allowed_sector_ids is None:
            allowed_sector_ids = self._sector_ids_with_raw_candidates(candidates)

        chosen = self._choose_sector_by_candidate_count(
            candidates,
            exclude_sector=old_sector,
            allowed_sector_ids=set(allowed_sector_ids) if allowed_sector_ids else None,
            respect_cooldown=respect_cooldown,
        )
        if not chosen and respect_cooldown:
            chosen = self._choose_sector_by_candidate_count(
                candidates,
                exclude_sector=old_sector,
                allowed_sector_ids=set(allowed_sector_ids) if allowed_sector_ids else None,
                respect_cooldown=False,
            )
        if chosen:
            st = self._last_sector_counts.get(chosen, {})
            self.get_logger().info(
                f"SECTOR_COUNT_SWITCH {old_sector} -> {chosen} "
                f"count={st.get('count', 0)} max_dist={st.get('max_dist', 0):.2f} "
                f"reason={reason} all_counts={self._format_sector_count_summary()}"
            )
            self._mark_sector_exhausted(old_sector, reason)
            self._activate_sector(
                chosen,
                f"candidate_count_switch from {old_sector}: {reason}",
            )
            self._clear_current_selection_for_sector_switch(
                f"candidate count switch to {chosen}"
            )
            self._last_direction_reason = (
                f"candidate_count_switch (chose {chosen} with most candidates)"
            )
            return chosen
        self._last_direction_reason = (
            f"candidate_count_switch skipped: no eligible candidate-count sector "
            f"from {old_sector}, reason={reason}"
        )
        return None

    def _maybe_init_or_switch_active_sector(
        self,
        valid_candidates: List[ExploreCandidate],
        available_sector_ids: List[str],
    ) -> None:
        """在 in_active 过滤之前统一处理：初始化 / 耗尽 / 超时 → 全部按候选点数量选 sector。"""
        if not self.sector_select_by_count:
            return

        old = self.active_area_id
        count_sectors = self._sector_ids_with_raw_candidates(valid_candidates)

        if old is None:
            chosen = self._choose_sector_by_candidate_count(
                valid_candidates,
                allowed_sector_ids=set(count_sectors or available_sector_ids),
                respect_cooldown=self.init_sector_respect_cooldown,
            )
            if chosen:
                st = self._last_sector_counts.get(chosen, {})
                self.get_logger().info(
                    f"SECTOR_COUNT_INIT -> {chosen} count={st.get('count', 0)} "
                    f"all_counts={self._format_sector_count_summary()}"
                )
                self._activate_sector(chosen, "candidate_count_init")
            return

        if not self._sector_is_exhausted(old):
            return

        exhaust_reason = self._sector_exhaust_reason(old) or "exhausted"
        self._switch_active_sector_by_candidate_count(
            valid_candidates,
            old,
            count_sectors,
            exhaust_reason,
            respect_cooldown=False,
        )

    def _sector_is_exhausted(self, sector_id: str) -> bool:
        """OR 耗尽判定：任意一个耗尽条件（无候选周期 / 失败次数 / 访问次数 / 超时）满足即返回 True。"""
        return bool(self._sector_exhaust_reason(sector_id))

    def _get_active_sector_raw_candidates(
        self, raw_candidates: List[ExploreCandidate]
    ) -> List[ExploreCandidate]:
        if not raw_candidates:
            return []
        if not self.direction_lock_enabled or not self.active_area_id:
            return list(raw_candidates)
        out: List[ExploreCandidate] = []
        for c in raw_candidates:
            if self._candidate_in_active_sector(c):
                out.append(c)
        return out

    def _project_candidate_with_overrides(
        self,
        raw_xy: Tuple[float, float],
        robot_xy: Tuple[float, float],
        max_radius_m: float,
        min_unknown_gain_cells: int,
        require_unknown_gain: bool,
        *,
        require_astar: bool = True,
        max_samples: Optional[int] = None,
    ) -> Optional[Tuple[float, float]]:
        """Local override projection without permanent mutation; uses try/finally for safety."""
        old_radius = self.projection_max_radius_m
        old_min_gain = self.min_unknown_gain_cells
        old_require = self.require_unknown_gain
        try:
            self.projection_max_radius_m = max_radius_m
            self.min_unknown_gain_cells = min_unknown_gain_cells
            self.require_unknown_gain = require_unknown_gain
            res = self.project_to_safe_free_goal_result(
                raw_xy,
                robot_xy,
                require_astar=require_astar,
                max_samples=max_samples,
            )
            if res.ok and res.goal_xy:
                return res.goal_xy
            return None
        finally:
            self.projection_max_radius_m = old_radius
            self.min_unknown_gain_cells = old_min_gain
            self.require_unknown_gain = old_require
    
    def _recover_candidates_in_active_sector(
        self,
        raw_candidates: List[ExploreCandidate],
        robot_xy: Tuple[float, float, float],
    ) -> List[ExploreCandidate]:
        self._last_recovery_debug = []
        self._last_recovery_candidates = []

        if not self.direction_lock_enabled:
            return []
        if not self.active_area_id:
            return []
        if not self.active_sector_recovery_enabled:
            return []

        robot_pos = (robot_xy[0], robot_xy[1])

        active_raw: List[ExploreCandidate] = []
        for c in raw_candidates:
            if self._candidate_in_active_sector(c):
                active_raw.append(c)

        if not active_raw:
            self._last_recovery_debug.append({
                "active_sector": self.active_area_id,
                "status": "no_raw_candidate_in_active_sector",
            })
            return []

        # 远点优先（raw 距离），分数第二
        active_raw.sort(key=self._active_sector_raw_distance_key, reverse=True)

        recovered: List[ExploreCandidate] = []
        max_try = max(1, int(self.max_expanded_candidates_per_tick))
        radius_schedule = [
            float(r) for r in self.projection_radius_schedule_m
            if float(r) > 0.0
        ]
        recovery_deadline = time.time() + max(0.35, float(self.max_validate_sec_per_tick))

        for base in active_raw[:max_try]:
            if time.time() > recovery_deadline:
                break
            raw_xy = base.raw_goal_xy or base.goal_xy

            for layer, radius_m in enumerate(radius_schedule):
                if time.time() > recovery_deadline:
                    break
                projected = self._project_candidate_with_overrides(
                    raw_xy=raw_xy,
                    robot_xy=robot_pos,
                    max_radius_m=radius_m,
                    min_unknown_gain_cells=(
                        self.relaxed_min_unknown_gain_cells
                        if self.allow_relax_unknown_gain
                        else self.min_unknown_gain_cells
                    ),
                    require_unknown_gain=False if self.allow_relax_unknown_gain else self.require_unknown_gain,
                )

                debug_item = {
                    "base_id": base.candidate_id,
                    "sector": self.active_area_id,
                    "layer": layer,
                    "radius_m": radius_m,
                    "raw_xy": [float(raw_xy[0]), float(raw_xy[1])],
                    "projected_xy": None,
                    "ok": False,
                    "reason": "",
                }

                if projected is None:
                    debug_item["reason"] = "projection_failed"
                    self._last_recovery_debug.append(debug_item)
                    continue

                cand = ExploreCandidate(
                    candidate_id=f"active_recover:{layer}:{make_candidate_id(base.mode, projected[0], projected[1])}",
                    mode="active_recovery",
                    goal_xy=projected,
                    goal_yaw=math.atan2(projected[1] - robot_pos[1], projected[0] - robot_pos[0]),
                    look_at=base.look_at,
                    semantic_score=base.semantic_score,
                    information_gain=max(base.information_gain, 0.10),
                    reachability=base.reachability,
                    novelty=base.novelty,
                    safety_margin=base.safety_margin,
                    qwen_text_score=base.qwen_text_score,
                    blacklist_penalty=base.blacklist_penalty,
                    repeated_observation_penalty=base.repeated_observation_penalty,
                    travel_cost_penalty=self._travel_cost_penalty(
                        math.hypot(projected[0] - robot_pos[0], projected[1] - robot_pos[1])
                    ),
                    known_map_bonus=base.known_map_bonus,
                    unknown_goal_penalty=0.0,
                    reason=f"active sector forced recovery layer={layer} radius={radius_m:.2f}",
                    source={
                        **base.source,
                        "recovery_layer": layer,
                        "projection_radius_m": radius_m,
                        "base_candidate_id": base.candidate_id,
                        "active_sector": self.active_area_id,
                    },
                    target_class=base.target_class,
                    raw_goal_xy=raw_xy,
                    projection_status=f"active_recovery_r{radius_m:.2f}",
                    sector_id=self.active_area_id,
                    forced_below_threshold=True,
                )

                # 必须仍然走安全验证，不能为了强制 sector 直接撞墙。人类已经够莽了，机器人别学。
                self._finalize_recovery_candidate(cand, robot_pos, layer)

                debug_item["projected_xy"] = [float(projected[0]), float(projected[1])]
                debug_item["ok"] = self._candidate_safe(cand)
                debug_item["reason"] = cand.reject_reason or cand.projection_status
                self._last_recovery_debug.append(debug_item)

                if self._candidate_safe(cand):
                    recovered.append(cand)

        recovered.sort(key=self._candidate_pick_key, reverse=True)

        self._last_recovery_candidates = recovered
        return recovered

    def _finalize_recovery_candidate(
        self,
        cand: ExploreCandidate,
        robot_xy: Tuple[float, float],
        layer: int,
    ) -> None:
        """Populate validation, projection_status, blacklist_penalty, A* path for recovery candidates.
        This ensures recovery points have the same audit info as normal candidates and do not bypass safety.
        """
        if cand is None or self.latest_map is None:
            return

        wx, wy = cand.goal_xy
        mx, my = world_to_map(self.latest_map, wx, wy)
        cfg = self._map_grid_cfg()
        on_known = is_known_free(self.latest_map, mx, my, cfg) if self.require_known_free else True
        clearance = self._clearance_at_map_xy(wx, wy)
        gain_cells = self._unknown_gain_cells_at_map_xy(wx, wy)

        # A* path (respect require_astar_path)
        path = None
        if self.require_astar_path:
            path = self._plan_candidate_path(robot_xy, cand.goal_xy)
        astar_ok = bool(path) and len(path) >= self.min_astar_path_points

        # blacklist penalty (critical: do not bypass)
        bp = self._blacklist_penalty(cand.goal_xy[0], cand.goal_xy[1], cand.target_class or "unknown")

        # build validation dict (same shape as normal candidates)
        cand.validation = {
            "ok": on_known and clearance >= self.min_goal_clearance_m and (not self.require_astar_path or astar_ok),
            "is_known_free": on_known,
            "clearance_m": round(clearance, 3),
            "unknown_gain_cells": gain_cells,
            "astar_ok": astar_ok,
            "reject_reason": None if (on_known and clearance >= self.min_goal_clearance_m) else "recovery_validation_failed",
        }

        cand.projection_status = f"recovery_layer{layer}_projected"
        cand.blacklist_penalty = bp
        cand.source = cand.source or {}
        if path:
            cand.source["planned_path"] = [[float(p[0]), float(p[1])] for p in path]
        cand.source["recovery_layer"] = layer
        cand.source["validation"] = cand.validation

    def _reproject_active_raw_candidates(
        self, raw_candidates: List[ExploreCandidate], robot_xy: Tuple[float, float]
    ) -> List[ExploreCandidate]:
        if not self.active_area_id:
            return []
        active_raw = self._get_active_sector_raw_candidates(raw_candidates)
        if not active_raw:
            return []
        recovered: List[ExploreCandidate] = []
        radii = self.projection_radius_schedule_m or [0.8, 1.2, 1.6]
        for raw_cand in active_raw[: self.max_expanded_candidates_per_tick * 2]:
            if len(recovered) >= self.max_expanded_candidates_per_tick:
                break
            raw_xy = raw_cand.goal_xy
            for r in radii:
                relaxed = self.relaxed_min_unknown_gain_cells if self.allow_relax_unknown_gain else self.min_unknown_gain_cells
                proj_xy = self._project_candidate_with_overrides(
                    raw_xy, robot_xy, float(r), relaxed, self.allow_relax_unknown_gain
                )
                if proj_xy is None:
                    continue
                # build a new candidate and validate fully (known_free/clearance/A* unchanged)
                new_cand = ExploreCandidate(
                    candidate_id=make_candidate_id("recovery_reproj", proj_xy[0], proj_xy[1]),
                    mode=raw_cand.mode or "frontier",
                    goal_xy=proj_xy,
                    goal_yaw=raw_cand.goal_yaw,
                    semantic_score=raw_cand.semantic_score,
                    information_gain=raw_cand.information_gain,
                    reachability=raw_cand.reachability,
                    novelty=raw_cand.novelty,
                    safety_margin=raw_cand.safety_margin,
                    qwen_text_score=raw_cand.qwen_text_score,
                    target_class=raw_cand.target_class,
                    reason="recovery_reprojected",
                    source={"type": "recovery", "layer": 1, "raw": list(raw_xy)},
                )
                new_cand.source["recovery_layer"] = 1
                self._finalize_recovery_candidate(new_cand, robot_xy, layer=1)
                recovered.append(new_cand)
                break
        if recovered:
            self.get_logger().info(
                f"RECOVERY inflate active={self.active_area_id} -> {len(recovered)} candidates "
                f"(radii={radii})"
            )
        return recovered

    def _mark_sector_active(self, sector_id: str, reason: str) -> None:
        """Delegate to centralized _activate_sector to keep timer consistent."""
        self._activate_sector(sector_id, reason)

    def _raw_distance_to_robot(self, candidate: ExploreCandidate) -> float:
        raw_xy = candidate.raw_goal_xy or candidate.goal_xy
        return self._distance_to_robot(raw_xy)

    def _active_sector_raw_distance_key(
        self, candidate: ExploreCandidate
    ) -> Tuple[float, float, float, float]:
        """active sector 内按 raw 黄点距离排序（远点优先）。"""
        dist = self._raw_distance_to_robot(candidate)
        return (dist, candidate.total_score, candidate.information_gain, candidate.safety_margin)

    def _farthest_raw_in_active_sector(
        self, raw_candidates: List[ExploreCandidate]
    ) -> Optional[ExploreCandidate]:
        active = [c for c in raw_candidates if self._candidate_in_active_sector(c)]
        if not active:
            return None
        active.sort(key=self._active_sector_raw_distance_key, reverse=True)
        return active[0]

    def _reset_candidate_to_raw(self, candidate: ExploreCandidate) -> Tuple[float, float]:
        raw_xy = candidate.raw_goal_xy or candidate.goal_xy
        candidate.goal_xy = raw_xy
        candidate.raw_goal_xy = raw_xy
        candidate.reject_reason = ""
        candidate.projection_status = ""
        candidate.validation = {}
        return raw_xy

    def _mark_forced_active_pick(
        self, candidate: ExploreCandidate, reason: str
    ) -> ExploreCandidate:
        candidate.forced_pick = True
        candidate.forced_below_threshold = True
        candidate.reason = f"{candidate.reason}; {reason}"
        return candidate

    def _force_accept_relaxed_explore_goal(
        self,
        candidate: ExploreCandidate,
        robot_xy: Tuple[float, float],
        deadline: float,
    ) -> Optional[ExploreCandidate]:
        """强制探索兜底：允许 unknown frontier，不做 A*，仅做基础安全校验。"""
        raw_xy = candidate.raw_goal_xy or candidate.goal_xy
        robot_pos = (robot_xy[0], robot_xy[1])

        def _apply_goal(goal_xy: Tuple[float, float], tag: str) -> ExploreCandidate:
            dist = math.hypot(goal_xy[0] - robot_pos[0], goal_xy[1] - robot_pos[1])
            candidate.goal_xy = goal_xy
            candidate.raw_goal_xy = raw_xy
            candidate.validation = {
                "ok": True,
                "reject_reason": "",
                "allow_unknown_goal": True,
                "raw_goal_xy": [raw_xy[0], raw_xy[1]],
                "projected_goal_xy": [goal_xy[0], goal_xy[1]],
            }
            candidate.reject_reason = ""
            candidate.projection_status = f"force_relaxed_{tag}"
            candidate.source = dict(candidate.source)
            candidate.source["fast_validation"] = True
            candidate.source["force_relaxed"] = True
            candidate.sector_id = self._sector_id_for_goal(raw_xy)
            candidate.travel_cost_penalty = self._travel_cost_penalty(dist)
            return self._mark_forced_active_pick(
                candidate, f"forced relaxed explore ({tag})"
            )

        for px, py in self._sample_projection_points(raw_xy):
            if time.time() >= deadline:
                break
            val = self.validate_nav_goal(
                (px, py),
                robot_pos,
                require_unknown_gain=False,
                require_astar=False,
                allow_unknown_goal=True,
            )
            if not val.get("ok"):
                continue
            tmp = ExploreCandidate(
                candidate_id=candidate.candidate_id,
                mode=candidate.mode,
                goal_xy=(px, py),
                goal_yaw=candidate.goal_yaw,
                look_at=candidate.look_at,
                semantic_score=candidate.semantic_score,
                information_gain=candidate.information_gain,
                reachability=candidate.reachability,
                novelty=candidate.novelty,
                safety_margin=candidate.safety_margin,
                reason=candidate.reason,
                source=dict(candidate.source),
                target_class=candidate.target_class,
                raw_goal_xy=raw_xy,
                sector_id=candidate.sector_id,
            )
            if not self._within_select_range(robot_pos, tmp):
                continue
            self.get_logger().info(
                f"FORCE_FARTHEST_ACTIVE ok(relaxed_snap) sector={self.active_area_id} "
                f"id={candidate.candidate_id} dist={self._raw_distance_to_robot(candidate):.2f}m"
            )
            return _apply_goal((px, py), "snap")

        val = self.validate_nav_goal(
            raw_xy,
            robot_pos,
            require_unknown_gain=False,
            require_astar=False,
            allow_unknown_goal=True,
        )
        if val.get("ok") and self._within_select_range(robot_pos, candidate):
            self.get_logger().info(
                f"FORCE_FARTHEST_ACTIVE ok(relaxed_raw) sector={self.active_area_id} "
                f"id={candidate.candidate_id} dist={self._raw_distance_to_robot(candidate):.2f}m"
            )
            return _apply_goal(raw_xy, "raw")

        return None

    def _force_validate_farthest_in_active_sector(
        self,
        raw_candidates: List[ExploreCandidate],
        robot_xy: Tuple[float, float],
    ) -> Optional[ExploreCandidate]:
        """无最优可探索点时：在 active sector 内强制选最远 raw 点并验证（含增强投影）。"""
        if not (
            self.direction_lock_enabled
            and self.active_area_id
            and self.force_active_sector_goal
            and self.force_farthest_when_no_best
        ):
            return None

        farthest = self._farthest_raw_in_active_sector(raw_candidates)
        if farthest is None:
            return None

        active_ranked = [
            c for c in raw_candidates if self._candidate_in_active_sector(c)
        ]
        active_ranked.sort(key=self._active_sector_raw_distance_key, reverse=True)
        try_limit = max(1, int(self.max_active_sector_validate_per_tick))

        deadline = time.time() + max(0.45, float(self.max_validate_sec_per_tick))

        old_radius = self.projection_max_radius_m
        old_min_gain = self.min_unknown_gain_cells
        old_require = self.require_unknown_gain
        schedule = [
            float(r) for r in (self.projection_radius_schedule_m or [])
            if float(r) > 0.0
        ]
        max_sched = max(schedule) if schedule else old_radius

        def _ok_fast(cand: ExploreCandidate, tag: str) -> ExploreCandidate:
            self.get_logger().info(
                f"FORCE_FARTHEST_ACTIVE ok({tag}) sector={self.active_area_id} "
                f"id={cand.candidate_id} dist={self._raw_distance_to_robot(cand):.2f}m fast=True"
            )
            return self._mark_forced_active_pick(cand, f"forced farthest {tag}")

        try:
            for base in active_ranked[:try_limit]:
                if time.time() >= deadline:
                    break
                dist = self._raw_distance_to_robot(base)
                self._reset_candidate_to_raw(base)

                if self._process_raw_candidate(base, robot_xy, fast=True):
                    return _ok_fast(base, "fast")

                self.projection_max_radius_m = max(old_radius, max_sched)
                if self.allow_relax_unknown_gain:
                    self.min_unknown_gain_cells = self.relaxed_min_unknown_gain_cells
                    self.require_unknown_gain = False

                self._reset_candidate_to_raw(base)
                if time.time() < deadline and self._process_raw_candidate(
                    base, robot_xy, fast=True
                ):
                    return _ok_fast(base, "relaxed_fast")

                robot_pos = (robot_xy[0], robot_xy[1])
                raw_xy = base.raw_goal_xy or base.goal_xy
                for layer, radius_m in enumerate(reversed(schedule)):
                    if time.time() >= deadline:
                        break
                    projected = self._project_candidate_with_overrides(
                        raw_xy=raw_xy,
                        robot_xy=robot_pos,
                        max_radius_m=radius_m,
                        min_unknown_gain_cells=(
                            self.relaxed_min_unknown_gain_cells
                            if self.allow_relax_unknown_gain
                            else self.min_unknown_gain_cells
                        ),
                        require_unknown_gain=(
                            False
                            if self.allow_relax_unknown_gain
                            else self.require_unknown_gain
                        ),
                        require_astar=False,
                        max_samples=8,
                    )
                    if projected is None:
                        continue

                    cand = ExploreCandidate(
                        candidate_id=(
                            f"force_far:{layer}:"
                            f"{make_candidate_id(base.mode, projected[0], projected[1])}"
                        ),
                        mode="force_farthest_active",
                        goal_xy=projected,
                        goal_yaw=math.atan2(
                            projected[1] - robot_pos[1], projected[0] - robot_pos[0]
                        ),
                        look_at=base.look_at,
                        semantic_score=base.semantic_score,
                        information_gain=max(base.information_gain, 0.10),
                        reachability=base.reachability,
                        novelty=base.novelty,
                        safety_margin=base.safety_margin,
                        qwen_text_score=base.qwen_text_score,
                        blacklist_penalty=base.blacklist_penalty,
                        repeated_observation_penalty=base.repeated_observation_penalty,
                        travel_cost_penalty=self._travel_cost_penalty(
                            math.hypot(
                                projected[0] - robot_pos[0], projected[1] - robot_pos[1]
                            )
                        ),
                        known_map_bonus=base.known_map_bonus,
                        unknown_goal_penalty=0.0,
                        reason=(
                            f"forced farthest active recovery layer={layer} "
                            f"radius={radius_m:.2f} dist={dist:.2f}m"
                        ),
                        source={
                            **base.source,
                            "force_farthest": True,
                            "base_candidate_id": base.candidate_id,
                            "recovery_layer": layer,
                            "projection_radius_m": radius_m,
                            "active_sector": self.active_area_id,
                        },
                        target_class=base.target_class,
                        raw_goal_xy=raw_xy,
                        projection_status=f"force_farthest_r{radius_m:.2f}",
                        sector_id=self.active_area_id,
                    )
                    if self._process_raw_candidate(cand, robot_pos, fast=True):
                        return _ok_fast(cand, f"recovery_r{radius_m:.2f}")

                relaxed = self._force_accept_relaxed_explore_goal(
                    base, robot_xy, deadline
                )
                if relaxed is not None:
                    return relaxed
        finally:
            self.projection_max_radius_m = old_radius
            self.min_unknown_gain_cells = old_min_gain
            self.require_unknown_gain = old_require

        fail_id = farthest.candidate_id if farthest else "n/a"
        fail_dist = self._raw_distance_to_robot(farthest) if farthest else 0.0
        self.get_logger().warn(
            f"FORCE_FARTHEST_ACTIVE failed sector={self.active_area_id} "
            f"id={fail_id} dist={fail_dist:.2f}m tried={min(len(active_ranked), try_limit)}"
        )
        return None

    def _candidate_in_active_sector(self, candidate: ExploreCandidate) -> bool:
        if not self.active_area_id:
            return True
        raw_xy = candidate.raw_goal_xy or candidate.goal_xy
        return self._sector_id_for_goal(raw_xy) == self.active_area_id

    def _filter_by_active_area(
        self, valid_candidates: List[ExploreCandidate]
    ) -> List[ExploreCandidate]:
        count_pool = self._sector_count_pool()

        # 耗尽/超时：按 raw 候选点数量切换 sector（不按 exhausted 过滤 allowed）
        if (
            self.direction_lock_enabled
            and self.sector_select_by_count
            and self.active_area_id
        ):
            if self._sector_is_exhausted(self.active_area_id):
                exhaust_reason = self._sector_exhaust_reason(self.active_area_id) or "time_limit"
                old_active = self.active_area_id
                self._mark_sector_exhausted(old_active, exhaust_reason)
                if count_pool:
                    self._switch_active_sector_by_candidate_count(
                        count_pool,
                        old_active,
                        self._sector_ids_with_raw_candidates(count_pool),
                        exhaust_reason,
                        respect_cooldown=False,
                    )

        if not valid_candidates:
            if self.direction_lock_enabled and self.active_area_id:
                st = self.sector_states.get(self.active_area_id)
                if st:
                    st.no_candidate_cycles += 1
                    st.last_update_time = time.time()
            return []

        if not self.direction_lock_enabled:
            return valid_candidates

        # 清理 exhausted 状态：如果全部 exhausted，就重新来一轮
        non_empty_sector_ids = self._sector_ids_with_raw_candidates(count_pool)
        available_sector_ids = [
            sid for sid in non_empty_sector_ids
            if not self._sector_is_exhausted(sid)
        ]

        if not available_sector_ids and self.reset_when_all_exhausted:
            for st in self.sector_states.values():
                st.status = "pending"
                st.no_candidate_cycles = 0
                st.failed_goals = 0
                st.visited_goals = 0
            available_sector_ids = non_empty_sector_ids
            self.active_area_id = None

        # 统一 sector 选择：初始化 / 耗尽 / 超时 → 全部按 raw 候选点数量
        self._maybe_init_or_switch_active_sector(count_pool, available_sector_ids)

        # 只在 active sector 内选点（按 raw 黄点所在扇区判断，与数量统计一致）
        in_active = [c for c in valid_candidates if self._candidate_in_active_sector(c)]
        if in_active:
            st = self.sector_states.get(self.active_area_id)
            if st:
                st.status = "active"
                st.no_candidate_cycles = 0
                st.last_update_time = time.time()
            return in_active

        # active sector 没候选：累加连续无候选计数（累积触发耗尽）
        if self.active_area_id:
            st = self.sector_states.get(self.active_area_id)
            if st:
                st.no_candidate_cycles += 1
                st.last_update_time = time.time()

        # 未耗尽：继续留在当前 sector，交给上层膨胀恢复
        if self.active_area_id and not self._sector_is_exhausted(self.active_area_id):
            return []

        # 已耗尽但当前 active 内无候选：按 raw 候选点数量切到别的 sector 并重试一次
        if self.active_area_id and self.sector_select_by_count:
            self._switch_active_sector_by_candidate_count(
                count_pool,
                self.active_area_id,
                self._sector_ids_with_raw_candidates(count_pool),
                self._sector_exhaust_reason(self.active_area_id) or "exhausted_no_in_active",
                respect_cooldown=False,
            )
            in_active_retry = [
                c for c in valid_candidates if self._candidate_in_active_sector(c)
            ]
            if in_active_retry:
                return in_active_retry

        return []

    def _pose_matches_map(self) -> bool:
        if self.latest_map is None:
            return False
        map_frame = str(self.latest_map.header.frame_id or self._frame_fixed)
        return self._pose_frame == map_frame

    def _scan_arrays(self) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        if self.latest_scan is None:
            return None, None
        ranges = [float(r) for r in self.latest_scan.ranges]
        angles = [
            float(self.latest_scan.angle_min + i * self.latest_scan.angle_increment)
            for i in range(len(ranges))
        ]
        return ranges, angles

    def _front_clearance(self) -> float:
        ranges, _ = self._scan_arrays()
        if not ranges:
            return 1.0
        valid = [r for r in ranges if 0.05 < r < 4.0 and not math.isinf(r)]
        return max(valid) if valid else 0.0

    def _next_cand_id(self) -> str:
        self._cand_counter += 1
        return f"cand_{self._cand_counter:03d}"

    def _confirmed_landmarks(self) -> List[Dict[str, Any]]:
        out = []
        for lm in self.semantic_map.get("landmarks") or []:
            state = str(lm.get("state", "")).lower()
            if state in ("confirmed", "active") or int(lm.get("seen_count", 0)) >= 3:
                out.append(lm)
        return out

    def _landmark_map_xy(self, lm: Dict[str, Any]) -> Tuple[float, float]:
        ox = float(lm.get("x", 0.0))
        oy = float(lm.get("y", 0.0))
        frame = str(
            lm.get("frame_id")
            or self.semantic_map.get("frame_id")
            or self._map_frame_id()
        )
        return self._to_map_xy(ox, oy, source_frame=frame)

    def _standoff_goal(
        self,
        obj_x: float,
        obj_y: float,
        robot_xy: Tuple[float, float],
        for_landmark: bool = False,
    ) -> Optional[Tuple[float, float, float]]:
        standoff = self._effective_standoff_m(for_landmark=for_landmark)
        if self.latest_map is not None:
            from src.planning.frontier_extractor import _find_standoff_goal

            res = float(self.latest_map.info.resolution)
            inflation = max(
                1, int(math.ceil(self._goal_inflation_radius_m() / res))
            )
            goal = _find_standoff_goal(
                self.latest_map,
                obj_x,
                obj_y,
                robot_xy,
                standoff,
                inflation,
                int(self.frontier_cfg.get("free_threshold", 20)),
                int(self.validation_occupied_threshold if self.goal_validation_enabled else self.frontier_cfg.get("occupied_threshold", 65)),
                int(self.frontier_cfg.get("unknown_value", -1)),
                allow_unknown_neighbors=not self.unknown_as_obstacle,
            )
            if goal:
                yaw = math.atan2(obj_y - goal[1], obj_x - goal[0])
                return goal[0], goal[1], yaw

        dx = robot_xy[0] - obj_x
        dy = robot_xy[1] - obj_y
        norm = math.hypot(dx, dy) or 1.0
        gx = obj_x + (dx / norm) * standoff
        gy = obj_y + (dy / norm) * standoff
        yaw = math.atan2(obj_y - gy, obj_x - gx)
        return gx, gy, yaw

    def _novelty_at(self, x: float, y: float) -> float:
        if not self.observed_sectors:
            return 0.8
        for sec in self.observed_sectors:
            sx = float(sec.get("x", sec.get("robot_x", 0.0)))
            sy = float(sec.get("y", sec.get("robot_y", 0.0)))
            if math.hypot(x - sx, y - sy) < 0.5:
                return 0.2
        return 0.8

    def _blacklist_penalty(self, x: float, y: float, target_class: str) -> float:
        now = time.time()
        penalty = 0.0
        self.blacklist_regions = [b for b in self.blacklist_regions if b.expire_time > now]
        for b in self.blacklist_regions:
            if math.hypot(x - b.x, y - b.y) <= b.radius:
                if b.target_class == target_class:
                    penalty = max(penalty, 1.0)
                else:
                    penalty = max(penalty, 0.4)
        return penalty

    def _target_visible(self) -> bool:
        bbox = self.latest_bbox
        if not bbox or not bbox.get("visible"):
            return False
        cls = str(bbox.get("class_name", bbox.get("class", "")))
        return is_target_match(self.parsed.target_aliases, cls) > 0

    def _map_grid_cfg(self) -> Dict[str, Any]:
        return dict(self.frontier_cfg)

    def _validation_grid_cfg(self) -> Dict[str, Any]:
        cfg = self._map_grid_cfg()
        if self.goal_validation_enabled:
            cfg = dict(cfg)
            cfg["occupied_threshold"] = self.validation_occupied_threshold
        return cfg

    def _goal_inflation_radius_m(self) -> float:
        return max(
            float(self.frontier_cfg.get("inflation_radius_m", 0.35)),
            self.robot_radius_m + self.safety_margin_m,
        )

    def _effective_standoff_m(self, for_landmark: bool = False) -> float:
        standoff = self.standoff_m
        if for_landmark:
            standoff = max(
                self.landmark_view_min_radius_m,
                min(self.landmark_view_max_radius_m, standoff),
            )
        return standoff

    def _goal_on_scanned_map(self, goal_xy: Tuple[float, float]) -> bool:
        if not self.require_goal_on_known_free:
            return True
        if self.latest_map is None or not self._map_coords_trusted():
            return False
        gx, gy = self._goal_map_xy(goal_xy)
        on_known, _, _ = goal_on_scanned_map(
            self.latest_map, gx, gy, self._map_grid_cfg()
        )
        return on_known

    def _cell_value_at_map_xy(self, wx: float, wy: float) -> Tuple[bool, int]:
        if self.latest_map is None:
            return False, 100
        cfg = self._map_grid_cfg()
        mx, my = world_to_map(self.latest_map, wx, wy)
        w = int(self.latest_map.info.width)
        h = int(self.latest_map.info.height)
        if mx < 0 or my < 0 or mx >= w or my >= h:
            return False, 100
        idx = my * w + mx
        if idx < 0 or idx >= len(self.latest_map.data):
            return False, 100
        return True, int(self.latest_map.data[idx])

    def _is_inflated_obstacle_at(self, wx: float, wy: float) -> bool:
        if self.latest_map is None:
            return True
        cfg = self._validation_grid_cfg()
        mx, my = world_to_map(self.latest_map, wx, wy)
        res = float(self.latest_map.info.resolution)
        inflation_cells = max(1, int(math.ceil(self._goal_inflation_radius_m() / res)))
        occupied_threshold = int(cfg.get("occupied_threshold", 65))
        unknown_value = int(cfg.get("unknown_value", -1))
        w = int(self.latest_map.info.width)
        h = int(self.latest_map.info.height)
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                nx, ny = mx + dx, my + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    if self.unknown_as_obstacle:
                        return True
                    continue
                val = int(self.latest_map.data[ny * w + nx])
                if val == unknown_value or val < 0:
                    if self.unknown_as_obstacle:
                        return True
                    continue
                if val >= occupied_threshold:
                    return True
        return False

    def _clearance_at_map_xy(self, wx: float, wy: float) -> float:
        if self.latest_map is None:
            return 0.0
        cfg = self._validation_grid_cfg()
        res = float(self.latest_map.info.resolution)
        mx, my = world_to_map(self.latest_map, wx, wy)
        occupied_threshold = int(cfg.get("occupied_threshold", 65))
        unknown_value = int(cfg.get("unknown_value", -1))
        w = int(self.latest_map.info.width)
        h = int(self.latest_map.info.height)
        max_cells = max(1, int(math.ceil(1.0 / res)))
        for radius in range(0, max_cells + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    nx, ny = mx + dx, my + dy
                    if nx < 0 or ny < 0 or nx >= w or ny >= h:
                        if self.unknown_as_obstacle:
                            return float(radius) * res
                        continue
                    val = int(self.latest_map.data[ny * w + nx])
                    if val == unknown_value or val < 0:
                        if self.unknown_as_obstacle:
                            return float(radius) * res
                        continue
                    if val >= occupied_threshold:
                        return float(radius) * res
        return 1.0

    def _unknown_gain_cells_at_map_xy(self, wx: float, wy: float) -> int:
        if self.latest_map is None:
            return 0
        cfg = self._validation_grid_cfg()
        mx, my = world_to_map(self.latest_map, wx, wy)
        res = float(self.latest_map.info.resolution)
        gain_radius_m = float(cfg.get("unknown_gain_radius_m", 0.6))
        radius_cells = max(1, int(math.ceil(gain_radius_m / res)))
        w = int(self.latest_map.info.width)
        h = int(self.latest_map.info.height)
        unknown_count = 0
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                nx, ny = mx + dx, my + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                if is_unknown_cell(self.latest_map, nx, ny, cfg):
                    unknown_count += 1
        return unknown_count

    def _unknown_gain_at_map_xy(self, wx: float, wy: float) -> float:
        if self.latest_map is None:
            return 0.0
        cfg = self._validation_grid_cfg()
        cells = self._unknown_gain_cells_at_map_xy(wx, wy)
        gain_radius_m = float(cfg.get("unknown_gain_radius_m", 0.6))
        res = float(self.latest_map.info.resolution)
        radius_cells = max(1, int(math.ceil(gain_radius_m / res)))
        total = (2 * radius_cells + 1) ** 2
        return cells / max(total, 1)

    def _raw_goal_is_safe_free(self, goal_xy: Tuple[float, float]) -> bool:
        if not self.goal_validation_enabled:
            return self._goal_on_scanned_map(goal_xy)
        if self.latest_map is None or not self._map_coords_trusted():
            return False
        gx, gy = self._goal_map_xy(goal_xy)
        inside, val = self._cell_value_at_map_xy(gx, gy)
        if not inside:
            return False
        cfg = self._validation_grid_cfg()
        mx, my = world_to_map(self.latest_map, gx, gy)
        if self.require_known_free and not is_known_free(self.latest_map, mx, my, cfg):
            return False
        if self._is_inflated_obstacle_at(gx, gy):
            return False
        if self._clearance_at_map_xy(gx, gy) < self.min_goal_clearance_m:
            return False
        return True

    def _sample_projection_points(
        self,
        raw_goal_xy: Tuple[float, float],
    ) -> List[Tuple[float, float]]:
        if self.latest_map is None:
            return []
        points: List[Tuple[float, float]] = []
        max_r = max(0.0, self.projection_max_radius_m)
        step = max(0.05, self.projection_step_m)
        rings = int(math.ceil(max_r / step))

        points.append(raw_goal_xy)
        for ri in range(1, rings + 1):
            r = ri * step
            n = max(16, int(math.ceil(2.0 * math.pi * r / step)))
            for k in range(n):
                a = 2.0 * math.pi * k / n
                points.append(
                    (raw_goal_xy[0] + r * math.cos(a), raw_goal_xy[1] + r * math.sin(a))
                )
        return points

    def project_to_safe_free_goal_result(
        self,
        raw_goal_xy: Tuple[float, float],
        robot_xy: Tuple[float, float],
        *,
        require_astar: bool = True,
        max_samples: Optional[int] = None,
    ) -> ProjectionResult:
        if self.latest_map is None:
            return ProjectionResult(False, status="no_map")
        if not self._map_coords_trusted():
            return ProjectionResult(False, status="map_untrusted")

        raw_validation = self.validate_nav_goal(
            raw_goal_xy,
            robot_xy,
            require_unknown_gain=False,
            require_astar=require_astar,
        )
        if raw_validation.get("ok"):
            return ProjectionResult(
                True,
                goal_xy=raw_goal_xy,
                status="raw_already_safe",
                validation=raw_validation,
                raw_validation=raw_validation,
                sampled_count=1,
                valid_sample_count=1,
            )

        if not self.projection_enabled:
            return ProjectionResult(
                False, status="projection_disabled", raw_validation=raw_validation
            )

        best = None
        sampled = 0
        valid_n = 0
        raw_x, raw_y = raw_goal_xy

        sample_cap = max_samples if max_samples is not None else self.projection_max_samples

        for p in self._sample_projection_points(raw_goal_xy):
            if sampled >= sample_cap:
                break
            sampled += 1
            cheap = self.validate_nav_goal(
                p,
                robot_xy,
                require_unknown_gain=False,
                require_astar=False,
            )
            if not cheap.get("inside_map") or not cheap.get("is_known_free"):
                continue
            if float(cheap.get("clearance_m", 0.0)) < self.min_goal_clearance_m:
                continue

            path = self._plan_candidate_path(robot_xy, p) if require_astar else []
            full = self.validate_nav_goal(
                p,
                robot_xy,
                planned_path=path if path else None,
                require_unknown_gain=False,
                require_astar=require_astar,
            )
            if not full.get("ok"):
                continue

            valid_n += 1
            repair_dist = math.hypot(p[0] - raw_x, p[1] - raw_y)
            clearance = float(full.get("clearance_m", 0.0))
            unknown_gain = float(full.get("unknown_gain", 0.0))
            path_len = len(path) if path else 999

            score = (
                1.00 * clearance
                + 0.50 * unknown_gain
                - 0.35 * repair_dist
                - 0.01 * path_len
            )
            if self.projection_prefer_nearest:
                score -= 0.20 * repair_dist

            if best is None or score > best[0]:
                best = (score, p, full)

        if best is None:
            return ProjectionResult(
                False,
                status="no_safe_projection",
                raw_validation=raw_validation,
                sampled_count=sampled,
                valid_sample_count=valid_n,
            )

        return ProjectionResult(
            True,
            goal_xy=best[1],
            status="projected_to_safe_free",
            validation=best[2],
            raw_validation=raw_validation,
            sampled_count=sampled,
            valid_sample_count=valid_n,
        )

    def project_to_safe_free_goal(
        self,
        raw_goal_xy: Tuple[float, float],
        robot_xy: Tuple[float, float],
    ) -> Optional[Tuple[float, float]]:
        res = self.project_to_safe_free_goal_result(raw_goal_xy, robot_xy)
        return res.goal_xy if res.ok else None

    def validate_nav_goal(
        self,
        goal_xy: Tuple[float, float],
        robot_xy: Tuple[float, float],
        planned_path: Optional[List[Tuple[float, float]]] = None,
        *,
        require_unknown_gain: Optional[bool] = None,
        require_astar: Optional[bool] = None,
        allow_unknown_goal: bool = False,
    ) -> Dict[str, Any]:
        if require_unknown_gain is None:
            require_unknown_gain = self.require_unknown_gain
        if require_astar is None:
            require_astar = self.require_astar_path

        result: Dict[str, Any] = {
            "inside_map": False,
            "cell_value": None,
            "is_known_free": False,
            "clearance_m": 0.0,
            "astar_ok": False,
            "unknown_gain": 0.0,
            "unknown_gain_cells": 0,
            "ok": False,
            "reject_reason": "",
        }
        if not self.goal_validation_enabled:
            if self.latest_map is None or not self._map_coords_trusted():
                result["reject_reason"] = "map_untrusted"
                return result
            if self._goal_on_scanned_map(goal_xy):
                result["ok"] = True
                result["inside_map"] = True
                result["is_known_free"] = True
            else:
                result["reject_reason"] = "unknown_cell"
            return result

        if self.latest_map is None or not self._map_coords_trusted():
            result["reject_reason"] = "map_untrusted"
            return result

        cfg = self._validation_grid_cfg()
        gx, gy = self._goal_map_xy(goal_xy)
        inside, cell_val = self._cell_value_at_map_xy(gx, gy)
        result["cell_value"] = cell_val
        if self.require_inside_map and not inside:
            result["reject_reason"] = "outside_map"
            return result
        result["inside_map"] = inside

        mx, my = world_to_map(self.latest_map, gx, gy)
        is_unknown = is_unknown_cell(self.latest_map, mx, my, cfg)
        if is_unknown and not allow_unknown_goal:
            result["reject_reason"] = "unknown_cell"
            return result

        occupied_threshold = int(cfg.get("occupied_threshold", 65))
        if cell_val >= occupied_threshold:
            result["reject_reason"] = "occupied_cell"
            return result

        if self.require_known_free and not allow_unknown_goal and not is_known_free(self.latest_map, mx, my, cfg):
            result["reject_reason"] = "unknown_cell"
            return result

        if self._is_inflated_obstacle_at(gx, gy):
            result["reject_reason"] = "inflated_obstacle"
            return result

        result["is_known_free"] = not is_unknown and is_known_free(
            self.latest_map, mx, my, cfg
        )
        result["clearance_m"] = round(self._clearance_at_map_xy(gx, gy), 3)
        min_clear = self.min_goal_clearance_m
        if allow_unknown_goal:
            min_clear = max(0.05, min_clear * 0.5)
        if result["clearance_m"] < min_clear:
            result["reject_reason"] = "low_clearance"
            return result

        gain_cells = self._unknown_gain_cells_at_map_xy(gx, gy)
        result["unknown_gain_cells"] = gain_cells
        result["unknown_gain"] = round(self._unknown_gain_at_map_xy(gx, gy), 4)
        if require_unknown_gain:
            if gain_cells < self.min_unknown_gain_cells:
                result["reject_reason"] = "low_unknown_gain"
                return result

        path = (
            planned_path
            if planned_path is not None
            else self._plan_candidate_path(robot_xy, goal_xy)
        )
        result["astar_ok"] = bool(path) and len(path) >= self.min_astar_path_points
        if require_astar and not result["astar_ok"]:
            result["reject_reason"] = "no_astar_path"
            return result

        result["ok"] = True
        return result

    def _process_raw_candidate(
        self,
        candidate: ExploreCandidate,
        robot_xy: Tuple[float, float],
        *,
        fast: bool = False,
    ) -> bool:
        raw_xy = candidate.raw_goal_xy or candidate.goal_xy
        candidate.raw_goal_xy = raw_xy

        proj = self.project_to_safe_free_goal_result(
            raw_xy,
            robot_xy,
            require_astar=not fast,
            max_samples=8 if fast else None,
        )
        candidate.projection_status = proj.status

        if not proj.ok or proj.goal_xy is None:
            candidate.reject_reason = proj.status or "no_safe_projection"
            candidate.validation = {
                "ok": False,
                "reject_reason": candidate.reject_reason,
                "raw_validation": proj.raw_validation,
                "sampled_count": proj.sampled_count,
                "valid_sample_count": proj.valid_sample_count,
            }
            return False

        candidate.goal_xy = proj.goal_xy
        candidate.validation = proj.validation

        if not self._within_select_range(robot_xy, candidate):
            candidate.reject_reason = "out_of_select_range"
            candidate.validation = {
                **candidate.validation,
                "ok": False,
                "reject_reason": "out_of_select_range",
            }
            return False

        path = self._plan_candidate_path(robot_xy, candidate.goal_xy) if not fast else []
        validation = self.validate_nav_goal(
            candidate.goal_xy,
            robot_xy,
            planned_path=path if path else None,
            require_unknown_gain=False,
            require_astar=not fast,
        )
        candidate.validation = {
            **validation,
            "raw_goal_xy": [raw_xy[0], raw_xy[1]],
            "projected_goal_xy": [candidate.goal_xy[0], candidate.goal_xy[1]],
            "projection_status": candidate.projection_status,
            "raw_validation": proj.raw_validation,
            "sampled_count": proj.sampled_count,
            "valid_sample_count": proj.valid_sample_count,
        }

        if not validation.get("ok"):
            candidate.reject_reason = str(validation.get("reject_reason", "invalid"))
            return False

        candidate.reject_reason = ""
        candidate.sector_id = self._sector_id_for_goal(candidate.goal_xy)
        dist = self._distance_to_robot(candidate.goal_xy)
        candidate.travel_cost_penalty = self._travel_cost_penalty(dist)
        candidate.source = dict(candidate.source)
        if path:
            candidate.source["planned_path"] = path
        candidate.source["raw_goal_xy"] = [raw_xy[0], raw_xy[1]]
        candidate.source["projection_status"] = candidate.projection_status
        candidate.source["validation"] = candidate.validation
        if fast:
            candidate.source["fast_validation"] = True

        try:
            candidate.information_gain = max(
                float(candidate.information_gain),
                float(validation.get("unknown_gain", 0.0)),
            )
        except Exception:
            pass

        return True

    def _effective_reselect_interval_sec(self) -> float:
        """候选点偏少或尚未选中时，加快重算频率。"""
        if self._selected is None:
            return self.reselect_interval_fast_sec
        if len(self._last_valid_candidates) < self.min_valid_candidates_for_slow_reselect:
            return self.reselect_interval_fast_sec
        return self.reselect_interval_sec

    def _rank_raw_for_validation(
        self, raw_candidates: List[ExploreCandidate]
    ) -> List[ExploreCandidate]:
        """验证预算有限时，优先验证 active sector 内的点（按 raw 扇区 + 远点优先）。"""
        if not self.direction_lock_enabled or not self.active_area_id:
            ranked = list(raw_candidates)
            ranked.sort(key=self._candidate_pick_key, reverse=True)
            return ranked
        active = [c for c in raw_candidates if self._candidate_in_active_sector(c)]
        other = [c for c in raw_candidates if not self._candidate_in_active_sector(c)]
        active.sort(key=self._active_sector_raw_distance_key, reverse=True)
        other.sort(key=self._candidate_pick_key, reverse=True)
        return active + other

    def _valid_candidates_in_active_sector(
        self, candidates: List[ExploreCandidate]
    ) -> List[ExploreCandidate]:
        if not self.direction_lock_enabled or not self.active_area_id:
            return list(candidates)
        return [c for c in candidates if self._candidate_in_active_sector(c)]

    def _prune_valid_candidate_cache(self, now: Optional[float] = None) -> None:
        if now is None:
            now = time.time()
        ttl = self.valid_candidate_cache_ttl_sec
        self._valid_candidates_cache = {
            cid: (ts, cand)
            for cid, (ts, cand) in self._valid_candidates_cache.items()
            if now - ts <= ttl
        }

    def _merge_valid_candidate_cache(
        self, newly_valid: List[ExploreCandidate], now: Optional[float] = None
    ) -> List[ExploreCandidate]:
        """跨 tick 累积已验证候选，避免每轮只保留 budget 个 valid。"""
        if now is None:
            now = time.time()
        self._prune_valid_candidate_cache(now)
        for cand in newly_valid:
            self._valid_candidates_cache[cand.candidate_id] = (now, cand)
        return [cand for _, cand in self._valid_candidates_cache.values()]

    def _build_valid_candidates(
        self,
        robot_xy: Tuple[float, float],
        raw_candidates: List[ExploreCandidate],
    ) -> List[ExploreCandidate]:
        valid: List[ExploreCandidate] = []
        reject_stats: Dict[str, int] = {}
        debug_items: List[Dict[str, Any]] = []

        self._last_raw_count = len(raw_candidates)
        ranked_raw = self._rank_raw_for_validation(raw_candidates)
        budget = self.max_candidates_validate_per_tick
        n = len(ranked_raw)
        force_active = (
            self.direction_lock_enabled
            and self.active_area_id
            and self.force_active_sector_goal
        )
        active_ranked = (
            [c for c in ranked_raw if self._candidate_in_active_sector(c)]
            if force_active
            else []
        )
        other_ranked = (
            [c for c in ranked_raw if not self._candidate_in_active_sector(c)]
            if force_active
            else ranked_raw
        )
        if force_active and active_ranked:
            # 批量验证改由 select_tick 内 _force_validate_farthest 承担，避免卡死
            to_process = []
            skipped = n
        elif n <= budget:
            to_process = ranked_raw
            skipped = 0
        else:
            start = self._validate_budget_offset % n
            to_process = [ranked_raw[(start + i) % n] for i in range(budget)]
            self._validate_budget_offset = (start + budget) % max(n, 1)
            skipped = n - budget
        if skipped > 0:
            reject_stats["skipped_budget"] = skipped

        validate_deadline = time.time() + self.max_validate_sec_per_tick
        wall_start = time.time()
        for idx, candidate in enumerate(to_process):
            if time.time() - wall_start > self.max_validate_sec_per_tick:
                reject_stats["skipped_time_budget"] = len(to_process) - idx
                break
            if time.time() > validate_deadline:
                reject_stats["skipped_time_budget"] = len(to_process) - idx
                break
            ok = self._process_raw_candidate(
                candidate, robot_xy, fast=bool(force_active)
            )

            item = self._candidate_summary(candidate, rank=0)
            item["raw_goal_xy"] = (
                list(candidate.raw_goal_xy) if candidate.raw_goal_xy else None
            )
            item["projection_status"] = candidate.projection_status
            item["reject_reason"] = candidate.reject_reason
            item["validation"] = candidate.validation
            debug_items.append(item)

            if not ok:
                reason = candidate.reject_reason or "invalid"
                reject_stats[reason] = reject_stats.get(reason, 0) + 1
                continue

            # 验证阶段不再按 min_hint_score 过滤（score 检查移至最终 pick 阶段）
            # forced_pick 候选（active sector 强制 pick）在 _pick_best_valid_candidate 中绕过该限制
            valid.append(candidate)

        merged_valid = self._merge_valid_candidate_cache(valid)
        merged_valid = self._valid_candidates_in_active_sector(merged_valid)
        self._last_valid_count = len(merged_valid)
        self._last_candidate_debug = debug_items[:80]
        self._last_pick_stats = {
            **self._last_pick_stats,
            "raw_count": len(raw_candidates),
            "valid_count": len(merged_valid),
            "valid_new_this_tick": len(valid),
            "valid_cache_size": len(self._valid_candidates_cache),
            "reject_stats": reject_stats,
            "active_area_id": self.active_area_id,
        }
        return merged_valid

    def _pick_best_valid_candidate(
        self,
        robot_xy: Tuple[float, float],
        valid_candidates: List[ExploreCandidate],
    ) -> Tuple[Optional[ExploreCandidate], List[Tuple[float, float]]]:
        return self._pick_best_from_pool(robot_xy, valid_candidates)

    def _pick_best_from_pool(
        self,
        robot_xy: Tuple[float, float],
        candidate_pool: List[ExploreCandidate],
    ) -> Tuple[Optional[ExploreCandidate], List[Tuple[float, float]]]:
        if not candidate_pool:
            return None, []
        for c in candidate_pool:
            dist = self._distance_to_robot(c.goal_xy)
            c.travel_cost_penalty = self._travel_cost_penalty(dist)
        ranked = sorted(candidate_pool, key=self._candidate_pick_key, reverse=True)
        if not ranked:
            return None, []
        best = ranked[0]
        if best.total_score < self.min_hint_score:
            if self.direction_lock_enabled and self.active_area_id and self.force_best_goal_in_active_sector:
                best.forced_below_threshold = True
                best.forced_pick = True
            else:
                self._last_selection_explanation = (
                    f"best score {best.total_score:.3f} < min_hint_score {self.min_hint_score}"
                )
                return None, []
        path = best.source.get("planned_path")
        if not isinstance(path, list) or len(path) < 2:
            path = self._plan_candidate_path(robot_xy, best.goal_xy)
        self._last_pick_stats = {
            **self._last_pick_stats,
            "selected": best.candidate_id,
            "active_area_id": self.active_area_id,
        }
        return best, list(path) if path else []

    def _apply_known_map_scoring(self, candidates: List[ExploreCandidate]) -> None:
        if self.latest_map is None or not self._map_coords_trusted():
            return
        cfg = self._map_grid_cfg()
        for cand in candidates:
            gx, gy = self._goal_map_xy(cand.goal_xy)
            on_known, is_edge, is_interior = goal_on_scanned_map(
                self.latest_map, gx, gy, cfg
            )
            if not on_known:
                cand.unknown_goal_penalty = self.unknown_goal_penalty_weight
                continue
            if cand.mode == "frontier" and is_edge:
                cand.known_map_bonus = self.known_map_bonus_weight * 0.95
                cand.reason = f"{cand.reason}; scanned frontier edge"
            elif is_edge:
                cand.known_map_bonus = self.known_map_bonus_weight * 0.85
            elif is_interior:
                cand.known_map_bonus = self.known_map_bonus_weight
                if cand.mode == "scanned_free":
                    cand.reason = "scanned known-free interior"
            cand.source = dict(cand.source)
            cand.source["on_known_free"] = True
            cand.source["is_frontier_edge"] = is_edge
            cand.source["is_interior_scanned"] = is_interior

    def _append_scanned_free_candidates(
        self,
        candidates: List[ExploreCandidate],
        robot_xy: Tuple[float, float],
        target_class: str,
    ) -> None:
        if (
            not self.scanned_free_candidates
            or not self._can_use_map_planning()
        ):
            return
        plan_robot = self._planning_robot_xy()
        if plan_robot is None:
            return
        goals = collect_scanned_free_goals(
            self.latest_map,
            plan_robot,
            self._map_grid_cfg(),
            self.min_goal_select_distance_m,
            self.max_goal_select_distance_m,
            self.scanned_free_max_candidates,
        )
        for fx, fy, yaw, is_edge in goals:
            px, py = self._to_pose_frame_xy(fx, fy)
            dist = math.hypot(px - robot_xy[0], py - robot_xy[1])
            cand = ExploreCandidate(
                candidate_id=make_candidate_id("scanned_free", px, py),
                mode="scanned_free_raw",
                goal_xy=(px, py),
                goal_yaw=yaw,
                look_at=(px, py),
                semantic_score=0.12,
                information_gain=0.45 if is_edge else 0.25,
                reachability=0.85,
                novelty=0.35 if is_edge else 0.2,
                safety_margin=min(1.0, self._front_clearance() / 2.0),
                travel_cost_penalty=self._travel_cost_penalty(dist),
                reason="scanned frontier edge" if is_edge else "scanned known-free cell",
                source={"type": "scanned_free", "is_frontier_edge": is_edge},
                target_class=target_class,
            )
            cand.blacklist_penalty = self._blacklist_penalty(px, py, target_class) * float(
                self.scoring_weights.get("blacklist_penalty", 0.3)
            )
            candidates.append(cand)

    def _append_free_space_fallback(
        self,
        candidates: List[ExploreCandidate],
        robot_xy: Tuple[float, float, float],
        ranges: List[float],
        angles: List[float],
        map_ok: bool,
    ) -> None:
        target_class = self.parsed.target_category
        scored_dirs = sorted(
            [
                (i, ranges[i], angles[i])
                for i in range(len(ranges))
                if 0.1 < ranges[i] < 4.0 and not math.isinf(ranges[i])
            ],
            key=lambda t: t[1],
            reverse=True,
        )
        for best_i, r, a in scored_dirs[:5]:
            step = min(
                max(r * 0.45, self.min_goal_select_distance_m),
                self.max_goal_select_distance_m * 0.95,
            )
            gx = robot_xy[0] + step * math.cos(robot_xy[2] + a)
            gy = robot_xy[1] + step * math.sin(robot_xy[2] + a)
            dist = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
            if not (
                self.min_goal_select_distance_m <= dist <= self.max_goal_select_distance_m
            ):
                continue
            candidates.append(
                ExploreCandidate(
                    candidate_id=make_candidate_id("free_space", gx, gy),
                    mode="free_space_raw",
                    goal_xy=(gx, gy),
                    goal_yaw=robot_xy[2] + a,
                    look_at=(gx, gy),
                    semantic_score=0.15,
                    information_gain=0.55,
                    reachability=0.75,
                    novelty=0.55,
                    safety_margin=min(1.0, r / 2.0),
                    travel_cost_penalty=self._travel_cost_penalty(dist),
                    reason="free_space fallback on scanned cell",
                    source={"type": "free_space"},
                    target_class=target_class,
                )
            )
            return

    def _generate_raw_candidates(self, robot_xy: Tuple[float, float]) -> List[ExploreCandidate]:
        candidates: List[ExploreCandidate] = []
        ranges, angles = self._scan_arrays()
        target_class = self.parsed.target_category
        map_ok = self._can_use_map_planning()
        plan_robot = self._planning_robot_xy() if map_ok else None
        self._prepare_viz_frame()

        if map_ok and plan_robot is not None:
            plan_xy = (plan_robot[0], plan_robot[1])
            context_objects = self.parsed.context_objects
            if not context_objects:
                for key in ("cup", "bottle", "backpack", "book", "plant"):
                    block = self.priors_cfg.get(key, {})
                    if block.get("context_objects"):
                        context_objects = block["context_objects"]
                        break

            for lm in self._confirmed_landmarks():
                cls = str(lm.get("class_name", ""))
                if is_target_match(self.parsed.target_aliases, cls) > 0:
                    continue
                sem = context_score(target_class, cls, self.priors_cfg)
                if sem <= 0.1:
                    continue
                ox, oy = self._landmark_map_xy(lm)
                standoff = self._standoff_goal(ox, oy, plan_xy, for_landmark=True)
                if standoff is None:
                    continue
                gx, gy, gyaw = standoff
                px, py = self._to_pose_frame_xy(gx, gy)
                look_px, look_py = self._to_pose_frame_xy(ox, oy)
                dist = math.hypot(px - robot_xy[0], py - robot_xy[1])
                landmark_id = str(lm.get("landmark_id") or "").strip()
                cand = ExploreCandidate(
                    candidate_id=f"landmark:{landmark_id}" if landmark_id else make_candidate_id("landmark", ox, oy),
                    mode="context_landmark_raw",
                    goal_xy=(px, py),
                    goal_yaw=gyaw,
                    look_at=(look_px, look_py),
                    semantic_score=sem,
                    information_gain=0.5,
                    reachability=0.75,
                    novelty=self._novelty_at(gx, gy),
                    safety_margin=min(1.0, self._front_clearance() / 2.0),
                    travel_cost_penalty=self._travel_cost_penalty(dist),
                    reason=f"near {cls}, partially unexplored",
                    source={"type": "semantic_context", "landmark_id": lm.get("landmark_id"), "class_name": cls},
                    target_class=target_class,
                )
                cand.blacklist_penalty = self._blacklist_penalty(px, py, target_class) * float(
                    self.scoring_weights.get("blacklist_penalty", 0.3)
                )
                candidates.append(cand)

        if map_ok and plan_robot is not None and bool(self.frontier_cfg.get("enabled", True)):
            plan_xy = (plan_robot[0], plan_robot[1])
            frontier_cfg_run = dict(self.frontier_cfg)
            frontier_cfg_run["allow_unknown_neighbors"] = not self.unknown_as_obstacle
            frontiers = extract_frontiers(
                self.latest_map, plan_xy, frontier_cfg_run, ranges, angles, plan_robot[2]
            )
            self._last_frontiers = frontiers
            for fg in frontiers:
                px, py = self._to_pose_frame_xy(fg.goal_xy[0], fg.goal_xy[1])
                lx, ly = self._to_pose_frame_xy(fg.cluster_center[0], fg.cluster_center[1])
                cand = ExploreCandidate(
                    candidate_id=make_candidate_id("frontier", px, py),
                    mode="frontier_raw",
                    goal_xy=(px, py),
                    goal_yaw=fg.goal_yaw,
                    look_at=(lx, ly),
                    semantic_score=0.1,
                    information_gain=fg.unknown_gain,
                    reachability=fg.reachability,
                    novelty=0.7,
                    safety_margin=min(1.0, fg.reachability),
                    travel_cost_penalty=self._travel_cost_penalty(fg.distance_m),
                    reason="frontier scanned edge",
                    source={"type": "frontier", "frontier_id": fg.frontier_id},
                    target_class=target_class,
                )
                cand.blacklist_penalty = self._blacklist_penalty(px, py, target_class) * float(
                    self.scoring_weights.get("blacklist_penalty", 0.3)
                )
                candidates.append(cand)

        if map_ok:
            self._append_scanned_free_candidates(candidates, robot_xy, target_class)

        w = self.scoring_weights
        for c in candidates:
            c.semantic_score *= float(w.get("semantic_score", 0.25))
            c.information_gain *= float(w.get("information_gain", 0.25))
            c.reachability *= float(w.get("reachability", 0.20))
            c.novelty *= float(w.get("novelty", 0.15))
            c.safety_margin *= float(w.get("safety_margin", 0.10))

        self._apply_known_map_scoring(candidates)

        if ranges and angles:
            self._append_free_space_fallback(
                candidates, robot_xy, ranges, angles, map_ok
            )

        for c in candidates:
            if c.raw_goal_xy is None:
                c.raw_goal_xy = c.goal_xy

        candidates = self._sector_balanced_downsample_candidates(
            candidates,
            max_total=self.max_raw_candidates_total,
        )
        return candidates

    def _generate_candidates(self, robot_xy: Tuple[float, float]) -> List[ExploreCandidate]:
        return self._generate_raw_candidates(robot_xy)

    def _astar_cfg(self) -> Dict[str, Any]:
        cfg = dict(self.frontier_cfg)
        cfg["allow_unknown"] = bool(self.planner_cfg.get("allow_unknown", False))
        return cfg

    def _goal_distance_m(self, robot_xy: Tuple[float, float], goal_xy: Tuple[float, float]) -> float:
        return math.hypot(goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1])

    def _within_select_range(self, robot_xy: Tuple[float, float], candidate: ExploreCandidate) -> bool:
        dist = self._goal_distance_m(robot_xy, candidate.goal_xy)
        return self.min_goal_select_distance_m <= dist <= self.max_goal_select_distance_m

    def _plan_candidate_path(
        self, robot_xy: Tuple[float, float], goal_xy: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        if not bool(self.planner_cfg.get("astar_enabled", False)) or self.latest_map is None:
            return []
        if not self._map_coords_trusted():
            return []
        start = self._to_map_xy(robot_xy[0], robot_xy[1])
        goal = self._goal_map_xy(goal_xy)
        path_map = plan_path(
            self.latest_map,
            start,
            goal,
            self._astar_cfg(),
        )
        if not path_map:
            return []
        return [self._to_pose_frame_xy(px, py) for px, py in path_map]

    def _rank_candidates(self, candidates: List[ExploreCandidate]) -> List[ExploreCandidate]:
        if self.qwen.enabled:
            summary = {
                "instruction": self.instruction,
                "target_aliases": self.parsed.target_aliases,
                "num_landmarks": len(self._confirmed_landmarks()),
            }
            cand_summaries = [
                {
                    "id": c.candidate_id,
                    "type": c.mode,
                    "semantic": c.source.get("class_name", ""),
                    "unknown_gain": c.information_gain,
                    "safe": c.safety_margin > 0.3,
                }
                for c in candidates[:5]
            ]
            qwen_result = self.qwen.rerank_candidates(self.instruction, summary, cand_summaries)
            self.pub_qwen.publish(String(data=json.dumps(qwen_result, ensure_ascii=False)))
            best_id = qwen_result.get("best_candidate_id")
            if best_id and qwen_result.get("enabled"):
                for c in candidates:
                    if c.candidate_id == best_id:
                        c.qwen_text_score = float(qwen_result.get("score", self.qwen.weight))
        return sorted(
            [c for c in candidates if self._candidate_safe(c)],
            key=lambda c: c.total_score,
            reverse=True,
        )

    def _pick_navigable_candidate(
        self, robot_xy: Tuple[float, float], candidates: List[ExploreCandidate]
    ) -> Tuple[Optional[ExploreCandidate], List[Tuple[float, float]]]:
        valid = self._build_valid_candidates(robot_xy, candidates)
        self._last_valid_candidates = valid
        return None, []

    def _score_candidates(self, candidates: List[ExploreCandidate]) -> Optional[ExploreCandidate]:
        ranked = self._rank_candidates(candidates)
        if not ranked:
            return None
        return ranked[0] if ranked[0].total_score >= self.min_hint_score else None

    def _candidate_summary(self, candidate: ExploreCandidate, rank: int) -> Dict[str, Any]:
        dist_m = None
        if self.robot_pose is not None:
            dist_m = round(
                self._goal_distance_m(self.robot_pose, candidate.goal_xy), 3
            )
        return {
            "rank": rank,
            "candidate_id": candidate.candidate_id,
            "mode": candidate.mode,
            "score": round(candidate.total_score, 3),
            "safe": self._candidate_safe(candidate),
            "dist_m": dist_m,
            "in_range": (
                self._within_select_range(self.robot_pose, candidate)
                if self.robot_pose is not None
                else None
            ),
            "goal_xy": [round(candidate.goal_xy[0], 3), round(candidate.goal_xy[1], 3)],
            "look_at": [round(candidate.look_at[0], 3), round(candidate.look_at[1], 3)],
            "reason": candidate.reason,
            "components": {
                "semantic": round(candidate.semantic_score, 3),
                "info_gain": round(candidate.information_gain, 3),
                "reach": round(candidate.reachability, 3),
                "novelty": round(candidate.novelty, 3),
                "safety": round(candidate.safety_margin, 3),
                "blacklist": round(candidate.blacklist_penalty, 3),
                "unknown": round(candidate.unknown_goal_penalty, 3),
                "travel_cost": round(candidate.travel_cost_penalty, 3),
                "known_map": round(candidate.known_map_bonus, 3),
            },
            "source": candidate.source,
            "reject_reason": candidate.reject_reason,
            "validation": candidate.validation,
            "raw_goal_xy": (
                [round(candidate.raw_goal_xy[0], 3), round(candidate.raw_goal_xy[1], 3)]
                if candidate.raw_goal_xy is not None
                else None
            ),
            "projection_status": candidate.projection_status,
            "sector_id": candidate.sector_id,
        }

    def _update_candidate_debug(self, candidates: List[ExploreCandidate]) -> None:
        if self._last_candidate_debug:
            ranked_debug = sorted(
                self._last_candidate_debug,
                key=lambda d: float(d.get("score", 0.0)),
                reverse=True,
            )
            for i, item in enumerate(ranked_debug[:8]):
                item["rank"] = i + 1
            self._last_candidate_summaries = ranked_debug[:8]
        else:
            ranked = sorted(candidates, key=lambda c: c.total_score, reverse=True)
            self._last_candidate_summaries = [
                self._candidate_summary(c, i + 1) for i, c in enumerate(ranked[:8])
            ]
        best = next(
            (c for c in (self._last_valid_candidates or candidates) if self._candidate_safe(c)),
            None,
        )
        self._last_best_candidate_id = best.candidate_id if best else None

    def _candidate_safe(self, candidate: ExploreCandidate) -> bool:
        if candidate.validation:
            return bool(candidate.validation.get("ok"))
        if candidate.unknown_goal_penalty > 0.5:
            return False
        if not self._goal_on_scanned_map(candidate.goal_xy):
            return False
        return (
            candidate.blacklist_penalty <= 0.0
            and candidate.reachability > 0.05
            and candidate.safety_margin > 0.05
        )

    def _distance_to_goal(self, goal_xy: Tuple[float, float]) -> float:
        if self.robot_pose is None:
            return 999.0
        return math.hypot(goal_xy[0] - self.robot_pose[0], goal_xy[1] - self.robot_pose[1])

    def _choose_sticky_candidate(
        self,
        best: Optional[ExploreCandidate],
        candidates: List[ExploreCandidate],
        now: float,
    ) -> Optional[ExploreCandidate]:
        current = self._selected
        by_id: Dict[str, ExploreCandidate] = {}
        for c in candidates:
            prev = by_id.get(c.candidate_id)
            if prev is None:
                by_id[c.candidate_id] = c
                continue
            if bool((c.validation or {}).get("ok")) and not bool(
                (prev.validation or {}).get("ok")
            ):
                by_id[c.candidate_id] = c
            elif getattr(c, "forced_pick", False) and not getattr(prev, "forced_pick", False):
                by_id[c.candidate_id] = c

        if current is not None and self.direction_lock_enabled and self.active_area_id:
            cur_sector = current.sector_id or self._sector_id_for_goal(current.goal_xy)
            if cur_sector != self.active_area_id:
                self._status_message = "cancel_cross_sector_goal"
                self._last_selection_explanation = (
                    f"cancel {current.candidate_id}: sector {cur_sector} != active {self.active_area_id}"
                )
                self._pending_switch_id = None
                self._pending_switch_count = 0
                self._selected_since = 0.0
                current = None

        if current is None:
            self._pending_switch_id = None
            self._pending_switch_count = 0
            if best is not None:
                self._selected_since = now
                self._last_selection_explanation = f"select initial {best.candidate_id}"
            else:
                self._last_selection_explanation = "no safe candidate above threshold"
            return best

        refreshed = by_id.get(current.candidate_id)
        if (
            refreshed is not None
            and current is not None
            and getattr(current, "forced_pick", False)
            and bool((current.validation or {}).get("ok"))
            and not bool((refreshed.validation or {}).get("ok"))
        ):
            refreshed = current
        if refreshed is None and getattr(current, "forced_pick", False):
            refreshed = current

        if refreshed is None or not self._candidate_safe(refreshed):
            if (
                current is not None
                and getattr(current, "forced_pick", False)
                and bool((current.validation or {}).get("ok"))
            ):
                self._last_selection_explanation = (
                    f"keep forced current {current.candidate_id}"
                )
                return current
            self._status_message = "current_goal_unsafe_cancel"
            self._last_selection_explanation = f"cancel current {current.candidate_id}: missing or unsafe"
            self._pending_switch_id = None
            self._pending_switch_count = 0
            self._selected_since = 0.0
            if best is not None:
                if best.candidate_id != current.candidate_id:
                    self._selected_since = now
                    self._last_selection_explanation += (
                        f"; select replacement {best.candidate_id}"
                    )
                    self._status_message = "current_goal_unsafe_cancel"
                else:
                    self._status_message = "selected"
                    self._last_selection_explanation = (
                        f"keep forced current {current.candidate_id}"
                    )
                    self._selected_since = max(self._selected_since, now - 1.0)
                return best
            if (
                getattr(current, "forced_pick", False)
                and bool((current.validation or {}).get("ok"))
            ):
                self._selected_since = now
                self._last_selection_explanation = (
                    f"keep forced current {current.candidate_id}"
                )
                return current
            return None

        if self.robot_pose is not None and not self._within_select_range(self.robot_pose, refreshed):
            self._status_message = "current_goal_out_of_range"
            self._last_selection_explanation = (
                f"cancel current {refreshed.candidate_id}: outside select range"
            )
            self._pending_switch_id = None
            self._pending_switch_count = 0
            self._selected_since = 0.0
            return best

        if self.robot_pose is not None and self.require_astar_path and not self.astar_fallback_bearing:
            if getattr(refreshed, "forced_pick", False) and refreshed.source.get(
                "force_relaxed"
            ):
                pass
            else:
                path = self._plan_candidate_path(self.robot_pose, refreshed.goal_xy)
                if not path:
                    self._status_message = "current_goal_no_astar_path"
                    self._last_selection_explanation = (
                        f"cancel current {refreshed.candidate_id}: astar path unavailable"
                    )
                    self._pending_switch_id = None
                    self._pending_switch_count = 0
                    self._selected_since = 0.0
                    return best

        if best is None or best.candidate_id == refreshed.candidate_id:
            self._pending_switch_id = None
            self._pending_switch_count = 0
            self._last_selection_explanation = f"keep current {refreshed.candidate_id}: still best"
            return refreshed

        if self.far_goal_priority_enabled and best is not None:
            cur_dist = self._distance_to_robot(refreshed.goal_xy)
            best_dist = self._distance_to_robot(best.goal_xy)
            if best_dist > cur_dist + 0.25:
                self._pending_switch_id = None
                self._pending_switch_count = 0
                self._selected_since = now
                self._status_message = "switch_farther_goal"
                self._last_selection_explanation = (
                    f"far-priority switch {refreshed.candidate_id}({cur_dist:.2f}m) "
                    f"-> {best.candidate_id}({best_dist:.2f}m)"
                )
                return best

        if self._nav_abort_reselect:
            self._nav_abort_reselect = False
            self._pending_switch_id = None
            self._pending_switch_count = 0
            self._selected_since = now
            self._status_message = "nav_abort_switch"
            self._last_selection_explanation = (
                f"nav abort immediate switch to {best.candidate_id}"
            )
            return best

        dist_to_current = self._distance_to_goal(refreshed.goal_xy)
        if dist_to_current > self.goal_switch_max_distance_m:
            self._pending_switch_id = None
            self._pending_switch_count = 0
            self._status_message = "keep_current_not_near_goal"
            self._last_selection_explanation = (
                f"keep current {refreshed.candidate_id}: "
                f"dist {dist_to_current:.2f}m > switch_max "
                f"{self.goal_switch_max_distance_m:.2f}m"
            )
            return refreshed

        if now - self._selected_since < self.keep_goal_min_sec:
            self._status_message = "keep_current_min_time"
            self._last_selection_explanation = (
                f"keep current {refreshed.candidate_id}: min hold "
                f"{now - self._selected_since:.1f}/{self.keep_goal_min_sec:.1f}s"
            )
            return refreshed

        if best.total_score < refreshed.total_score + self.switch_score_margin:
            self._status_message = "keep_current_score_margin"
            self._last_selection_explanation = (
                f"keep current {refreshed.candidate_id}: best {best.candidate_id} "
                f"margin {best.total_score - refreshed.total_score:.2f} < {self.switch_score_margin:.2f}"
            )
            self._pending_switch_id = None
            self._pending_switch_count = 0
            return refreshed

        if self._pending_switch_id != best.candidate_id:
            self._pending_switch_id = best.candidate_id
            self._pending_switch_count = 1
            self._status_message = "switch_candidate_pending"
            self._last_selection_explanation = (
                f"pending switch to {best.candidate_id}: confirm "
                f"1/{self.switch_confirm_count}"
            )
            return refreshed

        self._pending_switch_count += 1
        if self._pending_switch_count < self.switch_confirm_count:
            self._status_message = "switch_candidate_pending"
            self._last_selection_explanation = (
                f"pending switch to {best.candidate_id}: confirm "
                f"{self._pending_switch_count}/{self.switch_confirm_count}"
            )
            return refreshed

        self._pending_switch_id = None
        self._pending_switch_count = 0
        self._selected_since = now
        self._status_message = "switch_candidate_confirmed"
        self._last_selection_explanation = f"switch to {best.candidate_id}: confirmed better candidate"
        return best

    def _bearing_distance(
        self, robot_xy: Tuple[float, float], goal_xy: Tuple[float, float]
    ) -> Tuple[float, float]:
        dx = goal_xy[0] - robot_xy[0]
        dy = goal_xy[1] - robot_xy[1]
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - robot_xy[2]
        bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
        return bearing, dist

    def _publish_hint(self, selected: Optional[ExploreCandidate], robot_xy: Tuple[float, float]) -> None:
        now = time.time()
        if selected is None:
            reason = self._status_message
            if reason == "no_valid_safe_candidates":
                msg = "no valid safe candidates; wait for map update / slow scan"
            else:
                msg = "no semantic/frontier candidate"
            payload = {
                "stamp": now,
                "valid_sec": 1.0,
                "mode": "none",
                "score": 0.0,
                "reason": msg,
                "selection_status": reason,
            }
            self.pub_hint.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            return

        bearing, dist = self._bearing_distance(robot_xy, selected.goal_xy)
        path: List[Tuple[float, float]] = list(
            selected.source.get("planned_path") or self._last_path or []
        )
        if not path and bool(self.planner_cfg.get("astar_enabled", False)) and self.latest_map is not None:
            path = self._plan_candidate_path(robot_xy, selected.goal_xy)
        self._last_path = path
        use_astar = bool(path)
        nav_planner = "astar" if use_astar else "bearing_first"

        if self.require_astar_path and not path and not self.astar_fallback_bearing:
            none_payload = {
                "stamp": now,
                "valid_sec": 1.0,
                "mode": "none",
                "score": 0.0,
                "reason": "astar_path_unavailable_for_selected_goal",
                "candidate_id": selected.candidate_id,
                "astar_enabled": True,
                "astar_path_points": 0,
            }
            self.pub_hint.publish(String(data=json.dumps(none_payload, ensure_ascii=False)))
            stamp = self.get_clock().now().to_msg()
            self._publish_astar_markers(robot_xy, None, stamp)
            return

        if path:
            path_msg = NavPath()
            path_msg.header = Header()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            self._prepare_viz_frame()
            path_msg.header.frame_id = self._fixed_frame
            for px, py in path:
                vx, vy = self._xy_to_viz_frame(px, py)
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = vx
                ps.pose.position.y = vy
                path_msg.poses.append(ps)
            self.pub_path.publish(path_msg)

        payload = {
            "stamp": now,
            "valid_sec": self.valid_sec,
            "candidate_id": selected.candidate_id,
            "mode": selected.mode,
            "target_class": selected.target_class,
            "goal_frame": self._fixed_frame,
            "goal_pose": [selected.goal_xy[0], selected.goal_xy[1], selected.goal_yaw],
            "look_at": [selected.look_at[0], selected.look_at[1]],
            "goal_bearing_rad": bearing,
            "goal_distance_m": dist,
            "score": selected.total_score,
            "reason": selected.reason,
            "components": {
                "semantic_score": selected.semantic_score,
                "information_gain": selected.information_gain,
                "reachability": selected.reachability,
                "novelty": selected.novelty,
                "safety_margin": selected.safety_margin,
                "qwen_text_score": selected.qwen_text_score,
                "blacklist_penalty": selected.blacklist_penalty,
                "travel_cost_penalty": selected.travel_cost_penalty,
            },
            "source": selected.source,
            "selection_status": self._status_message,
            "selection_explanation": self._last_selection_explanation,
            "astar_enabled": bool(self.planner_cfg.get("astar_enabled", False)),
            "astar_path_points": len(path),
            "planned_path": [[px, py] for px, py in path],
            "nav_planner": nav_planner,
            "astar_fallback": not use_astar,
            "sector_id": selected.sector_id,
            "forced_below_threshold": bool(
                getattr(selected, "forced_below_threshold", False)
                or (isinstance(selected.source, dict) and selected.source.get("forced_below_threshold", False))
            ),
            "forced_pick": bool(getattr(selected, "forced_pick", False)),
            "force_reason": (selected.source.get("force_reason", "") if isinstance(selected.source, dict) else ""),
            "raw_score": float(selected.source.get("raw_score", selected.total_score) if isinstance(selected.source, dict) else selected.total_score),
            "min_hint_score": float(selected.source.get("min_hint_score", self.min_hint_score) if isinstance(selected.source, dict) else self.min_hint_score),
            "active_area_id": self.active_area_id,
            "raw_goal_xy": (
                [selected.raw_goal_xy[0], selected.raw_goal_xy[1]]
                if selected.raw_goal_xy is not None
                else None
            ),
            "projection_status": selected.projection_status,
            "validation": selected.validation,
        }
        self.pub_hint.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        stamp = self.get_clock().now().to_msg()
        self._publish_astar_markers(robot_xy, selected, stamp)

    def _map_frame_id(self) -> str:
        if self.latest_map is not None and self.latest_map.header.frame_id:
            return str(self.latest_map.header.frame_id)
        return self._frame_fixed or "map"

    def _map_coords_trusted(self) -> bool:
        self._tf_debug = {
            "map_frame": "",
            "base_frame": "",
            "pose_frame": self._pose_frame or "",
            "pose_source": getattr(self, "_pose_source", ""),
            "tf_map_base_ok": False,
            "tf_map_pose_ok": False,
            "map_coords_trusted": False,
            "last_tf_error": "",
            "tf_buffer_ok": self.tf_buffer is not None,
        }

        if self.latest_map is None:
            self._tf_debug["reason"] = "no_map"
            return False

        map_frame = self._map_frame_id()
        base_frame = self._base_frame or "base_link"

        self._tf_debug["map_frame"] = map_frame
        self._tf_debug["base_frame"] = base_frame
        self._tf_debug["pose_frame"] = self._pose_frame or ""

        tf_map_base = self._lookup_latest_transform(map_frame, base_frame, timeout_sec=0.2)
        self._tf_debug["tf_map_base_ok"] = tf_map_base is not None

        if tf_map_base is not None:
            self._tf_debug["map_coords_trusted"] = True
            self._tf_debug["pose_source"] = "tf_map_base"
            self._tf_debug["last_tf_error"] = ""
            return True

        if self._pose_frame:
            tf_map_pose = self._lookup_latest_transform(
                map_frame, self._pose_frame, timeout_sec=0.2
            )
            self._tf_debug["tf_map_pose_ok"] = tf_map_pose is not None
            if tf_map_pose is not None:
                self._tf_debug["map_coords_trusted"] = True
                self._tf_debug["pose_source"] = f"tf_map_{self._pose_frame}"
                self._tf_debug["last_tf_error"] = ""
                return True

        if self._pose_frame == map_frame:
            self._tf_debug["map_coords_trusted"] = True
            self._tf_debug["pose_source"] = "pose_frame_equals_map"
            self._tf_debug["last_tf_error"] = ""
            return True

        self._tf_debug["map_coords_trusted"] = False
        self._tf_debug["last_tf_error"] = getattr(self, "_last_tf_error", "")
        return False

    def _to_map_xy(self, x: float, y: float, source_frame: Optional[str] = None) -> Tuple[float, float]:
        src = source_frame or self._pose_frame
        dst = self._map_frame_id()
        if src == dst:
            return x, y
        if self.tf_buffer is None:
            return x, y
        try:
            tf = self.tf_buffer.lookup_transform(
                dst,
                src,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            q = tf.transform.rotation
            yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            return cos_y * x - sin_y * y + tx, sin_y * x + cos_y * y + ty
        except Exception:
            return x, y

    def _to_pose_frame_xy(self, x: float, y: float) -> Tuple[float, float]:
        src = self._map_frame_id()
        dst = self._pose_frame
        if src == dst:
            return x, y
        if self.tf_buffer is None:
            return x, y
        try:
            tf = self.tf_buffer.lookup_transform(
                dst,
                src,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            q = tf.transform.rotation
            yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            return cos_y * x - sin_y * y + tx, sin_y * x + cos_y * y + ty
        except Exception:
            return x, y

    def _planning_robot_xy(self) -> Optional[Tuple[float, float, float]]:
        if self.robot_pose is None:
            return None
        rx, ry, yaw = self.robot_pose
        mx, my = self._to_map_xy(rx, ry)
        return mx, my, yaw

    def _can_use_map_planning(self) -> bool:
        return self.latest_map is not None and self._map_coords_trusted()

    def _goal_map_xy(self, goal_xy: Tuple[float, float]) -> Tuple[float, float]:
        return self._to_map_xy(goal_xy[0], goal_xy[1])

    def _viz_frame_id(self) -> str:
        if self.latest_map is not None:
            return str(self.latest_map.header.frame_id or self._frame_fixed)
        return self._pose_frame

    def _xy_to_viz_frame(
        self,
        x: float,
        y: float,
        source_frame: Optional[str] = None,
    ) -> Tuple[float, float]:
        return self._to_map_xy(x, y, source_frame=source_frame or self._pose_frame)

    def _robot_xy_viz(self, robot_xy: Tuple[float, float, float]) -> Tuple[float, float, float]:
        vx, vy = self._xy_to_viz_frame(robot_xy[0], robot_xy[1])
        return vx, vy, robot_xy[2]

    def _prepare_viz_frame(self) -> None:
        self._fixed_frame = self._viz_frame_id()

    def _marker_sphere(
        self, ns: str, mid: int, x: float, y: float, color: Tuple[float, float, float, float], scale: float = 0.15
    ) -> Marker:
        m = Marker()
        m.header.frame_id = self._fixed_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.05
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        return m

    def _marker_header(self, stamp) -> Header:
        h = Header()
        h.stamp = stamp
        h.frame_id = self._fixed_frame
        return h

    def _marker_delete_all(self, ns: str, stamp) -> Marker:
        m = Marker()
        m.header = self._marker_header(stamp)
        m.ns = ns
        m.id = 0
        m.action = Marker.DELETEALL
        return m

    def _mode_color(self, mode: str, selected: bool = False, pending: bool = False) -> Tuple[float, float, float, float]:
        if selected:
            return (0.0, 1.0, 0.2, 1.0)
        if pending:
            return (1.0, 0.45, 0.0, 1.0)
        palette = {
            "target_landmark": (1.0, 0.85, 0.1, 0.95),
            "context_landmark": (1.0, 0.55, 0.1, 0.9),
            "frontier": (0.25, 0.55, 1.0, 0.9),
            "scanned_free": (0.2, 0.85, 0.55, 0.9),
            "free_space": (0.7, 0.7, 0.7, 0.85),
        }
        return palette.get(mode, (1.0, 1.0, 0.2, 0.9))

    def _publish_astar_markers(
        self,
        robot_xy: Tuple[float, float, float],
        selected: Optional[ExploreCandidate],
        stamp,
    ) -> None:
        self._prepare_viz_frame()
        robot_viz = self._robot_xy_viz(robot_xy)
        arr = MarkerArray()
        for ns in ("astar_path", "astar_waypoints", "astar_endpoints"):
            arr.markers.append(self._marker_delete_all(ns, stamp))
        if not selected or not self._last_path:
            self.pub_astar_markers.publish(arr)
            return

        line = Marker()
        line.header = self._marker_header(stamp)
        line.ns = "astar_path"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.06
        line.color = ColorRGBA(r=0.1, g=0.95, b=1.0, a=0.95)
        line.pose.orientation.w = 1.0
        line.points.append(Point(x=robot_viz[0], y=robot_viz[1], z=0.06))
        for px, py in self._last_path:
            vx, vy = self._xy_to_viz_frame(px, py)
            line.points.append(Point(x=vx, y=vy, z=0.06))
        arr.markers.append(line)

        for i, (px, py) in enumerate(self._last_path[:20]):
            vx, vy = self._xy_to_viz_frame(px, py)
            wp = self._marker_sphere("astar_waypoints", i + 1, vx, vy, (0.1, 0.8, 1.0, 0.85), 0.07)
            wp.header.stamp = stamp
            arr.markers.append(wp)

        start = self._marker_sphere(
            "astar_endpoints", 1, robot_viz[0], robot_viz[1], (0.0, 0.8, 1.0, 1.0), 0.1
        )
        start.header.stamp = stamp
        arr.markers.append(start)
        gx, gy = self._xy_to_viz_frame(selected.goal_xy[0], selected.goal_xy[1])
        end = self._marker_sphere("astar_endpoints", 2, gx, gy, (0.0, 0.5, 1.0, 1.0), 0.12)
        end.header.stamp = stamp
        arr.markers.append(end)
        self.pub_astar_markers.publish(arr)

    def _top_reject_reason(self) -> str:
        stats = self._last_pick_stats.get("reject_stats", {})
        if not stats:
            return "none"
        return max(stats.items(), key=lambda kv: kv[1])[0]

    def _publish_raw_candidate_pool_markers(
        self,
        candidates: List[ExploreCandidate],
        stamp,
        *,
        selected: Optional[ExploreCandidate] = None,
    ) -> None:
        """Foxglove 黄点：发布全部 raw 候选池，与验证 budget 无关。"""
        self._prepare_viz_frame()
        arr = MarkerArray()
        arr.markers.append(self._marker_delete_all("raw_goals", stamp))
        arr.markers.append(self._marker_delete_all("raw_pool_labels", stamp))

        limit = self.max_raw_candidate_markers
        for i, c in enumerate(candidates[:limit]):
            raw_xy = c.raw_goal_xy or c.goal_xy
            if raw_xy is None:
                continue
            rx, ry = self._xy_to_viz_frame(float(raw_xy[0]), float(raw_xy[1]))
            in_active = (
                self.direction_lock_enabled
                and self.active_area_id
                and self._candidate_in_active_sector(c)
            )
            if selected and c.candidate_id == selected.candidate_id:
                color = (1.0, 0.55, 0.0, 1.0)
                scale = 0.13
            elif in_active:
                color = (1.0, 0.85, 0.1, 0.95)
                scale = 0.10
            else:
                color = (0.85, 0.75, 0.15, 0.55)
                scale = 0.08

            raw_m = self._marker_sphere("raw_goals", i, rx, ry, color, scale)
            raw_m.pose.position.z = 0.12
            raw_m.header.stamp = stamp
            arr.markers.append(raw_m)

        if len(candidates) > limit:
            label = Marker()
            label.header = self._marker_header(stamp)
            label.ns = "raw_pool_labels"
            label.id = 0
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = 0.0
            label.pose.position.y = 0.0
            label.pose.position.z = 0.5
            label.scale.z = 0.14
            label.color = ColorRGBA(r=1.0, g=0.9, b=0.2, a=0.9)
            label.text = f"+{len(candidates) - limit} more raw"
            arr.markers.append(label)

        self.pub_projection_markers.publish(arr)

    def _publish_projection_debug_markers(
        self,
        robot_xy: Tuple[float, float, float],
        selected: Optional[ExploreCandidate],
        stamp,
    ) -> None:
        self._prepare_viz_frame()
        arr = MarkerArray()
        for ns in (
            "projected_goals",
            "rejected_goals",
            "projection_links",
            "reject_labels",
            "projection_radius",
        ):
            arr.markers.append(self._marker_delete_all(ns, stamp))

        debug_items = self._last_candidate_debug[: self.max_projection_debug_markers]
        link_id = 0
        label_id = 0
        for i, item in enumerate(debug_items):
            raw_xy = item.get("raw_goal_xy")
            goal_xy = item.get("goal_xy")
            if not raw_xy or len(raw_xy) < 2:
                continue
            rx, ry = self._xy_to_viz_frame(float(raw_xy[0]), float(raw_xy[1]))

            validation = item.get("validation") or {}
            ok = bool(validation.get("ok"))
            reject = str(item.get("reject_reason") or validation.get("reject_reason") or "")
            proj_status = str(item.get("projection_status") or "")

            if ok and goal_xy and len(goal_xy) >= 2:
                px, py = self._xy_to_viz_frame(float(goal_xy[0]), float(goal_xy[1]))
                proj_m = self._marker_sphere(
                    "projected_goals", i, px, py, (0.1, 0.95, 0.2, 0.95), 0.10
                )
                proj_m.header.stamp = stamp
                arr.markers.append(proj_m)
                link = Marker()
                link.header = self._marker_header(stamp)
                link.ns = "projection_links"
                link.id = link_id
                link_id += 1
                link.type = Marker.LINE_STRIP
                link.action = Marker.ADD
                link.scale.x = 0.03
                link.color = ColorRGBA(r=0.1, g=0.9, b=0.95, a=0.85)
                link.pose.orientation.w = 1.0
                link.points = [
                    Point(x=rx, y=ry, z=0.10),
                    Point(x=px, y=py, z=0.10),
                ]
                arr.markers.append(link)
            else:
                rej_m = self._marker_sphere(
                    "rejected_goals", i, rx, ry, (0.95, 0.15, 0.1, 0.85), 0.08
                )
                rej_m.header.stamp = stamp
                arr.markers.append(rej_m)

            sampled = validation.get("sampled_count", "")
            valid_n = validation.get("valid_sample_count", "")
            label = Marker()
            label.header = self._marker_header(stamp)
            label.ns = "reject_labels"
            label.id = label_id
            label_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = rx
            label.pose.position.y = ry
            label.pose.position.z = 0.32
            label.scale.z = 0.08
            label.color = ColorRGBA(r=1.0, g=0.95, b=0.7, a=0.98)
            label.text = (
                f"{item.get('candidate_id', '')}\n"
                f"{proj_status}\n"
                f"{reject}\n"
                f"s={sampled} v={valid_n}"
            )[:100]
            arr.markers.append(label)

        if selected and selected.raw_goal_xy:
            srx, sry = self._xy_to_viz_frame(
                selected.raw_goal_xy[0], selected.raw_goal_xy[1]
            )
            ring = Marker()
            ring.header = self._marker_header(stamp)
            ring.ns = "projection_radius"
            ring.id = 1
            ring.type = Marker.LINE_STRIP
            ring.action = Marker.ADD
            ring.scale.x = 0.02
            ring.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.7)
            ring.pose.orientation.w = 1.0
            n_seg = 48
            for k in range(n_seg + 1):
                a = 2.0 * math.pi * k / n_seg
                ring.points.append(
                    Point(
                        x=srx + self.projection_max_radius_m * math.cos(a),
                        y=sry + self.projection_max_radius_m * math.sin(a),
                        z=0.05,
                    )
                )
            arr.markers.append(ring)

        self.pub_projection_markers.publish(arr)

    def _publish_recovery_debug_markers(self, stamp) -> None:
        if not self.active_recovery_marker_enabled:
            return

        arr = MarkerArray()
        arr.markers.append(self._marker_delete_all("active_recovery_raw", stamp))
        arr.markers.append(self._marker_delete_all("active_recovery_projected", stamp))
        arr.markers.append(self._marker_delete_all("active_recovery_link", stamp))
        arr.markers.append(self._marker_delete_all("active_recovery_label", stamp))

        idx = 0
        for item in self._last_recovery_debug[:40]:
            raw_xy = item.get("raw_xy")
            proj_xy = item.get("projected_xy")
            if not raw_xy:
                continue

            rx, ry = self._xy_to_viz_frame(float(raw_xy[0]), float(raw_xy[1]))
            raw_m = self._marker_sphere(
                "active_recovery_raw",
                idx,
                rx,
                ry,
                (1.0, 0.4, 0.1, 0.85),
                0.08,
            )
            raw_m.header.stamp = stamp
            arr.markers.append(raw_m)

            if proj_xy:
                px, py = self._xy_to_viz_frame(float(proj_xy[0]), float(proj_xy[1]))
                proj_m = self._marker_sphere(
                    "active_recovery_projected",
                    idx,
                    px,
                    py,
                    (0.8, 0.0, 1.0, 0.95) if item.get("ok") else (1.0, 0.0, 0.0, 0.75),
                    0.12,
                )
                proj_m.header.stamp = stamp
                arr.markers.append(proj_m)

                line = Marker()
                line.header = proj_m.header
                line.ns = "active_recovery_link"
                line.id = idx
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.035
                line.color = ColorRGBA(r=0.8, g=0.2, b=1.0, a=0.8)
                line.points = [
                    Point(x=rx, y=ry, z=0.08),
                    Point(x=px, y=py, z=0.08),
                ]
                arr.markers.append(line)

                label = Marker()
                label.header = proj_m.header
                label.ns = "active_recovery_label"
                label.id = idx
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = px
                label.pose.position.y = py
                label.pose.position.z = 0.32
                label.scale.z = 0.10
                label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
                label.text = (
                    f"{item.get('sector')}\n"
                    f"r={item.get('radius_m')}\n"
                    f"{'OK' if item.get('ok') else 'X'} {str(item.get('reason', ''))[:18]}"
                )
                arr.markers.append(label)

            idx += 1

        self.pub_projection_markers.publish(arr)

    def _publish_selection_process_markers(
        self,
        robot_xy: Tuple[float, float, float],
        candidates: List[ExploreCandidate],
        selected: Optional[ExploreCandidate],
        stamp,
    ) -> None:
        self._prepare_viz_frame()
        robot_viz = self._robot_xy_viz(robot_xy)
        arr = MarkerArray()
        for ns in ("robot", "links", "pending", "status"):
            arr.markers.append(self._marker_delete_all(ns, stamp))

        robot_m = self._marker_sphere(
            "robot", 0, robot_viz[0], robot_viz[1], (0.0, 0.9, 0.9, 1.0), 0.14
        )
        robot_m.header.stamp = stamp
        arr.markers.append(robot_m)

        if selected:
            gx, gy = self._xy_to_viz_frame(selected.goal_xy[0], selected.goal_xy[1])
            link = Marker()
            link.header = self._marker_header(stamp)
            link.ns = "links"
            link.id = 1
            link.type = Marker.LINE_STRIP
            link.action = Marker.ADD
            link.scale.x = 0.04
            link.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9)
            link.pose.orientation.w = 1.0
            link.points = [
                Point(x=robot_viz[0], y=robot_viz[1], z=0.08),
                Point(x=gx, y=gy, z=0.08),
            ]
            arr.markers.append(link)

        if self._pending_switch_id:
            pending = next((c for c in candidates if c.candidate_id == self._pending_switch_id), None)
            if pending:
                px, py = self._xy_to_viz_frame(pending.goal_xy[0], pending.goal_xy[1])
                pm = self._marker_sphere("pending", 1, px, py, (1.0, 0.45, 0.0, 1.0), 0.16)
                pm.header.stamp = stamp
                arr.markers.append(pm)

        # Status text moved to /explore_hud; keep robot/links/pending markers only.
        arr.markers.append(self._marker_delete_all("status", stamp))
        self.pub_selection_markers.publish(arr)

    def _publish_markers(
        self,
        robot_xy: Tuple[float, float, float],
        candidates: List[ExploreCandidate],
        selected: Optional[ExploreCandidate],
        *,
        full: bool = True,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        self._prepare_viz_frame()
        robot_viz = self._robot_xy_viz(robot_xy)
        ranked = sorted(candidates, key=lambda c: c.total_score, reverse=True)
        rank_by_id = {c.candidate_id: i + 1 for i, c in enumerate(ranked)}

        cand_arr = MarkerArray()
        cand_arr.markers.append(self._marker_delete_all("candidates", stamp))
        cand_arr.markers.append(self._marker_delete_all("candidate_labels", stamp))
        for i, c in enumerate(candidates[: self.max_candidate_goal_markers]):
            is_sel = selected is not None and c.candidate_id == selected.candidate_id
            is_pending = c.candidate_id == self._pending_switch_id
            color = self._mode_color(c.mode, selected=is_sel, pending=is_pending)
            scale = 0.18 if is_sel else (0.14 if is_pending else 0.11)
            gx, gy = self._xy_to_viz_frame(c.goal_xy[0], c.goal_xy[1])
            m = self._marker_sphere("candidates", i, gx, gy, color, scale)
            m.header.stamp = stamp
            cand_arr.markers.append(m)

            rank = rank_by_id.get(c.candidate_id, 0)
            safe = "OK" if self._candidate_safe(c) else "X"
            label = Marker()
            label.header = m.header
            label.ns = "candidate_labels"
            label.id = 100 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = gx
            label.pose.position.y = gy
            label.pose.position.z = 0.28
            label.scale.z = 0.11
            label.color = ColorRGBA(r=1.0, g=1.0, b=0.95, a=0.98)
            label.text = (
                f"#{rank} {c.candidate_id}\n"
                f"{c.mode} {c.total_score:.2f} {safe}\n"
                f"{c.projection_status or '-'}\n"
                f"{(c.reject_reason or '-')[:24]}"
            )[:100]
            cand_arr.markers.append(label)
        self.pub_candidate_markers.publish(cand_arr)

        sel_arr = MarkerArray()
        sel_arr.markers.append(self._marker_delete_all("selected", stamp))
        sel_arr.markers.append(self._marker_delete_all("selected_arrow", stamp))
        sel_arr.markers.append(self._marker_delete_all("selected_label", stamp))
        if selected:
            gx, gy = self._xy_to_viz_frame(selected.goal_xy[0], selected.goal_xy[1])
            lx, ly = self._xy_to_viz_frame(selected.look_at[0], selected.look_at[1])
            m = self._marker_sphere("selected", 0, gx, gy, (0.0, 1.0, 0.0, 1.0), 0.22)
            m.header.stamp = stamp
            sel_arr.markers.append(m)
            arrow = Marker()
            arrow.header = m.header
            arrow.ns = "selected_arrow"
            arrow.id = 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=gx, y=gy, z=0.05),
                Point(x=lx, y=ly, z=0.05),
            ]
            arrow.scale.x = 0.05
            arrow.scale.y = 0.1
            arrow.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            sel_arr.markers.append(arrow)
            sel_label = Marker()
            sel_label.header = m.header
            sel_label.ns = "selected_label"
            sel_label.id = 2
            sel_label.type = Marker.TEXT_VIEW_FACING
            sel_label.action = Marker.ADD
            sel_label.pose.position.x = gx
            sel_label.pose.position.y = gy
            sel_label.pose.position.z = 0.42
            sel_label.scale.z = 0.13
            sel_label.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0)
            sel_label.text = f"SELECTED\n{selected.candidate_id}\n{selected.reason}"[:100]
            sel_arr.markers.append(sel_label)
        self.pub_selected_markers.publish(sel_arr)

        fr_arr = MarkerArray()
        fr_arr.markers.append(self._marker_delete_all("frontiers", stamp))
        fr_arr.markers.append(self._marker_delete_all("frontier_labels", stamp))
        for i, fg in enumerate(self._last_frontiers[:12]):
            fx, fy = self._xy_to_viz_frame(fg.goal_xy[0], fg.goal_xy[1])
            m = self._marker_sphere("frontiers", i, fx, fy, (0.2, 0.4, 1.0, 0.8), 0.08)
            m.header.stamp = stamp
            fr_arr.markers.append(m)
            fl = Marker()
            fl.header = m.header
            fl.ns = "frontier_labels"
            fl.id = 200 + i
            fl.type = Marker.TEXT_VIEW_FACING
            fl.action = Marker.ADD
            fl.pose.position.x = fx
            fl.pose.position.y = fy
            fl.pose.position.z = 0.2
            fl.scale.z = 0.09
            fl.color = ColorRGBA(r=0.6, g=0.8, b=1.0, a=0.95)
            fl.text = f"{fg.frontier_id}\ngain={fg.unknown_gain:.2f}"
            fr_arr.markers.append(fl)
        self.pub_frontier_markers.publish(fr_arr)
        self._publish_recovery_debug_markers(stamp)
        # 黄点 raw 池：每 tick 都更新，不依赖 full 或验证 budget
        self._publish_raw_candidate_pool_markers(candidates, stamp, selected=selected)

        if not full:
            return

        self._publish_selection_process_markers(robot_xy, candidates, selected, stamp)
        self._publish_astar_markers(robot_xy, selected, stamp)
        self._publish_projection_debug_markers(robot_xy, selected, stamp)

    def _select_tick(self) -> None:
        # 超时切换必须在 _select_busy 之前执行，否则长 tick 会挡住切换。
        self._maybe_force_active_sector_timeout()
        if self._select_busy:
            return
        self._select_busy = True
        try:
            self._select_tick_impl()
        except Exception as exc:
            self._status_message = "select_tick_error"
            self._last_selection_explanation = repr(exc)
            self.get_logger().error(f"select_tick failed: {exc!r}")
        finally:
            self._select_busy = False

    def _select_tick_impl(self) -> None:
        self._select_tick_count += 1
        if not self.explore_enabled:
            self._status_message = "semantic_explore.disabled"
            return
        if self._target_visible():
            self._status_message = "target_visible_hold"
            if self._selected is not None and self.robot_pose is not None:
                self._publish_hint(self._selected, self.robot_pose)
            return
        if not self._update_robot_pose() or self.robot_pose is None:
            self._status_message = "waiting_robot_pose"
            return
        if self.latest_map is None:
            self._status_message = "waiting_map"
            return
        now = time.time()
        robot_xy = self.robot_pose
        tick_sector = self.active_area_id

        if self._maybe_force_active_sector_timeout():
            # 超时切换后同 tick 继续在新 sector 选点，不要 return 卡住
            self._last_select_time = 0.0
            tick_sector = self.active_area_id

        if now - self._last_select_time < self._effective_reselect_interval_sec() and self._selected is not None:
            hold_ok = True
            if self.direction_lock_enabled and self.active_area_id:
                sel_sector = self._selected.sector_id or self._sector_id_for_goal(self._selected.goal_xy)
                if sel_sector != self.active_area_id:
                    hold_ok = False
            if hold_ok:
                self._publish_hint(self._selected, self.robot_pose)
                self._publish_markers(self.robot_pose, self._last_candidates, self._selected)
                return
        self._last_select_time = now
        self._status_message = "generating_candidates"
        raw_candidates = self._generate_raw_candidates(robot_xy)
        self._last_candidates = raw_candidates

        self._assign_sector_ids(raw_candidates)
        sector_pool = list(raw_candidates)
        self._build_sector_count_stats(sector_pool)

        if (
            self.direction_lock_enabled
            and self.sector_select_by_count
            and self.active_area_id is None
        ):
            chosen = self._choose_sector_by_candidate_count(
                sector_pool,
                allowed_sector_ids=set(self._sector_ids_with_raw_candidates(sector_pool)),
                respect_cooldown=self.init_sector_respect_cooldown,
                now=now,
            )
            if chosen:
                st = self._last_sector_counts.get(chosen, {})
                self.get_logger().info(
                    f"SECTOR_COUNT_INIT -> {chosen} count={st.get('count', 0)} "
                    f"all_counts={self._format_sector_count_summary()}"
                )
                self._activate_sector(chosen, "init_by_candidate_count")
                self._clear_current_selection_for_sector_switch(
                    f"init active sector by candidate count: {chosen}"
                )

        self._last_pick_stats = {
            "candidates_total": len(raw_candidates),
            "map_coords_trusted": self._map_coords_trusted(),
            "can_use_map_planning": self._can_use_map_planning(),
            "min_goal_clearance_m": self.min_goal_clearance_m,
            "min_unknown_gain_cells": self.min_unknown_gain_cells,
            "goal_validation_enabled": self.goal_validation_enabled,
            "projection_enabled": self.projection_enabled,
            "active_area_id": self.active_area_id,
        }
        if raw_candidates:
            self._publish_markers(robot_xy, raw_candidates, self._selected, full=False)
        self._status_message = "validating_candidates"
        valid_candidates = self._build_valid_candidates(robot_xy, raw_candidates)
        if (
            self.direction_lock_enabled
            and self.active_area_id
            and self.force_active_sector_goal
            and not self._valid_candidates_in_active_sector(valid_candidates)
        ):
            forced_far = self._force_validate_farthest_in_active_sector(
                raw_candidates, robot_xy
            )
            if forced_far is not None:
                valid_candidates = self._merge_valid_candidate_cache([forced_far], now)
                self._last_valid_candidates = valid_candidates
                self._last_valid_count = len(valid_candidates)
        self._last_valid_candidates = valid_candidates
        self._update_candidate_debug(raw_candidates)

        for c in valid_candidates:
            if not c.sector_id:
                c.sector_id = self._sector_id_for_goal(c.goal_xy)
            dist = self._distance_to_robot(c.goal_xy)
            c.travel_cost_penalty = self._travel_cost_penalty(dist)

        candidate_pool = valid_candidates
        best: Optional[ExploreCandidate] = None
        path: List[Tuple[float, float]] = []

        if self.direction_lock_enabled and self.active_area_id:
            active_valid = [
                c for c in candidate_pool
                if self._candidate_in_active_sector(c)
            ]
            if (
                not active_valid
                and self.force_active_sector_goal
                and self.force_farthest_when_no_best
            ):
                forced_far = self._force_validate_farthest_in_active_sector(
                    raw_candidates, robot_xy
                )
                if forced_far is not None:
                    active_valid = [forced_far]
                    self._valid_candidates_cache[forced_far.candidate_id] = (
                        now,
                        forced_far,
                    )
                    self._last_valid_candidates = self._merge_valid_candidate_cache(
                        [forced_far], now
                    )
                    self._last_valid_count = len(self._last_valid_candidates)
            if (
                not active_valid
                and self.force_active_sector_goal
                and self.active_sector_recovery_enabled
            ):
                st_pre = self.sector_states.get(self.active_area_id)
                cycles_pre = st_pre.no_candidate_cycles if st_pre is not None else 0
                if cycles_pre >= self.expand_after_no_candidate_cycles:
                    self._last_pick_stats = {
                        **getattr(self, "_last_pick_stats", {}),
                        "recovery_attempted": True,
                    }
                    active_valid = self._recover_candidates_in_active_sector(
                        raw_candidates,
                        robot_xy,
                    )
            if active_valid:
                candidate_pool = active_valid
                st = self.sector_states.get(self.active_area_id)
                if st is not None:
                    st.no_candidate_cycles = 0
                    st.last_update_time = now
            else:
                st = self.sector_states.get(self.active_area_id)
                if st is not None:
                    st.no_candidate_cycles += 1
                    st.last_update_time = now
                self._status_message = "active_sector_no_candidate"
                self._last_selection_explanation = (
                    f"active sector {self.active_area_id} has no valid/recovered candidate; "
                    f"no_candidate_cycles={st.no_candidate_cycles if st else 'n/a'}"
                )
                reason = self._sector_exhaust_reason(self.active_area_id)
                if reason and self.sector_select_by_count:
                    old_sector = self.active_area_id
                    switched = self._switch_active_sector_by_candidate_count(
                        sector_pool,
                        old_sector,
                        self._sector_ids_with_raw_candidates(sector_pool),
                        reason,
                        respect_cooldown=False,
                    )
                    if switched:
                        self._selected = None
                        self._last_select_time = 0.0
                        return
                self._selected = None
                self._publish_markers(robot_xy, raw_candidates, None)
                return

        best, path = self._pick_best_from_pool(robot_xy, candidate_pool)
        if best is not None and best.source.get("fast_validation"):
            fast_best = best
            projected_goal = tuple(best.goal_xy)
            self._reset_candidate_to_raw(best)
            full_deadline = time.time() + max(0.35, float(self.max_validate_sec_per_tick) * 0.5)
            if time.time() < full_deadline and self._process_raw_candidate(
                best, robot_xy, fast=False
            ):
                path = best.source.get("planned_path")
                if isinstance(path, list):
                    path = list(path)
            elif getattr(fast_best, "forced_pick", False):
                best = fast_best
                best.goal_xy = projected_goal
                path = best.source.get("planned_path") or []
                if isinstance(path, list):
                    path = list(path)
                self.get_logger().info(
                    f"FORCE_FARTHEST_ACTIVE keep fast-validated id={best.candidate_id}"
                )
            else:
                best = None
                path = []
        if best is None and self.direction_lock_enabled and self.active_area_id:
            if self.force_farthest_when_no_best and self.force_active_sector_goal:
                forced_far = self._force_validate_farthest_in_active_sector(
                    raw_candidates, robot_xy
                )
                if forced_far is not None:
                    best, path = self._pick_best_from_pool(robot_xy, [forced_far])
        if path:
            self._last_path = path

        # sector 中途切换：若已有 best 则继续发布；仅超时且无 best 时中断
        if self._active_sector_timed_out(now) and best is None:
            self._maybe_force_active_sector_timeout()
            if best is None and self.force_farthest_when_no_best:
                forced_far = self._force_validate_farthest_in_active_sector(
                    raw_candidates, robot_xy
                )
                if forced_far is not None:
                    best, path = self._pick_best_from_pool(robot_xy, [forced_far])
                    if path:
                        self._last_path = path
            if best is None:
                self._publish_markers(robot_xy, raw_candidates, None)
                return

        sticky_pool = list(candidate_pool or raw_candidates)
        if self._selected is not None:
            sticky_pool.append(self._selected)
        for vc in self._last_valid_candidates or []:
            sticky_pool.append(vc)
        selected = self._choose_sticky_candidate(best, sticky_pool, now)
        self._selected = selected
        if selected is None:
            if not self._last_valid_candidates:
                self._status_message = "no_valid_safe_candidates"
                self._last_selection_explanation = (
                    "no valid safe candidates after projection/validation; "
                    f"reject_stats={self._last_pick_stats.get('reject_stats', {})}"
                )
            else:
                stats = self._last_pick_stats
                if stats.get("candidates_total", 0) == 0:
                    self._status_message = "no_candidates_generated"
                elif stats.get("valid_count", 0) == 0:
                    self._status_message = "no_valid_safe_candidates"
                else:
                    self._status_message = "no_navigable_candidate"
        elif self._status_message not in (
            "keep_current_min_time",
            "keep_current_score_margin",
            "switch_candidate_pending",
            "switch_candidate_confirmed",
            "current_goal_unsafe_cancel",
        ):
            self._status_message = "selected"
        self._publish_hint(selected, robot_xy)
        self._publish_markers(robot_xy, raw_candidates, selected)
        # 事件日志：仅在选中目标发生变化时打印一次（每 tick 的常规状态由 compact 日志覆盖）
        sel_id = selected.candidate_id if selected is not None else None
        if sel_id != self._last_logged_hint_id:
            self._last_logged_hint_id = sel_id
            if selected is not None:
                path_len = len(selected.source.get("planned_path") or self._last_path or [])
                self.get_logger().info(
                    f"HINT id={selected.candidate_id} mode={selected.mode} "
                    f"score={selected.total_score:.3f} dist={self._distance_to_robot(selected.goal_xy):.2f}m "
                    f"forced={getattr(selected, 'forced_below_threshold', False)} "
                    f"sector={selected.sector_id} active={self.active_area_id} "
                    f"goal=({selected.goal_xy[0]:.2f},{selected.goal_xy[1]:.2f}) path={path_len}"
                )
            else:
                self.get_logger().info(
                    f"HINT none status={self._status_message} reason={self._last_selection_explanation}"
                )

    def _wrap_hud_line(self, text: str, width: Optional[int] = None) -> str:
        line = str(text or "").strip()
        if not line:
            return ""
        return textwrap.fill(
            line,
            width=width or self.hud_wrap_width,
            break_long_words=True,
            break_on_hyphens=False,
        )

    def _build_hud_text(self) -> str:
        lines: List[str] = []
        lines.append(f"STATUS: {self._status_message}")
        lines.append(f"raw={self._last_raw_count} valid={self._last_valid_count}")
        lines.append(f"top_reject={self._top_reject_str()}")
        sel = self._selected
        lines.append(f"sel={sel.candidate_id if sel else 'none'}")
        if self.active_area_id:
            active_st = self.sector_states.get(self.active_area_id)
            exhaust_info = ""
            if active_st:
                exhaust_info = (
                    f"nc={active_st.no_candidate_cycles}/{self.exhaust_no_candidate_cycles} "
                    f"f={active_st.failed_goals}/{self.exhaust_failed_goals} "
                    f"v={active_st.visited_goals}/{self.exhaust_visited_goals}"
                )
            elapsed = self._active_sector_elapsed_sec()
            sel_dist = ""
            if sel and self.robot_pose:
                d = math.hypot(
                    sel.goal_xy[0] - self.robot_pose[0],
                    sel.goal_xy[1] - self.robot_pose[1],
                )
                sel_dist = f" dist={d:.1f}m"
            lines.append(f"ACTIVE: {self.active_area_id}")
            lines.append(f"REASON: {self._last_direction_reason}")
            lines.append(f"EXHAUST: {exhaust_info}")
            lines.append(
                f"TIME: {elapsed:.0f}/{self.max_active_sector_sec:.0f}s "
                f"{self._sector_exhaust_reason(self.active_area_id)}"
            )
            lines.append(f"SELECTED: {sel.sector_id if sel else 'none'}{sel_dist}")
        if self._last_selection_explanation:
            lines.append(self._last_selection_explanation)
        wrapped = [self._wrap_hud_line(line) for line in lines if line]
        return "\n".join(wrapped)

    def _sector_counts_summary(self) -> Dict[str, int]:
        counts: Dict[str, int] = {}
        for candidate in self._sector_count_pool():
            sid = self._sector_id_for_candidate(candidate)
            counts[sid] = counts.get(sid, 0) + 1
        return counts

    def _build_state_payload(self) -> Dict[str, Any]:
        sel = self._selected
        summary = {
            "status": self._status_message,
            "active_sector": self.active_area_id,
            "sector_counts": self._sector_counts_summary(),
            "sector_progress": self._active_sector_progress(),
            "exhaust_reason": self._sector_exhaust_reason(self.active_area_id) if self.active_area_id else "",
            "valid": self._last_valid_count,
            "raw": self._last_raw_count,
            "selected": sel.candidate_id if sel else None,
            "selected_score": round(sel.total_score, 3) if sel else None,
            "forced_below_threshold": bool(getattr(sel, "forced_below_threshold", False)) if sel else False,
            "recovery_inflate": bool(isinstance(sel.source, dict) and sel.source.get("recovery_layer") is not None) if sel else False,
            "top_reject": self._top_reject_str(),
            "pose_ready": self.robot_pose is not None,
            "map_ready": self.latest_map is not None,
        }
        return {
            "summary": summary,
            "state": "SEMANTIC_EXPLORE" if self._selected else "SEARCH",
            "status": self._status_message,
            "selection_explanation": self._last_selection_explanation,
            "instruction": self.instruction,
            "target_aliases": self.parsed.target_aliases,
            "target_visible": self._target_visible(),
            "has_map": self.latest_map is not None,
            "has_scan": self.latest_scan is not None,
            "has_robot_pose": self.robot_pose is not None,
            "pose_frame": self._pose_frame,
            "pose_source": self._pose_source,
            "map_frame": str(self.latest_map.header.frame_id) if self.latest_map else None,
            "pose_map_aligned": self._pose_matches_map(),
            "viz_frame": self._viz_frame_id(),
            "map_coords_trusted": self._map_coords_trusted(),
            "tf_debug": dict(self._tf_debug),
            "pick_stats": self._last_pick_stats,
            "valid_candidates_count": len(self._last_valid_candidates),
            "active_area_id": self.active_area_id,
            "min_goal_clearance_m": self.min_goal_clearance_m,
            "min_unknown_gain_cells": self.min_unknown_gain_cells,
            "goal_validation_enabled": self.goal_validation_enabled,
            "projection_enabled": self.projection_enabled,
            "min_hint_score": self.min_hint_score,
            "num_landmarks": len(self._confirmed_landmarks()),
            "num_frontiers": len(self._last_frontiers),
            "num_candidates_last": len(self._last_candidates),
            "best_candidate_id": self._last_best_candidate_id,
            "candidates_ranked": self._last_candidate_summaries,
            "selected_candidate": self._selected.candidate_id if self._selected else None,
            "selected_mode": self._selected.mode if self._selected else None,
            "selected_score": round(self._selected.total_score, 3) if self._selected else 0.0,
            "selected_reason": self._selected.reason if self._selected else "",
            "selected_since_sec": round(time.time() - self._selected_since, 2) if self._selected_since else 0.0,
            "pending_switch_candidate": self._pending_switch_id,
            "pending_switch_count": self._pending_switch_count,
            "switch_confirm_target": self.switch_confirm_count,
            "keep_goal_min_sec": self.keep_goal_min_sec,
            "switch_score_margin": self.switch_score_margin,
            "goal_switch_max_distance_m": self.goal_switch_max_distance_m,
            "max_goal_select_distance_m": self.max_goal_select_distance_m,
            "require_astar_path": self.require_astar_path,
            "astar_fallback_bearing": self.astar_fallback_bearing,
            "require_goal_on_known_free": self.require_goal_on_known_free,
            "nav_abort_reselect": self._nav_abort_reselect,
            "last_nav_reject_reason": self._last_nav_reject_reason,
            "distance_to_selected_goal_m": (
                round(self._distance_to_goal(self._selected.goal_xy), 3) if self._selected else None
            ),
            "astar_enabled": bool(self.planner_cfg.get("astar_enabled", False)),
            "astar_path_points": len(self._last_path),
            "qwen_enabled": self.qwen.enabled,
            "fallback": self._selected is None,
            "blacklist_count": len(self.blacklist_regions),
            "stamp": time.time(),
            "candidate_pipeline": {
                "raw_count": self._last_raw_count,
                "valid_count": self._last_valid_count,
                "reject_stats": self._last_pick_stats.get("reject_stats", {}),
                "active_area_id": self.active_area_id,
            },
            "candidate_debug": self._last_candidate_debug[:30],
            "goal_validation_cfg": {
                "enabled": self.goal_validation_enabled,
                "min_clearance_m": self.min_goal_clearance_m,
                "require_known_free": self.require_known_free,
                "require_unknown_gain": self.require_unknown_gain,
                "min_unknown_gain_cells": self.min_unknown_gain_cells,
            },
            "projection_cfg": {
                "enabled": self.projection_enabled,
                "max_projection_radius_m": self.projection_max_radius_m,
                "projection_step_m": self.projection_step_m,
            },
            "planner_cfg": {
                "require_astar_path": self.require_astar_path,
                "astar_fallback_bearing": self.astar_fallback_bearing,
                "allow_unknown": self.planner_cfg.get("allow_unknown", False),
            },
            "direction_lock": {
                "enabled": self.direction_lock_enabled,
                "active_sector": self.active_area_id,
                "reason": self._last_direction_reason,
                "elapsed_sec": round(self._active_sector_elapsed_sec(), 1),
                "switch_count": self.active_area_switch_count,
                "active_area_id": self.active_area_id,
                "active_elapsed_sec": round(self._active_sector_elapsed_sec(), 1),
                "max_active_sec": self.max_active_sector_sec,
                "exhaust_logic": self.exhaust_logic,
                "exhausted_cooldown_sec": self.exhausted_cooldown_sec,
                "non_exhaust_reasons": sorted(list(self.sector_non_exhaust_reasons)),
                "visited_reasons": sorted(list(self.sector_visited_reasons)),
                "active_sector_recovery": {
                    "enabled": self.active_sector_recovery_enabled,
                    "attempted": bool(
                        self.active_area_id
                        and getattr(self, "_last_pick_stats", {}).get("recovery_attempted", False)
                    ),
                    "expand_after": self.expand_after_no_candidate_cycles,
                    "radii": self.projection_radius_schedule_m,
                    "last_debug": self._last_recovery_debug[:12],
                    "recovered_count": len(self._last_recovery_candidates),
                },
            },
            "sector_counts": self._last_sector_counts,
            "active_sector_recovery": {
                "active_sector": self.active_area_id,
                "enabled": self.active_sector_recovery_enabled,
                "last_debug": self._last_recovery_debug[:12],
                "recovered_count": len(self._last_recovery_candidates),
            },
        }

    def _publish_state_tick(self) -> None:
        try:
            self._maybe_force_active_sector_timeout()
            self._update_robot_pose()
            payload = self._build_state_payload()
            self.pub_state.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            self.pub_hud.publish(String(data=self._build_hud_text()))
            if self.robot_pose is not None and self._last_candidates:
                self._publish_markers(
                    self.robot_pose,
                    self._last_candidates,
                    self._selected,
                    full=False,
                )
            self._publish_explore_trajectory()
            self._publish_memory_markers()
        except Exception as exc:
            self.get_logger().error(f"publish_state_tick failed: {exc!r}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Explore goal selector")
    parser.add_argument("--config", default=str(ROOT / "configs/nav_yolo_lidar_semantic_explore.yaml"))
    parser.add_argument("--instruction", default=None)
    args = parser.parse_args()

    cfg = load_config(args.config)
    instruction = args.instruction or str(cfg.get("instruction", "find the target"))

    rclpy.init()
    node = ExploreGoalSelector(cfg, instruction)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
