#!/usr/bin/env python3
"""Explore goal selector: semantic map + frontier -> /explore_goal_hint."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path as PathLib
from typing import Any, Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

ROOT = PathLib(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.planning.frontier_extractor import extract_frontiers
from src.planning.grid_astar import plan_path
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
    from tf_transformations import euler_from_quaternion
except ImportError:
    tf2_ros = None
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
    reason: str = ""
    source: Dict[str, Any] = field(default_factory=dict)
    target_class: str = ""

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
            - self.blacklist_penalty
            - self.repeated_observation_penalty
            - self.travel_cost_penalty,
        )


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


class ExploreGoalSelector(Node):
    def __init__(self, cfg: Dict[str, Any], instruction: str):
        super().__init__("explore_goal_selector")
        self.cfg = cfg
        self.instruction = instruction or str(cfg.get("instruction", "find the target"))

        explore = _section(cfg, "semantic_explore")
        frontier_cfg = _section(cfg, "frontier")
        planner_cfg = _section(cfg, "planner")
        scoring = _section(cfg, "candidate_scoring")
        rates = _section(cfg, "rates")
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
        self.valid_sec = float(explore.get("max_hint_age_sec", 1.5))
        self.min_hint_score = float(explore.get("min_hint_score", 0.42))
        self.blacklist_radius_m = float(explore.get("blacklist_radius_m", 0.60))
        self.blacklist_ttl_sec = float(explore.get("blacklist_ttl_sec", 180.0))
        self.reselect_interval_sec = float(explore.get("reselect_interval_sec", 2.0))
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
        self.switch_score_margin = float(explore.get("switch_score_margin", 0.20))
        self.switch_confirm_count = max(1, int(explore.get("switch_confirm_count", 2)))
        self.standoff_m = float(frontier_cfg.get("observation_standoff_m", 0.65))
        self.scoring_weights = scoring
        self.frontier_cfg = frontier_cfg
        self.planner_cfg = planner_cfg
        self.selector_hz = float(rates.get("selector_hz", 2.0))
        self.state_hz = float(rates.get("state_pub_hz", 5.0))

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

        self.pub_hint = self.create_publisher(String, self.hint_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_path = self.create_publisher(
            NavPath, topics.get("explore_path", "/explore_path"), 10
        )
        self.pub_qwen = self.create_publisher(String, "/qwen_text_decision", 10)

        self.pub_candidate_markers = self.create_publisher(MarkerArray, "/explore_candidate_goals", 10)
        self.pub_selected_markers = self.create_publisher(MarkerArray, "/explore_selected_goal", 10)
        self.pub_frontier_markers = self.create_publisher(MarkerArray, "/explore_frontiers", 10)
        self.pub_selection_markers = self.create_publisher(MarkerArray, "/explore_selection_process", 10)
        self.pub_astar_markers = self.create_publisher(MarkerArray, "/explore_astar_markers", 10)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, "/semantic_map_json", self._on_semantic_map, 10)
        self.create_subscription(
            OccupancyGrid, frontier_cfg.get("map_topic", "/map"), self._on_map, map_qos
        )
        self.create_subscription(LaserScan, "/scan_filtered", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "/target_bbox_json", self._on_bbox, 10)
        self.create_subscription(String, "/nav_state", self._on_nav_state, 10)
        self.create_subscription(Odometry, mapping_topics.get("odom", "/odom"), self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self._on_pose, 10)

        self.tf_buffer = None
        self.tf_listener = None
        if tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0 / max(self.selector_hz, 0.1), self._select_tick)
        self.create_timer(1.0 / max(self.state_hz, 0.1), self._publish_state_tick)
        self.get_logger().info(f"explore_goal_selector ready instruction={self.instruction!r}")

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
        self.latest_map = msg

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
        self._blacklist_nav_failure(cand_id, goal_xy, reject_reason or "nav_observe_failed", now)

    def _update_robot_pose(self) -> bool:
        base = self._base_frame
        if self.tf_buffer is not None:
            for frame_id in (self._frame_fixed, self._frame_fallback):
                try:
                    tf = self.tf_buffer.lookup_transform(
                        frame_id,
                        base,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.2),
                    )
                    x = float(tf.transform.translation.x)
                    y = float(tf.transform.translation.y)
                    q = tf.transform.rotation
                    yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
                    self.robot_pose = (x, y, yaw)
                    self._pose_frame = frame_id
                    return True
                except Exception:
                    continue

        if self.latest_odom is not None:
            msg = self.latest_odom
            q = msg.pose.pose.orientation
            yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
            self.robot_pose = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                yaw,
            )
            self._pose_frame = str(msg.header.frame_id or self._frame_fallback)
            return True

        if self.latest_pose is not None:
            msg = self.latest_pose
            q = msg.pose.pose.orientation
            yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
            self.robot_pose = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                yaw,
            )
            self._pose_frame = str(msg.header.frame_id or self._frame_fixed)
            return True

        odom = self.semantic_map.get("robot_pose")
        if isinstance(odom, dict):
            self.robot_pose = (
                float(odom.get("x", 0.0)),
                float(odom.get("y", 0.0)),
                float(odom.get("yaw", 0.0)),
            )
            self._pose_frame = str(odom.get("frame_id", self._frame_fixed))
            return True
        return False

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

    def _standoff_goal(
        self, obj_x: float, obj_y: float, robot_xy: Tuple[float, float]
    ) -> Optional[Tuple[float, float, float]]:
        if self.latest_map is not None:
            from src.planning.frontier_extractor import _find_standoff_goal

            res = float(self.latest_map.info.resolution)
            inflation = max(1, int(math.ceil(self.frontier_cfg.get("inflation_radius_m", 0.25) / res)))
            goal = _find_standoff_goal(
                self.latest_map,
                obj_x,
                obj_y,
                robot_xy,
                self.standoff_m,
                inflation,
                int(self.frontier_cfg.get("free_threshold", 20)),
                int(self.frontier_cfg.get("occupied_threshold", 65)),
                int(self.frontier_cfg.get("unknown_value", -1)),
            )
            if goal:
                yaw = math.atan2(obj_y - goal[1], obj_x - goal[0])
                return goal[0], goal[1], yaw

        dx = robot_xy[0] - obj_x
        dy = robot_xy[1] - obj_y
        norm = math.hypot(dx, dy) or 1.0
        gx = obj_x + (dx / norm) * self.standoff_m
        gy = obj_y + (dy / norm) * self.standoff_m
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

    def _generate_candidates(self, robot_xy: Tuple[float, float]) -> List[ExploreCandidate]:
        candidates: List[ExploreCandidate] = []
        ranges, angles = self._scan_arrays()
        target_class = self.parsed.target_category
        map_aligned = self._pose_matches_map()
        marker_frame = (
            str(self.latest_map.header.frame_id)
            if self.latest_map is not None and map_aligned
            else self._pose_frame
        )
        self._fixed_frame = marker_frame

        if map_aligned:
            for lm in self._confirmed_landmarks():
                cls = str(lm.get("class_name", ""))
                if is_target_match(self.parsed.target_aliases, cls) <= 0:
                    continue
                ox, oy = float(lm.get("x", 0.0)), float(lm.get("y", 0.0))
                standoff = self._standoff_goal(ox, oy, robot_xy)
                if standoff is None:
                    continue
                gx, gy, gyaw = standoff
                dist = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
                landmark_id = str(lm.get("landmark_id") or "").strip()
                cand = ExploreCandidate(
                    candidate_id=f"landmark:{landmark_id}" if landmark_id else make_candidate_id("landmark", ox, oy),
                    mode="target_landmark",
                    goal_xy=(gx, gy),
                    goal_yaw=gyaw,
                    look_at=(ox, oy),
                    semantic_score=1.0,
                    information_gain=0.4,
                    reachability=0.8,
                    novelty=self._novelty_at(gx, gy),
                    safety_margin=min(1.0, self._front_clearance() / 2.0),
                    travel_cost_penalty=min(0.5, dist / 4.0),
                    reason=f"target landmark {cls} standoff",
                    source={"type": "target_landmark", "landmark_id": lm.get("landmark_id"), "class_name": cls},
                    target_class=target_class,
                )
                cand.blacklist_penalty = self._blacklist_penalty(gx, gy, target_class) * float(
                    self.scoring_weights.get("blacklist_penalty", 0.3)
                )
                candidates.append(cand)

        if map_aligned:
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
                ox, oy = float(lm.get("x", 0.0)), float(lm.get("y", 0.0))
                standoff = self._standoff_goal(ox, oy, robot_xy)
                if standoff is None:
                    continue
                gx, gy, gyaw = standoff
                dist = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
                landmark_id = str(lm.get("landmark_id") or "").strip()
                cand = ExploreCandidate(
                    candidate_id=f"landmark:{landmark_id}" if landmark_id else make_candidate_id("landmark", ox, oy),
                    mode="context_landmark",
                    goal_xy=(gx, gy),
                    goal_yaw=gyaw,
                    look_at=(ox, oy),
                    semantic_score=sem,
                    information_gain=0.5,
                    reachability=0.75,
                    novelty=self._novelty_at(gx, gy),
                    safety_margin=min(1.0, self._front_clearance() / 2.0),
                    travel_cost_penalty=min(0.5, dist / 4.0),
                    reason=f"near {cls}, partially unexplored",
                    source={"type": "semantic_context", "landmark_id": lm.get("landmark_id"), "class_name": cls},
                    target_class=target_class,
                )
                cand.blacklist_penalty = self._blacklist_penalty(gx, gy, target_class) * float(
                    self.scoring_weights.get("blacklist_penalty", 0.3)
                )
                candidates.append(cand)

        if map_aligned and bool(self.frontier_cfg.get("enabled", True)) and self.latest_map is not None:
            frontiers = extract_frontiers(
                self.latest_map, (robot_xy[0], robot_xy[1]), self.frontier_cfg, ranges, angles
            )
            self._last_frontiers = frontiers
            for fg in frontiers:
                gx, gy = fg.goal_xy
                cand = ExploreCandidate(
                    candidate_id=make_candidate_id("frontier", gx, gy),
                    mode="frontier",
                    goal_xy=(gx, gy),
                    goal_yaw=fg.goal_yaw,
                    look_at=fg.cluster_center,
                    semantic_score=0.1,
                    information_gain=fg.unknown_gain,
                    reachability=fg.reachability,
                    novelty=0.7,
                    safety_margin=min(1.0, fg.reachability),
                    travel_cost_penalty=min(0.5, fg.distance_m / 4.0),
                    reason="frontier unknown_gain",
                    source={"type": "frontier", "frontier_id": fg.frontier_id},
                    target_class=target_class,
                )
                cand.blacklist_penalty = self._blacklist_penalty(gx, gy, target_class) * float(
                    self.scoring_weights.get("blacklist_penalty", 0.3)
                )
                candidates.append(cand)

        if not candidates and ranges and angles:
            best_i = max(range(len(ranges)), key=lambda i: ranges[i] if 0.1 < ranges[i] < 4.0 else 0.0)
            r = ranges[best_i]
            a = angles[best_i]
            gx = robot_xy[0] + r * 0.5 * math.cos(robot_xy[2] + a)
            gy = robot_xy[1] + r * 0.5 * math.sin(robot_xy[2] + a)
            candidates.append(
                ExploreCandidate(
                    candidate_id=make_candidate_id("free_space", gx, gy),
                    mode="free_space",
                    goal_xy=(gx, gy),
                    goal_yaw=robot_xy[2] + a,
                    look_at=(gx, gy),
                    semantic_score=0.15,
                    information_gain=0.55,
                    reachability=0.75,
                    novelty=0.55,
                    safety_margin=min(1.0, r / 2.0),
                    reason="free_space fallback",
                    source={"type": "free_space"},
                    target_class=target_class,
                )
            )

        w = self.scoring_weights
        for c in candidates:
            c.semantic_score *= float(w.get("semantic_score", 0.25))
            c.information_gain *= float(w.get("information_gain", 0.25))
            c.reachability *= float(w.get("reachability", 0.20))
            c.novelty *= float(w.get("novelty", 0.15))
            c.safety_margin *= float(w.get("safety_margin", 0.10))

        return candidates

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
        return plan_path(
            self.latest_map,
            (robot_xy[0], robot_xy[1]),
            goal_xy,
            self._astar_cfg(),
        )

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
        in_range = [c for c in candidates if self._within_select_range(robot_xy, c)]
        ranked = self._rank_candidates(in_range)
        best_without_path: Optional[ExploreCandidate] = None
        for candidate in ranked:
            if candidate.total_score < self.min_hint_score:
                break
            path = self._plan_candidate_path(robot_xy, candidate.goal_xy)
            if path:
                return candidate, path
            if best_without_path is None:
                best_without_path = candidate
            if self.require_astar_path and not self.astar_fallback_bearing:
                continue
        if best_without_path is not None and (
            not self.require_astar_path or self.astar_fallback_bearing
        ):
            return best_without_path, []
        return None, []

    def _score_candidates(self, candidates: List[ExploreCandidate]) -> Optional[ExploreCandidate]:
        ranked = self._rank_candidates(candidates)
        if not ranked:
            return None
        return ranked[0] if ranked[0].total_score >= self.min_hint_score else None

    def _candidate_summary(self, candidate: ExploreCandidate, rank: int) -> Dict[str, Any]:
        return {
            "rank": rank,
            "candidate_id": candidate.candidate_id,
            "mode": candidate.mode,
            "score": round(candidate.total_score, 3),
            "safe": self._candidate_safe(candidate),
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
                "travel_cost": round(candidate.travel_cost_penalty, 3),
            },
            "source": candidate.source,
        }

    def _update_candidate_debug(self, candidates: List[ExploreCandidate]) -> None:
        ranked = sorted(candidates, key=lambda c: c.total_score, reverse=True)
        self._last_candidate_summaries = [
            self._candidate_summary(c, i + 1) for i, c in enumerate(ranked[:8])
        ]
        best = next((c for c in ranked if self._candidate_safe(c)), None)
        self._last_best_candidate_id = best.candidate_id if best else None

    def _candidate_safe(self, candidate: ExploreCandidate) -> bool:
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
        by_id = {c.candidate_id: c for c in candidates}

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
        if refreshed is None or not self._candidate_safe(refreshed):
            self._status_message = "current_goal_unsafe_cancel"
            self._last_selection_explanation = f"cancel current {current.candidate_id}: missing or unsafe"
            self._pending_switch_id = None
            self._pending_switch_count = 0
            self._selected_since = 0.0
            if best is not None and best.candidate_id != current.candidate_id:
                self._selected_since = now
                self._last_selection_explanation += f"; select replacement {best.candidate_id}"
                return best
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
            payload = {
                "stamp": now,
                "valid_sec": 1.0,
                "mode": "none",
                "score": 0.0,
                "reason": "no semantic/frontier candidate; fallback to free-space search",
            }
            self.pub_hint.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            return

        bearing, dist = self._bearing_distance(robot_xy, selected.goal_xy)
        path: List[Tuple[float, float]] = []
        if bool(self.planner_cfg.get("astar_enabled", False)) and self.latest_map is not None:
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
            path_msg.header.frame_id = self._fixed_frame
            for px, py in path:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = px
                ps.pose.position.y = py
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
        }
        self.pub_hint.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        stamp = self.get_clock().now().to_msg()
        self._publish_astar_markers(robot_xy, selected, stamp)

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
            "free_space": (0.7, 0.7, 0.7, 0.85),
        }
        return palette.get(mode, (1.0, 1.0, 0.2, 0.9))

    def _publish_astar_markers(
        self,
        robot_xy: Tuple[float, float, float],
        selected: Optional[ExploreCandidate],
        stamp,
    ) -> None:
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
        line.points.append(Point(x=robot_xy[0], y=robot_xy[1], z=0.06))
        for px, py in self._last_path:
            line.points.append(Point(x=px, y=py, z=0.06))
        arr.markers.append(line)

        for i, (px, py) in enumerate(self._last_path[:20]):
            wp = self._marker_sphere("astar_waypoints", i + 1, px, py, (0.1, 0.8, 1.0, 0.85), 0.07)
            wp.header.stamp = stamp
            arr.markers.append(wp)

        start = self._marker_sphere("astar_endpoints", 1, robot_xy[0], robot_xy[1], (0.0, 0.8, 1.0, 1.0), 0.1)
        start.header.stamp = stamp
        arr.markers.append(start)
        end = self._marker_sphere(
            "astar_endpoints",
            2,
            selected.goal_xy[0],
            selected.goal_xy[1],
            (0.0, 0.5, 1.0, 1.0),
            0.12,
        )
        end.header.stamp = stamp
        arr.markers.append(end)
        self.pub_astar_markers.publish(arr)

    def _publish_selection_process_markers(
        self,
        robot_xy: Tuple[float, float, float],
        candidates: List[ExploreCandidate],
        selected: Optional[ExploreCandidate],
        stamp,
    ) -> None:
        arr = MarkerArray()
        for ns in ("robot", "links", "pending", "status"):
            arr.markers.append(self._marker_delete_all(ns, stamp))

        robot_m = self._marker_sphere("robot", 0, robot_xy[0], robot_xy[1], (0.0, 0.9, 0.9, 1.0), 0.14)
        robot_m.header.stamp = stamp
        arr.markers.append(robot_m)

        if selected:
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
                Point(x=robot_xy[0], y=robot_xy[1], z=0.08),
                Point(x=selected.goal_xy[0], y=selected.goal_xy[1], z=0.08),
            ]
            arr.markers.append(link)

        if self._pending_switch_id:
            pending = next((c for c in candidates if c.candidate_id == self._pending_switch_id), None)
            if pending:
                pm = self._marker_sphere(
                    "pending",
                    1,
                    pending.goal_xy[0],
                    pending.goal_xy[1],
                    (1.0, 0.45, 0.0, 1.0),
                    0.16,
                )
                pm.header.stamp = stamp
                arr.markers.append(pm)

        status = Marker()
        status.header = self._marker_header(stamp)
        status.ns = "status"
        status.id = 1
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.x = robot_xy[0]
        status.pose.position.y = robot_xy[1]
        status.pose.position.z = 0.45
        status.scale.z = 0.14
        status.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.98)
        sel_id = selected.candidate_id if selected else "none"
        status.text = (
            f"{self._status_message}\n"
            f"sel={sel_id}\n"
            f"{self._last_selection_explanation}"
        )[:180]
        arr.markers.append(status)
        self.pub_selection_markers.publish(arr)

    def _publish_markers(
        self,
        robot_xy: Tuple[float, float, float],
        candidates: List[ExploreCandidate],
        selected: Optional[ExploreCandidate],
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        ranked = sorted(candidates, key=lambda c: c.total_score, reverse=True)
        rank_by_id = {c.candidate_id: i + 1 for i, c in enumerate(ranked)}

        cand_arr = MarkerArray()
        cand_arr.markers.append(self._marker_delete_all("candidates", stamp))
        cand_arr.markers.append(self._marker_delete_all("candidate_labels", stamp))
        for i, c in enumerate(candidates[:12]):
            is_sel = selected is not None and c.candidate_id == selected.candidate_id
            is_pending = c.candidate_id == self._pending_switch_id
            color = self._mode_color(c.mode, selected=is_sel, pending=is_pending)
            scale = 0.18 if is_sel else (0.14 if is_pending else 0.11)
            m = self._marker_sphere("candidates", i, c.goal_xy[0], c.goal_xy[1], color, scale)
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
            label.pose.position.x = c.goal_xy[0]
            label.pose.position.y = c.goal_xy[1]
            label.pose.position.z = 0.28
            label.scale.z = 0.11
            label.color = ColorRGBA(r=1.0, g=1.0, b=0.95, a=0.98)
            label.text = (
                f"#{rank} {c.candidate_id}\n"
                f"{c.mode} {c.total_score:.2f} {safe}"
            )[:80]
            cand_arr.markers.append(label)
        self.pub_candidate_markers.publish(cand_arr)

        sel_arr = MarkerArray()
        sel_arr.markers.append(self._marker_delete_all("selected", stamp))
        sel_arr.markers.append(self._marker_delete_all("selected_arrow", stamp))
        sel_arr.markers.append(self._marker_delete_all("selected_label", stamp))
        if selected:
            m = self._marker_sphere("selected", 0, selected.goal_xy[0], selected.goal_xy[1], (0.0, 1.0, 0.0, 1.0), 0.22)
            m.header.stamp = stamp
            sel_arr.markers.append(m)
            arrow = Marker()
            arrow.header = m.header
            arrow.ns = "selected_arrow"
            arrow.id = 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=selected.goal_xy[0], y=selected.goal_xy[1], z=0.05),
                Point(x=selected.look_at[0], y=selected.look_at[1], z=0.05),
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
            sel_label.pose.position.x = selected.goal_xy[0]
            sel_label.pose.position.y = selected.goal_xy[1]
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
            m = self._marker_sphere("frontiers", i, fg.goal_xy[0], fg.goal_xy[1], (0.2, 0.4, 1.0, 0.8), 0.08)
            m.header.stamp = stamp
            fr_arr.markers.append(m)
            fl = Marker()
            fl.header = m.header
            fl.ns = "frontier_labels"
            fl.id = 200 + i
            fl.type = Marker.TEXT_VIEW_FACING
            fl.action = Marker.ADD
            fl.pose.position.x = fg.goal_xy[0]
            fl.pose.position.y = fg.goal_xy[1]
            fl.pose.position.z = 0.2
            fl.scale.z = 0.09
            fl.color = ColorRGBA(r=0.6, g=0.8, b=1.0, a=0.95)
            fl.text = f"{fg.frontier_id}\ngain={fg.unknown_gain:.2f}"
            fr_arr.markers.append(fl)
        self.pub_frontier_markers.publish(fr_arr)

        self._publish_selection_process_markers(robot_xy, candidates, selected, stamp)
        self._publish_astar_markers(robot_xy, selected, stamp)

    def _select_tick(self) -> None:
        if not self.explore_enabled:
            self._status_message = "semantic_explore.disabled"
            return
        if self._target_visible():
            self._status_message = "target_visible_skip"
            return
        if not self._update_robot_pose() or self.robot_pose is None:
            self._status_message = "waiting_robot_pose"
            return
        if self.latest_map is None:
            self._status_message = "waiting_map"
            return
        now = time.time()
        if now - self._last_select_time < self.reselect_interval_sec and self._selected is not None:
            self._publish_hint(self._selected, self.robot_pose)
            self._publish_markers(self.robot_pose, self._last_candidates, self._selected)
            return
        self._last_select_time = now
        robot_xy = self.robot_pose
        candidates = self._generate_candidates(robot_xy)
        self._last_candidates = candidates
        self._update_candidate_debug(candidates)
        best, _path = self._pick_navigable_candidate(robot_xy, candidates)
        selected = self._choose_sticky_candidate(best, candidates, now)
        self._selected = selected
        if selected is None:
            self._status_message = "no_candidate_above_threshold"
        elif self._status_message not in (
            "keep_current_min_time",
            "keep_current_score_margin",
            "switch_candidate_pending",
            "switch_candidate_confirmed",
            "current_goal_unsafe_cancel",
        ):
            self._status_message = "selected"
        self._publish_hint(selected, robot_xy)
        self._publish_markers(robot_xy, candidates, selected)

    def _publish_state_tick(self) -> None:
        payload = {
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
            "map_frame": str(self.latest_map.header.frame_id) if self.latest_map else None,
            "pose_map_aligned": self._pose_matches_map(),
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
        }
        self.pub_state.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main() -> None:
    parser = argparse.ArgumentParser(description="Explore goal selector")
    parser.add_argument("--config", default=str(ROOT / "configs/nav_yolo_lidar_semantic_explore.yaml"))
    parser.add_argument("--instruction", default=None)
    args = parser.parse_args()

    cfg = load_config(args.config)
    instruction = args.instruction or str(cfg.get("instruction", "find the target"))

    rclpy.init()
    node = ExploreGoalSelector(cfg, instruction)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
