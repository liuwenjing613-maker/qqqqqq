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

        self.pub_hint = self.create_publisher(String, self.hint_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_path = self.create_publisher(
            NavPath, topics.get("explore_path", "/explore_path"), 10
        )
        self.pub_qwen = self.create_publisher(String, "/qwen_text_decision", 10)

        self.pub_candidate_markers = self.create_publisher(MarkerArray, "/explore_candidate_goals", 10)
        self.pub_selected_markers = self.create_publisher(MarkerArray, "/explore_selected_goal", 10)
        self.pub_frontier_markers = self.create_publisher(MarkerArray, "/explore_frontiers", 10)

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

    def _on_nav_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        failed_count = int(state.get("explore_failed_count", 0) or 0)
        if failed_count <= self._last_nav_failed_count:
            return
        self._last_nav_failed_count = failed_count
        cand_id = str(state.get("explore_candidate_id", ""))
        goal_pose = None
        if self._selected:
            goal_pose = self._selected.goal_xy
        if goal_pose:
            now = time.time()
            self.blacklist_regions.append(
                BlacklistRegion(
                    candidate_id=cand_id,
                    target_class=str(self.parsed.target_category),
                    x=goal_pose[0],
                    y=goal_pose[1],
                    radius=self.blacklist_radius_m,
                    reason="nav_observe_failed",
                    created_time=now,
                    expire_time=now + self.blacklist_ttl_sec,
                )
            )

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
                cand = ExploreCandidate(
                    candidate_id=self._next_cand_id(),
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
                cand = ExploreCandidate(
                    candidate_id=self._next_cand_id(),
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
                    candidate_id=self._next_cand_id(),
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
                    candidate_id=self._next_cand_id(),
                    mode="free_space",
                    goal_xy=(gx, gy),
                    goal_yaw=robot_xy[2] + a,
                    look_at=(gx, gy),
                    semantic_score=0.0,
                    information_gain=0.2,
                    reachability=0.6,
                    novelty=0.5,
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

    def _score_candidates(self, candidates: List[ExploreCandidate]) -> Optional[ExploreCandidate]:
        if not candidates:
            return None
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
        ranked = sorted(candidates, key=lambda c: c.total_score, reverse=True)
        return ranked[0] if ranked[0].total_score >= self.min_hint_score else None

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
        }
        self.pub_hint.publish(String(data=json.dumps(payload, ensure_ascii=False)))

        if bool(self.planner_cfg.get("astar_enabled", False)) and self.latest_map is not None:
            path = plan_path(
                self.latest_map,
                (robot_xy[0], robot_xy[1]),
                selected.goal_xy,
                self.frontier_cfg,
            )
            self._last_path = path
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

    def _publish_markers(self, candidates: List[ExploreCandidate], selected: Optional[ExploreCandidate]) -> None:
        stamp = self.get_clock().now().to_msg()
        cand_arr = MarkerArray()
        for i, c in enumerate(candidates[:12]):
            m = self._marker_sphere("candidates", i, c.goal_xy[0], c.goal_xy[1], (1.0, 1.0, 0.0, 0.9), 0.12)
            m.header.stamp = stamp
            cand_arr.markers.append(m)
            label = Marker()
            label.header = m.header
            label.ns = "candidate_labels"
            label.id = 100 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = c.goal_xy[0]
            label.pose.position.y = c.goal_xy[1]
            label.pose.position.z = 0.25
            label.scale.z = 0.12
            label.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=0.95)
            label.text = f"{c.candidate_id} {c.mode[:8]} {c.total_score:.2f}"
            cand_arr.markers.append(label)
        self.pub_candidate_markers.publish(cand_arr)

        sel_arr = MarkerArray()
        if selected:
            m = self._marker_sphere("selected", 0, selected.goal_xy[0], selected.goal_xy[1], (0.0, 1.0, 0.0, 1.0), 0.2)
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
        self.pub_selected_markers.publish(sel_arr)

        fr_arr = MarkerArray()
        for i, fg in enumerate(self._last_frontiers[:12]):
            m = self._marker_sphere("frontiers", i, fg.goal_xy[0], fg.goal_xy[1], (0.2, 0.4, 1.0, 0.8), 0.08)
            m.header.stamp = stamp
            fr_arr.markers.append(m)
        self.pub_frontier_markers.publish(fr_arr)

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
            self._publish_markers(self._last_candidates, self._selected)
            return
        self._last_select_time = now
        robot_xy = self.robot_pose
        candidates = self._generate_candidates(robot_xy)
        self._last_candidates = candidates
        selected = self._score_candidates(candidates)
        self._selected = selected
        self._status_message = "selected" if selected else "no_candidate_above_threshold"
        self._publish_hint(selected, robot_xy)
        self._publish_markers(candidates, selected)

    def _publish_state_tick(self) -> None:
        payload = {
            "state": "SEMANTIC_EXPLORE" if self._selected else "SEARCH",
            "status": self._status_message,
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
            "selected_candidate": self._selected.candidate_id if self._selected else None,
            "selected_reason": self._selected.reason if self._selected else "",
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
