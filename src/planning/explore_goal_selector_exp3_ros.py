#!/usr/bin/env python3
"""ROS explore goal selector node using EXP3 picking logic on top of EXP2 infrastructure."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.planning.explore_goal_selector_exp2 import (  # noqa: E402
    ExploreCandidate,
    ExploreGoalSelector,
    load_config,
    make_candidate_id,
)
from src.planning.explore_goal_selector_exp3 import ExploreGoalSelectorExp3  # noqa: E402


def _section(raw: Dict[str, Any], key: str) -> Dict[str, Any]:
    block = raw.get(key, {})
    return block if isinstance(block, dict) else {}


def _candidate_to_exp3_dict(candidate: ExploreCandidate) -> Dict[str, Any]:
    return {
        "x": candidate.goal_xy[0],
        "y": candidate.goal_xy[1],
        "score": candidate.total_score,
        "unknown_gain": candidate.information_gain,
        "reachable": candidate.reachability >= 0.5,
        "source": candidate.mode,
        "candidate_id": candidate.candidate_id,
        "reason": candidate.reason,
    }


def _goal_dict_xy(goal: Any) -> Optional[Tuple[float, float]]:
    if goal is None:
        return None
    if isinstance(goal, dict):
        x = goal.get("x")
        y = goal.get("y")
        if x is not None and y is not None:
            return float(x), float(y)
    x = getattr(goal, "x", None)
    y = getattr(goal, "y", None)
    if x is not None and y is not None:
        return float(x), float(y)
    return None


class ExploreGoalSelectorExp3Node(ExploreGoalSelector):
    """EXP2 ROS node with EXP3 sector/picking behavior and the same Foxglove topics."""

    def __init__(self, cfg: Dict[str, Any], instruction: str):
        super().__init__(cfg, instruction)
        self._exp3 = ExploreGoalSelectorExp3(cfg, logger=self.get_logger())
        # EXP3 owns sector lifecycle; disable EXP2 direction lock to avoid double filtering.
        self.direction_lock_enabled = False
        self.direction_sector_count = max(4, int(self._exp3.cfg.sector_count))
        self._exp3_debug: Dict[str, Any] = {}
        self.get_logger().info(
            "EXP3 ROS selector active "
            f"(sectors={self._exp3.cfg.sector_count}, "
            f"min_score={self._exp3.cfg.min_candidate_score})"
        )

    def _refresh_exp3_sector_preview(self) -> None:
        """Keep sector counts / active sector visible in Foxglove between full picks."""
        if self.robot_pose is None:
            return
        pool = self._last_candidates or self._last_valid_candidates or []
        if not pool:
            return
        upstream = [_candidate_to_exp3_dict(candidate) for candidate in pool]
        cvs = self._exp3.normalize_candidates(upstream, self.robot_pose)
        if not cvs:
            return
        sector_counts = self._exp3._sector_counts(cvs)
        now = time.time()
        sid = self._exp3._choose_active_sector(cvs, self.robot_pose, now)
        self._exp3.active_sector_id = sid
        preview = dict(self._exp3_debug)
        preview["sector_counts"] = sector_counts
        preview["active_sector_id"] = sid
        if "selected" not in preview:
            preview["selected"] = None
        self._exp3_debug = preview
        self._sync_active_sector_from_exp3()

    def _sync_active_sector_from_exp3(self) -> None:
        sid = self._exp3.active_sector_id
        if sid is None:
            return
        self.active_area_id = f"sector_{int(sid):02d}"
        self._last_direction_reason = str(
            (self._exp3_debug.get("recovery_level") or "exp3_count_based_sector")
        )

    def _sector_counts_summary(self) -> Dict[str, int]:
        counts = self._exp3_debug.get("sector_counts") or {}
        if isinstance(counts, dict) and counts:
            return {
                f"sector_{int(sid):02d}": int(count)
                for sid, count in counts.items()
            }
        return super()._sector_counts_summary()

    def _match_candidate(
        self,
        goal: Any,
        valid_candidates: List[ExploreCandidate],
        raw_candidates: List[ExploreCandidate],
    ) -> Optional[ExploreCandidate]:
        goal_id = goal.get("candidate_id") if isinstance(goal, dict) else None
        if goal_id:
            for pool in (valid_candidates, raw_candidates):
                for candidate in pool:
                    if candidate.candidate_id == goal_id:
                        return candidate

        goal_xy = _goal_dict_xy(goal)
        if goal_xy is None:
            return None
        gx, gy = goal_xy
        best: Optional[ExploreCandidate] = None
        best_dist = float("inf")
        for pool in (valid_candidates, raw_candidates):
            for candidate in pool:
                cx, cy = candidate.goal_xy
                dist = math.hypot(cx - gx, cy - gy)
                if dist < best_dist:
                    best_dist = dist
                    best = candidate
        if best is not None and best_dist <= 0.75:
            return best
        return None

    def _candidate_from_exp3_goal(
        self,
        goal: Dict[str, Any],
        robot_xy: Tuple[float, float, float],
    ) -> ExploreCandidate:
        x = float(goal["x"])
        y = float(goal["y"])
        source = str(goal.get("source", "exp3"))
        mode = "exp3_lidar" if source == "exp3_lidar_ray" else "exp3"
        dist = math.hypot(x - robot_xy[0], y - robot_xy[1])
        score = float(goal.get("exp3_score", goal.get("score", 0.2)))
        unknown_gain = float(goal.get("unknown_gain", 0.2))
        return ExploreCandidate(
            candidate_id=make_candidate_id(mode, x, y),
            mode=mode,
            goal_xy=(x, y),
            goal_yaw=math.atan2(y - robot_xy[1], x - robot_xy[0]),
            look_at=(x, y),
            semantic_score=0.1,
            information_gain=unknown_gain,
            reachability=0.85,
            novelty=0.4,
            safety_margin=0.7,
            travel_cost_penalty=min(1.0, dist / max(self.max_goal_select_distance_m, 0.1)),
            reason=str(goal.get("exp3_reason", goal.get("reason", "exp3_generated"))),
            source={"type": mode, "exp3": True, **goal},
            sector_id=f"sector_{int(goal.get('exp3_sector_id', 0)):02d}",
            forced_below_threshold=True,
            forced_pick=True,
        )

    def _pick_best_valid_candidate(
        self,
        robot_xy: Tuple[float, float],
        valid_candidates: List[ExploreCandidate],
    ) -> Tuple[Optional[ExploreCandidate], List[Tuple[float, float]]]:
        if not valid_candidates and not (self._last_candidates or []):
            return None, []

        pose = self.robot_pose or (robot_xy[0], robot_xy[1], 0.0)
        upstream = [
            _candidate_to_exp3_dict(candidate)
            for candidate in (self._last_candidates or valid_candidates)
        ]
        goal, debug = self._exp3.select_goal_with_debug(
            candidates=upstream,
            pose=pose,
            scan=self.latest_scan,
        )
        self._exp3_debug = dict(debug)
        self._sync_active_sector_from_exp3()

        if goal is None:
            self._last_selection_explanation = str(debug.get("reason") or "exp3_no_goal")
            self._last_pick_stats = {
                **getattr(self, "_last_pick_stats", {}),
                "exp3": debug,
                "active_area_id": self.active_area_id,
            }
            return None, []

        matched = self._match_candidate(goal, valid_candidates, self._last_candidates or [])
        if matched is None and isinstance(goal, dict):
            matched = self._candidate_from_exp3_goal(goal, pose)
        if matched is None:
            self._last_selection_explanation = "exp3_goal_match_failed"
            return None, []

        matched.forced_below_threshold = True
        matched.forced_pick = True
        matched.sector_id = self.active_area_id or matched.sector_id
        matched.reason = str(
            goal.get("exp3_reason", debug.get("selected", {}).get("reason", matched.reason))
        )
        matched.source = dict(matched.source)
        matched.source["exp3_debug"] = debug

        path = matched.source.get("planned_path")
        if not isinstance(path, list) or len(path) < 2:
            path = self._plan_candidate_path(robot_xy, matched.goal_xy)

        self._last_pick_stats = {
            **getattr(self, "_last_pick_stats", {}),
            "selected": matched.candidate_id,
            "active_area_id": self.active_area_id,
            "exp3": debug,
        }
        self._last_selection_explanation = (
            f"exp3 score={debug.get('selected', {}).get('score', '?')} "
            f"sector={self.active_area_id} "
            f"recovery={debug.get('recovery_level', '-')}"
        )
        return matched, list(path) if path else []

    def _build_hud_text(self) -> str:
        text = super()._build_hud_text()
        selected = self._exp3_debug.get("selected") or {}
        if selected:
            extra = (
                f"EXP3: sid={selected.get('sector_id')} "
                f"count={selected.get('sector_count')} "
                f"score={selected.get('score')} "
                f"recovery={self._exp3_debug.get('recovery_level', '-')}"
            )
            return f"{text}\n{extra}"
        return text

    def _build_state_payload(self) -> Dict[str, Any]:
        self._refresh_exp3_sector_preview()
        payload = super()._build_state_payload()
        summary = payload.get("summary", {})
        if isinstance(summary, dict):
            summary["profile"] = "exp3"
            summary["exp3"] = {
                "active_sector_id": self._exp3.active_sector_id,
                "recovery_level": self._exp3_debug.get("recovery_level"),
                "selected": self._exp3_debug.get("selected"),
                "sector_counts": self._exp3_debug.get("sector_counts"),
            }
            payload["summary"] = summary
        payload["exp3_debug"] = self._exp3_debug
        return payload


def main() -> None:
    parser = argparse.ArgumentParser(description="EXP3 explore goal selector (ROS)")
    parser.add_argument(
        "--config",
        default=str(ROOT / "configs/nav_yolo_lidar_semantic_explore_exp3.yaml"),
    )
    parser.add_argument("--instruction", default=None)
    args = parser.parse_args()

    cfg = load_config(args.config)
    instruction = args.instruction or str(cfg.get("instruction", "find the target"))

    rclpy.init()
    node = ExploreGoalSelectorExp3Node(cfg, instruction)
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
