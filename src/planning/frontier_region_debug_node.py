#!/usr/bin/env python3
"""ROS 2 observation-only frontier region diagnostic node."""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import ColorRGBA, Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.planning.frontier_region_debug_core import (  # noqa: E402
    FrontierAnalysisResult,
    MapMetadata,
    RobotPose2D,
    _CV2_AVAILABLE,
    analyze_frontier_regions,
    build_region_geometry_payload,
    build_region_snapshot_payload,
    generate_snapshot_id,
    grid_row_to_image_y,
    grid_to_world,
    occupancy_grid_to_array,
    render_annotated_map,
    result_to_dict,
    validate_config,
    world_to_grid,
)
from src.planning.region_candidate_guard import (  # noqa: E402
    ObservationWindow,
    RegionTrack,
    guard_regions_for_cycle,
    track_to_dict,
)
from src.planning.region_history_store import (  # noqa: E402
    ObservationPose,
    RegionHistoryStore,
)
from src.planning.robot_trajectory_store import RobotTrajectoryStore  # noqa: E402

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _observation_to_dict(obs: ObservationWindow) -> Dict[str, Any]:
    return {
        "state": obs.state,
        "window_id": obs.window_id,
        "accumulated_rotation_deg": obs.accumulated_rotation_deg,
        "translation_during_scan_m": obs.translation_during_scan_m,
        "settle_elapsed_s": obs.settle_elapsed_s,
        "map_stable_cycle_count": obs.map_stable_cycle_count,
    }


def _quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class JsonlWriter:
    def __init__(self, path: Path, flush: bool = True) -> None:
        self.path = path
        self.flush = flush
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = open(self.path, "a", encoding="utf-8")

    def write(self, record: Dict[str, Any]) -> None:
        self._fp.write(json.dumps(record, ensure_ascii=False) + "\n")
        if self.flush:
            self._fp.flush()

    def close(self) -> None:
        self._fp.close()


class FrontierRegionDebugNode(Node):
    NODE_NAME = "frontier_region_debug"

    def __init__(self, cfg: Dict[str, Any], run_dir: Path) -> None:
        super().__init__(self.NODE_NAME)
        self.cfg = cfg
        self.run_dir = run_dir
        self.cycle_id = 0
        self.last_cycle_status = "INIT"
        self._latest_grid: Optional[OccupancyGrid] = None
        self._latest_meta: Optional[MapMetadata] = None
        self._latest_result: Optional[FrontierAnalysisResult] = None
        self._latest_robot: Optional[RobotPose2D] = None
        self._latest_annotated_path: Optional[Path] = None
        self._snapshot_count = 0
        self._latest_snapshot_id: Optional[str] = None
        self._tf_ok = False
        self._tracks: Dict[str, RegionTrack] = {}
        self._observation = ObservationWindow()
        history_path = ROOT / "runtime/qwen_region_debug/region_history.json"
        self._history = RegionHistoryStore.load(history_path)
        self._history_path = history_path
        self._last_guard_summary: Dict[str, Any] = {}
        traj_cfg = cfg.get("trajectory", {})
        traj_runtime = ROOT / str(traj_cfg.get("runtime_file", "runtime/qwen_region_debug/trajectory_session.json"))
        self._trajectory = RobotTrajectoryStore(cfg, traj_runtime, map_frame=str(cfg.get("frames", {}).get("map", "map")))
        self._trajectory_status = "INIT"
        self._last_visited_grid_sig: Optional[Tuple[int, int, int, float, float]] = None

        topics = cfg["topics"]
        frames = cfg["frames"]
        self.map_frame = str(frames.get("map", "map"))
        self.robot_frame = str(frames.get("robot", "base_link"))
        self.tf_timeout_s = float(frames.get("tf_timeout_s", 0.5))

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, topics["map"], self._map_cb, map_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_heartbeat = self.create_publisher(String, topics["heartbeat"], 10)
        self.pub_map_health = self.create_publisher(String, topics["map_health_json"], 10)
        self.pub_frontier_stats = self.create_publisher(String, topics["frontier_stats_json"], 10)
        self.pub_regions = self.create_publisher(String, topics["regions_json"], 10)
        self.pub_rejections = self.create_publisher(String, topics["rejections_json"], 10)
        self.pub_frontier_markers = self.create_publisher(MarkerArray, topics["frontier_markers"], 10)
        self.pub_region_markers = self.create_publisher(MarkerArray, topics["region_markers"], 10)
        self.pub_rejected_markers = self.create_publisher(
            MarkerArray, topics["rejected_region_markers"], 10
        )
        self.pub_annotated = self.create_publisher(CompressedImage, topics["annotated_map"], 10)
        snap_topic = topics.get(
            "region_snapshot_json", "/qwen_explore_debug/region_snapshot_json"
        )
        self.pub_snapshot = self.create_publisher(String, snap_topic, 10)

        traj_topics = cfg.get("topics", {})
        self.pub_trajectory_path = self.create_publisher(
            NavPath, traj_topics.get("trajectory_path", "/qwen_explore_debug/trajectory_path"), 10
        )
        self.pub_trajectory_markers = self.create_publisher(
            MarkerArray,
            traj_topics.get("trajectory_markers", "/qwen_explore_debug/trajectory_markers"),
            10,
        )
        self.pub_visited_area_grid = self.create_publisher(
            OccupancyGrid,
            traj_topics.get("visited_area_grid", "/qwen_explore_debug/visited_area_grid"),
            10,
        )
        self.pub_trajectory_json = self.create_publisher(
            String, traj_topics.get("trajectory_json", "/qwen_explore_debug/trajectory_json"), 10
        )

        snap_cfg = cfg.get("snapshot", {})
        service_name = str(
            snap_cfg.get("service_name", "/qwen_explore_debug/capture_region_snapshot")
        )
        self._snapshot_expires_s = float(snap_cfg.get("expires_after_s", 300.0))
        self.create_service(Trigger, service_name, self._capture_snapshot_cb)
        obs_service = str(cfg.get("observation_gate", {}).get(
            "start_service", "/qwen_explore_debug/start_observation_window"
        ))
        self.create_service(Trigger, obs_service, self._start_observation_window_cb)
        reset_service = str(
            cfg.get("trajectory", {}).get(
                "reset_service", "/qwen_explore_debug/reset_trajectory_history"
            )
        )
        self.create_service(Trigger, reset_service, self._reset_trajectory_history_cb)

        log_cfg = cfg.get("logging", {})
        self._flush = bool(log_cfg.get("flush_every_record", True))
        self._write_jsonl = bool(log_cfg.get("write_jsonl", True))
        self._write_image = bool(log_cfg.get("write_annotated_image", True))
        if self._write_jsonl:
            self._jl_health = JsonlWriter(run_dir / "map_health.jsonl", self._flush)
            self._jl_cycles = JsonlWriter(run_dir / "frontier_cycles.jsonl", self._flush)
            self._jl_regions = JsonlWriter(run_dir / "regions.jsonl", self._flush)
            self._jl_rejections = JsonlWriter(run_dir / "rejections.jsonl", self._flush)
            self._jl_events = JsonlWriter(run_dir / "events.jsonl", self._flush)
            self._jl_tracks = JsonlWriter(run_dir / "region_tracks.jsonl", self._flush)
            self._jl_obs = JsonlWriter(run_dir / "observation_windows.jsonl", self._flush)
            self._jl_geo = JsonlWriter(run_dir / "geometric_scores.jsonl", self._flush)
            self._jl_traj = JsonlWriter(run_dir / "trajectory_samples.jsonl", self._flush)
        else:
            self._jl_health = self._jl_cycles = self._jl_regions = None
            self._jl_rejections = self._jl_events = None
            self._jl_tracks = self._jl_obs = self._jl_geo = None
            self._jl_traj = None

        period = float(cfg.get("node", {}).get("analysis_period_s", 1.0))
        self.create_timer(period, self._analysis_timer_cb)
        if bool(cfg.get("trajectory", {}).get("enabled", True)):
            traj_period = float(cfg.get("trajectory", {}).get("sample_period_s", 1.0))
            self.create_timer(traj_period, self._trajectory_timer_cb)
        self.get_logger().info(
            f"Frontier region debug node started (observation-only, period={period}s)"
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._latest_grid = msg
        _, meta = occupancy_grid_to_array(msg)
        self._latest_meta = meta

    def _lookup_robot_pose(self) -> Tuple[Optional[RobotPose2D], Optional[str], List[Dict[str, Any]]]:
        robot, code, errors, _, _ = self._lookup_robot_tf_detail()
        return robot, code, errors

    def _lookup_robot_tf_detail(
        self,
    ) -> Tuple[Optional[RobotPose2D], Optional[str], List[Dict[str, Any]], float, float]:
        errors: List[Dict[str, Any]] = []
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_s),
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = _quat_to_yaw(q)
            now_ns = self.get_clock().now().nanoseconds
            tf_ns = tf.header.stamp.sec * 1_000_000_000 + tf.header.stamp.nanosec
            tf_age_s = max(0.0, (now_ns - tf_ns) * 1e-9)
            tf_stamp_sec = float(tf.header.stamp.sec) + float(tf.header.stamp.nanosec) * 1e-9
            return (
                RobotPose2D(x=float(t.x), y=float(t.y), yaw_rad=yaw),
                None,
                errors,
                tf_stamp_sec,
                tf_age_s,
            )
        except TransformException as exc:
            msg = str(exc)
            if "extrapolation" in msg.lower():
                code = "TF_EXTRAPOLATION_ERROR"
            elif "timeout" in msg.lower():
                code = "TF_LOOKUP_TIMEOUT"
            else:
                code = "TF_MAP_BASE_MISSING"
            errors.append({"code": code, "detail": msg})
            return None, code, errors, 0.0, 0.0
        except Exception as exc:  # pragma: no cover
            errors.append({"code": "TF_INVALID_QUATERNION", "detail": str(exc)})
            return None, "TF_INVALID_QUATERNION", errors, 0.0, 0.0

    def _publish_heartbeat(
        self,
        map_received: bool,
        tf_ok: bool,
    ) -> None:
        merge_enabled = bool(self.cfg.get("region_merge", {}).get("enabled", True))
        payload = {
            "node": self.NODE_NAME,
            "alive": True,
            "cycle_id": self.cycle_id,
            "motion_enabled": False,
            "qwen_enabled": False,
            "nav2_enabled": False,
            "cmd_vel_publisher_created": False,
            "map_received": map_received,
            "tf_ok": tf_ok,
            "last_cycle_status": self.last_cycle_status,
            "merge_enabled": merge_enabled,
            "snapshot_service_ready": True,
            "latest_snapshot_id": self._latest_snapshot_id,
            "snapshot_count": self._snapshot_count,
            "observation_state": self._observation.state,
            "accumulated_rotation_deg": round(self._observation.accumulated_rotation_deg, 2),
            "translation_during_scan_m": round(self._observation.translation_during_scan_m, 3),
            "settle_elapsed_s": round(self._observation.settle_elapsed_s, 2),
            "map_stable_cycle_count": self._observation.map_stable_cycle_count,
            "snapshot_gate_ready": self._observation.snapshot_gate_ready(
                self._last_guard_summary.get("snapshot_eligible_count", 0)
            ),
            "guard_summary": self._last_guard_summary,
            "trajectory_session_id": self._trajectory.trajectory_session_id,
            "trajectory_revision": self._trajectory.trajectory_revision,
            "trajectory_vertex_count": len(self._trajectory.session.vertices),
            "trajectory_length_m": round(self._trajectory.session.trajectory_length_m, 3),
            "trajectory_status": self._trajectory_status,
            "timestamp": _utc_now_iso(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_heartbeat.publish(msg)

    def _log_jsonl(self, writer: Optional[JsonlWriter], record: Dict[str, Any]) -> None:
        if writer is not None:
            writer.write(record)

    def _write_latest(self, name: str, payload: Dict[str, Any]) -> None:
        path = self.run_dir / name
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _analysis_timer_cb(self) -> None:
        self.cycle_id += 1
        cycle = self.cycle_id
        try:
            if self._latest_grid is None or self._latest_meta is None:
                self.last_cycle_status = "WAITING_FOR_MAP"
                self.get_logger().warn(f"[MAP_HEALTH] cycle={cycle} status=WAITING_FOR_MAP map_received=false")
                self._publish_heartbeat(False, False)
                return

            robot, tf_code, tf_errors = self._lookup_robot_pose()
            tf_ok = robot is not None
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            map_age_s = now_sec - self._latest_meta.stamp_sec if self._latest_meta.stamp_sec > 0 else 0.0

            data, meta = occupancy_grid_to_array(self._latest_grid)

            if not tf_ok:
                health_payload = {
                    "cycle_id": cycle,
                    "status": "REJECTED",
                    "errors": tf_errors,
                    "warnings": [],
                    "map_received": True,
                    "map_stamp": self._latest_meta.stamp_sec,
                    "map_age_s": map_age_s,
                    "frame_id": meta.frame_id,
                    "width": meta.width,
                    "height": meta.height,
                    "resolution": meta.resolution,
                    "origin": [meta.origin_x, meta.origin_y],
                    "tf_ok": False,
                    "tf_error": tf_code,
                    "decision": "SKIP_FRONTIER_EXTRACTION",
                }
                self._emit_map_health_console(health_payload, robot=None)
                msg = String()
                msg.data = json.dumps(health_payload, ensure_ascii=False)
                self.pub_map_health.publish(msg)
                self._log_jsonl(self._jl_health, health_payload)
                self._write_latest("latest_map_health.json", health_payload)
                self.last_cycle_status = "TF_MISSING"
                self._tf_ok = False
                self._publish_heartbeat(True, False)
                return

            assert robot is not None
            result = analyze_frontier_regions(data, meta, robot, self.cfg, cycle_id=cycle)
            self._history.apply_to_tracks(self._tracks)  # type: ignore[arg-type]
            self._tracks, guarded = guard_regions_for_cycle(
                result.regions,
                self._tracks,
                cycle,
                now_sec,
                self.cfg,
                self._history.observation_poses,
                trajectory_session=self._trajectory.session,
            )
            for region, metrics, track in guarded:
                region.track_id = metrics.track_id
                region.stable = metrics.stable
                region.persistence_cycles = metrics.persistence_cycles
                region.age_s = metrics.age_s
                region.centroid_drift_m = metrics.centroid_drift_m
                region.bearing_drift_deg = metrics.bearing_drift_deg
                region.guard_cell_count_change_ratio = metrics.cell_count_change_ratio
                region.stability_rejection_reasons = list(metrics.stability_rejection_reasons)
                region.near_robot_penalty = metrics.near_robot_penalty
                region.recent_observation_penalty = metrics.recent_observation_penalty
                region.distance_to_nearest_observation_pose_m = metrics.distance_to_nearest_observation_pose_m
                region.nearest_trajectory_distance_m = metrics.nearest_trajectory_distance_m
                region.nearby_trajectory_vertex_count = metrics.nearby_trajectory_vertex_count
                region.nearby_recent_trajectory_count = metrics.nearby_recent_trajectory_count
                region.last_nearby_visit_age_s = metrics.last_nearby_visit_age_s
                region.trajectory_density_score = metrics.trajectory_density_score
                region.trajectory_novelty_score = metrics.trajectory_novelty_score
                region.trajectory_revisit_penalty = metrics.trajectory_revisit_penalty
                region.geo_score_before_trajectory = metrics.geo_score_before_trajectory
                region.geo_score_after_trajectory = metrics.geo_score_after_trajectory
                region.geo_score = metrics.geo_score
                region.geo_rank = metrics.geo_rank
                region.score_components = dict(metrics.score_components)
                region.penalty_components = dict(metrics.penalty_components)
                region.score_explanation = metrics.score_explanation
                region.blacklisted = track.blacklisted
                region.snapshot_eligible = metrics.snapshot_eligible and region.accepted
                region.visited_count = track.visit_count
                region.navigation_failure_count = track.navigation_failure_count
            self._observation.update(robot, now_sec, result, result.map_health, self.cfg)
            eligible = sum(1 for r in result.regions if r.snapshot_eligible and r.stable)
            self._last_guard_summary = {
                "track_count": len(self._tracks),
                "stable_count": sum(1 for r in result.regions if r.stable),
                "snapshot_eligible_count": eligible,
                "near_hard_reject": sum(1 for r in result.regions if "REGION_TOO_CLOSE_HARD" in r.stability_rejection_reasons),
                "near_soft_penalty": sum(1 for r in result.regions if r.near_robot_penalty > 0),
                "recent_observation_penalty": sum(1 for r in result.regions if r.recent_observation_penalty > 0),
            }
            self._log_jsonl(self._jl_tracks, {"cycle_id": cycle, "tracks": [track_to_dict(t) for t in self._tracks.values()]})
            self._log_jsonl(self._jl_obs, {"cycle_id": cycle, "observation": _observation_to_dict(self._observation)})
            for region in result.regions:
                self._log_jsonl(self._jl_geo, {
                    "cycle_id": cycle,
                    "region_id": region.region_id,
                    "track_id": region.track_id,
                    "geo_score": region.geo_score,
                    "geo_rank": region.geo_rank,
                    "score_components": region.score_components,
                    "penalty_components": region.penalty_components,
                    "snapshot_eligible": region.snapshot_eligible,
                    "stability_rejection_reasons": region.stability_rejection_reasons,
                })
            self._latest_result = result
            self._latest_robot = robot
            self._tf_ok = True
            self._process_result(result, meta, robot, map_age_s)
            self.last_cycle_status = result.stats.status
            self._publish_heartbeat(True, True)
        except Exception as exc:
            self.last_cycle_status = "CYCLE_ERROR"
            tb = traceback.format_exc()
            self.get_logger().error(
                f"[CYCLE_ERROR] cycle={cycle} exception_type={type(exc).__name__} "
                f"message={exc} node_continues=true\n{tb}"
            )
            event = {
                "cycle_id": cycle,
                "exception_type": type(exc).__name__,
                "message": str(exc),
                "traceback": tb,
                "node_continues": True,
                "timestamp": _utc_now_iso(),
            }
            self._log_jsonl(self._jl_events, event)
            self._publish_heartbeat(self._latest_grid is not None, False)

    def _emit_map_health_console(
        self,
        health: Dict[str, Any],
        robot: Optional[RobotPose2D],
    ) -> None:
        mh = health.get("map_health", health)
        self.get_logger().info(
            "[MAP_HEALTH]\n"
            f"cycle={health.get('cycle_id', self.cycle_id)}\n"
            f"status={mh.get('status', health.get('status'))}\n"
            f"map_received={health.get('map_received', True)}\n"
            f"map_stamp={health.get('map_stamp')}\n"
            f"map_age_s={health.get('map_age_s')}\n"
            f"frame_id={health.get('frame_id')}\n"
            f"size={health.get('width')}x{health.get('height')}\n"
            f"resolution_m={health.get('resolution')}\n"
            f"origin_xy=({health.get('origin', [0, 0])[0]},{health.get('origin', [0, 0])[1]})\n"
            f"expected_cells={mh.get('expected_cells')}\n"
            f"actual_cells={mh.get('actual_cells')}\n"
            f"free_cells={mh.get('free_cells')}\n"
            f"occupied_cells={mh.get('occupied_cells')}\n"
            f"unknown_cells={mh.get('unknown_cells')}\n"
            f"other_cells={mh.get('other_cells')}\n"
            f"free_ratio={mh.get('free_ratio')}\n"
            f"occupied_ratio={mh.get('occupied_ratio')}\n"
            f"unknown_ratio={mh.get('unknown_ratio')}\n"
            f"tf_ok={health.get('tf_ok', True)}\n"
            f"robot_map_xy=({robot.x if robot else 'n/a'},{robot.y if robot else 'n/a'})\n"
            f"robot_yaw_deg={math.degrees(robot.yaw_rad) if robot else 'n/a'}\n"
            f"robot_grid_rc=({mh.get('robot_grid_row')},{mh.get('robot_grid_col')})\n"
            f"robot_cell_value={mh.get('robot_cell_value')}\n"
            f"decision={health.get('decision', 'ALLOW_FRONTIER_EXTRACTION')}"
        )

    def _process_result(
        self,
        result: FrontierAnalysisResult,
        meta: MapMetadata,
        robot: RobotPose2D,
        map_age_s: float,
    ) -> None:
        cycle = result.cycle_id
        mh = result.map_health
        decision = "ALLOW_FRONTIER_EXTRACTION" if mh.ok else "SKIP_FRONTIER_EXTRACTION"

        health_payload = {
            "cycle_id": cycle,
            "status": mh.status,
            "errors": mh.errors,
            "warnings": mh.warnings,
            "map_received": True,
            "map_stamp": meta.stamp_sec,
            "map_age_s": map_age_s,
            "frame_id": meta.frame_id,
            "width": mh.width,
            "height": mh.height,
            "resolution": meta.resolution,
            "origin": [meta.origin_x, meta.origin_y],
            "expected_cells": mh.expected_cells,
            "actual_cells": mh.actual_cells,
            "free_cells": mh.free_cells,
            "occupied_cells": mh.occupied_cells,
            "unknown_cells": mh.unknown_cells,
            "other_cells": mh.other_cells,
            "free_ratio": mh.free_ratio,
            "occupied_ratio": mh.occupied_ratio,
            "unknown_ratio": mh.unknown_ratio,
            "tf_ok": True,
            "robot_map_xy": [robot.x, robot.y],
            "robot_yaw_deg": math.degrees(robot.yaw_rad),
            "robot_grid_row": mh.robot_grid_row,
            "robot_grid_col": mh.robot_grid_col,
            "robot_cell_value": mh.robot_cell_value,
            "decision": decision,
            "map_health": {
                "ok": mh.ok,
                "status": mh.status,
                "errors": mh.errors,
                "warnings": mh.warnings,
                "width": mh.width,
                "height": mh.height,
                "expected_cells": mh.expected_cells,
                "actual_cells": mh.actual_cells,
                "free_cells": mh.free_cells,
                "occupied_cells": mh.occupied_cells,
                "unknown_cells": mh.unknown_cells,
                "other_cells": mh.other_cells,
                "free_ratio": mh.free_ratio,
                "occupied_ratio": mh.occupied_ratio,
                "unknown_ratio": mh.unknown_ratio,
                "robot_grid_row": mh.robot_grid_row,
                "robot_grid_col": mh.robot_grid_col,
                "robot_cell_value": mh.robot_cell_value,
            },
        }
        self._emit_map_health_console(health_payload, robot)
        s = String()
        s.data = json.dumps(health_payload, ensure_ascii=False)
        self.pub_map_health.publish(s)
        self._log_jsonl(self._jl_health, health_payload)
        self._write_latest("latest_map_health.json", health_payload)

        if not mh.ok:
            return

        merge_enabled = bool(self.cfg.get("region_merge", {}).get("enabled", True))
        self.get_logger().info(
            "[REGION_MERGE_SUMMARY]\n"
            f"cycle={cycle}\n"
            f"raw_clusters={result.stats.raw_cluster_count}\n"
            f"merge_enabled={merge_enabled}\n"
            f"merge_pairs_considered={result.stats.merge_pairs_considered}\n"
            f"merge_pairs_accepted={result.stats.merge_pairs_accepted}\n"
            f"merged_groups={result.stats.merged_group_count}\n"
            f"clusters_after_merge={result.stats.cluster_count_after_merge}"
        )
        for entry in result.merge_log:
            self.get_logger().info(
                "[REGION_MERGE]\n"
                f"cycle={cycle}\n"
                f"source_clusters={entry.source_clusters}\n"
                f"frontier_gap_m={entry.frontier_gap_m:.3f}\n"
                f"centroid_distance_m={entry.centroid_distance_m:.3f}\n"
                f"bearing_difference_deg={entry.bearing_difference_deg:.2f}\n"
                f"direction_labels={entry.direction_labels}\n"
                f"result_cluster={entry.result_cluster}\n"
                f"result_frontier_cells={entry.result_frontier_cells}\n"
                f"decision={entry.decision}"
            )
        for rej in result.merge_reject_log[:5]:
            self.get_logger().debug(
                f"[REGION_MERGE_REJECT] cycle={cycle} "
                f"cluster_a={rej.get('cluster_a')} cluster_b={rej.get('cluster_b')} "
                f"frontier_gap_m={rej.get('frontier_gap_m')} "
                f"threshold={rej.get('threshold')} reason={rej.get('reason')}"
            )

        stats_payload = {
            "cycle_id": cycle,
            **result_to_dict(result)["stats"],
            "snapshot_count": self._snapshot_count,
            "latest_snapshot_id": self._latest_snapshot_id,
        }
        self.get_logger().info(
            "[FRONTIER_EXTRACT]\n"
            f"cycle={cycle}\n"
            f"free_cells_examined={stats_payload['free_cells_examined']}\n"
            f"raw_frontier_cells={stats_payload['raw_frontier_cells']}\n"
            f"removed_low_clearance={stats_payload['removed_low_clearance']}\n"
            f"remaining_frontier_cells={stats_payload['remaining_frontier_cells']}\n"
            f"raw_clusters={stats_payload['raw_cluster_count']}\n"
            f"merge_pairs_considered={stats_payload.get('merge_pairs_considered', 0)}\n"
            f"merge_pairs_accepted={stats_payload.get('merge_pairs_accepted', 0)}\n"
            f"merged_groups={stats_payload.get('merged_group_count', 0)}\n"
            f"clusters_after_merge={stats_payload.get('cluster_count_after_merge', 0)}\n"
            f"clusters_too_small={stats_payload['clusters_too_small']}\n"
            f"accepted_before_rank_limit={stats_payload.get('accepted_regions_before_rank_limit', 0)}\n"
            f"accepted_after_rank_limit={stats_payload.get('accepted_regions_after_rank_limit', 0)}\n"
            f"regions_accepted={stats_payload['accepted_region_count']}\n"
            f"regions_rejected={stats_payload['rejected_region_count']}\n"
            f"snapshot_count={self._snapshot_count}\n"
            f"latest_snapshot_id={self._latest_snapshot_id}\n"
            f"analysis_time_ms={stats_payload['analysis_time_ms']:.1f}"
        )
        s2 = String()
        s2.data = json.dumps(stats_payload, ensure_ascii=False)
        self.pub_frontier_stats.publish(s2)
        self._log_jsonl(self._jl_cycles, stats_payload)

        regions_payload = {
            "cycle_id": cycle,
            "regions": result_to_dict(result)["regions"],
        }
        s3 = String()
        s3.data = json.dumps(regions_payload, ensure_ascii=False)
        self.pub_regions.publish(s3)
        self._write_latest("latest_regions.json", regions_payload)

        reject_payload = {
            "cycle_id": cycle,
            "rejected_regions": result_to_dict(result)["rejected_regions"],
        }
        s4 = String()
        s4.data = json.dumps(reject_payload, ensure_ascii=False)
        self.pub_rejections.publish(s4)
        self._write_latest("latest_rejections.json", reject_payload)

        for region in result.regions:
            rdict = next(
                r for r in result_to_dict(result)["regions"] if r["region_id"] == region.region_id
            )
            self.get_logger().info(
                "[REGION]\n"
                f"cycle={cycle}\n"
                f"id={region.region_id}\n"
                f"accepted={region.accepted}\n"
                f"direction={region.direction_label}\n"
                f"centroid_map_xy=({region.centroid_x:.3f},{region.centroid_y:.3f})\n"
                f"nearest_map_xy=({region.nearest_x:.3f},{region.nearest_y:.3f})\n"
                f"distance_m={region.distance_to_robot_m:.3f}\n"
                f"bearing_global_deg={region.bearing_global_deg:.2f}\n"
                f"bearing_relative_deg={region.bearing_relative_deg:.2f}\n"
                f"frontier_cells={region.frontier_cell_count}\n"
                f"frontier_length_m={region.frontier_length_m:.3f}\n"
                f"unknown_gain_cells={region.unknown_gain_cells}\n"
                f"unknown_gain_ratio={region.unknown_gain_ratio:.3f}\n"
                f"minimum_clearance_m={region.minimum_clearance_m:.3f}\n"
                f"mean_clearance_m={region.mean_clearance_m:.3f}\n"
                f"diagnostic_priority={region.diagnostic_priority:.3f}\n"
                f"path_checked={region.path_checked}\n"
                f"reachable={region.reachable}\n"
                f"rejection_reasons={rdict.get('rejection_reasons', [])}"
            )
            self._log_jsonl(self._jl_regions, {"cycle_id": cycle, **rdict})

        rej_dict = result_to_dict(result)["rejected_regions"]
        for region in result.rejected_regions:
            rdict = next(r for r in rej_dict if r["region_id"] == region.region_id)
            reasons_str = ", ".join(
                f"{x['code']}(actual={x['actual']} threshold={x['threshold']})"
                for x in rdict.get("rejection_reasons", [])
            )
            self.get_logger().info(
                f"[REGION][REJECT] cycle={cycle} id={region.region_id} "
                f"direction={region.direction_label} "
                f"centroid_map_xy=({region.centroid_x:.3f},{region.centroid_y:.3f}) "
                f"reasons=[{reasons_str}]"
            )
            self._log_jsonl(self._jl_rejections, {"cycle_id": cycle, **rdict})

        self._publish_markers(result, meta, robot)
        self._publish_annotated_map(result, meta, robot, cycle)

    def _stamp_header(self) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self.map_frame
        return h

    def _trajectory_meta_dict(self) -> Dict[str, Any]:
        tcfg = self.cfg.get("trajectory", {})
        topics = self.cfg.get("topics", {})
        return {
            "trajectory_session_id": self._trajectory.trajectory_session_id,
            "trajectory_revision": self._trajectory.trajectory_revision,
            "trajectory_length_m": self._trajectory.session.trajectory_length_m,
            "trajectory_raw_sample_count": len(self._trajectory.session.raw_samples),
            "trajectory_vertex_count": len(self._trajectory.session.vertices),
            "visited_corridor_radius_m": float(tcfg.get("visited_corridor_radius_m", 0.35)),
            "trajectory_path_topic": topics.get(
                "trajectory_path", "/qwen_explore_debug/trajectory_path"
            ),
            "visited_area_grid_topic": topics.get(
                "visited_area_grid", "/qwen_explore_debug/visited_area_grid"
            ),
        }

    def _trajectory_overlay_dict(self, meta: MapMetadata) -> Dict[str, Any]:
        tcfg = self.cfg.get("trajectory", {})
        overlay: Dict[str, Any] = {
            "trajectory_session_id": self._trajectory.trajectory_session_id,
            "trajectory_revision": self._trajectory.trajectory_revision,
            "trajectory_vertex_count": len(self._trajectory.session.vertices),
            "trajectory_length_m": self._trajectory.session.trajectory_length_m,
            "vertices": [{"x": v.x, "y": v.y, "yaw_rad": v.yaw_rad} for v in self._trajectory.session.vertices],
            "observation_poses": list(self._history.observation_poses),
        }
        if bool(tcfg.get("draw_on_annotated_map", True)) and self._latest_meta is not None:
            overlay["visited_area_data"] = self._trajectory.build_visited_area_data(
                width=meta.width,
                height=meta.height,
                resolution=meta.resolution,
                origin_x=meta.origin_x,
                origin_y=meta.origin_y,
            )
        return overlay

    def _trajectory_timer_cb(self) -> None:
        if not bool(self.cfg.get("trajectory", {}).get("enabled", True)):
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        max_tf_age = float(self.cfg.get("trajectory", {}).get("max_tf_age_s", 0.30))
        robot, tf_code, _, tf_stamp_sec, tf_age_s = self._lookup_robot_tf_detail()
        if robot is None:
            sample = self._trajectory.ingest_tf_missing(now_sec)
            self._trajectory_status = tf_code or "TRAJECTORY_TF_MISSING"
            self._log_jsonl(
                self._jl_traj,
                {
                    "stamp_sec": now_sec,
                    "status": self._trajectory_status,
                    "valid": sample.valid,
                    "rejection_reason": sample.rejection_reason,
                },
            )
        else:
            sample, vertex, status = self._trajectory.ingest_tf_pose(
                stamp_sec=now_sec,
                x=robot.x,
                y=robot.y,
                yaw_rad=robot.yaw_rad,
                tf_stamp_sec=tf_stamp_sec,
                tf_age_s=tf_age_s,
                max_tf_age_s=max_tf_age,
            )
            self._trajectory_status = status
            record: Dict[str, Any] = {
                "stamp_sec": now_sec,
                "status": status,
                "valid": sample.valid,
                "x": sample.x,
                "y": sample.y,
                "yaw_deg": math.degrees(sample.yaw_rad),
                "rejection_reason": sample.rejection_reason,
            }
            if vertex is not None:
                record["vertex_id"] = vertex.vertex_id
                record["creation_reason"] = vertex.creation_reason
            self._log_jsonl(self._jl_traj, record)
            self._trajectory.maybe_save(now_sec)
        self._publish_trajectory_diagnostics()

    def _publish_trajectory_diagnostics(self) -> None:
        tcfg = self.cfg.get("trajectory", {})
        if self._latest_meta is None:
            return
        meta = self._latest_meta
        stamp = self._stamp_header()

        if bool(tcfg.get("publish_path", True)):
            path_msg = NavPath()
            path_msg.header = stamp
            for v in self._trajectory.session.vertices:
                ps = PoseStamped()
                ps.header = stamp
                ps.pose.position.x = v.x
                ps.pose.position.y = v.y
                ps.pose.position.z = 0.0
                ps.pose.orientation.z = math.sin(v.yaw_rad / 2.0)
                ps.pose.orientation.w = math.cos(v.yaw_rad / 2.0)
                path_msg.poses.append(ps)
            self.pub_trajectory_path.publish(path_msg)

        if bool(tcfg.get("publish_markers", True)):
            arr = MarkerArray()
            arr.markers.append(self._delete_all_marker("trajectory", 0))
            line = Marker()
            line.header = stamp
            line.ns = "trajectory"
            line.id = 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.06
            line.color = ColorRGBA(r=0.0, g=0.55, b=1.0, a=0.95)
            for v in self._trajectory.session.vertices:
                line.points.append(Point(x=v.x, y=v.y, z=0.05))
            arr.markers.append(line)

            obs = Marker()
            obs.header = stamp
            obs.ns = "trajectory"
            obs.id = 2
            obs.type = Marker.SPHERE_LIST
            obs.action = Marker.ADD
            obs.scale.x = obs.scale.y = obs.scale.z = 0.12
            obs.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9)
            for pose in self._history.observation_poses:
                obs.points.append(
                    Point(x=float(pose.get("x", 0)), y=float(pose.get("y", 0)), z=0.08)
                )
            arr.markers.append(obs)

            text = Marker()
            text.header = stamp
            text.ns = "trajectory"
            text.id = 3
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = meta.origin_x + 0.2
            text.pose.position.y = meta.origin_y + 0.2
            text.pose.position.z = 0.5
            text.scale.z = 0.14
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = (
                f"{self._trajectory.trajectory_session_id}\n"
                f"len={self._trajectory.session.trajectory_length_m:.2f}m\n"
                f"raw={len(self._trajectory.session.raw_samples)} "
                f"vtx={len(self._trajectory.session.vertices)}"
            )
            arr.markers.append(text)
            self.pub_trajectory_markers.publish(arr)

        if bool(tcfg.get("publish_visited_area_grid", True)):
            sig = (
                meta.width,
                meta.height,
                int(meta.resolution * 10000),
                meta.origin_x,
                meta.origin_y,
            )
            grid = OccupancyGrid()
            grid.header = stamp
            grid.info.resolution = meta.resolution
            grid.info.width = meta.width
            grid.info.height = meta.height
            grid.info.origin.position.x = meta.origin_x
            grid.info.origin.position.y = meta.origin_y
            grid.info.origin.orientation.w = 1.0
            grid.data = self._trajectory.build_visited_area_data(
                width=meta.width,
                height=meta.height,
                resolution=meta.resolution,
                origin_x=meta.origin_x,
                origin_y=meta.origin_y,
            )
            self._last_visited_grid_sig = sig
            self.pub_visited_area_grid.publish(grid)

        if bool(tcfg.get("publish_trajectory_json", True)):
            payload = self._trajectory.trajectory_json_payload()
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.pub_trajectory_json.publish(msg)

    def _reset_trajectory_history_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._trajectory.reset()
        self._trajectory.save_atomic()
        self._trajectory_status = "RESET"
        self._last_visited_grid_sig = None
        self._publish_trajectory_diagnostics()
        response.success = True
        response.message = (
            f"trajectory_reset session_id={self._trajectory.trajectory_session_id} "
            f"revision={self._trajectory.trajectory_revision}"
        )
        self.get_logger().info(f"[TRAJECTORY_RESET] {response.message}")
        return response

    def _delete_all_marker(self, ns: str, mid: int) -> Marker:
        m = Marker()
        m.header = self._stamp_header()
        m.ns = ns
        m.id = mid
        m.action = Marker.DELETEALL
        return m

    def _publish_markers(
        self,
        result: FrontierAnalysisResult,
        meta: MapMetadata,
        robot: RobotPose2D,
    ) -> None:
        cycle = result.cycle_id
        stamp = self._stamp_header()

        # Frontier points
        f_arr = MarkerArray()
        f_arr.markers.append(self._delete_all_marker("frontier_points", 0))
        if result.filtered_frontier_mask is not None:
            pts = Marker()
            pts.header = stamp
            pts.ns = "frontier_points"
            pts.id = 1
            pts.type = Marker.POINTS
            pts.action = Marker.ADD
            pts.scale.x = 0.05
            pts.scale.y = 0.05
            pts.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9)
            rows, cols = result.filtered_frontier_mask.nonzero()
            for r, c in zip(rows.tolist(), cols.tolist()):
                wx, wy = grid_to_world(int(r), int(c), meta)
                p = Point(x=wx, y=wy, z=0.05)
                pts.points.append(p)
            f_arr.markers.append(pts)
        self.pub_frontier_markers.publish(f_arr)

        self._publish_region_marker_set(
            result.regions, meta, robot, cycle, accepted=True
        )
        self._publish_region_marker_set(
            result.rejected_regions, meta, robot, cycle, accepted=False
        )

    def _publish_region_marker_set(
        self,
        regions: List[Any],
        meta: MapMetadata,
        robot: RobotPose2D,
        cycle: int,
        accepted: bool,
    ) -> None:
        ns_prefix = "accepted_regions" if accepted else "rejected_regions"
        arr = MarkerArray()
        arr.markers.append(self._delete_all_marker(ns_prefix, 0))
        pub = self.pub_region_markers if accepted else self.pub_rejected_markers
        base_id = 10
        for idx, region in enumerate(regions):
            stamp = self._stamp_header()
            color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.9) if accepted else ColorRGBA(
                r=0.9, g=0.1, b=0.1, a=0.9
            )
            sphere = Marker()
            sphere.header = stamp
            sphere.ns = ns_prefix
            sphere.id = base_id + idx * 4
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = region.centroid_x
            sphere.pose.position.y = region.centroid_y
            sphere.pose.position.z = 0.15
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.18
            sphere.color = color
            arr.markers.append(sphere)

            arrow = Marker()
            arrow.header = stamp
            arrow.ns = ns_prefix
            arrow.id = base_id + idx * 4 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points.append(Point(x=robot.x, y=robot.y, z=0.1))
            arrow.points.append(
                Point(x=region.nearest_x, y=region.nearest_y, z=0.1)
            )
            arrow.scale.x = 0.05
            arrow.scale.y = 0.1
            arrow.scale.z = 0.1
            arrow.color = color
            arr.markers.append(arrow)

            text = Marker()
            text.header = stamp
            text.ns = ns_prefix
            text.id = base_id + idx * 4 + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = region.centroid_x
            text.pose.position.y = region.centroid_y
            text.pose.position.z = 0.45
            status = "ACCEPT" if accepted else "REJECT"
            reject_codes = " ".join(r.code for r in region.rejection_reasons[:2])
            text.text = (
                f"{region.region_id}\n{region.direction_label}\n"
                f"d={region.distance_to_robot_m:.2f}m\n"
                f"gain={region.unknown_gain_cells}\n"
                f"clear={region.minimum_clearance_m:.2f}m\n"
                f"cells={region.frontier_cell_count}\n{status}"
                + (f"\n{reject_codes}" if reject_codes else "")
            )
            text.scale.z = 0.12
            text.color = color
            arr.markers.append(text)
        pub.publish(arr)

    def _publish_annotated_map(
        self,
        result: FrontierAnalysisResult,
        meta: MapMetadata,
        robot: RobotPose2D,
        cycle: int,
    ) -> None:
        if not self._write_image:
            return
        if not _CV2_AVAILABLE or cv2 is None:
            self.get_logger().warn(
                "[ANNOTATED_MAP][DISABLED] reason=CV2_NOT_AVAILABLE core_analysis_continues=true"
            )
            return
        data, _ = occupancy_grid_to_array(self._latest_grid)  # type: ignore[arg-type]
        overlay = None
        if bool(self.cfg.get("trajectory", {}).get("draw_on_annotated_map", True)):
            overlay = self._trajectory_overlay_dict(meta)
        img = render_annotated_map(data, meta, robot, result, cycle, self.cfg, overlay)
        if img is None:
            return
        img_path = self.run_dir / "latest_annotated_map.png"
        cv2.imwrite(str(img_path), img)
        self._latest_annotated_path = img_path
        ok, buf = cv2.imencode(".png", img)
        if ok:
            msg = CompressedImage()
            msg.header = self._stamp_header()
            msg.format = "png"
            msg.data = buf.tobytes()
            self.pub_annotated.publish(msg)

    def _start_observation_window_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        robot, _, _ = self._lookup_robot_pose()
        if robot is None:
            response.success = False
            response.message = "SNAPSHOT_TF_INVALID"
            return response
        now_s = self.get_clock().now().nanoseconds * 1e-9
        wid = f"OW_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')}"
        self._observation.start(wid, robot, now_s)
        self.get_logger().info(
            f"[OBSERVATION_WINDOW] started id={wid} state=OBSERVING rotation_deg=0"
        )
        response.success = True
        response.message = f"observation_window_id={wid} state=OBSERVING"
        return response

    def _capture_snapshot_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._latest_grid is None or self._latest_meta is None:
            response.success = False
            response.message = "SNAPSHOT_NO_MAP"
            return response
        if not self._tf_ok or self._latest_robot is None:
            response.success = False
            response.message = "SNAPSHOT_TF_INVALID"
            return response
        if self._latest_result is None:
            response.success = False
            response.message = "SNAPSHOT_ANALYSIS_NOT_READY"
            return response
        if self._latest_result.stats.status != "OK":
            response.success = False
            response.message = (
                f"SNAPSHOT_ANALYSIS_NOT_READY status={self._latest_result.stats.status}"
            )
            return response
        eligible = [r for r in self._latest_result.regions if r.snapshot_eligible and r.stable]
        require_obs = bool(self.cfg.get("observation_gate", {}).get("require_observation_window", True))
        block = self._observation.snapshot_block_reason(len(eligible), require_obs)
        if block:
            response.success = False
            response.message = block
            return response
        if not eligible:
            response.success = False
            response.message = "SNAPSHOT_NO_STABLE_REGION"
            return response
        if self._latest_annotated_path is None or not self._latest_annotated_path.exists():
            response.success = False
            response.message = "SNAPSHOT_ANNOTATED_MAP_UNAVAILABLE"
            return response

        snapshot_id = generate_snapshot_id(self._latest_result.cycle_id)
        capture_time = _utc_now_iso()
        snap_dir = self.run_dir / "snapshots" / snapshot_id
        snap_dir.mkdir(parents=True, exist_ok=True)

        annotated_dest = snap_dir / "annotated_map.png"
        import shutil

        shutil.copy2(self._latest_annotated_path, annotated_dest)

        payload = build_region_snapshot_payload(
            self._latest_result,
            self._latest_meta,
            self._latest_robot,
            snapshot_id,
            capture_time,
            self._snapshot_expires_s,
            str(annotated_dest),
            observation_meta={
                "observation_window_id": self._observation.window_id,
                "full_scan_completed": self._observation.accumulated_rotation_deg >= float(
                    self.cfg.get("observation_gate", {}).get("full_scan_threshold_deg", 350.0)
                ),
                "accumulated_rotation_deg": self._observation.accumulated_rotation_deg,
                "map_stable": self._observation.map_stable_cycle_count >= int(
                    self.cfg.get("observation_gate", {}).get("map_stable_cycles", 3)
                ),
                "robot_settled": self._observation.state in ("MAP_STABILIZING", "READY_FOR_SNAPSHOT"),
                "history_version": self._history.version,
                "guard_summary": self._last_guard_summary,
                "minimum_eligible_score": float(
                    self.cfg.get("geometric_scoring", {}).get("minimum_eligible_score", 0.40)
                ),
            },
            top_k=int(self.cfg.get("geometric_scoring", {}).get("top_k_for_qwen", 5)),
            trajectory_meta=self._trajectory_meta_dict(),
        )
        (snap_dir / "region_snapshot.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        if bool(self.cfg.get("snapshot", {}).get("write_region_geometry", True)):
            geo_payload = build_region_geometry_payload(
                self._latest_result,
                self._latest_meta,
                snapshot_id,
                top_k=int(self.cfg.get("geometric_scoring", {}).get("top_k_for_qwen", 5)),
                minimum_eligible_score=float(
                    self.cfg.get("geometric_scoring", {}).get("minimum_eligible_score", 0.40)
                ),
            )
            (snap_dir / "region_geometry.json").write_text(
                json.dumps(geo_payload, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        capture_meta = {
            "snapshot_id": snapshot_id,
            "cycle_id": self._latest_result.cycle_id,
            "capture_time": capture_time,
            "run_dir": str(self.run_dir),
            "snapshot_dir": str(snap_dir),
        }
        (snap_dir / "capture_meta.json").write_text(
            json.dumps(capture_meta, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self._write_latest("latest_region_snapshot.json", payload)

        pub = String()
        pub.data = json.dumps(payload, ensure_ascii=False)
        self.pub_snapshot.publish(pub)

        self._snapshot_count += 1
        self._latest_snapshot_id = snapshot_id
        known_cells = self._latest_result.map_health.free_cells + self._latest_result.map_health.occupied_cells
        top_label = payload["accepted_regions"][0]["label"] if payload["accepted_regions"] else ""
        top_track = payload["accepted_regions"][0].get("track_id", "") if payload["accepted_regions"] else ""
        self._history.add_observation_pose(
            ObservationPose(
                observation_pose_id=f"OP_{snapshot_id}",
                x=self._latest_robot.x,
                y=self._latest_robot.y,
                yaw=math.degrees(self._latest_robot.yaw_rad),
                capture_time=capture_time,
                snapshot_id=snapshot_id,
                selected_track_id=top_track,
                map_known_cell_count=known_cells,
            ),
            max_poses=int(self.cfg.get("region_history", {}).get("max_observation_poses", 50)),
        )
        self._trajectory.add_observation_pose_id(f"OP_{snapshot_id}")
        if top_track:
            self._history.update_track_stats(top_track, visit=True, capture_time=capture_time)
        self._history.save_atomic(self._history_path)
        labels = [r["label"] for r in payload["accepted_regions"]]
        self.get_logger().info(
            "[REGION_SNAPSHOT]\n"
            f"snapshot_id={snapshot_id}\n"
            f"cycle_id={self._latest_result.cycle_id}\n"
            f"accepted_regions={len(payload['accepted_regions'])}\n"
            f"labels={labels}\n"
            f"map_stamp={self._latest_meta.stamp_sec}\n"
            f"robot_pose=({self._latest_robot.x:.3f},{self._latest_robot.y:.3f})\n"
            f"annotated_map={annotated_dest}\n"
            f"snapshot_json={snap_dir / 'region_snapshot.json'}\n"
            f"status=SUCCESS"
        )
        response.success = True
        response.message = (
            f"snapshot_id={snapshot_id} regions={len(payload['accepted_regions'])} "
            f"directory={snap_dir}"
        )
        return response


def _git_info(project_dir: Path) -> Tuple[str, str, bool]:
    try:
        branch = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=project_dir,
            text=True,
        ).strip()
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=project_dir, text=True
        ).strip()
        dirty = bool(
            subprocess.check_output(
                ["git", "status", "--short"], cwd=project_dir, text=True
            ).strip()
        )
        return branch, commit, dirty
    except Exception:
        return "unknown", "unknown", False


def _write_run_meta(run_dir: Path, cfg: Dict[str, Any], config_path: Path) -> None:
    branch, commit, dirty = _git_info(ROOT)
    meta = {
        "run_id": run_dir.name,
        "project_dir": str(ROOT),
        "git_branch": branch,
        "git_commit": commit,
        "git_dirty": dirty,
        "hostname": socket.gethostname(),
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "start_time": _utc_now_iso(),
        "observation_only": True,
        "motion_enabled": False,
        "qwen_enabled": False,
        "nav2_enabled": False,
        "config_path": str(config_path),
    }
    (run_dir / "run_meta.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    import shutil

    shutil.copy2(config_path, run_dir / "resolved_config.yaml")


def main() -> int:
    parser = argparse.ArgumentParser(description="Frontier region debug node (observation-only)")
    parser.add_argument("--config", required=True, help="Path to YAML config")
    parser.add_argument("--run-dir", required=True, help="Run log directory")
    args = parser.parse_args()

    config_path = Path(args.config).expanduser().resolve()
    run_dir = Path(args.run_dir).expanduser().resolve()
    run_dir.mkdir(parents=True, exist_ok=True)

    raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    cfg_errors = validate_config(raw)
    if cfg_errors:
        print("[FATAL] configuration errors:", file=sys.stderr)
        for e in cfg_errors:
            print(f"  - {e}", file=sys.stderr)
        return 2

    _write_run_meta(run_dir, raw, config_path)
    (run_dir / "snapshots").mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = FrontierRegionDebugNode(raw, run_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
