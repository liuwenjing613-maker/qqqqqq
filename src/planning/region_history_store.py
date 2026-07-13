#!/usr/bin/env python3
"""Persistent observation pose and region visit history."""

from __future__ import annotations

import json
import os
import shutil
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


HISTORY_VERSION = "1.0"


@dataclass
class ObservationPose:
    observation_pose_id: str
    x: float
    y: float
    yaw: float
    capture_time: str
    snapshot_id: str
    selected_track_id: str = ""
    map_known_cell_count: int = 0


@dataclass
class RegionHistoryStore:
    version: str = HISTORY_VERSION
    observation_poses: List[Dict[str, Any]] = field(default_factory=list)
    track_stats: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path) -> "RegionHistoryStore":
        path = Path(path)
        if not path.is_file():
            return cls()
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
            return cls(
                version=str(raw.get("version", HISTORY_VERSION)),
                observation_poses=list(raw.get("observation_poses", [])),
                track_stats=dict(raw.get("track_stats", {})),
            )
        except (json.JSONDecodeError, OSError, TypeError):
            corrupt = path.with_suffix(path.suffix + f".corrupt_{int(time.time())}")
            try:
                shutil.copy2(path, corrupt)
            except OSError:
                pass
            return cls()

    def save_atomic(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        payload = {
            "version": self.version,
            "observation_poses": self.observation_poses,
            "track_stats": self.track_stats,
        }
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        with open(tmp, "rb") as fh:
            os.fsync(fh.fileno())
        os.replace(tmp, path)

    def add_observation_pose(
        self,
        pose: ObservationPose,
        max_poses: int = 50,
    ) -> None:
        self.observation_poses.append(asdict(pose))
        if len(self.observation_poses) > max_poses:
            self.observation_poses = self.observation_poses[-max_poses:]

    def update_track_stats(
        self,
        track_id: str,
        *,
        selection: bool = False,
        visit: bool = False,
        navigation_failure: bool = False,
        capture_time: str = "",
        blacklist_after: int = 3,
    ) -> None:
        stats = self.track_stats.setdefault(
            track_id,
            {
                "selection_count": 0,
                "visit_count": 0,
                "navigation_failure_count": 0,
                "blacklisted": False,
                "last_selected_time": None,
                "last_visited_time": None,
            },
        )
        if selection:
            stats["selection_count"] = int(stats.get("selection_count", 0)) + 1
            stats["last_selected_time"] = capture_time
        if visit:
            stats["visit_count"] = int(stats.get("visit_count", 0)) + 1
            stats["last_visited_time"] = capture_time
        if navigation_failure:
            stats["navigation_failure_count"] = int(stats.get("navigation_failure_count", 0)) + 1
        if int(stats.get("navigation_failure_count", 0)) >= blacklist_after:
            stats["blacklisted"] = True

    def apply_to_tracks(self, tracks: Dict[str, Any]) -> None:
        for tid, track in tracks.items():
            stats = self.track_stats.get(tid, {})
            track.selection_count = int(stats.get("selection_count", 0))
            track.visit_count = int(stats.get("visit_count", 0))
            track.navigation_failure_count = int(stats.get("navigation_failure_count", 0))
            track.blacklisted = bool(stats.get("blacklisted", False))
            track.last_selected_time = stats.get("last_selected_time")
            track.last_visited_time = stats.get("last_visited_time")
