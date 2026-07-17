#!/usr/bin/env python3
"""Tests for Qwen Nav2 goal JSON validation."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "nav"))
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "debug"))

from qwen_nav2_common import validate_goal_inputs  # noqa: E402


def _write(tmp: Path, name: str, payload: dict) -> Path:
    p = tmp / name
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return p


class TestGoalJsonValidation(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.session_id = "JQS_TEST"
        self.map_yaml = self.root / "map.yaml"
        self.map_yaml.write_text(
            "image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n",
            encoding="utf-8",
        )
        (self.root / "map.pgm").write_bytes(b"P5\n1 1\n255\n254\n")

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def _bundle(self) -> dict:
        return {
            "session_id": self.session_id,
            "bundle_fingerprint": "fp1",
            "candidates": [{"candidate_id": 2, "local_id": 2}],
        }

    def _goal(self) -> dict:
        return {
            "session_id": self.session_id,
            "map_yaml": str(self.map_yaml.resolve()),
            "bundle_fingerprint": "fp1",
            "selected_candidate_id": 2,
            "goal_pose_map": {"x": 1.0, "y": 2.0, "yaw_rad": -1.5},
        }

    def _pose(self) -> dict:
        return {"session_id": self.session_id, "frame_id": "map", "x": 0.1, "y": 0.2, "yaw": 0.0}

    def test_valid_goal_passes(self) -> None:
        goal = _write(self.root, "goal.json", self._goal())
        bundle = _write(self.root, "bundle.json", self._bundle())
        pose = _write(self.root, "pose.json", self._pose())
        parsed, msg = validate_goal_inputs(
            session_id=self.session_id,
            map_yaml=self.map_yaml,
            goal_json=goal,
            candidate_bundle=bundle,
            pose_json=pose,
        )
        self.assertEqual(msg, "ok")
        self.assertEqual(parsed.candidate_id, "2")

    def test_session_mismatch_fails(self) -> None:
        bad = self._goal()
        bad["session_id"] = "OTHER"
        goal = _write(self.root, "goal.json", bad)
        bundle = _write(self.root, "bundle.json", self._bundle())
        pose = _write(self.root, "pose.json", self._pose())
        with self.assertRaises(ValueError):
            validate_goal_inputs(
                session_id=self.session_id,
                map_yaml=self.map_yaml,
                goal_json=goal,
                candidate_bundle=bundle,
                pose_json=pose,
            )

    def test_candidate_missing_fails(self) -> None:
        bad = self._goal()
        bad["selected_candidate_id"] = 99
        goal = _write(self.root, "goal.json", bad)
        bundle = _write(self.root, "bundle.json", self._bundle())
        pose = _write(self.root, "pose.json", self._pose())
        with self.assertRaises(ValueError):
            validate_goal_inputs(
                session_id=self.session_id,
                map_yaml=self.map_yaml,
                goal_json=goal,
                candidate_bundle=bundle,
                pose_json=pose,
            )


if __name__ == "__main__":
    unittest.main()
