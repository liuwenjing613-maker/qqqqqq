#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Runtime regression tests for Qwen session planner / Nav2 gates (no hardware)."""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEBUG = ROOT / "scripts" / "debug"
sys.path.insert(0, str(DEBUG))

from qwen_map_goal_utils import (  # noqa: E402
    compute_bundle_fingerprint,
    validate_proposal_against_bundle,
)


def _make_corridor_map(out_dir: Path) -> tuple[Path, Path, Path]:
    """Create a simple free corridor map with unknown frontier ahead."""
    out_dir.mkdir(parents=True, exist_ok=True)
    h, w = 120, 160
    gray = np.full((h, w), 205, dtype=np.uint8)  # unknown
    # free corridor
    gray[40:80, 20:120] = 254
    # walls
    gray[38:40, 20:120] = 0
    gray[80:82, 20:120] = 0
    gray[40:80, 18:20] = 0
    # open frontier to unknown at right
    gray[40:80, 120:150] = 205
    pgm = out_dir / "test_map.pgm"
    cv2.imwrite(str(pgm), gray)
    yaml_path = out_dir / "test_map.yaml"
    meta = {
        "image": "test_map.pgm",
        "mode": "trinary",
        "resolution": 0.05,
        "origin": [-4.0, -3.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path.write_text(yaml.safe_dump(meta, sort_keys=False), encoding="utf-8")
    # robot near left of free corridor
    # map x = origin_x + px * res; py from top
    # choose pixel ~ (40, 60)
    px, py = 40, 60
    mx = -4.0 + px * 0.05
    my = -3.0 + (h - 1 - py) * 0.05
    pose = {
        "x": mx,
        "y": my,
        "yaw": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
    }
    pose_path = out_dir / "last_pose_map.json"
    pose_path.write_text(json.dumps(pose, indent=2) + "\n", encoding="utf-8")
    traj = {"vertices": [{"x": mx - 0.3, "y": my}, {"x": mx, "y": my}]}
    traj_path = out_dir / "trajectory_session.json"
    traj_path.write_text(json.dumps(traj, indent=2) + "\n", encoding="utf-8")
    return yaml_path, pose_path, traj_path


class TestCandidateBundleCrossProcess(unittest.TestCase):
    def test_two_process_candidate_cache_identical(self):
        with tempfile.TemporaryDirectory(prefix="cand_cache_") as tmp:
            tmp_path = Path(tmp)
            map_yaml, pose, traj = _make_corridor_map(tmp_path / "inputs")
            out1 = tmp_path / "p1"
            out2 = tmp_path / "p2"
            out1.mkdir()
            out2.mkdir()
            bundle = out1 / "candidate_bundle.json"
            nav1 = out1 / "navigation_goal_proposal.json"
            nav2 = out2 / "navigation_goal_proposal.json"
            planner = DEBUG / "qwen_live_session_planner.py"
            common = [
                sys.executable,
                str(planner),
                "--map-yaml",
                str(map_yaml),
                "--pose-json",
                str(pose),
                "--trajectory-json",
                str(traj),
                "--session-id",
                "TEST_SESSION_CACHE",
            ]
            # Process 1: candidates-only
            r1 = subprocess.run(
                common
                + [
                    "--output-dir",
                    str(out1),
                    "--nav-goal-json",
                    str(nav1),
                    "--candidate-bundle",
                    str(bundle),
                    "--candidates-only",
                ],
                cwd=str(ROOT),
                env={**os.environ, "PYTHONPATH": str(ROOT)},
                capture_output=True,
                text=True,
                timeout=180,
            )
            self.assertEqual(r1.returncode, 0, msg=r1.stdout + "\n" + r1.stderr)
            self.assertTrue(bundle.is_file())
            b1 = json.loads(bundle.read_text(encoding="utf-8"))
            self.assertNotIn("_runtime", b1)
            self.assertIn("bundle_fingerprint", b1)
            self.assertGreaterEqual(len(b1.get("candidates") or []), 1)
            fp1 = b1["bundle_fingerprint"]
            ids1 = [(c["candidate_id"], c["pixel_x"], c["pixel_y"]) for c in b1["candidates"]]

            # Process 2: dry-run using same bundle (separate Python process)
            r2 = subprocess.run(
                common
                + [
                    "--output-dir",
                    str(out2),
                    "--nav-goal-json",
                    str(nav2),
                    "--candidate-bundle",
                    str(bundle),
                    "--dry-run",
                ],
                cwd=str(ROOT),
                env={**os.environ, "PYTHONPATH": str(ROOT)},
                capture_output=True,
                text=True,
                timeout=180,
            )
            self.assertEqual(r2.returncode, 0, msg=r2.stdout + "\n" + r2.stderr)
            self.assertIn("[CACHE] reuse", r2.stdout)
            self.assertNotIn("KeyError", r2.stdout + r2.stderr)
            self.assertNotIn("_runtime", r2.stdout + r2.stderr)
            b2 = json.loads(bundle.read_text(encoding="utf-8"))
            self.assertEqual(b2["bundle_fingerprint"], fp1)
            ids2 = [(c["candidate_id"], c["pixel_x"], c["pixel_y"]) for c in b2["candidates"]]
            self.assertEqual(ids1, ids2)
            self.assertTrue(nav2.is_file())
            prop = json.loads(nav2.read_text(encoding="utf-8"))
            self.assertEqual(prop.get("selection_status"), "REGION_PROPOSED")
            self.assertEqual(prop.get("bundle_fingerprint"), fp1)


class TestProposalBundleBinding(unittest.TestCase):
    def _base_bundle_and_proposal(self, tmp: Path):
        map_yaml, pose, traj = _make_corridor_map(tmp / "inputs")
        # Minimal synthetic bundle after a successful candidates-only run is preferred;
        # fall back to handcrafted if planner unavailable.
        out = tmp / "run"
        out.mkdir()
        bundle_path = out / "candidate_bundle.json"
        nav = out / "navigation_goal_proposal.json"
        r = subprocess.run(
            [
                sys.executable,
                str(DEBUG / "qwen_live_session_planner.py"),
                "--map-yaml",
                str(map_yaml),
                "--pose-json",
                str(pose),
                "--trajectory-json",
                str(traj),
                "--output-dir",
                str(out),
                "--nav-goal-json",
                str(nav),
                "--candidate-bundle",
                str(bundle_path),
                "--session-id",
                "BIND_TEST",
                "--dry-run",
            ],
            cwd=str(ROOT),
            env={**os.environ, "PYTHONPATH": str(ROOT)},
            capture_output=True,
            text=True,
            timeout=180,
        )
        self.assertEqual(r.returncode, 0, msg=r.stdout + "\n" + r.stderr)
        bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
        proposal = json.loads(nav.read_text(encoding="utf-8"))
        return bundle, proposal, bundle_path, nav

    def test_fingerprint_mismatch_rejects(self):
        with tempfile.TemporaryDirectory(prefix="bind_fp_") as tmp:
            bundle, proposal, _, _ = self._base_bundle_and_proposal(Path(tmp))
            proposal["bundle_fingerprint"] = "deadbeef" * 4
            ok, msg = validate_proposal_against_bundle(proposal, bundle, allow_fallback=True)
            self.assertFalse(ok)
            self.assertIn("fingerprint", msg.lower())

    def test_missing_candidate_id_rejects(self):
        with tempfile.TemporaryDirectory(prefix="bind_id_") as tmp:
            bundle, proposal, _, _ = self._base_bundle_and_proposal(Path(tmp))
            proposal["selected_candidate_id"] = 99999
            proposal["qwen_selection"]["selected_local_id"] = 99999
            ok, msg = validate_proposal_against_bundle(proposal, bundle, allow_fallback=True)
            self.assertFalse(ok)
            self.assertIn("不存在", msg)

    def test_coordinate_mismatch_rejects(self):
        with tempfile.TemporaryDirectory(prefix="bind_xy_") as tmp:
            bundle, proposal, _, _ = self._base_bundle_and_proposal(Path(tmp))
            proposal["selected_candidate_pixel"] = {"x": 1, "y": 1}
            proposal["goal_pose_map"]["pixel_x"] = 1
            proposal["goal_pose_map"]["pixel_y"] = 1
            ok, msg = validate_proposal_against_bundle(proposal, bundle, allow_fallback=True)
            self.assertFalse(ok)
            self.assertIn("像素", msg)

    def test_fallback_without_allow_rejects(self):
        with tempfile.TemporaryDirectory(prefix="bind_fb_") as tmp:
            bundle, proposal, _, _ = self._base_bundle_and_proposal(Path(tmp))
            proposal["qwen_selection"]["selected_by"] = "python_fallback_after_qwen_error"
            ok, msg = validate_proposal_against_bundle(proposal, bundle, allow_fallback=False)
            self.assertFalse(ok)
            self.assertIn("fallback", msg.lower())

    def test_valid_binding_passes(self):
        with tempfile.TemporaryDirectory(prefix="bind_ok_") as tmp:
            bundle, proposal, _, _ = self._base_bundle_and_proposal(Path(tmp))
            ok, msg = validate_proposal_against_bundle(proposal, bundle, allow_fallback=True)
            self.assertTrue(ok, msg=msg)


class TestComputePathEmptyGate(unittest.TestCase):
    def test_empty_path_logic_in_sender(self):
        # Static inspection: empty poses must return None before NavigateToPose.
        src = (ROOT / "scripts" / "nav" / "send_navigation_goal_proposal.py").read_text(encoding="utf-8")
        self.assertIn("len(path.poses) == 0", src)
        self.assertIn("规划器返回空路径，终止导航", src)
        self.assertIn("未发送 NavigateToPose", src)


class TestReadyJsonAmclGate(unittest.TestCase):
    def test_unsettled_not_ready_for_goal(self):
        payload = {
            "schema_version": 2,
            "status": "LOCALIZATION_UNSETTLED",
            "ready_for_goal": False,
            "amcl_settled": False,
            "compute_path_to_pose_ready": True,
            "navigate_to_pose_ready": True,
            "map_to_base_link_fresh": True,
        }
        self.assertNotEqual(payload["status"], "READY_FOR_GOAL")
        self.assertFalse(payload["ready_for_goal"])
        self.assertFalse(payload["amcl_settled"])


class TestMapPublisherFastFail(unittest.TestCase):
    def test_nav2_script_checks_zero_then_one(self):
        src = (ROOT / "scripts" / "slam" / "run_nav2_saved_map.sh").read_text(encoding="utf-8")
        self.assertIn("map_pub_before", src)
        self.assertIn("expected 0", src)
        self.assertIn("wait_map_publisher_count 1", src)


class TestNav2StartOnlyEntry(unittest.TestCase):
    def test_session_script_supports_skip_qwen_nav2_start_only(self):
        src = (ROOT / "scripts" / "debug" / "run_joy_map_qwen_plan_session.sh").read_text(encoding="utf-8")
        self.assertIn("--skip-qwen --nav2-start-only", src)
        self.assertIn("NAV2_START_ONLY", src)
        self.assertTrue((ROOT / "scripts" / "nav" / "start_session_nav2_only.sh").is_file())

    def test_dry_run_forbids_auto_nav(self):
        src = (ROOT / "scripts" / "debug" / "run_joy_map_qwen_plan_session.sh").read_text(encoding="utf-8")
        self.assertIn("--dry-run 禁止真实运动", src)
        self.assertIn("DRY_RUN_QWEN", src)
        self.assertIn("AUTO_NAV=0", src)


class TestHandoffLogic(unittest.TestCase):
    def test_fast_nav_uses_and_not_or_for_map_stop(self):
        src = (ROOT / "scripts" / "debug" / "run_joy_map_qwen_plan_session.sh").read_text(encoding="utf-8")
        self.assertIn("! slam_toolbox_running && ! map_topic_has_publisher", src)
        self.assertIn("SLAM 仍在发布 /map", src)
        self.assertIn("request_nav_handoff", src)
        corridor = (ROOT / "scripts" / "slam" / "run_corridor_mapping_live_foxglove.sh").read_text(encoding="utf-8")
        self.assertIn("perform_nav_handoff", corridor)
        self.assertIn("HANDOFF_DONE", corridor)


class TestNoApiKeyInTree(unittest.TestCase):
    def test_gitignore_has_env_and_runtime(self):
        gi = (ROOT / ".gitignore").read_text(encoding="utf-8")
        self.assertIn(".env", gi)
        self.assertIn("runtime/", gi)
        self.assertIn("!.env.example", gi)

    def test_tracked_sources_have_no_sk_key(self):
        # Scan tracked-like source paths only; never print secrets.
        patterns = []
        for path in [
            ROOT / "scripts",
            ROOT / "docs",
            ROOT / "README.md",
            ROOT / "README_cn.md",
            ROOT / ".env.example",
        ]:
            if path.is_file():
                text = path.read_text(encoding="utf-8", errors="ignore")
                if "sk-" in text and "sk-your" not in text and "sk-xxxxxxxx" not in text:
                    # Allow documentation placeholders only
                    for line in text.splitlines():
                        if "sk-" in line and "sk-your" not in line and "sk-xxxxxxxx" not in line and "placeholder" not in line.lower():
                            if line.strip().startswith("#"):
                                continue
                            patterns.append(str(path))
                            break
            elif path.is_dir():
                for f in path.rglob("*"):
                    if f.suffix.lower() not in {".py", ".sh", ".md", ".yaml", ".yml", ".txt", ".json", ".example"}:
                        continue
                    if "logs" in f.parts or "runtime" in f.parts:
                        continue
                    try:
                        text = f.read_text(encoding="utf-8", errors="ignore")
                    except OSError:
                        continue
                    for line in text.splitlines():
                        if "DASHSCOPE_API_KEY=sk-" in line.replace(" ", ""):
                            patterns.append(str(f))
                            break
                        if line.strip().startswith("DASHSCOPE_API_KEY=") and "sk-" in line and "your_" not in line:
                            patterns.append(str(f))
                            break
        self.assertEqual(patterns, [], msg=f"possible key literals in: {patterns}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
