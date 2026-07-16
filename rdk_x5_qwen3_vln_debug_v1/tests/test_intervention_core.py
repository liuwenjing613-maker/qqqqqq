#!/usr/bin/env python3
from __future__ import annotations

import math
import unittest

from intervention.core import (
    Candidate,
    CandidateSummary,
    DecisionKind,
    InterventionConfig,
    InterventionCore,
    PoseSample,
    ServoSample,
)


def cfg(**overrides):
    base = dict(
        enabled=True,
        startup_grace_sec=0.0,
        cooldown_sec=1.0,
        servo_max_age_sec=1.0,
        odom_max_age_sec=1.0,
        candidates_max_age_sec=3.0,
    )
    base.update(overrides)
    return InterventionConfig(**base)


def servo(t, **kw):
    values = dict(
        stamp=t,
        state="SEARCHING",
        result="TARGET_INFERRED",
        action="POINT",
        request_id=int(t * 10),
        point_role="search",
        horizontal_error=0.0,
        limited_vx=0.0,
        limited_wz=0.0,
        spawn_scan_phase="IDLE",
        view_adjust_phase="IDLE",
        emergency_reverse_active=False,
        turn_pending_action=None,
        mission_success=False,
        motion_enabled=True,
    )
    values.update(kw)
    return ServoSample(**values)


def pose(t, x=0.0, y=0.0, yaw=0.0):
    return PoseSample(t, x, y, yaw)


def summary(t, generation, candidates, **kw):
    return CandidateSummary(
        stamp=t,
        generation=generation,
        candidates=tuple(candidates),
        **kw,
    )


class InterventionCoreTests(unittest.TestCase):
    def healthy_core(self, t=10.0, config=None):
        core = InterventionCore(config or cfg(), start_stamp=0.0)
        core.update_pose(pose(t))
        core.update_servo(servo(t))
        return core

    def test_disabled_preserves_ego(self):
        core = self.healthy_core(config=cfg(enabled=False))
        d = core.evaluate(10.0)
        self.assertEqual(d.kind, DecisionKind.KEEP_EGO)
        self.assertEqual(d.reason, "DISABLED")

    def test_spawn_scan_blocks_intervention(self):
        core = InterventionCore(cfg(), start_stamp=0.0)
        core.update_pose(pose(10.0))
        core.update_servo(
            servo(10.0, state="SPAWN_SCAN", spawn_scan_phase="TURNING")
        )
        d = core.evaluate(10.0)
        self.assertEqual(d.kind, DecisionKind.KEEP_EGO)
        self.assertTrue(d.reason.startswith("STATE_") or d.reason.startswith("SPAWN_"))

    def test_target_visible_always_keeps_ego(self):
        core = InterventionCore(cfg(), start_stamp=0.0)
        core.update_pose(pose(10.0))
        core.update_servo(
            servo(
                10.0,
                result="TARGET_VISIBLE",
                point_role="target",
                action="POINT",
            )
        )
        d = core.evaluate(10.0)
        self.assertEqual(d.kind, DecisionKind.KEEP_EGO)
        self.assertEqual(d.reason, "TARGET_VISIBLE")

    def test_turn_pending_blocks_intervention(self):
        core = InterventionCore(cfg(), start_stamp=0.0)
        core.update_pose(pose(10.0))
        core.update_servo(servo(10.0, turn_pending_action="TURN_LEFT"))
        d = core.evaluate(10.0)
        self.assertEqual(d.kind, DecisionKind.KEEP_EGO)
        self.assertEqual(d.reason, "TURN_PENDING")

    def test_branch_requires_two_distinct_updates(self):
        core = self.healthy_core(10.0)
        candidates = [
            Candidate("F1", -65.0, score=0.72),
            Candidate("F2", 10.0, score=0.68),
        ]
        core.update_candidates(
            summary(10.0, 1, candidates, decision_distance_m=0.8)
        )
        self.assertEqual(core.evaluate(10.0).kind, DecisionKind.KEEP_EGO)
        core.update_candidates(
            summary(10.1, 2, candidates, decision_distance_m=0.8)
        )
        core.update_pose(pose(10.1))
        core.update_servo(servo(10.1))
        d = core.evaluate(10.1)
        self.assertEqual(d.kind, DecisionKind.MAP_QWEN)
        self.assertEqual(d.reason, "BRANCH_AMBIGUOUS")
        self.assertEqual(set(d.candidate_ids), {"F1", "F2"})

    def test_clear_geometric_winner_does_not_call_qwen(self):
        core = self.healthy_core(10.0)
        candidates = [
            Candidate("F1", -70.0, score=0.92),
            Candidate("F2", 20.0, score=0.40),
        ]
        core.update_candidates(summary(10.0, 1, candidates, decision_distance_m=0.7))
        core.update_candidates(summary(10.1, 2, candidates, decision_distance_m=0.7))
        core.update_pose(pose(10.1))
        core.update_servo(servo(10.1))
        d = core.evaluate(10.1)
        self.assertEqual(d.kind, DecisionKind.KEEP_EGO)

    def test_returned_junction_single_candidate_goes_direct(self):
        core = self.healthy_core(10.0)
        candidates = [Candidate("B7", 88.0, score=0.5)]
        core.update_candidates(
            summary(
                10.0,
                1,
                candidates,
                returned_to_junction=True,
                unseen_candidate_count=1,
                junction_id="J3",
            )
        )
        core.update_candidates(
            summary(
                10.1,
                2,
                candidates,
                returned_to_junction=True,
                unseen_candidate_count=1,
                junction_id="J3",
            )
        )
        core.update_pose(pose(10.1))
        core.update_servo(servo(10.1))
        d = core.evaluate(10.1)
        self.assertEqual(d.kind, DecisionKind.MAP_DIRECT)
        self.assertEqual(d.reason, "RETURNED_JUNCTION_SINGLE")
        self.assertEqual(d.candidate_ids, ("B7",))

    def test_no_progress_escalates_only_after_local_recovery_cycle(self):
        c = cfg(
            progress_window_sec=6.0,
            progress_recovery_grace_sec=4.0,
            progress_recovery_success_distance_m=0.18,
        )
        core = InterventionCore(c, start_stamp=0.0)
        t = 1.0
        request_id = 0
        while t <= 7.2:
            request_id += 1
            core.update_pose(pose(t, 0.0, 0.0))
            core.update_servo(
                servo(t, request_id=request_id, limited_vx=0.04)
            )
            t += 0.1
        first = core.evaluate(7.2)
        self.assertEqual(first.kind, DecisionKind.RECOVERY)
        self.assertEqual(first.reason, "NO_PROGRESS_LOCAL_RECOVERY")

        while t <= 11.4:
            request_id += 1
            core.update_pose(pose(t, 0.0, 0.0))
            core.update_servo(
                servo(t, request_id=request_id, limited_vx=0.04)
            )
            core.evaluate(t)
            t += 0.1

        candidates = [
            Candidate("F1", -60.0, score=0.6),
            Candidate("F2", 60.0, score=0.5),
        ]
        while t <= 17.7:
            request_id += 1
            core.update_pose(pose(t, 0.0, 0.0))
            core.update_servo(
                servo(t, request_id=request_id, limited_vx=0.04)
            )
            t += 0.1
        core.update_candidates(summary(17.6, 1, candidates))
        d = core.evaluate(17.7)
        self.assertEqual(d.kind, DecisionKind.MAP_QWEN)
        self.assertEqual(d.reason, "NO_PROGRESS_AFTER_RECOVERY")

    def test_recovery_motion_cancels_no_progress_escalation(self):
        core = InterventionCore(cfg(), start_stamp=0.0)
        t = 1.0
        request_id = 0
        while t <= 7.2:
            request_id += 1
            core.update_pose(pose(t, 0.0, 0.0))
            core.update_servo(servo(t, request_id=request_id, limited_vx=0.04))
            t += 0.1
        self.assertEqual(core.evaluate(7.2).kind, DecisionKind.RECOVERY)
        core.update_pose(pose(8.0, 0.20, 0.0))
        core.update_servo(servo(8.0, request_id=999, limited_vx=0.04))
        d = core.evaluate(8.0)
        self.assertEqual(d.kind, DecisionKind.KEEP_EGO)
        self.assertEqual(d.reason, "EGO_HEALTHY")

    def test_action_oscillation_with_one_candidate_goes_direct(self):
        core = InterventionCore(cfg(), start_stamp=0.0)
        candidate = Candidate("F9", 40.0, score=0.7)
        core.update_candidates(summary(20.0, 1, [candidate]))
        actions = [
            "TURN_LEFT",
            "TURN_RIGHT",
            "TURN_LEFT",
            "TURN_RIGHT",
            "TURN_LEFT",
            "TURN_RIGHT",
            "TURN_LEFT",
            "TURN_RIGHT",
        ]
        for i, action in enumerate(actions):
            t = 20.0 + i * 0.9
            core.update_pose(pose(t, 0.0, 0.0))
            core.update_servo(
                servo(
                    t,
                    request_id=i + 1,
                    action=action,
                    limited_vx=0.0,
                )
            )
        # Refresh candidate summary so it remains inside the 3 s freshness gate.
        core.update_candidates(summary(26.3, 2, [candidate]))
        d = core.evaluate(26.3)
        self.assertEqual(d.kind, DecisionKind.MAP_DIRECT)
        self.assertEqual(d.reason, "ACTION_OSCILLATION")

    def test_two_emergency_reverse_edges_trigger_map_analysis(self):
        core = InterventionCore(cfg(), start_stamp=0.0)
        core.update_candidates(
            summary(10.0, 1, [Candidate("F1", 0.0, score=0.5)])
        )
        seq = [
            (10.0, False),
            (10.5, True),
            (11.0, False),
            (12.0, True),
            (12.5, False),
        ]
        for i, (t, active) in enumerate(seq):
            core.update_pose(pose(t))
            core.update_servo(
                servo(t, request_id=i + 1, emergency_reverse_active=active)
            )
        core.update_candidates(
            summary(12.5, 2, [Candidate("F1", 0.0, score=0.5)])
        )
        d = core.evaluate(12.5)
        self.assertEqual(d.kind, DecisionKind.MAP_DIRECT)
        self.assertEqual(d.reason, "REPEATED_EMERGENCY_REVERSE")

    def test_motion_disabled_blocks_intervention(self):
        core = InterventionCore(cfg(), start_stamp=0.0)
        core.update_pose(pose(10.0))
        core.update_servo(servo(10.0, motion_enabled=False))
        candidates = [
            Candidate("F1", -70.0, score=0.6),
            Candidate("F2", 70.0, score=0.59),
        ]
        core.update_candidates(summary(10.0, 1, candidates, decision_distance_m=0.5))
        core.update_candidates(summary(10.1, 2, candidates, decision_distance_m=0.5))
        d = core.evaluate(10.1)
        self.assertEqual(d.kind, DecisionKind.KEEP_EGO)
        self.assertEqual(d.reason, "MOTION_DISABLED")

    def test_inconsistent_returned_junction_summary_does_not_choose_arbitrary_direct(self):
        core = self.healthy_core(10.0)
        candidates = [
            Candidate("F1", -60.0, score=0.61),
            Candidate("F2", 60.0, score=0.60),
        ]
        for generation, t in [(1, 10.0), (2, 10.1)]:
            core.update_candidates(
                summary(
                    t, generation, candidates,
                    returned_to_junction=True,
                    unseen_candidate_count=1,
                    junction_id="J_bad",
                )
            )
        core.update_pose(pose(10.1))
        core.update_servo(servo(10.1))
        d = core.evaluate(10.1)
        self.assertEqual(d.kind, DecisionKind.MAP_QWEN)
        self.assertEqual(d.reason, "RETURNED_JUNCTION_MULTI")


if __name__ == "__main__":
    unittest.main()
