#!/usr/bin/env python3
"""Readable dry simulation of the five first-version intervention conditions."""

from __future__ import annotations

from intervention.core import (
    Candidate,
    CandidateSummary,
    InterventionConfig,
    InterventionCore,
    PoseSample,
    ServoSample,
)


def s(t, rid, **kw):
    data = dict(
        stamp=t,
        state="SEARCHING",
        result="TARGET_INFERRED",
        action="POINT",
        request_id=rid,
        point_role="search",
        horizontal_error=0.0,
        limited_vx=0.04,
        limited_wz=0.0,
        spawn_scan_phase="IDLE",
        view_adjust_phase="IDLE",
        emergency_reverse_active=False,
        turn_pending_action=None,
        mission_success=False,
        motion_enabled=True,
    )
    data.update(kw)
    return ServoSample(**data)


def show(label, decision):
    print(f"{label:35s} -> {decision.kind.value:10s} {decision.reason}")


def main():
    cfg = InterventionConfig(
        enabled=True,
        startup_grace_sec=0.0,
        servo_max_age_sec=1.0,
        odom_max_age_sec=1.0,
        candidates_max_age_sec=3.0,
    )

    # 1. Healthy first-person navigation.
    core = InterventionCore(cfg, 0.0)
    core.update_pose(PoseSample(10.0, 0.0, 0.0, 0.0))
    core.update_servo(s(10.0, 1))
    show("healthy corridor", core.evaluate(10.0))

    # 2. Persistent ambiguous fork.
    cand = (
        Candidate("F_left", -65.0, 0.70),
        Candidate("F_right", 60.0, 0.66),
    )
    core.update_candidates(CandidateSummary(10.0, 1, cand, decision_distance_m=0.8))
    show("fork first candidate update", core.evaluate(10.0))
    core.update_candidates(CandidateSummary(10.1, 2, cand, decision_distance_m=0.8))
    core.update_pose(PoseSample(10.1, 0.0, 0.0, 0.0))
    core.update_servo(s(10.1, 2))
    show("fork persistent second update", core.evaluate(10.1))

    # 3. Returned junction with only one unseen branch.
    core = InterventionCore(cfg, 0.0)
    core.update_pose(PoseSample(20.0, 0.0, 0.0, 0.0))
    core.update_servo(s(20.0, 1))
    only = (Candidate("B_unseen", 90.0, 0.5),)
    for generation, t in [(1, 20.0), (2, 20.1)]:
        core.update_candidates(
            CandidateSummary(
                t,
                generation,
                only,
                returned_to_junction=True,
                unseen_candidate_count=1,
                junction_id="J2",
            )
        )
    core.update_pose(PoseSample(20.1, 0.0, 0.0, 0.0))
    core.update_servo(s(20.1, 2))
    show("returned junction, one unseen", core.evaluate(20.1))

    # 4. Spawn/turn/target guards.
    core = InterventionCore(cfg, 0.0)
    core.update_pose(PoseSample(30.0, 0.0, 0.0, 0.0))
    core.update_servo(s(30.0, 1, state="SPAWN_SCAN", spawn_scan_phase="TURNING"))
    show("birth scan guard", core.evaluate(30.0))
    core.update_servo(s(30.1, 2, turn_pending_action="TURN_LEFT"))
    core.update_pose(PoseSample(30.1, 0.0, 0.0, 0.0))
    show("normal TURN guard", core.evaluate(30.1))
    core.update_servo(s(30.2, 3, result="TARGET_VISIBLE", point_role="target"))
    core.update_pose(PoseSample(30.2, 0.0, 0.0, 0.0))
    show("target takeover guard", core.evaluate(30.2))


if __name__ == "__main__":
    main()
