#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.fsm.nav_state_machine import NavFSMConfig, NavObservation, NavState, NavStateMachine


def obs(now=0.0, **kwargs):
    data = dict(
        now=now,
        image_fresh=True,
        scan_fresh=True,
        require_lidar=True,
        target_visible=False,
        target_stale=False,
        target_score=0.0,
        target_score_ok=False,
        target_u=None,
        target_v=None,
        target_centered=False,
        front_distance=1.2,
    )
    data.update(kwargs)
    return NavObservation(**data)


def target_obs(now=0.0, **kwargs):
    data = dict(
        target_visible=True,
        target_score=0.5,
        target_score_ok=True,
        target_u=320.0,
        target_v=240.0,
        target_centered=True,
    )
    data.update(kwargs)
    return obs(now=now, **data)


def test_search_candidate_track_after_stable_frames():
    fsm = NavStateMachine(NavFSMConfig(stable_frames_required=3))
    assert fsm.update(obs(0.0)).state == NavState.WAIT_SENSORS
    assert fsm.update(obs(0.1)).state == NavState.SEARCH
    assert fsm.update(target_obs(0.2)).state == NavState.CANDIDATE_LOCK
    assert fsm.update(target_obs(0.3)).state == NavState.CANDIDATE_LOCK
    assert fsm.update(target_obs(0.4)).state == NavState.TRACK


def test_lost_frames_enter_recovery():
    fsm = NavStateMachine(NavFSMConfig(stable_frames_required=1, lost_frames_limit=5))
    fsm.update(obs(0.0))
    fsm.update(obs(0.1))
    fsm.update(target_obs(0.2))
    fsm.update(target_obs(0.3))
    assert fsm.state == NavState.TRACK
    for i in range(4):
        assert fsm.update(obs(0.4 + i * 0.1)).state == NavState.TRACK
    assert fsm.update(obs(0.9)).state == NavState.LOST_RECOVERY


def test_emergency_enters_blocked_from_any_state():
    fsm = NavStateMachine(NavFSMConfig())
    fsm.update(obs(0.0))
    fsm.update(obs(0.1))
    assert fsm.update(obs(0.2, emergency=True)).state == NavState.BLOCKED


def test_arrive_frames_then_success_without_qwen():
    fsm = NavStateMachine(NavFSMConfig(stable_frames_required=1, arrive_required_frames=4))
    fsm.update(obs(0.0))
    fsm.update(obs(0.1))
    fsm.update(target_obs(0.2))
    fsm.update(target_obs(0.3))
    assert fsm.state == NavState.TRACK
    for i in range(3):
        assert fsm.update(target_obs(0.4 + i * 0.1, front_distance=0.65)).state == NavState.TRACK
    assert fsm.update(target_obs(0.8, front_distance=0.65)).state == NavState.ARRIVE_VERIFY
    for i in range(3):
        assert fsm.update(target_obs(0.9 + i * 0.1, front_distance=0.65)).state == NavState.ARRIVE_VERIFY
    assert fsm.update(target_obs(1.2, front_distance=0.65)).state == NavState.SUCCESS


def test_stale_target_cannot_enter_track():
    fsm = NavStateMachine(NavFSMConfig(stable_frames_required=1))
    fsm.update(obs(0.0))
    fsm.update(obs(0.1))
    result = fsm.update(target_obs(0.2, target_stale=True))
    assert result.state == NavState.SEARCH


if __name__ == "__main__":
    test_search_candidate_track_after_stable_frames()
    test_lost_frames_enter_recovery()
    test_emergency_enters_blocked_from_any_state()
    test_arrive_frames_then_success_without_qwen()
    test_stale_target_cannot_enter_track()
    print("PASS test_nav_state_machine")
