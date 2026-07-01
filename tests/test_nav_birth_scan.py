#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.config.nav_birth_scan import birth_scan_duration_sec, load_birth_scan_config
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
        target_center_error_px=0.0,
    )
    data.update(kwargs)
    return obs(now=now, **data)


def birth_cfg(**kwargs):
    base = dict(
        birth_scan_enabled=True,
        birth_scan_wait_sec=5.0,
        birth_scan_wz=0.03,
        birth_scan_deg=360.0,
        stable_frames_required=1,
        max_task_sec=0,
    )
    base.update(kwargs)
    return NavFSMConfig(**base)


def test_birth_wait_then_scanning():
    fsm = NavStateMachine(birth_cfg())
    assert fsm.update(obs(0.0)).state == NavState.WAIT_SENSORS
    assert fsm.update(obs(0.1)).state == NavState.BIRTH_WAIT
    assert fsm.update(obs(2.0)).state == NavState.BIRTH_WAIT
    assert fsm.update(obs(5.1)).state == NavState.SCANNING


def test_birth_wait_target_goes_candidate_lock():
    fsm = NavStateMachine(birth_cfg())
    fsm.update(obs(0.0))
    fsm.update(obs(0.1))
    assert fsm.update(target_obs(1.0)).state == NavState.CANDIDATE_LOCK
    assert fsm.birth_phase_completed


def test_scanning_target_goes_candidate_lock():
    fsm = NavStateMachine(birth_cfg(birth_scan_wait_sec=0.0))
    fsm.update(obs(0.0))
    fsm.update(obs(0.1))
    fsm.update(obs(0.2))
    assert fsm.state == NavState.SCANNING
    assert fsm.update(target_obs(0.3)).state == NavState.CANDIDATE_LOCK


def test_scanning_complete_goes_search():
    scan_deg = 18.0
    duration = birth_scan_duration_sec(scan_deg, 0.03)
    fsm = NavStateMachine(birth_cfg(birth_scan_wait_sec=0.0, birth_scan_deg=scan_deg))
    fsm.update(obs(0.0))
    fsm.update(obs(0.1))
    fsm.update(obs(0.2))
    assert fsm.state == NavState.SCANNING
    t_enter = 0.2
    assert fsm.update(obs(t_enter + duration - 0.01)).state == NavState.SCANNING
    assert fsm.update(obs(t_enter + duration + 0.01)).state == NavState.SEARCH
    assert fsm.birth_phase_completed


def test_birth_scan_disabled_keeps_legacy_flow():
    fsm = NavStateMachine(NavFSMConfig(stable_frames_required=1, birth_scan_enabled=False))
    assert fsm.update(obs(0.0)).state == NavState.WAIT_SENSORS
    assert fsm.update(obs(0.1)).state == NavState.SEARCH


def test_scanning_ignores_emergency():
    fsm = NavStateMachine(birth_cfg(birth_scan_wait_sec=0.0))
    fsm.update(obs(0.0))
    fsm.update(obs(0.1))
    fsm.update(obs(0.2))
    assert fsm.state == NavState.SCANNING
    assert fsm.update(obs(0.3, emergency=True, front_distance=0.08)).state == NavState.SCANNING


def test_load_birth_scan_effective_wz_clipped():
    cfg = load_birth_scan_config(
        {
            "birth_scan": {"enabled": True, "scan_wz": 0.2, "scan_deg": 360.0},
            "chassis": {"max_wz": 0.06},
        }
    )
    assert cfg["effective_scan_wz"] == 0.06
    assert abs(cfg["scan_duration_sec"] - birth_scan_duration_sec(360.0, 0.06)) < 1e-6


def test_load_birth_scan_config():
    cfg = load_birth_scan_config(
        {"birth_scan": {"enabled": True, "wait_sec": 5.0, "scan_wz": 0.03, "scan_deg": 360.0}}
    )
    assert cfg["enabled"] is True
    assert cfg["wait_sec"] == 5.0
    assert cfg["scan_wz"] == 0.03
    assert abs(cfg["scan_duration_sec"] - birth_scan_duration_sec(360.0, 0.03)) < 1e-6


if __name__ == "__main__":
    test_birth_wait_then_scanning()
    test_birth_wait_target_goes_candidate_lock()
    test_scanning_target_goes_candidate_lock()
    test_scanning_complete_goes_search()
    test_birth_scan_disabled_keeps_legacy_flow()
    test_scanning_ignores_emergency()
    test_load_birth_scan_effective_wz_clipped()
    test_load_birth_scan_config()
    print("PASS test_nav_birth_scan")
