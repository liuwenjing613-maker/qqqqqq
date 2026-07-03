#!/usr/bin/env python3
import math
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
        birth_scan_effective_wz=0.03,
        birth_scan_deg=360.0,
        birth_scan_max_rotations=2.0,
        birth_scan_max_wall_timeout_sec=0.0,
        stable_frames_required=1,
        max_task_sec=0,
    )
    base.update(kwargs)
    return NavFSMConfig(**base)


def enter_scanning(fsm: NavStateMachine, t0: float = 0.0) -> float:
    fsm.update(obs(t0))
    fsm.update(obs(t0 + 0.1))
    fsm.update(obs(t0 + 0.2))
    assert fsm.state == NavState.SCANNING
    return t0 + 0.2


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
    enter_scanning(fsm)
    assert fsm.update(target_obs(0.3)).state == NavState.CANDIDATE_LOCK


def advance_scanning(fsm: NavStateMachine, t_start: float, duration: float, dt: float = 0.05) -> float:
    t = t_start + dt
    while t <= t_start + duration + dt:
        fsm.update(obs(t))
        t += dt
    return t


def test_scanning_complete_goes_search():
    scan_deg = 18.0
    max_rot = 1
    wz = 0.03
    duration = birth_scan_duration_sec(scan_deg * max_rot, wz)
    fsm = NavStateMachine(
        birth_cfg(
            birth_scan_wait_sec=0.0,
            birth_scan_deg=scan_deg,
            birth_scan_max_rotations=max_rot,
            birth_scan_effective_wz=wz,
        )
    )
    t_enter = enter_scanning(fsm)
    advance_scanning(fsm, t_enter, duration - 0.05)
    assert fsm.state == NavState.SCANNING
    result = fsm.update(obs(t_enter + duration + 0.2))
    assert result.state == NavState.SEARCH
    assert fsm.birth_phase_completed


def test_max_two_rotations_total():
    wz = 0.03
    duration = birth_scan_duration_sec(720.0, wz)
    fsm = NavStateMachine(
        birth_cfg(
            birth_scan_wait_sec=0.0,
            birth_scan_deg=360.0,
            birth_scan_max_rotations=2.0,
            birth_scan_effective_wz=wz,
        )
    )
    t_enter = enter_scanning(fsm)
    advance_scanning(fsm, t_enter, duration - 0.05)
    assert fsm.state == NavState.SCANNING
    result = fsm.update(obs(t_enter + duration + 0.2))
    assert result.state == NavState.SEARCH
    assert result.reason == "birth_scan_budget_exhausted"
    assert fsm.birth_phase_completed
    assert fsm.birth_scan_yaw_accumulated_rad >= math.radians(720.0) - 1e-3


def test_sensor_stale_holds_scanning_not_birth_wait():
    fsm = NavStateMachine(birth_cfg(birth_scan_wait_sec=0.0))
    enter_scanning(fsm)
    fsm.birth_scan_yaw_accumulated_rad = 0.5
    result = fsm.update(obs(0.3, scan_fresh=False))
    assert result.state == NavState.SCANNING
    assert result.reason == "scanning"
    result = fsm.update(obs(0.4, scan_fresh=True))
    assert result.state == NavState.SCANNING


def test_birth_scan_budget_exhausted_no_reentry():
    fsm = NavStateMachine(birth_cfg(birth_scan_wait_sec=0.0))
    enter_scanning(fsm)
    fsm.birth_scan_yaw_accumulated_rad = fsm._birth_scan_budget_rad()
    result = fsm.update(obs(0.3))
    assert result.state == NavState.SEARCH
    assert fsm.birth_phase_completed
    fsm.update(obs(1.0, scan_fresh=False))
    result = fsm.update(obs(1.1))
    assert result.state == NavState.SEARCH
    assert not fsm._should_start_or_resume_birth_scan()


def test_birth_scan_disabled_keeps_legacy_flow():
    fsm = NavStateMachine(NavFSMConfig(stable_frames_required=1, birth_scan_enabled=False))
    assert fsm.update(obs(0.0)).state == NavState.WAIT_SENSORS
    assert fsm.update(obs(0.1)).state == NavState.SEARCH


def test_scanning_ignores_emergency():
    fsm = NavStateMachine(birth_cfg(birth_scan_wait_sec=0.0))
    enter_scanning(fsm)
    assert fsm.update(obs(0.3, emergency=True, front_distance=0.08)).state == NavState.SCANNING


def test_load_birth_scan_effective_wz_clipped():
    cfg = load_birth_scan_config(
        {
            "birth_scan": {"enabled": True, "scan_wz": 0.2, "scan_deg": 360.0},
            "chassis": {"max_wz": 0.06},
        }
    )
    assert cfg["effective_scan_wz"] == 0.06
    assert cfg["max_rotations"] == 2.0
    assert abs(cfg["max_total_scan_deg"] - 720.0) < 1e-6
    assert abs(cfg["scan_duration_sec"] - birth_scan_duration_sec(360.0, 0.06)) < 1e-6
    assert abs(cfg["max_total_duration_sec"] - birth_scan_duration_sec(720.0, 0.06)) < 1e-6


def test_load_birth_scan_config():
    cfg = load_birth_scan_config(
        {"birth_scan": {"enabled": True, "wait_sec": 5.0, "scan_wz": 0.03, "scan_deg": 360.0}}
    )
    assert cfg["enabled"] is True
    assert cfg["wait_sec"] == 5.0
    assert cfg["scan_wz"] == 0.03
    assert cfg["max_rotations"] == 2.0
    assert abs(cfg["scan_duration_sec"] - birth_scan_duration_sec(360.0, 0.03)) < 1e-6
    assert abs(cfg["max_total_duration_sec"] - birth_scan_duration_sec(720.0, 0.03)) < 1e-6


def test_birth_wait_holds_through_image_stale():
    fsm = NavStateMachine(birth_cfg())
    fsm.update(obs(0.0))
    fsm.update(obs(0.1))
    assert fsm.state == NavState.BIRTH_WAIT
    assert fsm.update(obs(0.2, image_fresh=False)).state == NavState.BIRTH_WAIT
    assert fsm.update(obs(0.2, image_fresh=False)).reason == "birth_wait"


def test_wait_sensors_recovers_birth_with_scan_only():
    fsm = NavStateMachine(birth_cfg())
    fsm.update(obs(0.0))
    result = fsm.update(obs(0.1, image_fresh=False, scan_fresh=True))
    assert result.state == NavState.BIRTH_WAIT
    assert result.reason == "sensor_ready_birth_wait"


def test_birth_wait_timer_survives_wait_sensors_bounce():
    fsm = NavStateMachine(birth_cfg(birth_scan_wait_sec=1.0))
    fsm.update(obs(0.0))
    fsm.update(obs(0.1))
    assert fsm._birth_wait_started_at == 0.1
    fsm.state = NavState.WAIT_SENSORS
    result = fsm.update(obs(0.5, scan_fresh=True))
    assert result.state == NavState.BIRTH_WAIT
    assert fsm._birth_wait_started_at == 0.1
    result = fsm.update(obs(1.15, scan_fresh=True))
    assert result.state == NavState.SCANNING
    assert result.reason == "birth_wait_timeout_scan"


if __name__ == "__main__":
    test_birth_wait_then_scanning()
    test_birth_wait_target_goes_candidate_lock()
    test_scanning_target_goes_candidate_lock()
    test_scanning_complete_goes_search()
    test_max_two_rotations_total()
    test_sensor_stale_holds_scanning_not_birth_wait()
    test_birth_scan_budget_exhausted_no_reentry()
    test_birth_scan_disabled_keeps_legacy_flow()
    test_scanning_ignores_emergency()
    test_load_birth_scan_effective_wz_clipped()
    test_load_birth_scan_config()
    test_birth_wait_holds_through_image_stale()
    test_wait_sensors_recovers_birth_with_scan_only()
    test_birth_wait_timer_survives_wait_sensors_bounce()
    print("PASS test_nav_birth_scan")
