#!/usr/bin/env python3
"""Offline timeline simulation of the control-source transfer handshake.

This is intentionally ROS-free. It validates the ordering invariants used by
third_view_intervention_node.py: EGO -> HOLD -> request -> MAP -> HOLD -> fresh
first-person result -> EGO, plus ACK timeout and target takeover paths.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Sim:
    stop_hold: float = 0.30
    ack_timeout: float = 6.0
    resume_min_hold: float = 0.30
    resume_fresh_timeout: float = 3.0
    phase: str = "EGO"
    mode: str = "EGO"
    phase_started: float = 0.0
    request_published: bool = False
    resume_ref: int = -1
    latest_request_id: int = 10
    fresh_ready: bool = False

    def trigger(self, now: float) -> None:
        assert self.phase == "EGO"
        self.phase, self.mode, self.phase_started = "STOPPING", "HOLD", now

    def tick(self, now: float) -> None:
        if self.phase == "STOPPING" and now - self.phase_started >= self.stop_hold:
            self.request_published = True
            self.phase, self.phase_started = "WAIT_ACK", now
        elif self.phase == "WAIT_ACK" and now - self.phase_started >= self.ack_timeout:
            self.begin_resume(now)
        elif self.phase == "RESUMING":
            if self.latest_request_id > self.resume_ref:
                self.fresh_ready = True
            age = now - self.phase_started
            if age >= self.resume_min_hold and (
                self.fresh_ready or age >= self.resume_fresh_timeout
            ):
                self.phase, self.mode = "EGO", "EGO"

    def ack(self, now: float) -> None:
        assert self.phase == "WAIT_ACK"
        self.phase, self.mode, self.phase_started = "MAP_NAV", "MAP", now

    def completed(self, now: float) -> None:
        assert self.phase == "MAP_NAV"
        self.begin_resume(now)

    def target(self, now: float, request_id: int) -> None:
        assert self.phase in {"STOPPING", "WAIT_ACK", "MAP_NAV"}
        self.latest_request_id = request_id
        self.phase, self.mode, self.phase_started = "RESUMING", "HOLD", now
        self.resume_ref = request_id
        self.fresh_ready = True

    def begin_resume(self, now: float) -> None:
        self.resume_ref = self.latest_request_id
        self.fresh_ready = False
        self.phase, self.mode, self.phase_started = "RESUMING", "HOLD", now


def print_state(label: str, t: float, sim: Sim) -> None:
    print(
        f"{label:32s} t={t:4.1f} phase={sim.phase:9s} "
        f"mode={sim.mode:4s} request={sim.request_published}"
    )


def normal_path() -> None:
    sim = Sim()
    sim.trigger(0.0)
    print_state("trigger branch", 0.0, sim)
    assert sim.mode == "HOLD"
    sim.tick(0.31)
    print_state("stop hold complete", 0.31, sim)
    assert sim.phase == "WAIT_ACK" and sim.request_published
    sim.ack(0.70)
    print_state("teammate accepted", 0.70, sim)
    assert sim.mode == "MAP"
    sim.completed(2.00)
    print_state("A* completed", 2.00, sim)
    assert sim.mode == "HOLD"
    sim.tick(2.40)
    assert sim.phase == "RESUMING"  # no fresh first-person result yet
    sim.latest_request_id = 11
    sim.tick(2.90)
    print_state("fresh ego result", 2.90, sim)
    assert sim.phase == "EGO" and sim.mode == "EGO"


def timeout_path() -> None:
    sim = Sim()
    sim.trigger(0.0)
    sim.tick(0.31)
    sim.tick(6.32)
    print_state("ACK timeout -> hold", 6.32, sim)
    assert sim.phase == "RESUMING" and sim.mode == "HOLD"
    sim.latest_request_id = 11
    sim.tick(6.70)
    print_state("ACK timeout -> ego", 6.70, sim)
    assert sim.phase == "EGO"


def target_takeover_path() -> None:
    sim = Sim()
    sim.trigger(0.0)
    sim.tick(0.31)
    sim.ack(0.50)
    sim.target(1.20, request_id=11)
    print_state("fresh target cancels map", 1.20, sim)
    assert sim.mode == "HOLD"
    sim.tick(1.51)
    print_state("target servo resumes", 1.51, sim)
    assert sim.phase == "EGO" and sim.mode == "EGO"


def main() -> None:
    normal_path()
    timeout_path()
    target_takeover_path()
    print("TRANSFER HANDSHAKE SIMULATION PASSED")


if __name__ == "__main__":
    main()
