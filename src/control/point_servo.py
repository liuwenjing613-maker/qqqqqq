#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Any, Dict, Optional


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class PointServoConfig:
    image_width: int = 640
    image_height: int = 480
    max_vx: float = 0.06
    steer_vx: float = 0.04
    max_wz: float = 0.06
    kp_turn: float = 0.12
    center_deadband: float = 0.06
    turn_only_threshold: float = 0.20
    turn_only_vx: float = 0.04
    cmd_wz_deadband: float = 0.006


@dataclass
class ServoCommand:
    vx: float = 0.0
    wz: float = 0.0


@dataclass
class PointServoResult:
    cmd: ServoCommand
    state: str
    reason: str
    ex: Optional[float] = None


class PointServo:
    def __init__(self, cfg: Optional[PointServoConfig] = None):
        self.cfg = cfg or PointServoConfig()

    def compute_cmd(self, target: Dict[str, Any]) -> PointServoResult:
        if not target or not target.get("visible", False) or target.get("stale", False):
            return PointServoResult(ServoCommand(), "LOST_STOP", "target_not_visible")

        u = target.get("u", target.get("cx"))
        if u is None:
            return PointServoResult(ServoCommand(), "LOST_STOP", "target_missing_u")

        ex = (float(u) - self.cfg.image_width / 2.0) / max(float(self.cfg.image_width), 1.0)
        wz = clamp(-self.cfg.kp_turn * ex, -self.cfg.max_wz, self.cfg.max_wz)
        if abs(wz) < self.cfg.cmd_wz_deadband:
            wz = 0.0

        abs_ex = abs(ex)
        if abs_ex <= self.cfg.center_deadband:
            return PointServoResult(
                ServoCommand(vx=self.cfg.max_vx, wz=0.0),
                "FORWARD",
                "target_centered",
                ex,
            )

        if abs_ex < self.cfg.turn_only_threshold:
            return PointServoResult(
                ServoCommand(vx=self.cfg.steer_vx, wz=wz),
                "FORWARD_STEER",
                "target_steer",
                ex,
            )

        return PointServoResult(
            ServoCommand(vx=self.cfg.turn_only_vx, wz=wz),
            "TURN_ONLY",
            "target_turn_only",
            ex,
        )
