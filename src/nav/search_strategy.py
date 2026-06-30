#!/usr/bin/env python3
"""Target search memory and direction helpers for shared_nav."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class TargetSearchMemory:
    last_visible_time: Optional[float] = None
    last_target_u: Optional[float] = None
    last_target_ex: Optional[float] = None
    search_turn_dir: float = 1.0
    search_turn_locked_until: float = 0.0
    search_mode: str = "init"


def turn_dir_from_ex(ex: float) -> float:
    """Match PointServo sign: target on right (ex>0) -> negative wz."""
    if abs(ex) < 1e-6:
        return 1.0
    return -1.0 if ex > 0 else 1.0


def compute_loss_age(now: float, last_visible_time: Optional[float]) -> float:
    if last_visible_time is None:
        return float("inf")
    return max(0.0, now - last_visible_time)


def should_use_free_space(loss_age_sec: float, enabled: bool, after_sec: float) -> bool:
    return bool(enabled) and loss_age_sec >= after_sec


def pick_clearance_turn_dir(
    left: Optional[float],
    right: Optional[float],
    min_delta: float,
    current_dir: float,
) -> Tuple[float, str]:
    if left is None and right is None:
        return current_dir, "unknown"
    if left is not None and right is not None and abs(left - right) < min_delta:
        return current_dir, "hold"
    if right is None or (left is not None and left >= right):
        return 1.0, "left"
    return -1.0, "right"
