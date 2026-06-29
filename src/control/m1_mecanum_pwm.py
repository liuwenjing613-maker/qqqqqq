#!/usr/bin/env python3
"""M1 mecanum wheel layout and PWM mixing for open-loop set_motor()."""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

# Yahboom expansion-board doc (M1): M1=FL, M2=RL, M3=FR, M4=RR
# Wire order on board: FL, RL, FR, RR
YAHBOOM_M1_LAYOUT = "fl-rl-fr-rr"

WHEEL_LAYOUTS: Dict[str, Dict[str, int]] = {
    # Official Yahboom M1 board order — use this as default.
    "fl-rl-fr-rr": {"FL": 1, "RL": 2, "FR": 3, "RR": 4},
    "yahboom": {"FL": 1, "RL": 2, "FR": 3, "RR": 4},
    "m1": {"FL": 1, "RL": 2, "FR": 3, "RR": 4},
    # Textbook mecanum motor order (only if wiring differs)
    "fl-fr-rl-rr": {"FL": 1, "FR": 2, "RL": 3, "RR": 4},
    # Previous experimental mapping — kept for A/B tests
    "fl-fr-rr-rl": {"FL": 1, "FR": 2, "RR": 3, "RL": 4},
}


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def pwm_scalar(speed: float, deadband: float, gain: float) -> float:
    if abs(speed) < 1e-6:
        return 0.0
    return math.copysign(deadband + abs(speed) * gain, speed)


def mecanum_wheel_pwm(vx_pwm: float, wz_pwm: float) -> Dict[str, float]:
    """Standard mecanum IK with vy=0."""
    return {
        "FL": vx_pwm - wz_pwm,
        "FR": vx_pwm + wz_pwm,
        "RL": vx_pwm - wz_pwm,
        "RR": vx_pwm + wz_pwm,
    }


def map_wheels_to_motors(wheels: Dict[str, float], layout: str) -> Tuple[float, float, float, float]:
    key = str(layout).lower().replace("_", "-")
    mapping = WHEEL_LAYOUTS.get(key)
    if mapping is None:
        raise ValueError(
            f"unknown wheel_layout={layout!r}; choices: {', '.join(sorted(set(WHEEL_LAYOUTS)))}"
        )

    motors = [0.0, 0.0, 0.0, 0.0]
    for wheel, motor_idx in mapping.items():
        motors[motor_idx - 1] = float(wheels[wheel])
    return motors[0], motors[1], motors[2], motors[3]


def pwm_from_twist(
    vx: float,
    wz: float,
    *,
    vx_pwm_deadband: float,
    wz_pwm_deadband: float,
    vx_pwm_gain: float,
    wz_pwm_gain: float,
    pwm_max: float,
    wheel_layout: str = YAHBOOM_M1_LAYOUT,
) -> Tuple[float, float, float, float, float, float]:
    """Convert body vx/wz to four motor PWM targets."""
    vx_pwm = pwm_scalar(vx, vx_pwm_deadband, vx_pwm_gain)
    wz_pwm = pwm_scalar(wz, wz_pwm_deadband, wz_pwm_gain)

    wheels = mecanum_wheel_pwm(vx_pwm, wz_pwm)
    m1, m2, m3, m4 = map_wheels_to_motors(wheels, wheel_layout)

    m1 = clamp(m1, -pwm_max, pwm_max)
    m2 = clamp(m2, -pwm_max, pwm_max)
    m3 = clamp(m3, -pwm_max, pwm_max)
    m4 = clamp(m4, -pwm_max, pwm_max)
    return vx_pwm, wz_pwm, m1, m2, m3, m4


def describe_layout(layout: str) -> str:
    key = str(layout).lower().replace("_", "-")
    mapping = WHEEL_LAYOUTS.get(key, {})
    if not mapping:
        return f"unknown layout {layout!r}"
    parts = [f"M{idx}={wheel}" for wheel, idx in sorted(mapping.items(), key=lambda x: x[1])]
    return f"{key}: " + ", ".join(parts)
