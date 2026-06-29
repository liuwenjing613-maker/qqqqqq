#!/usr/bin/env python3
"""Print M1 mecanum PWM mixing for each wheel layout."""

from __future__ import annotations

import os
import sys

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.control.m1_mecanum_pwm import WHEEL_LAYOUTS, describe_layout, pwm_from_twist


def show(label: str, vx: float, wz: float, layout: str) -> None:
    vx_pwm, wz_pwm, m1, m2, m3, m4 = pwm_from_twist(
        vx,
        wz,
        vx_pwm_deadband=6.0,
        wz_pwm_deadband=8.0,
        vx_pwm_gain=180.0,
        wz_pwm_gain=120.0,
        pwm_max=30.0,
        wheel_layout=layout,
    )
    print(
        f"{label:10s} [{layout:12s}] vx={vx:.2f} wz={wz:.2f} -> "
        f"M1={m1:5.0f} M2={m2:5.0f} M3={m3:5.0f} M4={m4:5.0f}  "
        f"(vx_pwm={vx_pwm:.1f} wz_pwm={wz_pwm:.1f})"
    )


def main() -> None:
    print("=== Yahboom M1 mecanum PWM mixing ===")
    print("Official board order: M1=FL, M2=RL, M3=FR, M4=RR\n")

    layouts = sorted(set(WHEEL_LAYOUTS.keys()))
    for layout in layouts:
        print(describe_layout(layout))
    print()

    for layout in ("fl-rl-fr-rr", "fl-fr-rl-rr", "fl-fr-rr-rl"):
        show("forward", 0.03, 0.0, layout)
        show("turn-left", 0.0, 0.12, layout)
        print()

    print("Correct turn-left on Yahboom M1 should look like:")
    print("  M1=-  M2=-  M3=+  M4=+   (left wheels same, right wheels same)")
    print("\nWrong turn (looks like strafe) looks like:")
    print("  M1=-  M2=+  M3=-  M4=+   (diagonal pattern)")


if __name__ == "__main__":
    main()
