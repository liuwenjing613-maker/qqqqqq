#!/usr/bin/env python3
"""Ground spin test: try each wheel layout with pure wz via set_motor PWM bridge logic."""

from __future__ import annotations

import argparse
import os
import sys
import time

from Rosmaster_Lib import Rosmaster

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.control.m1_mecanum_pwm import describe_layout, pwm_from_twist


def stop(bot: Rosmaster, repeat: int = 5) -> None:
    for _ in range(repeat):
        bot.set_motor(0, 0, 0, 0)
        time.sleep(0.08)


def pulse_turn(bot: Rosmaster, layout: str, wz: float, duration: float, pwm_max: float) -> None:
    _, _, m1, m2, m3, m4 = pwm_from_twist(
        0.0,
        wz,
        vx_pwm_deadband=6.0,
        wz_pwm_deadband=8.0,
        vx_pwm_gain=180.0,
        wz_pwm_gain=120.0,
        pwm_max=pwm_max,
        wheel_layout=layout,
    )
    print(
        f"layout={layout} wz={wz:.2f} -> set_motor({int(m1)}, {int(m2)}, {int(m3)}, {int(m4)}) for {duration:.1f}s"
    )
    bot.set_motor(int(m1), int(m2), int(m3), int(m4))
    time.sleep(duration)
    stop(bot)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--wz", type=float, default=0.12)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rest", type=float, default=1.0)
    parser.add_argument("--pwm-max", type=float, default=30.0)
    parser.add_argument(
        "--layouts",
        default="fl-rl-fr-rr,fl-fr-rl-rr,fl-fr-rr-rl",
        help="Comma-separated layouts to test",
    )
    args = parser.parse_args()

    layouts = [x.strip() for x in args.layouts.split(",") if x.strip()]
    bot = Rosmaster(car_type=1, com=args.port)
    bot.create_receive_threading()
    time.sleep(0.6)
    stop(bot)

    print("=== M1 turn layout test ===")
    print("Watch the robot: should rotate in place, NOT strafe sideways.")
    print("Stop with Ctrl+C if behavior is wrong.\n")

    try:
        for layout in layouts:
            print(describe_layout(layout))
            pulse_turn(bot, layout, args.wz, args.duration, args.pwm_max)
            print(f"rest {args.rest:.1f}s\n")
            time.sleep(args.rest)
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        stop(bot)
        print("DONE")


if __name__ == "__main__":
    main()
