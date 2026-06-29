#!/usr/bin/env python3
"""Sweep equal PWM on all four motors via Rosmaster set_motor()."""

from __future__ import annotations

import argparse
import time

from Rosmaster_Lib import Rosmaster


def parse_pwm_list(text: str) -> list[int]:
    return [int(float(x.strip())) for x in text.split(",") if x.strip()]


def stop_robot(bot: Rosmaster, repeat: int = 5, interval: float = 0.08) -> None:
    for _ in range(repeat):
        try:
            bot.set_motor(0, 0, 0, 0)
        except Exception:
            pass
        time.sleep(interval)


def main() -> None:
    parser = argparse.ArgumentParser(description="Sweep M1 motor PWM values on all wheels")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--pwms", default="6,8,10,12,14,16,18", help="Comma-separated PWM values")
    parser.add_argument("--duration", type=float, default=2.0, help="Hold seconds per PWM step")
    parser.add_argument("--rest", type=float, default=1.0, help="Rest seconds after each step")
    args = parser.parse_args()

    pwms = parse_pwm_list(args.pwms)
    if not pwms:
        raise SystemExit("No PWM values provided")

    bot = Rosmaster(com=args.port)
    bot.create_receive_threading()
    time.sleep(0.6)
    stop_robot(bot)

    print(f"===== M1 PWM sweep start: {pwms} =====")
    try:
        for pwm in pwms:
            print(f"\n--- set_motor({pwm}, {pwm}, {pwm}, {pwm}) for {args.duration:.1f}s ---")
            bot.set_motor(pwm, pwm, pwm, pwm)
            time.sleep(max(0.0, float(args.duration)))
            stop_robot(bot)
            print(f"stopped, rest {args.rest:.1f}s")
            time.sleep(max(0.0, float(args.rest)))
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        stop_robot(bot)
        print("===== M1 PWM sweep done =====")


if __name__ == "__main__":
    main()
