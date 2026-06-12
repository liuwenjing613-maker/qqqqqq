#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster


def clamp_angle(x):
    return max(0, min(180, int(x)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--s1", type=int, default=90)
    parser.add_argument("--s2", type=int, default=90)
    parser.add_argument("--s3", type=int, default=90)
    parser.add_argument("--s4", type=int, default=90)
    parser.add_argument("--only", type=int, default=0, help="0 means all; 1~4 means only one servo")
    args = parser.parse_args()

    bot = Rosmaster(com=args.port)
    time.sleep(0.2)

    angles = {
        1: clamp_angle(args.s1),
        2: clamp_angle(args.s2),
        3: clamp_angle(args.s3),
        4: clamp_angle(args.s4),
    }

    print("=== Gimbal center once ===")
    print(f"port: {args.port}")
    print(f"angles: {angles}")

    if args.only in [1, 2, 3, 4]:
        sid = args.only
        print(f"Set only servo S{sid} -> {angles[sid]}")
        bot.set_pwm_servo(sid, angles[sid])
    else:
        print("Set all PWM servos")
        bot.set_pwm_servo_all(
            angles[1],
            angles[2],
            angles[3],
            angles[4],
        )

    time.sleep(0.5)
    print("Done.")


if __name__ == "__main__":
    main()
