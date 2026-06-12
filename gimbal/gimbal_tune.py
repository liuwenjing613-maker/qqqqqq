#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster


def clamp_angle(x):
    return max(0, min(180, int(x)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--yaw-id", type=int, default=1)
    parser.add_argument("--pitch-id", type=int, default=2)
    parser.add_argument("--yaw", type=int, default=90)
    parser.add_argument("--pitch", type=int, default=90)
    args = parser.parse_args()

    bot = Rosmaster(com=args.port)
    time.sleep(0.2)

    yaw = clamp_angle(args.yaw)
    pitch = clamp_angle(args.pitch)

    print("=== Gimbal tune ===")
    print(f"port: {args.port}")
    print(f"yaw servo: S{args.yaw_id}, angle={yaw}")
    print(f"pitch servo: S{args.pitch_id}, angle={pitch}")

    bot.set_pwm_servo(args.yaw_id, yaw)
    time.sleep(0.3)
    bot.set_pwm_servo(args.pitch_id, pitch)
    time.sleep(0.5)

    print("Done.")


if __name__ == "__main__":
    main()
