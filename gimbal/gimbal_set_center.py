#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster


DEFAULT_PORT = "/dev/myserial"

# 按你的实际测试结果修改这里
DEFAULT_YAW_ID = 2
DEFAULT_PITCH_ID = 3
DEFAULT_YAW_ANGLE = 60
DEFAULT_PITCH_ANGLE = 130


def clamp_angle(x):
    return max(0, min(180, int(x)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--yaw-id", type=int, default=DEFAULT_YAW_ID)
    parser.add_argument("--pitch-id", type=int, default=DEFAULT_PITCH_ID)
    parser.add_argument("--yaw", type=int, default=DEFAULT_YAW_ANGLE)
    parser.add_argument("--pitch", type=int, default=DEFAULT_PITCH_ANGLE)
    args = parser.parse_args()

    yaw = clamp_angle(args.yaw)
    pitch = clamp_angle(args.pitch)

    print("=== Set gimbal center ===")
    print(f"port: {args.port}")
    print(f"yaw: S{args.yaw_id} -> {yaw}")
    print(f"pitch: S{args.pitch_id} -> {pitch}")

    bot = Rosmaster(com=args.port)
    time.sleep(0.2)

    bot.set_pwm_servo(args.yaw_id, yaw)
    time.sleep(0.3)
    bot.set_pwm_servo(args.pitch_id, pitch)
    time.sleep(0.5)

    print("Gimbal center set.")


if __name__ == "__main__":
    main()
