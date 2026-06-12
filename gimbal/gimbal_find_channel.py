#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--base", type=int, default=90)
    parser.add_argument("--delta", type=int, default=10)
    parser.add_argument("--sleep", type=float, default=0.8)
    args = parser.parse_args()

    bot = Rosmaster(com=args.port)
    time.sleep(0.2)

    base = args.base
    delta = args.delta

    print("=== Find gimbal servo channel ===")
    print("每次只让一路舵机在 base±delta 小范围摆动。")
    print("观察哪一路对应水平转动，哪一路对应俯仰转动。")
    print("如果有机械卡住或角度异常，立即 Ctrl+C。")

    try:
        # 先全部回 90
        bot.set_pwm_servo_all(base, base, base, base)
        time.sleep(1.0)

        for sid in [1, 2, 3, 4]:
            input(f"\n准备测试 S{sid}，按 Enter 后它会小幅摆动：")

            print(f"S{sid}: {base} -> {base + delta}")
            bot.set_pwm_servo(sid, base + delta)
            time.sleep(args.sleep)

            print(f"S{sid}: {base + delta} -> {base - delta}")
            bot.set_pwm_servo(sid, base - delta)
            time.sleep(args.sleep)

            print(f"S{sid}: back to {base}")
            bot.set_pwm_servo(sid, base)
            time.sleep(args.sleep)

        print("\n测试完成。请记录：")
        print("水平 yaw 舵机是 S几？")
        print("俯仰 pitch 舵机是 S几？")

    except KeyboardInterrupt:
        print("\nInterrupted, set all to base.")
        bot.set_pwm_servo_all(base, base, base, base)


if __name__ == "__main__":
    main()
