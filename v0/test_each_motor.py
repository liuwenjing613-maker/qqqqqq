#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster


def safe_stop(bot):
    bot.set_motor(0, 0, 0, 0)
    bot.set_car_motion(0, 0, 0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--duration", type=float, default=0.8)
    args = parser.parse_args()

    print("=== Individual motor test ===")
    print("必须确认四个轮子架空！")
    print("这个测试会逐个驱动 M1/M2/M3/M4。")
    print("你需要记录每次到底哪个实际轮子转。")
    input("确认轮子架空后按 Enter：")

    bot = Rosmaster(car_type=1, com=args.port)
    bot.set_car_type(1)
    bot.create_receive_threading()
    time.sleep(0.5)

    tests = [
        ("M1 only", (args.speed, 0, 0, 0)),
        ("M2 only", (0, args.speed, 0, 0)),
        ("M3 only", (0, 0, args.speed, 0)),
        ("M4 only", (0, 0, 0, args.speed)),
    ]

    try:
        safe_stop(bot)
        time.sleep(0.5)

        try:
            print("version:", bot.get_version())
            print("battery:", bot.get_battery_voltage())
        except Exception as e:
            print("read status failed:", e)

        for name, speeds in tests:
            print("\n======================")
            print(name, speeds)
            print("======================")
            input("准备观察这个电机，按 Enter 开始：")

            try:
                before = bot.get_motor_encoder()
                print("encoder before:", before)
            except Exception as e:
                print("encoder before read failed:", e)

            bot.set_motor(*speeds)
            time.sleep(args.duration)
            safe_stop(bot)
            time.sleep(0.5)

            try:
                after = bot.get_motor_encoder()
                print("encoder after:", after)
            except Exception as e:
                print("encoder after read failed:", e)

            print("请记录：刚才哪个轮子转了？方向是否正常？")

        print("\n反向测试，每个电机反转一次。")
        input("按 Enter 开始反向测试：")

        reverse_tests = [
            ("M1 reverse", (-args.speed, 0, 0, 0)),
            ("M2 reverse", (0, -args.speed, 0, 0)),
            ("M3 reverse", (0, 0, -args.speed, 0)),
            ("M4 reverse", (0, 0, 0, -args.speed)),
        ]

        for name, speeds in reverse_tests:
            print("\n======================")
            print(name, speeds)
            print("======================")
            input("准备观察这个电机，按 Enter 开始：")
            bot.set_motor(*speeds)
            time.sleep(args.duration)
            safe_stop(bot)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        safe_stop(bot)
        print("STOP.")


if __name__ == "__main__":
    main()
