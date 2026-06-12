#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--speed", type=int, default=15)
    args = parser.parse_args()

    print("=== set_car_run test ===")
    print("必须确认四轮架空。")
    input("确认后按 Enter：")

    bot = Rosmaster(car_type=1, com=args.port)
    bot.set_car_type(1)
    bot.create_receive_threading()
    time.sleep(0.5)

    tests = [
        (1, "forward"),
        (2, "backward"),
        (3, "left"),
        (4, "right"),
        (5, "spin left"),
        (6, "spin right"),
    ]

    try:
        bot.set_car_run(0, 0)
        time.sleep(0.5)

        for state, name in tests:
            print("\n====================")
            print(state, name)
            print("====================")
            input("准备观察，按 Enter 开始：")
            bot.set_car_run(state, args.speed, False)
            time.sleep(0.8)
            bot.set_car_run(0, 0, False)
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        bot.set_car_run(0, 0, False)
        bot.set_car_motion(0, 0, 0)
        bot.set_motor(0, 0, 0, 0)
        print("STOP.")


if __name__ == "__main__":
    main()
