#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    args = parser.parse_args()

    bot = Rosmaster(com=args.port)
    bot.create_receive_threading()
    time.sleep(0.3)

    print("=== Start deadzone test ===")
    print("必须架空轮子。每个速度持续 1 秒。")
    input("确认轮子架空后按 Enter：")

    speeds = [0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.10, 0.12]

    try:
        for vx in speeds:
            print(f"\nTEST vx={vx:.2f}")
            bot.set_car_motion(vx, 0.0, 0.0)
            time.sleep(1.0)
            bot.set_car_motion(0.0, 0.0, 0.0)
            time.sleep(0.8)
            input("观察轮子是否能从静止启动。按 Enter 测下一个速度：")

    except KeyboardInterrupt:
        pass
    finally:
        bot.set_car_motion(0.0, 0.0, 0.0)
        print("STOP")

if __name__ == "__main__":
    main()
