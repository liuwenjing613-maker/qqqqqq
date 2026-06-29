#!/usr/bin/env python3
import argparse
import json
import sys
import time

from Rosmaster_Lib import Rosmaster

PROJECT_ROOT = __import__("os").path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.control.m1_rosmaster_extensions import apply_m1_runtime_sanitize


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--pid-kp", type=float, default=1.2)
    parser.add_argument("--pid-ki", type=float, default=0.05)
    parser.add_argument("--pid-kd", type=float, default=0.02)
    parser.add_argument("--write-flash", action="store_true",
                        help="谨慎使用：把参数永久写入 MCU Flash")
    args = parser.parse_args()

    bot = Rosmaster(com=args.port)
    bot.create_receive_threading()
    time.sleep(0.8)

    steps = apply_m1_runtime_sanitize(
        bot,
        pid_kp=args.pid_kp,
        pid_ki=args.pid_ki,
        pid_kd=args.pid_kd,
        write_flash=args.write_flash,
        log=print,
    )
    print(json.dumps(steps, ensure_ascii=False, indent=2))

    print("\n===== idle check =====")
    for i in range(10):
        print(i, bot.get_motion_data())
        time.sleep(0.1)

    bot.set_car_motion(0.0, 0.0, 0.0)


if __name__ == "__main__":
    main()
