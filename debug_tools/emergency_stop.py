#!/usr/bin/env python3
import argparse
import time
from Rosmaster_Lib import Rosmaster


def main():
    parser = argparse.ArgumentParser(description="Emergency stop for Yahboom Rosmaster M1.")
    parser.add_argument("--port", default="/dev/myserial", help="Serial port, e.g. /dev/myserial or /dev/ttyUSB0")
    parser.add_argument("--repeat", type=int, default=8, help="How many stop commands to send")
    parser.add_argument("--interval", type=float, default=0.08, help="Interval between stop commands")
    args = parser.parse_args()

    bot = Rosmaster(com=args.port)
    bot.create_receive_threading()
    time.sleep(0.2)

    for _ in range(args.repeat):
        bot.set_car_motion(0.0, 0.0, 0.0)
        time.sleep(args.interval)

    print(f"[OK] Emergency stop sent on {args.port}")


if __name__ == "__main__":
    main()
