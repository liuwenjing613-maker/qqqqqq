#!/usr/bin/env python3
import argparse
from Rosmaster_Lib import Rosmaster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    args = parser.parse_args()

    bot = Rosmaster(com=args.port)
    bot.set_car_motion(0.0, 0.0, 0.0)
    print("STOP command sent.")


if __name__ == "__main__":
    main()
