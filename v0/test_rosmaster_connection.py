#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster


def try_read(name, func):
    try:
        value = func()
        print(f"[OK] {name}: {value}")
        return value
    except Exception as e:
        print(f"[ERR] {name}: {repr(e)}")
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial", help="serial port, e.g. /dev/myserial or /dev/ttyUSB0")
    args = parser.parse_args()

    print("=== Rosmaster connection test ===")
    print(f"Using port: {args.port}")

    bot = Rosmaster(com=args.port)

    print("Starting receive thread...")
    bot.create_receive_threading()
    time.sleep(0.5)

    try_read("get_version", bot.get_version)
    try_read("get_battery_voltage", bot.get_battery_voltage)
    try_read("get_motion_data", bot.get_motion_data)
    try_read("get_accelerometer_data", bot.get_accelerometer_data)
    try_read("get_gyroscope_data", bot.get_gyroscope_data)

    print("Sending stop command...")
    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(0.2)

    print("Done.")


if __name__ == "__main__":
    main()
