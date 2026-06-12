#!/usr/bin/env python3
import time
import argparse
from src.control.chassis_controller import ChassisController


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    args = parser.parse_args()

    car = ChassisController(
        port=args.port,
        max_vx=0.08,
        max_vy=0.04,
        max_wz=0.30,
        watchdog_timeout=0.5,
    )

    print("status:", car.get_status())

    print("请确认轮子架空。")
    input("按 Enter 开始：")

    try:
        print("forward")
        car.set_velocity(0.06, 0.0, 0.0)
        time.sleep(2.0)
        car.stop()
        time.sleep(1.2)

        print("rotate left")
        car.set_velocity(0.0, 0.0, 0.30)
        time.sleep(2.0)
        car.stop()
        time.sleep(1.2)

        print("watchdog test: send forward once, then sleep 2s")
        car.set_velocity(0.06, 0.0, 0.0)
        time.sleep(1.2)
        print("如果 watchdog 正常，小车应该早已自动停止。")

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        car.close()
        print("closed.")


if __name__ == "__main__":
    main()
