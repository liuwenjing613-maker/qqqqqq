#!/usr/bin/env python3
import time
import argparse
from src.control.chassis_controller import ChassisController


def hold_cmd(car, vx, vy, wz, duration=1.2, dt=0.1):
    """
    持续发送速度命令，避免 watchdog 0.5 秒自动停车。
    """
    print(f"CMD HOLD: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}, duration={duration:.1f}s")
    start = time.time()
    while time.time() - start < duration:
        car.set_velocity(vx, vy, wz)
        time.sleep(dt)
    car.stop()
    time.sleep(0.8)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    args = parser.parse_args()

    car = ChassisController(
        port=args.port,
        max_vx=0.15,
        max_vy=0.10,
        max_wz=0.60,
        watchdog_timeout=0.5,
    )

    print("=== ChassisController speed sweep test ===")
    print("这个测试会持续发送速度命令，避免 watchdog 误判停车。")
    print("建议先架空测试，再落地测试。")
    input("确认安全后按 Enter 开始：")

    try:
        print("status:", car.get_status())
        car.stop()

        # 前进速度阶梯
        for vx in [0.04, 0.06, 0.08, 0.10, 0.12]:
            print("\n==============================")
            print(f"forward vx={vx}")
            print("==============================")
            input("按 Enter 执行：")
            hold_cmd(car, vx, 0.0, 0.0, duration=1.2)

        # 后退速度阶梯
        for vx in [-0.06, -0.08, -0.10]:
            print("\n==============================")
            print(f"backward vx={vx}")
            print("==============================")
            input("按 Enter 执行：")
            hold_cmd(car, vx, 0.0, 0.0, duration=1.2)

        # 旋转测试
        for wz in [0.30, 0.40, 0.50]:
            print("\n==============================")
            print(f"rotate left wz={wz}")
            print("==============================")
            input("按 Enter 执行：")
            hold_cmd(car, 0.0, 0.0, wz, duration=1.2)

        for wz in [-0.30, -0.40, -0.50]:
            print("\n==============================")
            print(f"rotate right wz={wz}")
            print("==============================")
            input("按 Enter 执行：")
            hold_cmd(car, 0.0, 0.0, wz, duration=1.2)

        print("\n测试完成。")

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        car.close()
        print("Final stop sent.")


if __name__ == "__main__":
    main()
