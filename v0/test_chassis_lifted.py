#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class SafeRosmaster:
    def __init__(self, port="/dev/myserial"):
        self.bot = Rosmaster(car_type=1, com=port)
        self.bot.set_car_type(1)
        self.bot.create_receive_threading()
        time.sleep(0.5)

        # 第一阶段强制限速
        self.max_vx = 0.05
        self.max_vy = 0.05
        self.max_wz = 0.25

    def set_velocity(self, vx, vy, wz):
        vx = clamp(vx, -self.max_vx, self.max_vx)
        vy = clamp(vy, -self.max_vy, self.max_vy)
        wz = clamp(wz, -self.max_wz, self.max_wz)
        print(f"CMD: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
        self.bot.set_car_motion(vx, vy, wz)

    def stop(self):
        print("CMD: STOP")
        self.bot.set_car_motion(0.0, 0.0, 0.0)

    def read_status(self):
        try:
            print("version:", self.bot.get_version())
        except Exception as e:
            print("version read failed:", e)

        try:
            print("battery:", self.bot.get_battery_voltage())
        except Exception as e:
            print("battery read failed:", e)

        try:
            print("motion:", self.bot.get_motion_data())
        except Exception as e:
            print("motion read failed:", e)


def run_step(car, name, vx, vy, wz, duration=1.0):
    print("\n==============================")
    print(name)
    print("==============================")
    car.set_velocity(vx, vy, wz)
    time.sleep(duration)
    car.stop()
    time.sleep(0.8)
    car.read_status()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    args = parser.parse_args()

    car = SafeRosmaster(port=args.port)

    print("=== M1 lifted chassis test ===")
    print("必须确认：四个轮子已经架空，不接触地面。")
    print("按 Enter 后开始，每个动作只持续 1 秒。")
    print("如果异常，请立即 Ctrl+C 或关闭小车电源。")
    input("确认轮子架空后按 Enter：")

    try:
        car.read_status()

        run_step(car, "1. forward: vx > 0，预期四轮转动，小车前进方向", 0.05, 0.0, 0.0)
        run_step(car, "2. backward: vx < 0，预期四轮反向，小车后退方向", -0.05, 0.0, 0.0)
        run_step(car, "3. left shift: vy > 0，预期麦轮横向左移方向", 0.0, 0.05, 0.0)
        run_step(car, "4. right shift: vy < 0，预期麦轮横向右移方向", 0.0, -0.05, 0.0)
        run_step(car, "5. rotate left: wz > 0，预期原地左旋", 0.0, 0.0, 0.25)
        run_step(car, "6. rotate right: wz < 0，预期原地右旋", 0.0, 0.0, -0.25)

        print("\n全部测试完成，已停车。")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt, stopping...")
    finally:
        car.stop()
        time.sleep(0.2)


if __name__ == "__main__":
    main()
