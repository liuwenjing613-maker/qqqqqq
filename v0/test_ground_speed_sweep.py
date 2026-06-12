#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class GroundSpeedSweep:
    def __init__(self, port="/dev/myserial"):
        self.bot = Rosmaster(car_type=1, com=port)
        self.bot.set_car_type(1)
        self.bot.create_receive_threading()
        time.sleep(0.5)

        # 第一轮落地测试上限，不要太高
        self.max_vx = 0.15
        self.max_vy = 0.10
        self.max_wz = 0.60

    def set_motion(self, vx, vy, wz):
        vx = clamp(vx, -self.max_vx, self.max_vx)
        vy = clamp(vy, -self.max_vy, self.max_vy)
        wz = clamp(wz, -self.max_wz, self.max_wz)

        print(f"CMD vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
        self.bot.set_car_motion(vx, vy, wz)

    def stop(self):
        print("CMD STOP")
        self.bot.set_car_motion(0.0, 0.0, 0.0)

    def status(self):
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

    def run_once(self, name, vx, vy, wz, duration=1.0):
        print("\n==============================")
        print(name)
        print("==============================")
        input("确认周围安全，按 Enter 执行这一档速度：")

        self.set_motion(vx, vy, wz)
        time.sleep(duration)
        self.stop()
        time.sleep(0.8)

        print("观察结果：")
        print("1. 完全不动")
        print("2. 轻微抖动但没走")
        print("3. 能慢慢走")
        print("4. 速度太快")
        self.status()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    args = parser.parse_args()

    car = GroundSpeedSweep(port=args.port)

    print("=== M1 ground speed sweep test ===")
    print("用途：找到小车落地后最小可动速度。")
    print("注意：场地必须空旷，手靠近电源开关。")
    print("如果异常，立刻 Ctrl+C 或关闭小车电源。")
    input("确认小车已落地、周围安全后按 Enter：")

    try:
        car.status()
        car.stop()

        # 前进速度阶梯
        forward_speeds = [0.06, 0.08, 0.10, 0.12, 0.15]
        for vx in forward_speeds:
            car.run_once(f"forward vx={vx}", vx, 0.0, 0.0, duration=1.0)

        # 后退速度阶梯
        backward_speeds = [-0.06, -0.08, -0.10, -0.12]
        for vx in backward_speeds:
            car.run_once(f"backward vx={vx}", vx, 0.0, 0.0, duration=1.0)

        # 原地旋转速度阶梯
        rotate_speeds = [0.25, 0.35, 0.45, 0.60]
        for wz in rotate_speeds:
            car.run_once(f"rotate left wz={wz}", 0.0, 0.0, wz, duration=1.0)

        for wz in [-0.25, -0.35, -0.45, -0.60]:
            car.run_once(f"rotate right wz={wz}", 0.0, 0.0, wz, duration=1.0)

        print("\n测试完成。请根据观察结果记录最小可动速度。")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        car.stop()
        time.sleep(0.2)
        print("Final stop sent.")


if __name__ == "__main__":
    main()
