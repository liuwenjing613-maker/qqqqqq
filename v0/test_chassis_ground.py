#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class GroundTestCar:
    def __init__(self, port="/dev/myserial"):
        self.bot = Rosmaster(com=port)
        self.bot.create_receive_threading()
        time.sleep(0.5)

        # 落地仍然限速
        self.max_vx = 0.15
        self.max_vy = 0.10
        self.max_wz = 0.60

    def move(self, vx, vy, wz, duration):
        vx = clamp(vx, -self.max_vx, self.max_vx)
        vy = clamp(vy, -self.max_vy, self.max_vy)
        wz = clamp(wz, -self.max_wz, self.max_wz)

        print(f"move: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}, t={duration:.1f}s")
        self.bot.set_car_motion(vx, vy, wz)
        time.sleep(duration)
        self.stop()
        time.sleep(0.5)

    def stop(self):
        print("stop")
        self.bot.set_car_motion(0.0, 0.0, 0.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    args = parser.parse_args()

    car = GroundTestCar(port=args.port)

    print("=== M1 ground low speed test ===")
    print("请确认小车在空旷地面，前方无遮挡。")
    print("每次只运动很短时间。异常立刻 Ctrl+C 或关电源。")
    input("确认后按 Enter 开始：")

    try:
        car.move(0.05, 0.0, 0.0, 1.2)    # 前进
        car.move(-0.05, 0.0, 0.0, 1.2)   # 后退
        car.move(0.0, 0.0, 0.22, 2.0)    # 左旋
        car.move(0.0, 0.0, -0.22, 2.0)   # 右旋

        # 横移对地面要求高，最后再测
        input("准备测试横移，确认左右无遮挡后按 Enter：")
        car.move(0.0, 0.12, 0.0, 1.8)    # 左平移
        car.move(0.0, -0.12, 0.0, 1.8)   # 右平移

        print("落地低速测试完成。")

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        car.stop()


if __name__ == "__main__":
    main()
