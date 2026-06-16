#!/usr/bin/env python3
"""
落地底盘单项测试：前进 / 左转 / 右转。
直连 Rosmaster，不经过 ROS / MVP，用于排查硬件与最小可动速度。

用法:
  python3 debug_tools/test_chassis_ground_basic.py --action all
  python3 debug_tools/test_chassis_ground_basic.py --action forward --vx 0.04 --duration 1.5
  python3 debug_tools/test_chassis_ground_basic.py --action left --wz 0.10
"""

import argparse
import os
import sys
import time

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from Rosmaster_Lib import Rosmaster
from src.config.mvp_tune import DEFAULT_TUNE_PATH, load_mvp_tune


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class GroundBasicTest:
    def __init__(self, port, max_vx, max_wz, kick_wz=0.24, kick_duration=0.12):
        self.bot = Rosmaster(com=port)
        self.bot.create_receive_threading()
        time.sleep(0.5)
        self.max_vx = float(max_vx)
        self.max_wz = float(max_wz)
        self.kick_wz = float(kick_wz)
        self.kick_duration = float(kick_duration)

    def status(self):
        try:
            battery = self.bot.get_battery_voltage()
        except Exception:
            battery = None
        print(f"battery={battery}")
        if battery is not None and battery < 1.0:
            print("[WARN] battery 接近 0，可能串口不对，请检查 --port")

    def move_rotate_kick(self, cruise_wz, duration):
        """先 kick 突破静摩擦，再以巡航 wz 转动。"""
        cruise_wz = clamp(cruise_wz, -self.max_wz, self.max_wz)
        sign = 1.0 if cruise_wz > 0 else -1.0
        kick = sign * min(abs(self.kick_wz), self.max_wz)
        cruise = duration - self.kick_duration
        if cruise < 0.2:
            cruise = 0.2
        print(
            f">>> rotate kick wz={kick:+.3f} for {self.kick_duration:.2f}s, "
            f"then cruise wz={cruise_wz:+.3f} for {cruise:.1f}s"
        )
        self.bot.set_car_motion(0.0, 0.0, kick)
        time.sleep(self.kick_duration)
        self.bot.set_car_motion(0.0, 0.0, cruise_wz)
        time.sleep(cruise)
        self.stop()
        time.sleep(0.5)

    def move(self, vx, wz, duration, use_rotate_kick=False):
        if use_rotate_kick and abs(vx) < 1e-4 and abs(wz) > 1e-4:
            self.move_rotate_kick(wz, duration)
            return
        vx = clamp(vx, -self.max_vx, self.max_vx)
        wz = clamp(wz, -self.max_wz, self.max_wz)
        print(f">>> move vx={vx:+.3f} wz={wz:+.3f} duration={duration:.1f}s")
        self.bot.set_car_motion(vx, 0.0, wz)
        time.sleep(duration)
        self.stop()
        time.sleep(0.5)

    def stop(self):
        print(">>> stop")
        self.bot.set_car_motion(0.0, 0.0, 0.0)


ACTIONS = {
    "forward": (0.04, 0.0, 1.2, "前进", False),
    "left": (0.0, 0.16, 2.0, "原地左转(kick→巡航)", True),
    "right": (0.0, -0.16, 2.0, "原地右转(kick→巡航)", True),
}

ROTATE_SWEEP_WZ = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50]


def main():
    tune = load_mvp_tune(DEFAULT_TUNE_PATH)
    parser = argparse.ArgumentParser(description="落地底盘前进/左转/右转单项测试")
    parser.add_argument("--mvp-tune-config", default=DEFAULT_TUNE_PATH)
    parser.add_argument("--port", default=tune["chassis_port"])
    parser.add_argument("--max-vx", type=float, default=tune["chassis_max_vx"])
    parser.add_argument("--max-wz", type=float, default=tune["chassis_max_wz"])
    parser.add_argument(
        "--action",
        choices=["forward", "left", "right", "all"],
        default="all",
        help="测试动作；all=依次前进/左转/右转",
    )
    parser.add_argument("--vx", type=float, default=None, help="覆盖前进速度")
    parser.add_argument("--wz", type=float, default=None, help="覆盖转向角速度")
    parser.add_argument("--duration", type=float, default=None, help="覆盖持续时间(秒)")
    parser.add_argument("--yes", action="store_true", help="跳过交互确认")
    parser.add_argument(
        "--sweep-rotate",
        action="store_true",
        help="扫描多档 wz，找落地最小可转速度",
    )
    args = parser.parse_args()

    if args.mvp_tune_config != DEFAULT_TUNE_PATH:
        tune = load_mvp_tune(args.mvp_tune_config)
        if args.port == tune["chassis_port"] or args.port == load_mvp_tune()["chassis_port"]:
            args.port = tune["chassis_port"]
        args.max_vx = tune["chassis_max_vx"]
        args.max_wz = tune["chassis_max_wz"]

    print("=== 落地底盘基础测试（直连 M1）===")
    print(f"port={args.port} max_vx={args.max_vx} max_wz={args.max_wz}")
    print("请确认：小车在地面、周围空旷，手能随时断电。")
    if not args.yes:
        input("按 Enter 开始：")

    car = GroundBasicTest(
        args.port,
        args.max_vx,
        args.max_wz,
        kick_wz=tune.get("kick_wz", 0.24),
        kick_duration=tune.get("kick_duration", 0.12),
    )
    car.status()
    car.stop()

    if args.sweep_rotate:
        print("\n=== 转向速度扫描（先左转，每档按 Enter）===")
        try:
            for wz in ROTATE_SWEEP_WZ:
                if not args.yes:
                    input(f"准备左转 wz={wz:+.2f}，按 Enter：")
                car.move(0.0, wz, 1.2)
            print("\n扫描完成。记录从哪一档开始能明显转动。")
        except KeyboardInterrupt:
            print("\n中断。")
        finally:
            car.stop()
        return

    actions = ["forward", "left", "right"] if args.action == "all" else [args.action]

    try:
        for name in actions:
            vx, wz, duration, label, use_kick = ACTIONS[name]
            if args.vx is not None:
                vx = args.vx
            if args.wz is not None:
                wz = args.wz
            if args.duration is not None:
                duration = args.duration

            print(f"\n--- {label} ---")
            if args.action == "all" and not args.yes:
                input(f"准备 {label}，按 Enter 执行：")
            car.move(vx, wz, duration, use_rotate_kick=use_kick)

        print("\n测试完成。")
    except KeyboardInterrupt:
        print("\n中断。")
    finally:
        car.stop()


if __name__ == "__main__":
    main()
