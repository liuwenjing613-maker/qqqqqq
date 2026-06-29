#!/usr/bin/env python3
import json
import statistics
import sys
import time

from Rosmaster_Lib import Rosmaster

PROJECT_ROOT = __import__("os").path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)
from src.control.m1_rosmaster_extensions import apply_m1_runtime_sanitize

PORT = "/dev/ttyUSB0"


def sample_motion(bot, duration=4.0, rate_hz=20.0):
    dt = 1.0 / rate_hz
    vals = []
    end = time.time() + duration
    while time.time() < end:
        vals.append(bot.get_motion_data())
        time.sleep(dt)
    vx = [v[0] for v in vals]
    return {
        "count": len(vx),
        "mean_vx": statistics.mean(vx),
        "mean_abs_vx": statistics.mean(abs(x) for x in vx),
        "median_abs_vx": statistics.median(abs(x) for x in vx),
        "max_abs_vx": max(abs(x) for x in vx),
        "samples": vals[:5],
    }


def main():
    bot = Rosmaster(com=PORT)
    bot.create_receive_threading()
    time.sleep(0.8)

    sanitize = apply_m1_runtime_sanitize(bot, log=print)
    print("\n=== sanitize summary ===")
    print(json.dumps({k: v.get("ok") for k, v in sanitize.items() if isinstance(v, dict)}, indent=2))

    results = []
    for vx_cmd in [0.01, 0.02, 0.03, 0.04]:
        bot.set_car_motion(0.0, 0.0, 0.0)
        time.sleep(0.5)
        print(f"\n=== hold vx={vx_cmd} ===")
        bot.set_car_motion(vx_cmd, 0.0, 0.0)
        time.sleep(1.0)
        stats = sample_motion(bot, duration=4.0)
        stats["cmd_vx"] = vx_cmd
        results.append(stats)
        print(json.dumps(stats, indent=2))
        bot.set_car_motion(0.0, 0.0, 0.0)
        time.sleep(0.8)

    print("\n=== summary table ===")
    for r in results:
        print(
            f"cmd={r['cmd_vx']:.3f}  feedback_mean_abs={r['mean_abs_vx']:.4f}  "
            f"median_abs={r['median_abs_vx']:.4f}  max_abs={r['max_abs_vx']:.4f}"
        )

    bot.set_car_motion(0.0, 0.0, 0.0)


if __name__ == "__main__":
    main()
