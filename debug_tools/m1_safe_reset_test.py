#!/usr/bin/env python3
import sys
import time

from Rosmaster_Lib import Rosmaster

PROJECT_ROOT = __import__("os").path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.control.m1_rosmaster_extensions import apply_m1_runtime_sanitize

PORT = "/dev/ttyUSB0"

bot = Rosmaster(com=PORT)
bot.create_receive_threading()
time.sleep(0.8)

apply_m1_runtime_sanitize(bot, log=print)

print("\n=== idle after reset ===")
for i in range(20):
    print(i, bot.get_motion_data())
    time.sleep(0.1)

# 安全短脉冲测试，轮子必须悬空
for vx in [0.01, 0.02, 0.03, 0.04]:
    print(f"\n=== short pulse vx={vx} ===")
    bot.set_car_motion(vx, 0.0, 0.0)
    for i in range(6):
        print(i, bot.get_motion_data())
        time.sleep(2.0)
    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(1.0)

bot.set_car_motion(0.0, 0.0, 0.0)
print("DONE")
