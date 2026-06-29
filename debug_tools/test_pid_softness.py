#!/usr/bin/env python3
import argparse
import time
from Rosmaster_Lib import Rosmaster

def stop(bot):
    for _ in range(8):
        bot.set_car_motion(0, 0, 0)
        time.sleep(0.05)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--kp", type=float, default=1.0)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument("--vx", type=float, default=0.03)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()

    bot = Rosmaster(com=args.port)
    bot.create_receive_threading()
    time.sleep(0.3)

    print("old pid:", bot.get_motion_pid())
    bot.set_pid_param(args.kp, args.ki, args.kd, forever=False)
    time.sleep(0.2)
    print("new pid:", bot.get_motion_pid())

    start = time.time()
    while time.time() - start < args.duration:
        bot.set_car_motion(args.vx, 0.0, args.wz)
        vx, vy, vz = bot.get_motion_data()
        print(f"cmd vx={args.vx:.3f}, wz={args.wz:.3f} | fb vx={vx:.3f}, vy={vy:.3f}, wz={vz:.3f}")
        time.sleep(0.1)

    stop(bot)

if __name__ == "__main__":
    main()