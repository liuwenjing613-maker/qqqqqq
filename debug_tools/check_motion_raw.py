import time
from Rosmaster_Lib import Rosmaster

PORT = "/dev/ttyUSB0"   # 如果你的底盘是 /dev/ttyUSB1，就改这里

bot = Rosmaster(com=PORT)
bot.create_receive_threading()
time.sleep(1.0)

print("version:", bot.get_version() if hasattr(bot, "get_version") else "no get_version")

if hasattr(bot, "get_car_type_from_machine"):
    try:
        print("car_type_from_machine:", bot.get_car_type_from_machine())
    except Exception as e:
        print("get_car_type_from_machine error:", repr(e))

if hasattr(bot, "get_motion_pid"):
    try:
        print("motion_pid:", bot.get_motion_pid())
    except Exception as e:
        print("get_motion_pid error:", repr(e))

print("\n=== idle motion data ===")
for i in range(20):
    print(i, bot.get_motion_data())
    time.sleep(0.2)

print("\n=== cmd vx=0.04 ===")
bot.set_car_motion(0.04, 0.0, 0.0)
for i in range(30):
    print(i, bot.get_motion_data())
    time.sleep(0.2)

bot.set_car_motion(0.0, 0.0, 0.0)
time.sleep(0.5)

print("\n=== stopped motion data ===")
for i in range(20):
    print(i, bot.get_motion_data())
    time.sleep(0.2)

bot.set_car_motion(0.0, 0.0, 0.0)
