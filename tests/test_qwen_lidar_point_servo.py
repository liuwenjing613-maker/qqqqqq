#!/usr/bin/env python3
import os
import sys
# 基于当前脚本的绝对路径，定位到上一层目录（src 所在的项目根目录）
current_file_path = os.path.abspath(__file__)    # 当前脚本的绝对路径
current_dir = os.path.dirname(current_file_path) # 当前脚本所在文件夹
parent_dir = os.path.dirname(current_dir)        # 上一层目录（src 所在目录）

# 将父目录加入 Python 模块搜索路径
sys.path.append(parent_dir)
from src.control.qwen_lidar_point_servo import QwenLidarPointServo

servo = QwenLidarPointServo(image_width=1280)

cases = [
    ({"visible": True, "u": 640}, 1.2, 1.2),
    ({"visible": True, "u": 300}, 1.2, 1.2),
    ({"visible": True, "u": 980}, 1.2, 1.2),
    ({"visible": True, "u": 640}, 0.45, 0.45),
    ({"visible": True, "u": 640}, 0.25, 0.25),
    ({"visible": False}, 1.2, None),
]

for target, front, td in cases:
    res = servo.compute_cmd(target, front, td)
    print(target, "front", front, "target_d", td, "=>", res.state, res.cmd.linear.x, res.cmd.angular.z, res.reason)