#!/usr/bin/env python3
import os, sys
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)
from src.control.qwen_lidar_point_servo import QwenLidarPointServo

servo = QwenLidarPointServo(image_width=1280, require_lidar=True)
cases = [
    ({"visible": True, "u": 640}, 1.2, 1.2),
    ({"visible": True, "u": 300}, 1.2, 1.2),
    ({"visible": True, "u": 980}, 1.2, 1.2),
    ({"visible": True, "u": 640}, 0.45, 0.45),
    ({"visible": True, "u": 640}, 0.25, 0.25),
    ({"visible": True, "u": 640}, None, None),
    ({"visible": False}, 1.2, None),
]
for target, front, td in cases:
    res = servo.compute_cmd(target, front, td)
    print(target, "front", front, "target_d", td, "=>", res.state, res.cmd.linear.x, res.cmd.angular.z, res.reason)
