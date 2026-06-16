#!/usr/bin/env python3
"""停车及时性：平滑器零速立即刹停。"""

import os
import sys
import unittest

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.control.cmd_smoother import CmdSmoother
from src.control.mvp_visual_servo import MVPVisualServo


class TestCmdSmootherFastStop(unittest.TestCase):
    def test_zero_target_stops_immediately(self):
        s = CmdSmoother(alpha=0.8, max_vx_delta=0.01, max_wz_delta=0.01)
        s.update(0.04, 0.0)
        self.assertGreater(s.vx, 0.0)
        vx, wz = s.update(0.0, 0.0)
        self.assertEqual(vx, 0.0)
        self.assertEqual(wz, 0.0)


class TestVisualServoArrive(unittest.TestCase):
    def test_stop_at_arrive_threshold(self):
        servo = MVPVisualServo(
            image_width=1280,
            max_vx=0.04,
            arrive_area_ratio=0.14,
        )
        state, cmd = servo.compute_cmd({"visible": True, "cx": 640, "area_ratio": 0.15})
        self.assertEqual(state, "ARRIVED_STOP")
        self.assertEqual(cmd.linear.x, 0.0)


if __name__ == "__main__":
    unittest.main()
