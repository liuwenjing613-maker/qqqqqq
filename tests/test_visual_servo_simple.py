#!/usr/bin/env python3
"""简单视觉伺服：ex 符号与转向输出。"""

import os
import sys
import unittest

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.control.mvp_visual_servo import MVPVisualServo


class TestVisualServoSimple(unittest.TestCase):
    def _cmd(self, cx, area_ratio=0.05):
        servo = MVPVisualServo(
            image_width=1280,
            max_vx=0.04,
            kp_turn=0.175,
            turn_threshold=0.25,
            max_wz=0.08,
            wz_deadzone=0.08,
            cmd_wz_deadzone=0.01,
            forward_turn_scale=0.25,
        )
        return servo.compute_cmd({"visible": True, "cx": cx, "area_ratio": area_ratio})

    def test_mid_offset_below_cmd_deadzone_goes_straight(self):
        # cx=434 → ex≈-0.161，之前会给 cmd_wz≈0.004 并触发底盘 kick
        state, cmd = self._cmd(434)
        self.assertEqual(state, "FORWARD")
        self.assertGreater(cmd.linear.x, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)

    def test_larger_mid_offset_steers_slowly(self):
        # cx=340 → ex≈-0.234，仍小于 turn_threshold，但已超过 cmd_wz_deadzone
        state, cmd = self._cmd(340)
        ex = (340 - 640) / 1280.0
        self.assertLess(ex, 0.0)
        self.assertEqual(state, "FORWARD")
        self.assertGreater(cmd.linear.x, 0.0)
        self.assertGreater(cmd.angular.z, 0.0)
        self.assertLess(cmd.angular.z, 0.015)

    def test_large_offset_turns_in_place_slowly(self):
        # cx=1106 → ex≈+0.36，日志中的场景
        state, cmd = self._cmd(1106)
        self.assertEqual(state, "TURN_ONLY")
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertLess(cmd.angular.z, -0.05)
        self.assertGreater(cmd.angular.z, -0.08)

    def test_right_target_positive_ex_turns_right(self):
        state, cmd = self._cmd(950)
        ex = (950 - 640) / 1280.0
        self.assertGreater(ex, 0.0)
        self.assertEqual(state, "FORWARD")
        self.assertGreater(cmd.linear.x, 0.0)
        self.assertLess(cmd.angular.z, 0.0)
        self.assertGreater(cmd.angular.z, -0.015)

    def test_slight_left_still_steers_while_forward(self):
        # cx=580 → ex=-0.046875，小于 deadzone，应前进且不转
        state, cmd = self._cmd(580)
        self.assertEqual(state, "FORWARD")
        self.assertGreater(cmd.linear.x, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)


if __name__ == "__main__":
    unittest.main()
