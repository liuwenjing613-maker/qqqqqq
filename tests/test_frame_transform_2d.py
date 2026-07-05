#!/usr/bin/env python3
import math
import unittest

from src.nav.frame_transform_2d import Transform2D, hint_goal_frame, transform_points


class TestTransform2D(unittest.TestCase):
    def test_identity(self):
        t = Transform2D.identity()
        self.assertEqual(t.apply(1.2, -0.4), (1.2, -0.4))

    def test_translate(self):
        t = Transform2D(1.0, 0.0, 2.0, 3.0)
        self.assertEqual(t.apply(1.0, 1.0), (3.0, 4.0))

    def test_rotate_90_deg(self):
        yaw = math.pi / 2.0
        t = Transform2D(math.cos(yaw), math.sin(yaw), 0.0, 0.0)
        x, y = t.apply(1.0, 0.0)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 1.0, places=6)

    def test_transform_path(self):
        yaw = math.pi
        t = Transform2D(math.cos(yaw), math.sin(yaw), 1.0, 0.0)
        out = transform_points([(0.0, 0.0), (1.0, 0.0)], t)
        self.assertAlmostEqual(out[0][0], 1.0, places=6)
        self.assertAlmostEqual(out[0][1], 0.0, places=6)
        self.assertAlmostEqual(out[1][0], 0.0, places=6)
        self.assertAlmostEqual(out[1][1], 0.0, places=6)

    def test_hint_goal_frame_default(self):
        self.assertEqual(hint_goal_frame({}, "map"), "map")
        self.assertEqual(hint_goal_frame({"goal_frame": "odom"}, "map"), "odom")


if __name__ == "__main__":
    unittest.main()
