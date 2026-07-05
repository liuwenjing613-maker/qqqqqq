#!/usr/bin/env python3
import math
import unittest

from src.nav.frame_transform_2d import Transform2D


class TestExploreFrameGeometry(unittest.TestCase):
    def test_map_goal_bearing_after_shift_to_odom(self):
        map_to_odom = Transform2D(1.0, 0.0, 2.0, 1.0)
        goal_odom = map_to_odom.apply(1.0, 0.0)
        self.assertAlmostEqual(goal_odom[0], 3.0, places=6)
        self.assertAlmostEqual(goal_odom[1], 1.0, places=6)

        robot_xy = (0.0, 0.0)
        robot_yaw = 0.0
        bearing = math.atan2(goal_odom[1] - robot_xy[1], goal_odom[0] - robot_xy[0]) - robot_yaw
        self.assertAlmostEqual(bearing, math.atan2(1.0, 3.0), places=6)

    def test_map_path_first_point_matches_goal_transform(self):
        map_to_odom = Transform2D(
            math.cos(0.5),
            math.sin(0.5),
            -1.0,
            0.5,
        )
        path_map = [(0.0, 0.0), (1.0, 0.0)]
        path_odom = map_to_odom.apply_path(path_map)
        goal_odom = map_to_odom.apply(1.0, 0.0)
        self.assertAlmostEqual(path_odom[-1][0], goal_odom[0], places=6)
        self.assertAlmostEqual(path_odom[-1][1], goal_odom[1], places=6)


if __name__ == "__main__":
    unittest.main()
