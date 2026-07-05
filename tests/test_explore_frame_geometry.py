#!/usr/bin/env python3
import math
import unittest

from src.nav.frame_transform_2d import Transform2D


def _bearing(robot_xy, robot_yaw, goal_xy):
    dx = goal_xy[0] - robot_xy[0]
    dy = goal_xy[1] - robot_xy[1]
    bearing = math.atan2(dy, dx) - robot_yaw
    return (bearing + math.pi) % (2 * math.pi) - math.pi


class TestExploreFrameGeometry(unittest.TestCase):
    def test_map_goal_bearing_after_shift_to_odom(self):
        map_to_odom = Transform2D(1.0, 0.0, 2.0, 1.0)
        goal_odom = map_to_odom.apply(1.0, 0.0)
        self.assertAlmostEqual(goal_odom[0], 3.0, places=6)
        self.assertAlmostEqual(goal_odom[1], 1.0, places=6)

        robot_xy = (0.0, 0.0)
        robot_yaw = 0.0
        bearing = _bearing(robot_xy, robot_yaw, goal_odom)
        self.assertAlmostEqual(bearing, math.atan2(1.0, 3.0), places=6)

    def test_map_frame_bearing_matches_selector(self):
        robot_map = (0.1, 0.05, 0.2)
        goal_map = (0.02, -0.43)
        bearing = _bearing(robot_map[:2], robot_map[2], goal_map)
        self.assertLess(bearing, 0.0)
        self.assertGreater(bearing, -2.1)

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
