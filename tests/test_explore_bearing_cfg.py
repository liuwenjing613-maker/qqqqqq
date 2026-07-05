#!/usr/bin/env python3
import unittest

from src.apps.run_shared_nav_semantic_explore import explore_bearing_cfg


class TestExploreBearingCfg(unittest.TestCase):
    def test_reads_safe_goal_projection_bearing_first(self):
        cfg = {
            "safe_goal_projection": {
                "bearing_first": {
                    "align_max_sec": 10.0,
                    "turn_in_place_threshold_rad": 0.18,
                }
            },
            "planner": {"bearing_first": {"align_max_sec": 2.5}},
        }
        bearing = explore_bearing_cfg(cfg)
        self.assertEqual(bearing["align_max_sec"], 10.0)
        self.assertEqual(bearing["turn_in_place_threshold_rad"], 0.18)

    def test_falls_back_to_planner_bearing_first(self):
        cfg = {
            "planner": {"bearing_first": {"max_vx": 0.05}},
        }
        bearing = explore_bearing_cfg(cfg)
        self.assertEqual(bearing["max_vx"], 0.05)


if __name__ == "__main__":
    unittest.main()
