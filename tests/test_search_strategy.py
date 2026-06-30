#!/usr/bin/env python3
import unittest

from src.nav.search_strategy import (
    compute_loss_age,
    pick_clearance_turn_dir,
    should_use_free_space,
    turn_dir_from_ex,
)


class TestSearchStrategy(unittest.TestCase):
    def test_turn_dir_from_ex(self):
        self.assertEqual(turn_dir_from_ex(0.2), -1.0)
        self.assertEqual(turn_dir_from_ex(-0.2), 1.0)
        self.assertEqual(turn_dir_from_ex(0.0), 1.0)

    def test_loss_age_never_seen(self):
        self.assertEqual(compute_loss_age(10.0, None), float("inf"))

    def test_loss_age_recent(self):
        self.assertAlmostEqual(compute_loss_age(10.0, 8.0), 2.0)

    def test_should_use_free_space(self):
        self.assertFalse(should_use_free_space(5.0, True, 8.0))
        self.assertTrue(should_use_free_space(9.0, True, 8.0))
        self.assertFalse(should_use_free_space(9.0, False, 8.0))

    def test_pick_clearance_prefers_open_side(self):
        d, side = pick_clearance_turn_dir(2.0, 0.5, 0.15, 1.0)
        self.assertEqual(d, 1.0)
        self.assertEqual(side, "left")

    def test_pick_clearance_holds_when_similar(self):
        d, side = pick_clearance_turn_dir(1.0, 0.95, 0.15, -1.0)
        self.assertEqual(d, -1.0)
        self.assertEqual(side, "hold")


if __name__ == "__main__":
    unittest.main()
