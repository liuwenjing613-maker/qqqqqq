from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from control.qwen_visual_servo import (  # noqa: E402
    CommandRateLimiter,
    QwenVisualServo,
    RateLimitConfig,
    ServoConfig,
    ServoInput,
)


class ServoTests(unittest.TestCase):
    def setUp(self) -> None:
        self.servo = QwenVisualServo(ServoConfig())

    def input(self, **kwargs) -> ServoInput:
        values = dict(
            now_sec=10.0,
            state="TARGET_LOCKED",
            result="TARGET_VISIBLE",
            point_role="target",
            point_x=480.0,
            image_width=960,
            confidence=0.9,
            latency_ms=800.0,
            result_received_sec=9.9,
            point_streak=1,
            front_distance=1.0,
            scan_received_sec=9.9,
        )
        values.update(kwargs)
        return ServoInput(**values)

    def test_centered_target_point_moves_forward(self) -> None:
        decision = self.servo.compute(self.input())
        self.assertGreater(decision.vx, 0.0)
        self.assertAlmostEqual(decision.wz, 0.0, places=4)

    def test_search_hint_in_inferred_state_moves(self) -> None:
        decision = self.servo.compute(
            self.input(
                state="TARGET_INFERRED",
                result="SEARCH_HINT",
                point_role="search",
                point_x=480.0,
                confidence=0.2,
            )
        )
        self.assertGreater(decision.vx, 0.0)
        self.assertIn("search", decision.reason)

    def test_search_hint_in_searching_state_turns(self) -> None:
        decision = self.servo.compute(
            self.input(
                state="SEARCHING",
                result="SEARCH_HINT",
                point_role="search",
                point_x=800.0,
            )
        )
        self.assertLess(decision.wz, 0.0)
        self.assertEqual(decision.vx, 0.0)

    def test_observe_state_with_point_moves(self) -> None:
        decision = self.servo.compute(
            self.input(state="OBSERVE", result="TARGET_VISIBLE")
        )
        self.assertGreater(decision.vx, 0.0)

    def test_verify_state_with_point_moves(self) -> None:
        decision = self.servo.compute(
            self.input(
                state="VERIFY",
                result="VERIFY_SUCCESS",
                point_role="verify",
            )
        )
        self.assertGreater(decision.vx, 0.0)

    def test_low_confidence_point_still_moves_by_default(self) -> None:
        decision = self.servo.compute(self.input(confidence=0.0))
        self.assertGreater(decision.vx, 0.0)

    def test_no_point_stops(self) -> None:
        decision = self.servo.compute(
            self.input(result="SEARCH_NO_HINT", point_role="none", point_x=None)
        )
        self.assertTrue(decision.hard_stop)
        self.assertEqual(decision.reason, "no_valid_pixel")

    def test_paused_blocks_even_with_point(self) -> None:
        decision = self.servo.compute(self.input(state="PAUSED"))
        self.assertTrue(decision.hard_stop)
        self.assertEqual((decision.vx, decision.wz), (0.0, 0.0))

    def test_success_blocks_even_with_old_point(self) -> None:
        decision = self.servo.compute(self.input(state="SUCCESS"))
        self.assertTrue(decision.hard_stop)
        self.assertEqual((decision.vx, decision.wz), (0.0, 0.0))

    def test_first_point_can_move_forward(self) -> None:
        decision = self.servo.compute(self.input(point_streak=1))
        self.assertGreater(decision.vx, 0.0)

    def test_stale_frame_stops(self) -> None:
        decision = self.servo.compute(
            self.input(now_sec=12.0, result_received_sec=10.0, latency_ms=800.0)
        )
        self.assertTrue(decision.hard_stop)
        self.assertEqual(decision.reason, "result_receive_timeout")

    def test_emergency_obstacle_stops(self) -> None:
        decision = self.servo.compute(self.input(front_distance=0.20))
        self.assertTrue(decision.hard_stop)
        self.assertEqual(decision.reason, "emergency_obstacle")

    def test_slow_zone_scales_forward(self) -> None:
        clear = self.servo.compute(self.input(front_distance=1.0))
        slow = self.servo.compute(self.input(front_distance=0.53))
        self.assertGreater(clear.vx, slow.vx)
        self.assertGreater(slow.vx, 0.0)

    def test_rate_limiter_hard_stop_is_immediate(self) -> None:
        limiter = CommandRateLimiter(RateLimitConfig())
        vx, _ = limiter.step(0.04, 0.0, 0.1)
        self.assertGreater(vx, 0.0)
        self.assertEqual(limiter.step(0.0, 0.0, 0.1, hard_stop=True), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
