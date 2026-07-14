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
    ViewAdjustConfig,
    ViewAdjustController,
    ViewAdjustPhase,
)


class ServoTests(unittest.TestCase):
    def setUp(self) -> None:
        self.servo = QwenVisualServo(ServoConfig())

    def input(self, **kwargs) -> ServoInput:
        values = dict(
            now_sec=10.0,
            state="TARGET_LOCKED",
            result="TARGET_VISIBLE",
            action="POINT",
            point_role="target",
            point_x=480.0,
            image_width=960,
            confidence=0.0,
            latency_ms=800.0,
            result_received_sec=9.9,
            point_streak=1,
            front_distance=1.0,
            scan_received_sec=9.9,
        )
        values.update(kwargs)
        return ServoInput(**values)

    def test_point_keeps_old_servo_behavior(self) -> None:
        decision = self.servo.compute(self.input())
        self.assertGreater(decision.vx, 0.0)
        self.assertAlmostEqual(decision.wz, 0.0, places=4)

    def test_search_point_still_moves(self) -> None:
        decision = self.servo.compute(
            self.input(
                state="TARGET_INFERRED",
                result="TARGET_INFERRED",
                point_role="search",
            )
        )
        self.assertGreater(decision.vx, 0.0)

    def test_turn_never_enters_pixel_servo(self) -> None:
        decision = self.servo.compute(
            self.input(action="TURN_RIGHT", point_x=None)
        )
        self.assertTrue(decision.hard_stop)
        self.assertEqual(decision.reason, "action_turn_right")

    def test_stop_never_enters_pixel_servo(self) -> None:
        decision = self.servo.compute(self.input(action="STOP", point_x=None))
        self.assertTrue(decision.hard_stop)
        self.assertEqual((decision.vx, decision.wz), (0.0, 0.0))

    def test_view_adjust_one_pulse_per_request(self) -> None:
        view = ViewAdjustController(
            ViewAdjustConfig(turn_pulse_sec=1.0, settle_sec=0.2)
        )
        self.assertTrue(view.start("TURN_RIGHT", 7, 10.0))
        self.assertFalse(view.start("TURN_RIGHT", 7, 10.1))
        d = view.update(10.4)
        self.assertEqual(d.phase, ViewAdjustPhase.TURNING)
        self.assertLess(d.wz, 0.0)
        d = view.update(11.05)
        self.assertEqual(d.phase, ViewAdjustPhase.SETTLING)
        self.assertEqual(d.wz, 0.0)
        d = view.update(11.3)
        self.assertEqual(d.phase, ViewAdjustPhase.WAITING_FRESH_RESULT)
        self.assertTrue(d.request_fresh_observation)
        d2 = view.update(11.4)
        self.assertFalse(d2.request_fresh_observation)

    def test_left_and_right_signs(self) -> None:
        view = ViewAdjustController(ViewAdjustConfig())
        view.start("TURN_LEFT", 1, 0.0)
        self.assertGreater(view.update(0.1).wz, 0.0)
        view.start("TURN_RIGHT", 2, 1.0)
        self.assertLess(view.update(1.1).wz, 0.0)

    def test_rate_limiter_hard_stop_is_immediate(self) -> None:
        limiter = CommandRateLimiter(RateLimitConfig())
        vx, _ = limiter.step(0.04, 0.0, 0.1)
        self.assertGreater(vx, 0.0)
        self.assertEqual(
            limiter.step(0.0, 0.0, 0.1, hard_stop=True),
            (0.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()
