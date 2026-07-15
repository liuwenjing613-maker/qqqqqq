from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from control.qwen_visual_servo import (  # noqa: E402
    CommandRateLimiter,
    EmergencyReverseConfig,
    EmergencyReverseController,
    QwenVisualServo,
    RateLimitConfig,
    ServoConfig,
    ServoInput,
    TurnPendingConfig,
    TurnPendingGate,
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

    def test_target_visible_moves_when_lidar_null(self) -> None:
        decision = self.servo.compute(
            self.input(front_distance=None, scan_received_sec=None)
        )
        self.assertFalse(decision.hard_stop)
        self.assertGreater(decision.vx, 0.0)
        self.assertEqual(decision.obstacle_scale, 1.0)
        self.assertEqual(decision.reason, "continuous_visual_servo")

    def test_target_visible_moves_when_lidar_stale(self) -> None:
        decision = self.servo.compute(
            self.input(
                front_distance=0.30,
                scan_received_sec=0.0,
                now_sec=10.0,
            )
        )
        self.assertFalse(decision.hard_stop)
        self.assertGreater(decision.vx, 0.0)
        self.assertEqual(decision.obstacle_scale, 1.0)

    def test_fresh_close_lidar_still_emergency_stops(self) -> None:
        decision = self.servo.compute(
            self.input(front_distance=0.20, scan_received_sec=9.9)
        )
        self.assertTrue(decision.hard_stop)
        self.assertEqual(decision.reason, "emergency_obstacle")

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
            ViewAdjustConfig(
                pre_turn_stop_sec=0.15,
                turn_pulse_sec=1.0,
                settle_sec=0.2,
            )
        )
        self.assertTrue(view.start("TURN_RIGHT", 7, 10.0))
        self.assertFalse(view.start("TURN_RIGHT", 7, 10.1))
        d = view.update(10.05)
        self.assertEqual(d.phase, ViewAdjustPhase.PRE_TURN_STOP)
        self.assertEqual((d.vx, d.wz), (0.0, 0.0))
        self.assertTrue(d.hard_stop)
        d = view.update(10.4)
        self.assertEqual(d.phase, ViewAdjustPhase.TURNING)
        self.assertLess(d.wz, 0.0)
        d = view.update(11.2)
        self.assertEqual(d.phase, ViewAdjustPhase.SETTLING)
        self.assertEqual(d.wz, 0.0)
        d = view.update(11.4)
        self.assertEqual(d.phase, ViewAdjustPhase.WAITING_FRESH_RESULT)
        self.assertTrue(d.request_fresh_observation)
        d2 = view.update(11.5)
        self.assertFalse(d2.request_fresh_observation)

    def test_left_and_right_signs(self) -> None:
        view = ViewAdjustController(ViewAdjustConfig())
        view.start("TURN_LEFT", 1, 0.0)
        self.assertGreater(view.update(0.2).wz, 0.0)
        view.start("TURN_RIGHT", 2, 1.0)
        self.assertLess(view.update(1.2).wz, 0.0)

    def test_rotate_only_band_uses_fixed_wz(self) -> None:
        servo = QwenVisualServo(
            ServoConfig(
                kp_wz=0.01,
                rotate_only_wz=0.06,
                max_wz=0.06,
                center_deadband=0.2,
                turn_only_threshold=0.7,
                angular_sign=-1.0,
            )
        )
        half = 0.5 * 959.0
        point_x = half + 0.8 * half
        decision = servo.compute(
            self.input(point_x=point_x, image_width=960)
        )
        self.assertEqual(decision.reason, "rotate_only_large_error")
        self.assertAlmostEqual(decision.wz, -0.06, places=4)

    def test_steer_drive_band_still_uses_kp(self) -> None:
        servo = QwenVisualServo(
            ServoConfig(
                kp_wz=0.01,
                rotate_only_wz=0.06,
                max_wz=0.06,
                center_deadband=0.2,
                turn_only_threshold=0.7,
                angular_sign=-1.0,
            )
        )
        half = 0.5 * 959.0
        point_x = half + 0.65 * half
        decision = servo.compute(
            self.input(point_x=point_x, image_width=960)
        )
        self.assertIn("visual_servo", decision.reason)
        self.assertAlmostEqual(decision.wz, -0.0065, places=4)

    def test_turn_pending_requires_consecutive_near_lidar(self) -> None:
        gate = TurnPendingGate(
            TurnPendingConfig(
                entry_distance=0.45,
                entry_frames=3,
                pending_vx=0.04,
            )
        )
        self.assertTrue(gate.start("TURN_RIGHT", 9))
        self.assertAlmostEqual(gate.desired_vx(0.07, 0.07), 0.04)
        gate.update_scan(0.9)
        self.assertFalse(gate.ready)
        gate.update_scan(0.44)
        gate.update_scan(0.43)
        self.assertFalse(gate.ready)
        gate.update_scan(0.44)
        self.assertTrue(gate.ready)

    def test_emergency_reverse_latches_until_clearance(self) -> None:
        # Trigger on stop_distance (0.42), release at 0.42 + 0.18 = 0.60.
        reverse = EmergencyReverseController(
            EmergencyReverseConfig(
                trigger_distance=0.42,
                clearance=0.18,
                reverse_vx=-0.055,
            )
        )
        self.assertFalse(reverse.update(0.43))
        self.assertTrue(reverse.update(0.42))
        self.assertTrue(reverse.update(0.50))
        self.assertTrue(reverse.update(0.59))
        self.assertFalse(reverse.update(0.60))

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
