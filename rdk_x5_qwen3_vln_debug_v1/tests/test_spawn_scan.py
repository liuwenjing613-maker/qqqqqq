from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from control.qwen_visual_servo import (  # noqa: E402
    SpawnScanConfig,
    SpawnScanController,
    SpawnScanPhase,
)
from qwen_vln.prompt_manager import PromptManager  # noqa: E402
from qwen_vln.qwen_client import parse_model_output  # noqa: E402
from qwen_vln.state_machine import (  # noqa: E402
    NavigationStateMachine,
    StateMachineConfig,
)
from qwen_vln.types import ModelResult, PromptMode, VlnState  # noqa: E402


class SpawnScanProtocolTests(unittest.TestCase):
    def test_parse_inferred_scores(self) -> None:
        r = parse_model_output(
            '{"p":null,"t":0.12,"r":0.34,"q":0.56}',
            PromptMode.SPAWN_SCAN,
            960,
            540,
        )
        self.assertEqual(r.result, "TARGET_INFERRED")
        self.assertEqual(r.action, "STOP")
        self.assertIsNone(r.point)
        self.assertAlmostEqual(r.score_t, 0.12)
        self.assertAlmostEqual(r.score_r, 0.34)
        self.assertAlmostEqual(r.score_q, 0.56)

    def test_parse_visible_point(self) -> None:
        r = parse_model_output(
            '{"p":[500,400],"t":0.9,"r":0.5,"q":0.8}',
            PromptMode.SPAWN_SCAN,
            960,
            540,
        )
        self.assertEqual(r.result, "TARGET_VISIBLE")
        self.assertEqual(r.action, "POINT")
        self.assertIsNotNone(r.point)

    def test_legacy_s_field_ignored(self) -> None:
        # Older model outputs may still include s; visibility follows p only.
        r = parse_model_output(
            '{"s":"V|I","p":[500,400],"t":0.9,"r":0.5,"q":0.8}',
            PromptMode.SPAWN_SCAN,
            960,
            540,
        )
        self.assertEqual(r.result, "TARGET_VISIBLE")
        self.assertIsNotNone(r.point)

    def test_prompt_is_standalone(self) -> None:
        pm = PromptManager(str(ROOT / "prompts"))
        text = pm.build(PromptMode.SPAWN_SCAN, "find the bottle", 960, 540)
        self.assertIn("SPAWN_SCAN", text)
        self.assertIn("find the bottle", text)
        self.assertNotIn('"s"', text)
        self.assertNotIn("TURN_LEFT", text)
        self.assertNotIn("Mode: OBSERVE", text)

    def test_hud_format_line(self) -> None:
        from qwen_vln.visualizer import SpawnScanHud

        line = SpawnScanHud(
            phase="DWELL",
            sector=1,
            scores=[0.12, 0.45, None, None, None, None],
            best_sector=1,
            sector_deg=60.0,
        ).format_line()
        self.assertIn("SPAWN:", line)
        self.assertIn("DWELL", line)
        self.assertIn("0.45", line)
        self.assertIn("face->S1(+60L)", line)
        self.assertNotIn("FRAME", line)


class SpawnScanFsmTests(unittest.TestCase):
    def test_starts_in_spawn_scan(self) -> None:
        fsm = NavigationStateMachine(StateMachineConfig())
        fsm.set_instruction("find bottle")
        fsm.mark_image_ready()
        self.assertEqual(fsm.state, VlnState.SPAWN_SCAN)
        self.assertEqual(fsm.prompt_mode(), PromptMode.SPAWN_SCAN)
        self.assertFalse(fsm.should_request())

    def test_visible_aborts_to_track(self) -> None:
        fsm = NavigationStateMachine(StateMachineConfig())
        fsm.has_image = True
        fsm.instruction = "find bottle"
        fsm.command("spawn_scan")
        fsm.apply_result(
            ModelResult(
                result="TARGET_VISIBLE",
                point=None,
                point_role="target",
                label="",
                reason_code="spawn_v",
                action="POINT",
                score_q=0.9,
            ),
            PromptMode.SPAWN_SCAN,
        )
        self.assertEqual(fsm.state, VlnState.TARGET_LOCKED)

    def test_inferred_stays_in_spawn(self) -> None:
        fsm = NavigationStateMachine(StateMachineConfig())
        fsm.has_image = True
        fsm.instruction = "find bottle"
        fsm.command("spawn_scan")
        fsm.apply_result(
            ModelResult(
                result="TARGET_INFERRED",
                point=None,
                point_role="none",
                label="",
                reason_code="spawn_i",
                action="STOP",
                score_q=0.4,
            ),
            PromptMode.SPAWN_SCAN,
        )
        self.assertEqual(fsm.state, VlnState.SPAWN_SCAN)


class SpawnScanControllerTests(unittest.TestCase):
    def test_dwell_request_then_odom_turn_and_best_return(self) -> None:
        ctrl = SpawnScanController(
            SpawnScanConfig(
                sectors=3,
                sector_deg=90.0,
                wz=0.06,
                settle_sec=0.5,
                dwell_sec=2.0,
                yaw_tolerance_deg=2.0,
            )
        )
        ctrl.start(0.0)
        d0 = ctrl.update(0.4, 0.0, True)
        self.assertFalse(d0.request_observation)
        d1 = ctrl.update(0.5, 0.0, True)
        self.assertTrue(d1.request_observation)
        self.assertEqual(d1.phase, SpawnScanPhase.DWELL)
        self.assertFalse(
            ctrl.note_result("TARGET_INFERRED", 0.2, request_id=1)
        )

        d2 = ctrl.update(2.0, 0.0, True)
        self.assertEqual(d2.phase, SpawnScanPhase.TURNING)
        self.assertGreater(d2.wz, 0.0)

        # Integrate ~90 deg left via odom.
        yaw = 0.0
        for _ in range(20):
            yaw += math.radians(5.0)
            d = ctrl.update(3.0, yaw, True)
            if d.phase == SpawnScanPhase.DWELL:
                break
        self.assertEqual(ctrl.phase, SpawnScanPhase.DWELL)
        self.assertEqual(ctrl.sector_index, 1)

        ctrl.update(3.5, yaw, True)
        ctrl.note_result("TARGET_INFERRED", 0.9, request_id=2)
        # Skip remaining dwell with high score on sector 1; finish sector 2 low.
        ctrl.update(5.5, yaw, True)
        # turn to sector 2
        for _ in range(20):
            yaw += math.radians(5.0)
            d = ctrl.update(6.0, yaw, True)
            if d.phase == SpawnScanPhase.DWELL:
                break
        self.assertEqual(ctrl.sector_index, 2)
        ctrl.update(6.5, yaw, True)
        ctrl.note_result("TARGET_INFERRED", 0.1, request_id=3)
        d_end = ctrl.update(8.5, yaw, True)
        # Best is sector 1; from sector 2 left steps = (1-2)%3 = 2 -> returning
        self.assertIn(
            d_end.phase,
            {SpawnScanPhase.RETURNING, SpawnScanPhase.DONE},
        )
        self.assertEqual(ctrl.best_sector, 1)

    def test_dwell_waits_for_score_before_turn(self) -> None:
        ctrl = SpawnScanController(
            SpawnScanConfig(
                sectors=3,
                sector_deg=90.0,
                settle_sec=0.5,
                dwell_sec=2.0,
                result_wait_sec=3.0,
            )
        )
        ctrl.start(0.0)
        ctrl.update(0.5, 0.0, True)
        # Min dwell met but no score yet -> keep dwelling.
        d = ctrl.update(2.0, 0.0, True)
        self.assertEqual(d.phase, SpawnScanPhase.DWELL)
        self.assertEqual(d.reason, "spawn_scan_wait_score")
        self.assertFalse(
            ctrl.note_result("TARGET_INFERRED", 0.42, request_id=1)
        )
        self.assertAlmostEqual(ctrl.scores[0], 0.42)
        d2 = ctrl.update(2.1, 0.0, True)
        self.assertEqual(d2.phase, SpawnScanPhase.TURNING)

    def test_late_score_accepted_while_turning(self) -> None:
        ctrl = SpawnScanController(
            SpawnScanConfig(
                sectors=3,
                settle_sec=0.1,
                dwell_sec=0.2,
                result_wait_sec=0.0,
            )
        )
        ctrl.start(0.0)
        ctrl.update(0.1, 0.0, True)
        d = ctrl.update(0.2, 0.0, True)
        self.assertEqual(d.phase, SpawnScanPhase.TURNING)
        self.assertIsNone(ctrl.scores[0])
        self.assertFalse(
            ctrl.note_result("TARGET_INFERRED", 0.77, request_id=9)
        )
        self.assertAlmostEqual(ctrl.scores[0], 0.77)

    def test_cancel_keeps_scores_for_hud(self) -> None:
        ctrl = SpawnScanController(SpawnScanConfig(sectors=3))
        ctrl.start(0.0)
        ctrl.scores = [0.1, 0.9, 0.2]
        ctrl._best_sector = 1
        ctrl.cancel()
        self.assertEqual(ctrl.phase, SpawnScanPhase.IDLE)
        self.assertEqual(ctrl.scores, [0.1, 0.9, 0.2])
        self.assertEqual(ctrl.best_sector, 1)

    def test_visible_aborts(self) -> None:
        ctrl = SpawnScanController(SpawnScanConfig())
        ctrl.start(0.0)
        ctrl.update(0.5, 0.0, True)
        aborted = ctrl.note_result("TARGET_VISIBLE", 0.95, request_id=1)
        self.assertTrue(aborted)
        self.assertEqual(ctrl.phase, SpawnScanPhase.IDLE)


if __name__ == "__main__":
    unittest.main()
