import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from qwen_vln.prompt_manager import PromptManager
from qwen_vln.qwen_client import parse_model_output
from qwen_vln.state_machine import NavigationStateMachine, StateMachineConfig
from qwen_vln.types import ModelResult, PromptMode, VlnState


class ParserTest(unittest.TestCase):
    def test_visible_point(self):
        raw = json.dumps(
            {
                "result": "TARGET_VISIBLE",
                "point": {"x": 120, "y": 80},
                "point_role": "target",
                "confidence": 0.9,
                "label": "bottle",
                "reason_code": "exact_target_visible",
            }
        )
        result = parse_model_output(raw, PromptMode.OBSERVE, 640, 480)
        self.assertEqual(result.point.x, 120)
        self.assertEqual(result.point.y, 80)

    def test_out_of_bounds_rejected(self):
        raw = json.dumps(
            {
                "result": "TARGET_VISIBLE",
                "point": {"x": 1000, "y": 80},
                "point_role": "target",
                "confidence": 0.9,
                "label": "bottle",
                "reason_code": "exact_target_visible",
            }
        )
        with self.assertRaises(ValueError):
            parse_model_output(raw, PromptMode.OBSERVE, 640, 480)

    def test_wrong_result_for_mode_rejected(self):
        raw = json.dumps(
            {
                "result": "SEARCH_HINT",
                "point": {"x": 320, "y": 240},
                "point_role": "search",
                "confidence": 0.7,
                "label": "table_area",
                "reason_code": "target_likely_near_table",
            }
        )
        with self.assertRaises(ValueError):
            parse_model_output(raw, PromptMode.OBSERVE, 640, 480)


    def test_missing_field_rejected(self):
        raw = json.dumps(
            {
                "result": "TARGET_NOT_VISIBLE",
                "point": None,
                "point_role": "none",
                "confidence": 0.9,
                "label": "",
            }
        )
        with self.assertRaises(ValueError):
            parse_model_output(raw, PromptMode.OBSERVE, 640, 480)

    def test_search_no_hint_has_no_point(self):
        raw = json.dumps(
            {
                "result": "SEARCH_NO_HINT",
                "point": None,
                "point_role": "none",
                "confidence": 0.8,
                "label": "",
                "reason_code": "no_visible_semantic_clue",
            }
        )
        result = parse_model_output(raw, PromptMode.SEARCH, 640, 480)
        self.assertIsNone(result.point)


class StateMachineTest(unittest.TestCase):
    def setUp(self):
        self.fsm = NavigationStateMachine(StateMachineConfig())
        self.fsm.set_instruction("find the bottle")
        self.fsm.mark_image_ready()

    def test_initial_observe(self):
        self.assertEqual(self.fsm.state, VlnState.OBSERVE)
        self.assertEqual(self.fsm.prompt_mode(), PromptMode.OBSERVE)

    def test_manual_commands(self):
        self.fsm.command("search")
        self.assertEqual(self.fsm.state, VlnState.SEARCHING)
        self.fsm.command("inferred")
        self.assertEqual(self.fsm.state, VlnState.TARGET_INFERRED)
        self.fsm.command("verify")
        self.assertEqual(self.fsm.state, VlnState.VERIFY)
        self.fsm.command("pause")
        self.assertEqual(self.fsm.state, VlnState.PAUSED)

    def test_search_hint_enters_inferred(self):
        self.fsm.command("search")
        self.fsm.mark_request_started(now=10.0)
        result = ModelResult(
            result="SEARCH_HINT",
            point=None,
            point_role="search",
            confidence=0.8,
            label="table area",
            reason_code="target_likely_near_table",
            raw_text="{}",
        )
        self.fsm.apply_result(result, PromptMode.SEARCH)
        self.assertEqual(self.fsm.state, VlnState.TARGET_INFERRED)
        self.assertEqual(self.fsm.prompt_mode(), PromptMode.SEARCH)
        self.assertFalse(self.fsm.should_request(now=11.0))
        self.assertTrue(self.fsm.should_request(now=13.1))

    def test_same_state_result_keeps_interval(self):
        self.fsm.command("search")
        self.fsm.mark_request_started(now=10.0)
        result = ModelResult(
            result="SEARCH_NO_HINT",
            point=None,
            point_role="none",
            confidence=0.8,
            label="",
            reason_code="no_visible_semantic_clue",
            raw_text="{}",
        )
        self.fsm.apply_result(result, PromptMode.SEARCH)
        self.assertEqual(self.fsm.state, VlnState.SEARCHING)
        self.assertFalse(self.fsm.should_request(now=11.0))
        self.assertTrue(self.fsm.should_request(now=13.1))

    def test_manual_same_state_invalidates_generation(self):
        self.fsm.command("search")
        generation = self.fsm.generation
        self.fsm.command("search")
        self.assertGreater(self.fsm.generation, generation)


class PromptTest(unittest.TestCase):
    def test_prompt_has_exact_dimensions(self):
        prompt = PromptManager(str(ROOT / "prompts")).build(
            PromptMode.OBSERVE,
            "find bottle",
            960,
            540,
        )
        self.assertIn("960 x 540", prompt)
        self.assertIn("[0, 959]", prompt)
        self.assertIn("[0, 539]", prompt)
        self.assertIn("TARGET_NOT_VISIBLE", prompt)
        self.assertIn("JSON", prompt)

    def test_search_prompt_allows_no_hint(self):
        prompt = PromptManager(str(ROOT / "prompts")).build(
            PromptMode.SEARCH,
            "find bottle",
            640,
            480,
        )
        self.assertIn("SEARCH_NO_HINT", prompt)
        self.assertIn("NOT a traversable path point", prompt)


if __name__ == "__main__":
    unittest.main()
