import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from qwen_vln.prompt_manager import PromptManager
from qwen_vln.qwen_client import parse_model_output
from qwen_vln.state_machine import NavigationStateMachine, StateMachineConfig
from qwen_vln.types import ModelResult, PixelPoint, PromptMode, VlnState


class ParserTest(unittest.TestCase):
    def test_visible_point_norm1000_to_pixel(self):
        raw = json.dumps({"s": "V", "p": [120, 80]})
        result = parse_model_output(raw, PromptMode.OBSERVE, 640, 480)
        # round(120/1000*639)=77, round(80/1000*479)=38
        self.assertEqual(result.result, "TARGET_VISIBLE")
        self.assertEqual(result.point_role, "target")
        self.assertEqual(result.point.x, 77)
        self.assertEqual(result.point.y, 38)
        self.assertNotIn("confidence", result.to_dict())

    def test_norm1000_corners_map_to_image_corners(self):
        raw = json.dumps({"s": "V", "p": [1000, 1000]})
        result = parse_model_output(raw, PromptMode.OBSERVE, 960, 1280)
        self.assertEqual(result.point.x, 959)
        self.assertEqual(result.point.y, 1279)

    def test_norm1000_center_maps_near_image_center(self):
        raw = json.dumps({"s": "V", "p": [476, 458]})
        result = parse_model_output(raw, PromptMode.OBSERVE, 960, 1280)
        # round(476/1000*959)=456, round(458/1000*1279)=586
        self.assertEqual(result.point.x, 456)
        self.assertEqual(result.point.y, 586)

    def test_out_of_bounds_rejected(self):
        raw = json.dumps({"s": "V", "p": [1001, 80]})
        with self.assertRaises(ValueError):
            parse_model_output(raw, PromptMode.OBSERVE, 640, 480)

    def test_unit_interval_rejected(self):
        raw = json.dumps({"s": "V", "p": [0.48, 0.46]})
        with self.assertRaises(ValueError):
            parse_model_output(raw, PromptMode.OBSERVE, 960, 1280)

    def test_wrong_status_for_mode_rejected(self):
        raw = json.dumps({"s": "S", "p": [320, 240]})
        with self.assertRaises(ValueError):
            parse_model_output(raw, PromptMode.OBSERVE, 640, 480)

    def test_missing_field_rejected(self):
        raw = json.dumps({"s": "I"})
        with self.assertRaises(ValueError):
            parse_model_output(raw, PromptMode.OBSERVE, 640, 480)

    def test_target_inferred_compact(self):
        raw = json.dumps({"s": "I", "p": [520, 680]})
        result = parse_model_output(raw, PromptMode.SEARCH, 640, 480)
        self.assertEqual(result.result, "TARGET_INFERRED")
        self.assertEqual(result.point_role, "search")
        self.assertIsNotNone(result.point)

    def test_null_point_rejected(self):
        raw = json.dumps({"s": "I", "p": None})
        with self.assertRaises(ValueError):
            parse_model_output(raw, PromptMode.SEARCH, 640, 480)

    def test_verify_codes(self):
        ok = parse_model_output(
            json.dumps({"s": "S", "p": [400, 300]}),
            PromptMode.VERIFY,
            640,
            480,
        )
        self.assertEqual(ok.result, "VERIFY_SUCCESS")
        self.assertEqual(ok.point_role, "verify")
        fail = parse_model_output(
            json.dumps({"s": "F", "p": [500, 700]}),
            PromptMode.VERIFY,
            640,
            480,
        )
        self.assertEqual(fail.result, "VERIFY_FAILED")
        self.assertEqual(fail.point_role, "search")

    def test_truncated_json_recovered(self):
        raw = '{"s":"V","p":[450, 430]'
        result = parse_model_output(raw, PromptMode.SEARCH, 576, 768)
        self.assertEqual(result.result, "TARGET_VISIBLE")
        # round(450/1000*575)=259, round(430/1000*767)=330
        self.assertEqual(result.point.x, 259)
        self.assertEqual(result.point.y, 330)

    def test_extra_confidence_key_ignored(self):
        raw = json.dumps({"s": "V", "p": [500, 500], "c": 95})
        result = parse_model_output(raw, PromptMode.OBSERVE, 640, 480)
        self.assertEqual(result.result, "TARGET_VISIBLE")
        self.assertNotIn("confidence", result.to_dict())


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

    def test_target_inferred_enters_inferred_state(self):
        self.fsm.command("search")
        self.fsm.mark_request_started(now=10.0)
        result = ModelResult(
            result="TARGET_INFERRED",
            point=PixelPoint(x=320, y=360),
            point_role="search",
            label="table area",
            reason_code="compact_i",
            raw_text="{}",
        )
        self.fsm.apply_result(result, PromptMode.SEARCH)
        self.assertEqual(self.fsm.state, VlnState.TARGET_INFERRED)
        self.assertEqual(self.fsm.prompt_mode(), PromptMode.SEARCH)
        self.assertFalse(self.fsm.should_request(now=11.0))
        self.assertTrue(self.fsm.should_request(now=13.1))

    def test_same_search_family_keeps_interval(self):
        self.fsm.command("search")
        self.fsm.mark_request_started(now=10.0)
        result = ModelResult(
            result="TARGET_INFERRED",
            point=PixelPoint(x=300, y=400),
            point_role="search",
            label="",
            reason_code="compact_i",
            raw_text="{}",
        )
        self.fsm.apply_result(result, PromptMode.SEARCH)
        # SEARCHING -> TARGET_INFERRED stays in the search-prompt family.
        self.assertEqual(self.fsm.state, VlnState.TARGET_INFERRED)
        self.assertFalse(self.fsm.should_request(now=11.0))
        self.assertTrue(self.fsm.should_request(now=13.1))

    def test_manual_same_state_invalidates_generation(self):
        self.fsm.command("search")
        generation = self.fsm.generation
        self.fsm.command("search")
        self.assertGreater(self.fsm.generation, generation)


class PromptTest(unittest.TestCase):
    def test_prompt_has_compact_schema(self):
        prompt = PromptManager(str(ROOT / "prompts")).build(
            PromptMode.OBSERVE,
            "find bottle",
            960,
            540,
        )
        self.assertIn("960 x 540", prompt)
        self.assertIn("[0, 1000]", prompt)
        self.assertIn('"s":"V|I|S|F"', prompt)
        self.assertIn("Never use original pixel coordinates", prompt)
        self.assertIn('s="V"', prompt)
        self.assertIn('s="I"', prompt)
        self.assertNotIn('"c"', prompt)
        self.assertNotIn("confidence", prompt.lower())

    def test_search_prompt_uses_compact_status(self):
        prompt = PromptManager(str(ROOT / "prompts")).build(
            PromptMode.SEARCH,
            "find bottle",
            640,
            480,
        )
        self.assertIn('s="I"', prompt)
        self.assertIn("exploration waypoint", prompt)
        self.assertNotIn("SEARCH_NO_HINT", prompt)
        self.assertNotIn("TARGET_INFERRED", prompt)
        self.assertNotIn("lower c", prompt)


if __name__ == "__main__":
    unittest.main()
