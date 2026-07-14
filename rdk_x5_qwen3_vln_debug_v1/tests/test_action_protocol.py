from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from qwen_vln.qwen_client import parse_model_output  # noqa: E402
from qwen_vln.types import PromptMode  # noqa: E402


class ActionProtocolTests(unittest.TestCase):
    def test_visible_point(self) -> None:
        r = parse_model_output(
            '{"s":"V","a":"POINT","p":[500,600],"c":91}',
            PromptMode.OBSERVE,
            960,
            720,
        )
        self.assertEqual(r.action, "POINT")
        self.assertIsNotNone(r.point)
        self.assertEqual(r.confidence, 91.0)

    def test_inferred_turn_right(self) -> None:
        r = parse_model_output(
            '{"s":"I","a":"TURN_RIGHT","p":null,"c":78}',
            PromptMode.SEARCH,
            960,
            720,
        )
        self.assertEqual(r.action, "TURN_RIGHT")
        self.assertIsNone(r.point)

    def test_turn_rejects_non_null_point(self) -> None:
        with self.assertRaises(ValueError):
            parse_model_output(
                '{"s":"I","a":"TURN_LEFT","p":[100,600],"c":80}',
                PromptMode.SEARCH,
                960,
                720,
            )

    def test_point_requires_point(self) -> None:
        with self.assertRaises(ValueError):
            parse_model_output(
                '{"s":"I","a":"POINT","p":null,"c":80}',
                PromptMode.SEARCH,
                960,
                720,
            )

    def test_visible_cannot_turn(self) -> None:
        with self.assertRaises(ValueError):
            parse_model_output(
                '{"s":"V","a":"TURN_RIGHT","p":null,"c":80}',
                PromptMode.OBSERVE,
                960,
                720,
            )

    def test_legacy_point_is_backward_compatible(self) -> None:
        r = parse_model_output(
            '{"s":"I","p":[800,650]}',
            PromptMode.SEARCH,
            960,
            720,
        )
        self.assertEqual(r.action, "POINT")
        self.assertIsNotNone(r.point)
        self.assertEqual(r.confidence, 0.0)

    def test_truncated_turn_tail_recovery(self) -> None:
        r = parse_model_output(
            '{"s":"I","a":"TURN_LEFT","p":null,"c":73',
            PromptMode.SEARCH,
            960,
            720,
        )
        self.assertEqual(r.action, "TURN_LEFT")
        self.assertIsNone(r.point)
        self.assertEqual(r.confidence, 73.0)


if __name__ == "__main__":
    unittest.main()
