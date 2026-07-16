from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from qwen_vln.image_prep import resize_for_api, smart_resize_hw  # noqa: E402
from qwen_vln.qwen_client import (  # noqa: E402
    _norm1000_to_pixel,
    parse_model_output,
    pixel_to_norm1000,
)
from qwen_vln.types import PromptMode  # noqa: E402


class CoordinateMappingTests(unittest.TestCase):
    def test_official_center_maps_to_mid_pixel(self) -> None:
        # Official: pixel = round(coord/1000 * size)
        self.assertEqual(_norm1000_to_pixel(500, 960), 480)
        self.assertEqual(_norm1000_to_pixel(0, 960), 0)
        self.assertEqual(_norm1000_to_pixel(1000, 960), 959)

    def test_roundtrip_near_identity(self) -> None:
        for size in (405, 540, 868, 960):
            for pixel in (0, size // 4, size // 2, size - 1):
                norm = pixel_to_norm1000(pixel, size)
                back = _norm1000_to_pixel(norm, size)
                self.assertLessEqual(abs(back - pixel), 1)

    def test_parse_uses_official_mapping(self) -> None:
        r = parse_model_output(
            '{"s":"V","a":"POINT","p":[450,462],"c":0}',
            PromptMode.SEARCH,
            392,
            532,
        )
        self.assertIsNotNone(r.point)
        self.assertEqual(r.point.x, _norm1000_to_pixel(450, 392))
        self.assertEqual(r.point.y, _norm1000_to_pixel(462, 532))


class SmartResizeTests(unittest.TestCase):
    def test_dimensions_divisible_by_32(self) -> None:
        h, w = smart_resize_hw(540, 960, min_pixels=65536, max_pixels=442368)
        self.assertEqual(h % 32, 0)
        self.assertEqual(w % 32, 0)
        self.assertLessEqual(h * w, 442368)
        self.assertGreaterEqual(h * w, 65536)

    def test_portrait_batch_fit_then_snap(self) -> None:
        # 4284x5712 phone photo path used by offline batch.
        img = np.zeros((5712, 4284, 3), dtype=np.uint8)
        out = resize_for_api(
            img, 960, 540, min_pixels=65536, max_pixels=442368
        )
        h, w = out.shape[:2]
        self.assertEqual(h % 32, 0)
        self.assertEqual(w % 32, 0)
        self.assertLessEqual(h * w, 442368)
        # Must not stay at naive 405x540 (not factor-32 aligned).
        self.assertNotEqual((h, w), (540, 405))
        # Portrait snaps to 416x544 under factor-32 (not the old 392x532 / factor-28).
        self.assertEqual((h, w), (544, 416))


if __name__ == "__main__":
    unittest.main()
