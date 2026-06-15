#!/usr/bin/env python3
"""target_backend_yolo 坐标解析与同义词匹配单元测试。"""

import os
import sys
import unittest

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.perception.target_backend_yolo import (
    _matches_target_class,
    _normalize_raw_rect,
    _parse_roi_to_image_bbox,
    _scale_bbox,
    parse_target_classes,
)

IMAGE_W = 1280
IMAGE_H = 720


class TestParseTargetClasses(unittest.TestCase):
    def test_empty_string_means_no_filter(self):
        self.assertEqual(parse_target_classes(""), [])
        self.assertEqual(parse_target_classes("   "), [])
        self.assertEqual(parse_target_classes(None), [])

    def test_splits_csv(self):
        self.assertEqual(parse_target_classes("backpack, handbag"), ["backpack", "handbag"])


class TestClassMatching(unittest.TestCase):
    def test_no_filter_accepts_all(self):
        self.assertTrue(_matches_target_class("person", []))
        self.assertTrue(_matches_target_class("backpack", []))

    def test_synonym_bag_matches_backpack(self):
        self.assertTrue(_matches_target_class("backpack", ["bag"]))
        self.assertTrue(_matches_target_class("handbag", ["backpack"]))
        self.assertTrue(_matches_target_class("suitcase", ["bag"]))

    def test_substring_red_backpack(self):
        self.assertTrue(_matches_target_class("backpack", ["red backpack"]))


class TestNormalizeRawRect(unittest.TestCase):
    def test_good_backpack_x2y2(self):
        x, y, w, h = _normalize_raw_rect(306, 185, 463, 315, 640, 640)
        self.assertEqual((x, y, w, h), (306, 185, 157, 130))

    def test_wide_suitcase_x2y2(self):
        x, y, w, h = _normalize_raw_rect(127, 0, 638, 365, 640, 640)
        self.assertEqual(w, 511)
        self.assertEqual(h, 365)
        self.assertLessEqual(x + w, 640)

    def test_right_edge_fp(self):
        x, y, w, h = _normalize_raw_rect(529, 124, 639, 282, 640, 640)
        self.assertEqual((x, y, w, h), (529, 124, 110, 158))


class TestScaleBBox640(unittest.TestCase):
    def test_good_backpack_to_1280(self):
        x, y, w, h = _normalize_raw_rect(306, 185, 463, 315, 640, 640)
        sx, sy, sw, sh = _scale_bbox(x, y, w, h, IMAGE_W, IMAGE_H)
        self.assertAlmostEqual(sx, 612, delta=2)
        self.assertAlmostEqual(sw, 314, delta=2)
        self.assertAlmostEqual(sh, 146, delta=2)
        area_ratio = (sw * sh) / float(IMAGE_W * IMAGE_H)
        self.assertLess(area_ratio, 0.15)

    def test_wide_box_stays_in_frame(self):
        x, y, w, h = _normalize_raw_rect(127, 0, 638, 365, 640, 640)
        sx, sy, sw, sh = _scale_bbox(x, y, w, h, IMAGE_W, IMAGE_H)
        self.assertLessEqual(sx + sw, IMAGE_W + 2)


class TestParseRoiAutoDetect(unittest.TestCase):
    def test_yolo_log_center_backpack_640(self):
        """640 空间 x1,y1,x2,y2 → 1280 图上 x≈554"""
        bx, by, bw, bh = _parse_roi_to_image_bbox(
            277.437, 163.435, 469.631, 319.201, IMAGE_W, IMAGE_H
        )
        self.assertGreaterEqual(bx, 500)
        self.assertLessEqual(bx, 700)
        area = (bw * bh) / float(IMAGE_W * IMAGE_H)
        self.assertLess(area, 0.15)
        self.assertGreater(area, 0.02)

    def test_1280_native_not_double_scaled(self):
        """1280 原生 x2,y2 不应被推到 x>1000"""
        bx, by, bw, bh = _parse_roi_to_image_bbox(
            554, 183, 938, 359, IMAGE_W, IMAGE_H
        )
        self.assertLess(bx + bw, IMAGE_W + 5)
        self.assertGreaterEqual(bx, 500)
        self.assertLessEqual(bx, 700)

    def test_double_scale_would_be_wrong(self):
        """旧逻辑错误示例：1280 值经 640 normalize + scale 会偏右"""
        nx, ny, nw, nh = _normalize_raw_rect(554, 183, 938, 359, 640, 640)
        wrong_x, _, _, _ = _scale_bbox(nx, ny, nw, nh, IMAGE_W, IMAGE_H, 640, 640)
        fixed_x, _, _, _ = _parse_roi_to_image_bbox(
            554, 183, 938, 359, IMAGE_W, IMAGE_H
        )
        self.assertGreater(wrong_x, 1000)
        self.assertLess(fixed_x, 700)

    def test_right_edge_fp_stays_on_right_not_offscreen(self):
        bx, by, bw, bh = _parse_roi_to_image_bbox(
            466.105, 136.966, 632.667, 280.623, IMAGE_W, IMAGE_H
        )
        self.assertLessEqual(bx + bw, IMAGE_W + 2)
        self.assertGreaterEqual(bx, 800)


if __name__ == "__main__":
    unittest.main()
