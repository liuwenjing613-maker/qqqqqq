#!/usr/bin/env python3
"""
离线验证 YOLO ROI 坐标解析（无需 ROS）。

用 YOLO 日志中的真实 rect 与 PerceptionTargets 可能出现的 1280 原生格式对照，
打印各解释路径下的 bbox，确认 auto-detect 选中正确结果。

用法:
  python3 scripts/verify_yolo_coord_parsing.py
"""
import os
import sys

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.perception.target_backend_yolo import (
    _bbox_plausibility,
    _normalize_raw_rect,
    _parse_roi_to_image_bbox,
    _scale_bbox,
)

IMAGE_W = 1280
IMAGE_H = 720

# 来自 yolo_diag_yolo_world.log 的真实检测（640 空间 x1,y1,x2,y2）
CASES = [
    {
        "name": "center_backpack_640",
        "desc": "YOLO log: 277,163,469,319 backpack score~0.004",
        "rect": (277.437, 163.435, 469.631, 319.201),
    },
    {
        "name": "center_backpack_640_v2",
        "desc": "YOLO log: 306,185,463,315",
        "rect": (306.893, 185.701, 463.078, 314.899),
    },
    {
        "name": "right_edge_fp_640",
        "desc": "YOLO log: 466,136,632,280 右缘误检",
        "rect": (466.105, 136.966, 632.667, 280.623),
    },
    {
        "name": "wide_suitcase_640",
        "desc": "YOLO log: 127,0,638,365 宽框",
        "rect": (127.266, 0, 638.423, 365.567),
    },
    {
        "name": "center_backpack_1280_native",
        "desc": "若 PerceptionTargets 已是 1280 空间（约 640 值 x2）",
        "rect": (554, 183, 938, 359),
    },
    {
        "name": "wrong_double_scale_source",
        "desc": "1280 x2/y2 被当 640 normalize 再 scale 的典型错误输入",
        "rect": (554, 183, 938, 359),
    },
]


def interpret_paths(x_off, y_off, w_f, h_f):
    rows = []
    for label, cw, ch, do_scale in (
        ("640_norm+scale", 640, 640, True),
        ("1280_norm+no_scale", IMAGE_W, IMAGE_H, False),
        ("640_norm+no_scale", 640, 640, False),
    ):
        nx, ny, nw, nh = _normalize_raw_rect(x_off, y_off, w_f, h_f, cw, ch)
        if nw <= 0 or nh <= 0:
            continue
        if do_scale:
            bx, by, bw, bh = _scale_bbox(nx, ny, nw, nh, IMAGE_W, IMAGE_H, cw, ch)
        else:
            bx, by, bw, bh = nx, ny, nw, nh
        score = _bbox_plausibility(bx, by, bw, bh, IMAGE_W, IMAGE_H)
        rows.append((label, [bx, by, bw, bh], score))
    auto = _parse_roi_to_image_bbox(x_off, y_off, w_f, h_f, IMAGE_W, IMAGE_H)
    return rows, auto


def main():
    print(f"image={IMAGE_W}x{IMAGE_H}\n")
    for case in CASES:
        x_off, y_off, w_f, h_f = case["rect"]
        print(f"=== {case['name']} ===")
        print(case["desc"])
        print(f"  rect_raw=({x_off:.1f}, {y_off:.1f}, {w_f:.1f}, {h_f:.1f})")
        rows, auto = interpret_paths(x_off, y_off, w_f, h_f)
        for label, bbox, score in rows:
            print(f"  {label:22s} bbox={bbox} plausibility={score:.3f}")
        print(f"  {'AUTO_PICK':22s} bbox={list(auto)}")
        print()

    print("期望: center_backpack 的 AUTO_PICK x 约 500~700，area_ratio < 0.15")
    bx, by, bw, bh = _parse_roi_to_image_bbox(277.437, 163.435, 469.631, 319.201, IMAGE_W, IMAGE_H)
    area = (bw * bh) / float(IMAGE_W * IMAGE_H)
    ok = 500 <= bx <= 700 and area < 0.15
    print(f"center_backpack check: bbox=[{bx},{by},{bw},{bh}] area={area:.3f} -> {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
