#!/usr/bin/env python3
import cv2
import numpy as np


def find_red_target(frame):
    """
    红色书包目标检测后端：与 red_backpack_debug.py 保持一致。
    目标：明显红色书包能够被稳定找到。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 放宽红色范围：兼容暗红、橙红、压缩偏色
    lower_red1 = np.array([0, 50, 35])
    upper_red1 = np.array([18, 255, 255])

    lower_red2 = np.array([165, 50, 35])
    upper_red2 = np.array([180, 255, 255])

    mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    H, W = frame.shape[:2]

    if not contours:
        return {"visible": False, "reason": "no_contour"}

    candidates = []

    for c in contours:
        area = cv2.contourArea(c)
        if area <= 0:
            continue

        x, y, w, h = cv2.boundingRect(c)
        cx = x + w / 2.0
        cy = y + h / 2.0

        bbox_area = w * h
        contour_area_ratio = area / float(W * H)
        bbox_area_ratio = bbox_area / float(W * H)
        aspect = w / float(h)

        # 与 red_backpack_debug.py 保持一致
        if contour_area_ratio < 0.004:
            continue

        if w < 35 or h < 35:
            continue

        if aspect < 0.20 or aspect > 4.0:
            continue

        if cy > H * 0.92 and contour_area_ratio < 0.025:
            continue

        candidates.append({
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "cx": cx,
            "cy": cy,
            "area": area,
            "bbox_area": bbox_area,
            "area_ratio": contour_area_ratio,
            "bbox_area_ratio": bbox_area_ratio,
            "aspect": aspect,
        })

    if not candidates:
        return {"visible": False, "reason": "filtered_small_or_noise"}

    best = max(candidates, key=lambda item: item["area_ratio"])

    return {
        "visible": True,
        "class_name": "red_backpack",
        "score": 1.0,
        "bbox": [int(best["x"]), int(best["y"]), int(best["w"]), int(best["h"])],
        "cx": float(best["cx"]),
        "cy": float(best["cy"]),
        "area_ratio": float(best["area_ratio"]),
        "bbox_area_ratio": float(best["bbox_area_ratio"]),
        "aspect": float(best["aspect"]),
        "reason": "valid_red_backpack_color_region",
    }
