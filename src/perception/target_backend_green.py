#!/usr/bin/env python3
import cv2
import numpy as np


def find_green_target(frame):
    """
    输入 BGR 图像。
    输出统一 target dict：
    {
      "visible": bool,
      "class_name": "green_target",
      "score": 1.0,
      "bbox": [x, y, w, h],
      "cx": cx,
      "cy": cy,
      "area_ratio": area_ratio
    }
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_green1 = np.array([0, 80, 60])
    upper_green1 = np.array([10, 255, 255])
    lower_green2 = np.array([170, 80, 60])
    upper_green2 = np.array([180, 255, 255])

    mask = cv2.inRange(hsv, lower_green1, upper_green1) | cv2.inRange(hsv, lower_green2, upper_green2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return {"visible": False}

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)

    H, W = frame.shape[:2]
    area_ratio = area / float(W * H)

    if area < 500:
        return {"visible": False}

    x, y, w, h = cv2.boundingRect(c)
    cx = x + w / 2.0
    cy = y + h / 2.0

    return {
        "visible": True,
        "class_name": "green_target",
        "score": 1.0,
        "bbox": [int(x), int(y), int(w), int(h)],
        "cx": float(cx),
        "cy": float(cy),
        "area_ratio": float(area_ratio),
    }
