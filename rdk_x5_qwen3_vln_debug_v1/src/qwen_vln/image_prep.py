"""Client-side image prep matching Qwen3-VL smart_resize (factor=32).

Resize the frame we send AND draw on to the same token grid the model uses,
so [0,1000] relative points map 1:1 onto the visualized canvas.
"""
from __future__ import annotations

import math
from typing import Tuple

import cv2
import numpy as np

# Qwen3-VL: ViT patch=16, spatial merge=2 => factor 32 (Qwen2.5-VL used 28).
_QWEN_FACTOR = 32
_MAX_RATIO = 200


def _round_by_factor(number: float, factor: int) -> int:
    return int(round(number / factor) * factor)


def _ceil_by_factor(number: float, factor: int) -> int:
    return int(math.ceil(number / factor) * factor)


def _floor_by_factor(number: float, factor: int) -> int:
    return int(math.floor(number / factor) * factor)


def smart_resize_hw(
    height: int,
    width: int,
    *,
    factor: int = _QWEN_FACTOR,
    min_pixels: int = 65536,
    max_pixels: int = 442368,
) -> Tuple[int, int]:
    """Return (height, width) divisible by factor and within pixel budget."""
    if height <= 0 or width <= 0:
        raise ValueError(f"invalid image size {width}x{height}")
    if max_pixels < min_pixels:
        raise ValueError("max_pixels must be >= min_pixels")
    if max(height, width) / min(height, width) > _MAX_RATIO:
        raise ValueError(
            f"absolute aspect ratio must be < {_MAX_RATIO}, "
            f"got {max(height, width) / min(height, width)}"
        )

    h_bar = max(factor, _round_by_factor(height, factor))
    w_bar = max(factor, _round_by_factor(width, factor))
    if h_bar * w_bar > max_pixels:
        beta = math.sqrt((height * width) / float(max_pixels))
        h_bar = max(factor, _floor_by_factor(height / beta, factor))
        w_bar = max(factor, _floor_by_factor(width / beta, factor))
    elif h_bar * w_bar < min_pixels:
        beta = math.sqrt(float(min_pixels) / float(height * width))
        h_bar = max(factor, _ceil_by_factor(height * beta, factor))
        w_bar = max(factor, _ceil_by_factor(width * beta, factor))
    return int(h_bar), int(w_bar)


def resize_for_api(
    image: np.ndarray,
    max_width: int,
    max_height: int = 0,
    *,
    min_pixels: int = 65536,
    max_pixels: int = 442368,
) -> np.ndarray:
    """Fit into optional box, then Qwen smart_resize for token-grid alignment."""
    if image is None or image.size == 0:
        raise ValueError("empty image")
    height, width = image.shape[:2]
    scale = 1.0
    if max_width > 0 and width > max_width:
        scale = min(scale, max_width / float(width))
    if max_height > 0 and height > max_height:
        scale = min(scale, max_height / float(height))
    if scale < 0.999:
        width = max(1, int(round(width * scale)))
        height = max(1, int(round(height * scale)))
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

    height, width = image.shape[:2]
    new_h, new_w = smart_resize_hw(
        height,
        width,
        min_pixels=int(min_pixels),
        max_pixels=int(max_pixels),
    )
    if new_h == height and new_w == width:
        return image
    return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
