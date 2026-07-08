#!/usr/bin/env python3
"""COCO 80-class names shared by YOLO detection and semantic mapping."""

from __future__ import annotations

from typing import FrozenSet, Tuple

# YOLOv5s tag v7.0 detect model — standard COCO 80 classes (index order matters).
COCO_CLASS_NAMES: Tuple[str, ...] = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
)

# Typical movers: observed but not fused into static landmarks.
COCO_DYNAMIC_CLASSES: FrozenSet[str] = frozenset({
    "person",
    "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
})

# Handheld / small items — lower detection score threshold in semantic filter.
COCO_SMALL_OBJECTS: FrozenSet[str] = frozenset({
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "sports ball", "frisbee", "baseball bat", "baseball glove", "tennis racket", "kite",
    "remote", "cell phone", "mouse", "keyboard", "book", "clock", "vase", "scissors",
    "teddy bear", "toothbrush", "hair drier", "tie", "handbag", "umbrella",
    "skateboard", "surfboard", "skis", "snowboard",
})

# Furniture / appliances / large fixtures — larger landmark merge radius.
COCO_LARGE_OBJECTS: FrozenSet[str] = frozenset({
    "bench", "backpack", "suitcase", "chair", "couch", "potted plant", "bed", "dining table",
    "toilet", "tv", "laptop", "microwave", "oven", "toaster", "sink", "refrigerator",
    "fire hydrant", "parking meter", "stop sign", "traffic light",
    "bicycle", "car", "motorcycle", "bus", "train", "truck", "boat", "airplane",
})


def coco_class_csv() -> str:
    return ",".join(COCO_CLASS_NAMES)


def is_all_coco_mode(value: str) -> bool:
    return str(value or "").strip().lower() in ("all", "all_coco", "coco", "coco80")


def expand_class_list(values: list[str], *, auto_set: FrozenSet[str]) -> list[str]:
    """Return auto_set when values is empty or requests all COCO classes."""
    if not values:
        return sorted(auto_set)
    if len(values) == 1 and is_all_coco_mode(values[0]):
        return sorted(auto_set)
    return values
