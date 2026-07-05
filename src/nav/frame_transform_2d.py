#!/usr/bin/env python3
"""Small 2D frame transform helpers for nav explore geometry."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Transform2D:
    """Rigid 2D transform: p_target = R(yaw) @ p_source + t."""

    cos_yaw: float
    sin_yaw: float
    tx: float
    ty: float

    @classmethod
    def identity(cls) -> "Transform2D":
        return cls(1.0, 0.0, 0.0, 0.0)

    def apply(self, x: float, y: float) -> Tuple[float, float]:
        return (
            self.cos_yaw * x - self.sin_yaw * y + self.tx,
            self.sin_yaw * x + self.cos_yaw * y + self.ty,
        )

    def apply_path(self, path: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        return [self.apply(px, py) for px, py in path]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform2d_from_tf_message(tf_msg) -> Transform2D:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
    return Transform2D(math.cos(yaw), math.sin(yaw), float(t.x), float(t.y))


def transform_points(
    path: Iterable[Tuple[float, float]],
    transform: Transform2D,
) -> List[Tuple[float, float]]:
    return transform.apply_path(list(path))


def hint_goal_frame(hint: dict, default_frame: str) -> str:
    frame = hint.get("goal_frame")
    if frame is None or str(frame).strip() == "":
        return default_frame
    return str(frame)
