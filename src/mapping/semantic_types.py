#!/usr/bin/env python3
"""Datatypes for semantic mapping overlay (observations, landmarks, viewpoints)."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class SemanticObservation:
    obs_id: str
    stamp: float

    fixed_frame: str
    robot_x: float
    robot_y: float
    robot_yaw: float

    class_name: str
    score: float
    bbox_xyxy: List[float]
    u: float
    v: float
    area_ratio: float

    bearing_rad: float
    range_m: Optional[float]
    object_x: Optional[float]
    object_y: Optional[float]
    position_sigma: float

    source: str
    quality: str
    range_source: str
    image_path: Optional[str] = None
    raw_json: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SemanticObservation":
        return cls(**{k: data.get(k) for k in cls.__dataclass_fields__ if k in data})


@dataclass
class SemanticLandmark:
    landmark_id: str
    class_name: str

    frame_id: str
    x: float
    y: float

    confidence: float
    seen_count: int
    first_seen_time: float
    last_seen_time: float

    state: str
    sources: List[str] = field(default_factory=list)
    obs_ids: List[str] = field(default_factory=list)
    covariance_xy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    best_image_path: Optional[str] = None
    note: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SemanticLandmark":
        return cls(**{k: data.get(k) for k in cls.__dataclass_fields__ if k in data})


@dataclass
class ViewpointNode:
    node_id: str
    stamp: float

    frame_id: str
    x: float
    y: float
    yaw: float

    observed_classes: List[str] = field(default_factory=list)
    image_path: Optional[str] = None

    view_fov_deg: float = 70.0
    view_range_m: float = 3.0
    visited: bool = True
    note: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ViewpointNode":
        return cls(**{k: data.get(k) for k in cls.__dataclass_fields__ if k in data})


def json_dumps(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
