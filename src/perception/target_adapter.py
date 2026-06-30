#!/usr/bin/env python3
import json
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional

from src.perception.target_bbox_parser import TargetBBoxParser, TargetBBoxParserConfig


@dataclass
class NavTarget:
    visible: bool
    u: Optional[float]
    v: Optional[float]
    score: float = 0.0
    stale: bool = False
    source: str = ""
    class_name: str = ""
    reason: str = ""
    bbox: Optional[list] = None
    area_ratio: Optional[float] = None
    stamp_time: float = 0.0
    vote_reason: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class TargetAdapter:
    def __init__(
        self,
        image_width: int = 640,
        image_height: int = 480,
        target_words: Optional[list] = None,
        min_score: float = 0.0,
        min_area_ratio: float = 0.0,
        max_area_ratio: float = 1.0,
        accept_unknown_class: bool = True,
        bbox_stale_sec: float = 0.45,
    ):
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.bbox_stale_sec = float(bbox_stale_sec)
        self._last_yolo_raw_json: Dict[str, Any] = {}
        self._last_yolo_json_time = 0.0
        self.bbox_parser = TargetBBoxParser(
            TargetBBoxParserConfig(
                target_words=list(target_words or []),
                target_min_score=float(min_score),
                target_stable_frames=1,
                target_lost_timeout_sec=float(bbox_stale_sec),
                target_memory_sec=float(bbox_stale_sec),
                accept_unknown_class=bool(accept_unknown_class),
                bbox_min_area_ratio=float(min_area_ratio),
                bbox_max_area_ratio=float(max_area_ratio),
            )
        )

    def update_image_geometry(self, width: int, height: int) -> None:
        if width and height:
            self.image_width = int(width)
            self.image_height = int(height)

    def from_color(self, frame: Any, color: str) -> NavTarget:
        if color == "green":
            from src.perception.target_backend_green import find_green_target

            raw = find_green_target(frame)
        else:
            from src.perception.target_backend_red import find_red_target

            raw = find_red_target(frame)
        return self._from_backend_dict(raw, source="color")

    def update_yolo_bbox_json(self, json_msg: str, now: Optional[float] = None) -> NavTarget:
        now = float(now if now is not None else time.time())
        data = self._safe_json(json_msg)
        self._last_yolo_raw_json = data
        self._last_yolo_json_time = now
        parsed = self.bbox_parser.update_json(json_msg, self.image_width, self.image_height)
        return self._from_backend_dict(parsed, source="yolo_bbox", now=now, raw_json=data)

    def current_yolo_target(self, now: Optional[float] = None) -> NavTarget:
        now = float(now if now is not None else time.time())
        raw = self.bbox_parser.get_target(now, self.image_width, self.image_height)
        raw_json = self._last_yolo_raw_json if now - self._last_yolo_json_time <= self.bbox_stale_sec else None
        return self._from_backend_dict(raw, source="yolo_bbox", now=now, raw_json=raw_json)

    def from_qwen_result(self, qwen_result: Dict[str, Any], now: Optional[float] = None) -> NavTarget:
        now = float(now if now is not None else time.time())
        if not isinstance(qwen_result, dict):
            return NavTarget(False, None, None, source="qwen_point", reason="invalid_qwen_result", stamp_time=now)

        point = qwen_result.get("target_point")
        if isinstance(point, (list, tuple)) and len(point) >= 2:
            u, v = point[0], point[1]
        else:
            u = qwen_result.get("u", qwen_result.get("cx"))
            v = qwen_result.get("v", qwen_result.get("cy"))

        if u is None or v is None or not qwen_result.get("visible", True):
            return NavTarget(False, None, None, source="qwen_point", reason="qwen_no_point", stamp_time=now)

        return NavTarget(
            visible=True,
            u=float(u),
            v=float(v),
            score=float(qwen_result.get("score", qwen_result.get("confidence", 1.0))),
            stale=False,
            source="qwen_point",
            class_name=str(qwen_result.get("class_name", "")),
            reason=str(qwen_result.get("reason", "qwen_point")),
            stamp_time=now,
        )

    def _from_backend_dict(
        self,
        raw: Dict[str, Any],
        source: str,
        now: Optional[float] = None,
        raw_json: Optional[Dict[str, Any]] = None,
        **kwargs: Any,
    ) -> NavTarget:
        del kwargs
        now = float(now if now is not None else time.time())
        if not isinstance(raw, dict) or not raw.get("visible", False):
            reason = "not_visible"
            if isinstance(raw, dict):
                reason = str(raw.get("reason", reason))
            return NavTarget(False, None, None, source=source, reason=reason, stamp_time=now)

        u = raw.get("u", raw.get("cx"))
        v = raw.get("v", raw.get("cy"))
        bbox = raw.get("bbox")
        area_ratio = raw.get("area_ratio", raw.get("bbox_area_ratio"))

        if (u is None or v is None) and isinstance(bbox, (list, tuple)) and len(bbox) == 4:
            x, y, a, b = [float(x) for x in bbox]
            if a > x and b > y:
                w = a - x
                h = b - y
            else:
                w = a
                h = b
            u = x + w / 2.0
            v = y + h / 2.0
            if area_ratio is None:
                area_ratio = (w * h) / max(float(self.image_width * self.image_height), 1.0)

        if u is None or v is None:
            return NavTarget(False, None, None, source=source, reason="missing_point", stamp_time=now)

        age_sec = float(raw.get("age_sec", 0.0) or 0.0)
        vote_reason = str(raw.get("vote_reason", raw.get("reason", "")))
        stale = bool(raw.get("stale", False))
        if age_sec > self.bbox_stale_sec:
            stale = True
        if vote_reason == "hold_last_target":
            stale = True
        if raw_json and bool(raw_json.get("stale", False)):
            stale = True

        raw_source = str(raw.get("source", source) or source)
        if source == "yolo_bbox" and raw_source == "yolo":
            raw_source = "yolo_bbox"

        return NavTarget(
            visible=not stale,
            u=float(u),
            v=float(v),
            score=float(raw.get("score", 0.0) or 0.0),
            stale=stale,
            source=raw_source,
            class_name=str(raw.get("class_name", raw.get("class", "")) or ""),
            reason=str(raw.get("reason", "ok") or "ok"),
            bbox=list(bbox) if isinstance(bbox, (list, tuple)) else None,
            area_ratio=float(area_ratio) if area_ratio is not None else None,
            stamp_time=now,
            vote_reason=vote_reason,
        )

    @staticmethod
    def _safe_json(text: str) -> Dict[str, Any]:
        try:
            data = json.loads(text)
        except Exception:
            return {}
        return data if isinstance(data, dict) else {}
