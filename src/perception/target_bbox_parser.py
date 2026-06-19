import json
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class TargetBBoxParserConfig:
    target_words: List[str]
    target_min_score: float = 0.006
    target_stable_frames: int = 1
    target_lost_timeout_sec: float = 0.8
    target_memory_sec: float = 1.5
    accept_unknown_class: bool = True
    bbox_min_area_ratio: float = 0.0005
    bbox_max_area_ratio: float = 0.70
    bbox_edge_margin_px: int = 4


class TargetBBoxParser:
    def __init__(self, cfg: TargetBBoxParserConfig):
        self.cfg = cfg
        self.last_msg_time: float = 0.0
        self.last_target: Optional[Dict[str, Any]] = None
        self.stable_frames: int = 0
        self.pending_stable_frames: int = 0

    def update_json(self, text: str, image_width: int, image_height: int) -> Dict[str, Any]:
        now = time.time()
        try:
            data = json.loads(text)
        except Exception as e:
            return {"visible": False, "reason": f"json_error:{e}"}

        if isinstance(data, dict) and data.get("visible") is False:
            self.last_msg_time = now
            self.stable_frames = 0
            self.pending_stable_frames = 0
            return {"visible": False, "reason": data.get("reason", "not_visible")}

        candidates = self._extract_candidates(data)
        best = self._select_best(candidates, image_width, image_height)

        if best is None:
            self.last_msg_time = now
            self.stable_frames = 0
            self.pending_stable_frames = 0
            self.last_target = None
            return {"visible": False, "reason": "no_valid_box"}

        self.last_msg_time = now
        self.pending_stable_frames += 1

        if self.pending_stable_frames < max(1, int(self.cfg.target_stable_frames)):
            return {
                "visible": False,
                "reason": "stabilizing",
                "stable_frames": self.pending_stable_frames,
            }

        self.stable_frames = self.pending_stable_frames
        best["visible"] = True
        best["stable_frames"] = self.stable_frames
        best["age_sec"] = 0.0
        best["source"] = "yolo"
        self.last_target = best
        return best

    def get_target(self, now: Optional[float], image_width: int, image_height: int) -> Dict[str, Any]:
        del image_width, image_height
        if now is None:
            now = time.time()

        if self.last_target is None:
            return {"visible": False, "reason": "no_target"}

        age = now - self.last_msg_time
        if age <= self.cfg.target_lost_timeout_sec:
            out = dict(self.last_target)
            out["age_sec"] = age
            out["visible"] = True
            return out

        if age <= self.cfg.target_memory_sec:
            out = dict(self.last_target)
            out["age_sec"] = age
            out["visible"] = False
            out["reason"] = "memory_only_target_lost"
            return out

        return {"visible": False, "reason": "target_timeout"}

    def _extract_candidates(self, data: Any) -> List[Dict[str, Any]]:
        candidates = []

        if not isinstance(data, dict):
            return candidates

        if "bbox" in data:
            candidates.append({
                "bbox": data.get("bbox"),
                "score": data.get("score", data.get("confidence", 1.0)),
                "class_name": data.get("class", data.get("class_name", data.get("label", ""))),
                "cx": data.get("cx"),
                "cy": data.get("cy"),
            })

        if "target_bbox" in data:
            candidates.append({
                "bbox": data.get("target_bbox"),
                "score": data.get("score", data.get("confidence", 1.0)),
                "class_name": data.get("class", data.get("class_name", data.get("target_class", ""))),
                "cx": data.get("cx"),
                "cy": data.get("cy"),
            })

        tb = data.get("target_box")
        if isinstance(tb, dict) and "bbox" in tb:
            candidates.append({
                "bbox": tb.get("bbox"),
                "score": tb.get("score", tb.get("confidence", 1.0)),
                "class_name": tb.get("class", tb.get("class_name", tb.get("label", ""))),
                "cx": tb.get("cx"),
                "cy": tb.get("cy"),
            })

        boxes = data.get("boxes")
        if isinstance(boxes, list):
            for b in boxes:
                if not isinstance(b, dict):
                    continue
                candidates.append({
                    "bbox": b.get("bbox", b.get("box")),
                    "score": b.get("score", b.get("confidence", 1.0)),
                    "class_name": b.get("class", b.get("class_name", b.get("label", ""))),
                    "cx": b.get("cx"),
                    "cy": b.get("cy"),
                })

        cx = data.get("cx")
        cy = data.get("cy")
        if cx is not None and cy is not None and not candidates:
            candidates.append({
                "bbox": None,
                "score": data.get("score", data.get("confidence", 1.0)),
                "class_name": data.get("class", data.get("class_name", data.get("label", ""))),
                "cx": cx,
                "cy": cy,
            })

        return candidates

    def _select_best(self, candidates: List[Dict[str, Any]], image_width: int, image_height: int) -> Optional[Dict[str, Any]]:
        valid = []
        for c in candidates:
            parsed = self._parse_candidate(c, image_width, image_height)
            if parsed is None:
                continue
            valid.append(parsed)

        if not valid:
            return None

        valid.sort(
            key=lambda x: (
                1 if x["target_word_hit"] else 0,
                x["score"],
                min(x["area_ratio"], 0.20),
                -abs(x["cx"] - image_width / 2.0) / max(image_width, 1),
            ),
            reverse=True,
        )
        return valid[0]

    @staticmethod
    def _normalize_bbox(
        bbox: Any,
        image_width: int,
        image_height: int,
    ) -> Optional[Tuple[float, float, float, float]]:
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return None

        try:
            a, b, c, d = [float(v) for v in bbox]
        except Exception:
            return None

        if c > a and d > b:
            x1, y1, x2, y2 = a, b, c, d
        elif c > 0 and d > 0:
            x1, y1 = a, b
            x2, y2 = a + c, b + d
        else:
            return None

        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1

        margin = 0
        x1 = clamp(x1, margin, image_width - 1)
        x2 = clamp(x2, margin, image_width - 1)
        y1 = clamp(y1, margin, image_height - 1)
        y2 = clamp(y2, margin, image_height - 1)

        if x2 - x1 <= 1 or y2 - y1 <= 1:
            return None

        return x1, y1, x2, y2

    def _parse_candidate(self, c: Dict[str, Any], image_width: int, image_height: int) -> Optional[Dict[str, Any]]:
        bbox = c.get("bbox")
        norm = self._normalize_bbox(bbox, image_width, image_height) if bbox is not None else None

        cx_raw = c.get("cx")
        cy_raw = c.get("cy")

        center_only = False
        if norm is None:
            if cx_raw is None or cy_raw is None:
                return None
            try:
                cx = float(cx_raw)
                cy = float(cy_raw)
            except Exception:
                return None
            x1 = cx - 1.0
            y1 = cy - 1.0
            x2 = cx + 1.0
            y2 = cy + 1.0
            area_ratio = self.cfg.bbox_min_area_ratio
            center_only = True
        else:
            x1, y1, x2, y2 = norm
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w = x2 - x1
            h = y2 - y1
            area_ratio = (w * h) / max(float(image_width * image_height), 1.0)

        margin = self.cfg.bbox_edge_margin_px
        if not center_only:
            if x1 < margin or y1 < margin:
                return None
            if x2 > image_width - 1 - margin or y2 > image_height - 1 - margin:
                return None

            if area_ratio < self.cfg.bbox_min_area_ratio:
                return None
            if area_ratio > self.cfg.bbox_max_area_ratio:
                return None

        score = float(c.get("score", 1.0))
        if score < self.cfg.target_min_score:
            return None

        cls = str(c.get("class_name", "") or "").strip().lower()
        target_words = [str(x).lower() for x in self.cfg.target_words]
        hit = any(t in cls or cls in t for t in target_words if t and cls)

        if (not hit) and (not self.cfg.accept_unknown_class) and cls:
            return None

        return {
            "bbox": [x1, y1, x2, y2],
            "u": cx,
            "v": cy,
            "cx": cx,
            "cy": cy,
            "score": score,
            "class_name": cls,
            "target_word_hit": hit,
            "area_ratio": area_ratio,
        }
