from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any, Dict, Optional, Tuple


class VlnState(str, Enum):
    WAIT_IMAGE = "WAIT_IMAGE"
    OBSERVE = "OBSERVE"
    TARGET_LOCKED = "TARGET_LOCKED"
    TARGET_INFERRED = "TARGET_INFERRED"
    SEARCHING = "SEARCHING"
    VERIFY = "VERIFY"
    SUCCESS = "SUCCESS"
    PAUSED = "PAUSED"
    ERROR = "ERROR"


class PromptMode(str, Enum):
    OBSERVE = "observe"
    TRACK = "track"
    SEARCH = "search"
    VERIFY = "verify"


@dataclass(frozen=True)
class PixelPoint:
    x: int
    y: int

    def as_tuple(self) -> Tuple[int, int]:
        return self.x, self.y


@dataclass
class ModelResult:
    result: str
    point: Optional[PixelPoint]
    point_role: str
    label: str
    reason_code: str
    # V3 protocol additions. Defaults keep old named/positional construction valid.
    action: str = "POINT"
    confidence: float = 0.0
    raw_text: str = ""
    latency_ms: float = 0.0
    request_id: int = 0
    image_width: int = 0
    image_height: int = 0

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        if self.point is not None:
            data["point"] = {"x": self.point.x, "y": self.point.y}
        else:
            data["point"] = None
        return data
