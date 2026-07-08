#!/usr/bin/env python3
"""Read Horizon RDK BPU utilization from sysfs (lightweight, cached)."""

from __future__ import annotations

import time
from typing import Optional

_RATIO_PATHS = (
    "/sys/devices/system/bpu/ratio",
    "/sys/devices/platform/soc/3a000000.bpu/ratio",
)


class BpuUtilizationReader:
    """Poll BPU busy ratio (0-100%) with a short cache to limit sysfs reads."""

    def __init__(self, refresh_sec: float = 0.4):
        self.refresh_sec = max(0.1, float(refresh_sec))
        self._cached_ratio: Optional[float] = None
        self._cached_at = 0.0

    def read_percent(self, now: Optional[float] = None) -> Optional[float]:
        now = float(now if now is not None else time.time())
        if (
            self._cached_ratio is not None
            and now - self._cached_at < self.refresh_sec
        ):
            return self._cached_ratio

        for path in _RATIO_PATHS:
            try:
                with open(path, "r", encoding="ascii") as handle:
                    value = float(handle.read().strip())
                self._cached_ratio = max(0.0, min(100.0, value))
                self._cached_at = now
                return self._cached_ratio
            except (OSError, ValueError):
                continue
        return self._cached_ratio

    def format_line(self, prefix: str = "BPU") -> str:
        pct = self.read_percent()
        if pct is None:
            return f"{prefix} util=n/a"
        return f"{prefix} util={pct:.0f}%"
