#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.perception.bpu_utilization import BpuUtilizationReader


def test_format_line_with_cached_value():
    reader = BpuUtilizationReader(refresh_sec=10.0)
    now = __import__("time").time()
    reader._cached_ratio = 12.5
    reader._cached_at = now
    assert reader.format_line() == "BPU util=12%"
    assert reader.format_line(prefix="HW") == "HW util=12%"


def test_format_line_na_when_unavailable():
    reader = BpuUtilizationReader(refresh_sec=10.0)
    reader._cached_ratio = None
    reader._cached_at = 0.0
    import src.perception.bpu_utilization as mod
    old_paths = mod._RATIO_PATHS
    mod._RATIO_PATHS = ("/nonexistent/bpu/ratio",)
    try:
        assert reader.format_line() == "BPU util=n/a"
    finally:
        mod._RATIO_PATHS = old_paths


def test_read_percent_uses_cache():
    reader = BpuUtilizationReader(refresh_sec=10.0)
    reader._cached_ratio = 7.0
    reader._cached_at = 100.0
    assert reader.read_percent(now=100.5) == 7.0


if __name__ == "__main__":
    test_format_line_with_cached_value()
    test_format_line_na_when_unavailable()
    test_read_percent_uses_cache()
    print("PASS test_bpu_utilization")
