#!/usr/bin/env python3
import os
import sys

import pytest

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_ollama_client import parse_nav_result


def test_norm1000_center_maps_to_original_center():
    raw = {"u": 500, "v": 500}
    r = parse_nav_result(
        raw,
        orig_w=1280,
        orig_h=1707,
        model_w=192,
        model_h=256,
        sx=1280 / 192,
        sy=1707 / 256,
        coord_mode="norm1000",
    )
    assert r["usable"] is True
    assert abs(r["u"] - 639.5) < 2
    assert abs(r["v"] - 853.0) < 2


def test_model_coord_maps_to_original():
    raw = {"u": 96, "v": 128}
    r = parse_nav_result(
        raw,
        orig_w=1280,
        orig_h=1707,
        model_w=192,
        model_h=256,
        sx=1280 / 192,
        sy=1707 / 256,
        coord_mode="model",
    )
    assert r["usable"] is True
    assert abs(r["u"] - 640) < 2
    assert abs(r["v"] - 853.5) < 2


def test_null_uv_means_not_visible():
    raw = {"u": None, "v": None}
    r = parse_nav_result(
        raw,
        orig_w=1280,
        orig_h=1707,
        model_w=192,
        model_h=256,
        sx=1280 / 192,
        sy=1707 / 256,
        coord_mode="norm1000",
    )
    assert r["usable"] is False
    assert r["u"] is None
    assert r["v"] is None
    assert r["_coord_reason"] == "missing_point"


def test_norm1000_out_of_range_invalid():
    raw = {"u": 1200, "v": 500}
    r = parse_nav_result(
        raw,
        orig_w=1280,
        orig_h=1707,
        model_w=192,
        model_h=256,
        sx=1280 / 192,
        sy=1707 / 256,
        coord_mode="norm1000",
    )
    assert r["usable"] is False
    assert r["_coord_invalid"] is True
    assert r["u"] is None
    assert r["v"] is None


def test_norm1000_array_uv_takes_center():
    """Model sometimes returns u/v as [min, max] spans; use center."""
    raw = {"u": [432, 567], "v": [432, 568]}
    r = parse_nav_result(
        raw,
        orig_w=1280,
        orig_h=1707,
        model_w=192,
        model_h=256,
        sx=1280 / 192,
        sy=1707 / 256,
        coord_mode="norm1000",
    )
    assert r["usable"] is True
    assert r["_raw_u"] == pytest.approx(499.5)
    assert r["_raw_v"] == pytest.approx(500.0)
    assert abs(r["u"] - 638.86) < 2
    assert abs(r["v"] - 853.0) < 2


def test_extract_xy_fallback():
    raw = {"x": 500, "y": 500}
    r = parse_nav_result(
        raw,
        orig_w=1280,
        orig_h=1707,
        model_w=192,
        model_h=256,
        sx=1280 / 192,
        sy=1707 / 256,
        coord_mode="norm1000",
    )
    assert r["usable"] is True
