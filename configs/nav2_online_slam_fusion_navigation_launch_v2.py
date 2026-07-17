#!/usr/bin/env python3
"""Navigation-only Nav2 launch for a live slam_toolbox map.

It reuses the repository's proven navigation launch, but changes the final
velocity_smoother output from /cmd_vel to /map_qwen_plan/cmd_vel_raw.  Therefore
Nav2 cannot bypass the V1 EGO/MAP/HOLD safety mux.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ORIGINAL = Path(__file__).with_name("nav2_click_nav_navigation_launch.py")
if not ORIGINAL.is_file():
    raise RuntimeError(f"missing proven navigation launch: {ORIGINAL}")

spec = importlib.util.spec_from_file_location("nav2_click_nav_navigation_base_v2", ORIGINAL)
if spec is None or spec.loader is None:
    raise RuntimeError(f"cannot load {ORIGINAL}")
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)

# controller/behaviors -> cmd_vel_nav -> velocity_smoother -> raw backend topic
module._VEL_SMOOTHER_REMAPS = [
    ("cmd_vel", "cmd_vel_nav"),
    ("cmd_vel_smoothed", "/map_qwen_plan/cmd_vel_raw"),
]


def generate_launch_description():
    return module.generate_launch_description()
