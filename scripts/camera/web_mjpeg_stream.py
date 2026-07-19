#!/usr/bin/env python3
"""Backward-compatible entry — delegates to robot_web_dashboard.py.

This script no longer opens /dev/video0. Use robot camera stack first, then:
  bash scripts/camera/start_web_camera.sh
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_web_dashboard import main  # noqa: E402

if __name__ == "__main__":
    main()
