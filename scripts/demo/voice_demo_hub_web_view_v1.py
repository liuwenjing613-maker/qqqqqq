#!/usr/bin/env python3
"""Deprecated — use scripts/camera/robot_web_dashboard.py instead."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "camera"))
from robot_web_dashboard import main  # noqa: E402

if __name__ == "__main__":
    main()
