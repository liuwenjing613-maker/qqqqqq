from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from qwen_vln.types import ModelResult, PixelPoint, VlnState  # noqa: E402
from qwen_vln.visualizer import ResultVisualizer, ServoZoneOverlay  # noqa: E402


class ServoZoneOverlayTests(unittest.TestCase):
    def test_draw_servo_zones_matches_control_thresholds(self) -> None:
        zones = ServoZoneOverlay(
            max_vx=0.07,
            max_wz=0.05,
            kp_wz=0.01,
            angular_sign=-1.0,
            center_deadband=0.20,
            turn_only_threshold=0.70,
            cmd_wz_deadband=0.006,
        )
        vis = ResultVisualizer(servo_zones=zones)
        frame = np.zeros((540, 960, 3), dtype=np.uint8)
        result = ModelResult(
            result="TARGET_VISIBLE",
            point=PixelPoint(x=480, y=270),
            point_role="target",
            label="",
            reason_code="test",
            action="POINT",
            request_id=1,
        )
        out = vis.draw(
            frame,
            VlnState.TARGET_LOCKED,
            "find the bottle",
            result,
            False,
        )
        self.assertEqual(out.shape, frame.shape)
        self.assertEqual(vis._zone_name_for_point(480, 960), "[deadband]")
        # |e|=0.5 is between 0.20 and 0.70
        half = 0.5 * 959.0
        x = int(round(half + 0.5 * half))
        self.assertEqual(vis._zone_name_for_point(x, 960), "[steer+drive]")
        x_outer = int(round(half + 0.85 * half))
        self.assertEqual(vis._zone_name_for_point(x_outer, 960), "[rotate-only]")


if __name__ == "__main__":
    unittest.main()
