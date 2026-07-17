#!/usr/bin/env python3
"""Offline protocol walk-through from live frontier candidates to bridge output."""
from __future__ import annotations

import math
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from fusion.live_frontier_backend_core_v2 import (  # noqa: E402
    FrontierConfig,
    GridMeta,
    RobotPose2D,
    candidate_summary_payload,
    extract_frontier_candidates,
)
from fusion.online_map_protocol import (  # noqa: E402
    BridgeConfig,
    BridgeSession,
    build_backend_request,
    normalize_backend_status,
    normalize_candidate_summary,
)


def main() -> int:
    grid = np.full((120, 120), 100, dtype=np.int16)
    grid[50:70, 50:70] = 0
    grid[55:65, 18:50] = 0
    grid[55:65, 70:102] = 0
    grid[70:102, 55:65] = 0
    grid[55:65, 8:18] = -1
    grid[55:65, 102:112] = -1
    grid[102:112, 55:65] = -1
    meta = GridMeta(120, 120, 0.05, -3.0, -3.0)
    robot = RobotPose2D(0.0, 0.0, math.pi / 2)
    cfg = FrontierConfig(
        obstacle_inflation_m=0.12,
        min_frontier_cells=4,
        min_goal_distance_m=0.3,
        max_goal_distance_m=3.0,
        min_heading_separation_deg=25.0,
    )
    candidates, diag = extract_frontier_candidates(grid, meta, robot, cfg)
    assert len(candidates) >= 2, diag
    backend_summary = candidate_summary_payload(
        candidates, map_version="sim-map", probe_id="probe-1", diagnostics=diag
    )
    intervention_summary = normalize_candidate_summary(backend_summary)
    assert len(intervention_summary["candidates"]) >= 2
    assert all("pose" in item for item in intervention_summary["candidates"])

    candidate_ids = [item["id"] for item in intervention_summary["candidates"][:2]]
    intervention_request = {
        "request_id": "intervention-000001",
        "decision": "MAP_QWEN",
        "reason_code": "BRANCH_AMBIGUOUS",
        "candidate_ids": candidate_ids,
        "robot_pose": {"x": 0.0, "y": 0.0, "yaw": robot.yaw},
    }
    backend_request = build_backend_request(
        intervention_request,
        instruction="find the bottle",
        map_topic="/map",
        odom_topic="/odom",
        scan_topic="/scan_filtered",
        use_memory=False,
    )
    assert backend_request["operation"] == "SELECT_AND_NAVIGATE"
    assert backend_request["options"]["keep_mapping_alive"] is True

    bridge = BridgeSession(
        BridgeConfig(
            request_timeout_sec=8.0,
            backend_heartbeat_timeout_sec=8.0,
            backend_cmd_timeout_sec=0.4,
            max_linear_x=0.055,
            max_angular_z=0.060,
            require_final_orientation=True,
        )
    )
    bridge.start("intervention-000001", 0.0)
    accepted = normalize_backend_status(
        {"request_id": "intervention-000001", "status": "QWEN_SELECTING"},
        active_request_id="intervention-000001",
        require_final_orientation=True,
    )
    assert accepted and bridge.on_backend_status(accepted, 0.2) == "ACTIVE"
    assert bridge.on_backend_cmd(0.09, -0.10, 0.3)
    vx, wz, reason = bridge.output_cmd(0.31)
    assert abs(vx - 0.055) < 1e-9 and abs(wz + 0.060) < 1e-9 and reason == "backend_cmd"

    done = normalize_backend_status(
        {
            "request_id": "intervention-000001",
            "status": "COMPLETED",
            "final_orientation_done": True,
        },
        active_request_id="intervention-000001",
        require_final_orientation=True,
    )
    assert done and done["status"] == "COMPLETED"
    assert bridge.on_backend_status(done, 1.0) == "COMPLETED"
    vx, wz, _ = bridge.output_cmd(1.01)
    assert vx == 0.0 and wz == 0.0
    print("simulate_fullflow_v2_protocol: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
