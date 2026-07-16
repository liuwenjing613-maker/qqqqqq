from __future__ import annotations

import json

from fusion.online_map_protocol import (
    BridgeConfig,
    BridgeSession,
    build_backend_request,
    normalize_backend_status,
    normalize_candidate_summary,
)


def show(label, value):
    print(f"{label:34s} -> {value}")


def main():
    summary = normalize_candidate_summary(
        {
            "map_seq": 88,
            "distance_to_decision_m": 0.74,
            "candidate_points": [
                {
                    "candidate_id": "F_LEFT",
                    "relative_heading_deg": 65,
                    "final_score": 0.73,
                    "goal_pose": {"x": 1.0, "y": 0.8, "yaw": 1.1},
                },
                {
                    "candidate_id": "F_RIGHT",
                    "relative_heading_deg": -63,
                    "final_score": 0.70,
                    "goal_pose": {"x": 1.1, "y": -0.7, "yaw": -1.0},
                },
            ],
        }
    )
    show("candidate normalization", [c["id"] for c in summary["candidates"]])

    request = build_backend_request(
        {
            "request_id": "intervention-000001",
            "decision": "MAP_QWEN",
            "reason_code": "BRANCH_AMBIGUOUS",
            "candidate_ids": ["F_LEFT", "F_RIGHT"],
            "robot_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        },
        instruction="find the bottle",
        map_topic="/map",
        odom_topic="/odom",
        scan_topic="/scan_filtered",
        use_memory=False,
    )
    show("backend operation", request["operation"])
    assert request["operation"] == "SELECT_AND_NAVIGATE"

    session = BridgeSession(
        BridgeConfig(
            request_timeout_sec=8.0,
            backend_heartbeat_timeout_sec=8.0,
            backend_cmd_timeout_sec=0.4,
            cancel_hold_sec=0.4,
            max_linear_x=0.06,
            max_angular_z=0.06,
        )
    )
    session.start("intervention-000001", 0.0)
    show("after intervention request", session.state)
    show("pre-backend output", session.output_cmd(0.1))
    assert session.output_cmd(0.1)[:2] == (0.0, 0.0)

    accepted = normalize_backend_status(
        {"request_id": "intervention-000001", "state": "QWEN_SELECTING"},
        active_request_id=session.active_request_id,
        require_final_orientation=False,
    )
    session.on_backend_status(accepted, 0.5)
    show("after backend ACK", session.state)
    show("ACK but no cmd", session.output_cmd(0.6))

    assert session.on_backend_cmd(0.08, -0.09, 0.7)
    show("clamped map command", session.output_cmd(0.8))
    assert session.output_cmd(0.8)[:2] == (0.06, -0.06)

    completed = normalize_backend_status(
        {
            "request_id": "intervention-000001",
            "state": "ARRIVED_ALIGNED",
            "final_orientation_done": True,
        },
        active_request_id=session.active_request_id,
        require_final_orientation=True,
    )
    result = session.on_backend_status(completed, 2.0)
    show("arrival result", result)
    assert result == "COMPLETED"
    session.reset("manager_resuming_ego")
    show("after return to ego", session.state)

    # Stale old status may not revive MAP.
    stale = normalize_backend_status(
        {"request_id": "intervention-000001", "state": "NAVIGATING"},
        active_request_id=None,
        require_final_orientation=False,
    )
    show("stale status after completion", stale)
    assert stale is None

    # Target takeover path blocks map velocity immediately.
    session.start("intervention-000002", 10.0)
    session.on_backend_status(
        {"request_id": "intervention-000002", "status": "NAVIGATING"}, 10.1
    )
    session.on_backend_cmd(0.03, 0.01, 10.2)
    session.cancel(10.3, "fresh_first_person_target_visible")
    show("target takeover output", session.output_cmd(10.31))
    assert session.output_cmd(10.31)[:2] == (0.0, 0.0)

    print("\nSIMULATION PASS")
    print(json.dumps({"scenarios": 2, "memory_enabled": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
