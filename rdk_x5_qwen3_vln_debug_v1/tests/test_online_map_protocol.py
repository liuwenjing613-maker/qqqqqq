from __future__ import annotations

import unittest

from fusion.online_map_protocol import (
    BridgeConfig,
    BridgeSession,
    build_backend_request,
    normalize_backend_status,
    normalize_candidate_summary,
)


class CandidateProtocolTest(unittest.TestCase):
    def test_aliases_and_pose_are_normalized(self):
        value = normalize_candidate_summary(
            {
                "map_seq": 7,
                "distance_to_decision_m": 0.8,
                "candidate_points": [
                    {
                        "candidate_id": "F1",
                        "relative_heading_deg": 62,
                        "final_score": 0.72,
                        "is_reachable": True,
                        "goal_pose": {"x": 1, "y": 2, "yaw": 0.3},
                    }
                ],
            }
        )
        self.assertEqual(value["map_version"], "7")
        self.assertEqual(value["decision_distance_m"], 0.8)
        self.assertEqual(value["candidates"][0]["id"], "F1")
        self.assertEqual(value["candidates"][0]["heading_deg"], 62.0)
        self.assertEqual(value["candidates"][0]["pose"]["x"], 1.0)

    def test_direct_request_uses_single_candidate_without_qwen(self):
        req = build_backend_request(
            {
                "request_id": "r1",
                "decision": "MAP_DIRECT",
                "candidate_ids": ["F1"],
            },
            instruction="find bottle",
            map_topic="/map",
            odom_topic="/odom",
            scan_topic="/scan_filtered",
            use_memory=False,
        )
        self.assertEqual(req["operation"], "NAVIGATE_CANDIDATE")
        self.assertEqual(req["selected_candidate_id"], "F1")
        self.assertFalse(req["options"]["use_memory"])

    def test_failure_without_candidates_requests_extract_select(self):
        req = build_backend_request(
            {
                "request_id": "r2",
                "decision": "MAP_QWEN",
                "candidate_ids": [],
            },
            instruction="find bottle",
            map_topic="/map",
            odom_topic="/odom",
            scan_topic="/scan",
            use_memory=False,
        )
        self.assertEqual(req["operation"], "EXTRACT_SELECT_AND_NAVIGATE")

    def test_stale_backend_status_is_ignored(self):
        self.assertIsNone(
            normalize_backend_status(
                {"request_id": "old", "status": "COMPLETED"},
                active_request_id="new",
                require_final_orientation=False,
            )
        )

    def test_orientation_gate_keeps_map_active(self):
        value = normalize_backend_status(
            {"request_id": "r", "status": "COMPLETED"},
            active_request_id="r",
            require_final_orientation=True,
        )
        self.assertEqual(value["status"], "NAVIGATING")
        self.assertTrue(value["waiting_final_orientation"])
        value = normalize_backend_status(
            {
                "request_id": "r",
                "status": "COMPLETED",
                "final_orientation_done": True,
            },
            active_request_id="r",
            require_final_orientation=True,
        )
        self.assertEqual(value["status"], "COMPLETED")


class BridgeSessionTest(unittest.TestCase):
    def setUp(self):
        self.session = BridgeSession(
            BridgeConfig(
                request_timeout_sec=2.0,
                backend_heartbeat_timeout_sec=3.0,
                backend_cmd_timeout_sec=0.4,
                cancel_hold_sec=0.3,
                max_linear_x=0.06,
                max_angular_z=0.06,
            )
        )

    def test_waits_with_zero_until_backend_status(self):
        self.session.start("r1", 0.0)
        self.assertEqual(self.session.output_cmd(0.1)[:2], (0.0, 0.0))
        self.assertFalse(self.session.on_backend_cmd(0.03, 0.01, 0.2))
        event = self.session.on_backend_status(
            {"request_id": "r1", "status": "ACCEPTED"}, 0.3
        )
        self.assertEqual(event, "ACTIVE")
        self.assertTrue(self.session.on_backend_cmd(0.2, -0.2, 0.4))
        x, z, reason = self.session.output_cmd(0.5)
        self.assertEqual((x, z), (0.06, -0.06))
        self.assertEqual(reason, "backend_cmd")

    def test_stale_command_fails_to_zero(self):
        self.session.start("r1", 0.0)
        self.session.on_backend_status(
            {"request_id": "r1", "status": "NAVIGATING"}, 0.1
        )
        self.session.on_backend_cmd(0.03, 0.01, 0.2)
        self.assertEqual(self.session.output_cmd(0.7)[:2], (0.0, 0.0))

    def test_timeout_reports_failure(self):
        self.session.start("r1", 0.0)
        self.assertEqual(self.session.tick(2.1), "FAILED")
        self.assertEqual(self.session.last_reason, "backend_request_timeout")

    def test_cancel_immediately_blocks_command(self):
        self.session.start("r1", 0.0)
        self.session.on_backend_status(
            {"request_id": "r1", "status": "NAVIGATING"}, 0.1
        )
        self.session.on_backend_cmd(0.03, 0.01, 0.2)
        self.session.cancel(0.3, "target_visible")
        self.assertEqual(self.session.output_cmd(0.31)[:2], (0.0, 0.0))
        self.assertEqual(self.session.tick(0.61), "CANCEL_COMPLETE")
        self.assertEqual(self.session.state, "IDLE")


if __name__ == "__main__":
    unittest.main()
