#!/usr/bin/env python3
"""Generate a temporary Nav2 parameter file for live-map fusion V2."""
from __future__ import annotations

import argparse
from pathlib import Path
import yaml


def nested(root, *keys):
    cur = root
    for key in keys:
        cur = cur.setdefault(key, {})
    return cur


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()
    src = Path(args.input)
    dst = Path(args.output)
    cfg = yaml.safe_load(src.read_text(encoding="utf-8")) or {}

    bt = nested(cfg, "bt_navigator", "ros__parameters")
    # Milliseconds. The stock 5000ms service wait was too short on RDK under load
    # (compute_path_to_pose ack + costmap clear both timed out ~5s).
    bt["wait_for_service_timeout"] = 30000
    bt["default_server_timeout"] = 30000
    bt["bt_loop_duration"] = 30

    planner = nested(cfg, "planner_server", "ros__parameters", "GridBased")
    planner["use_astar"] = True
    planner["allow_unknown"] = True
    planner["tolerance"] = 0.20
    planner["max_planning_time"] = 10.0

    planner_srv = nested(cfg, "planner_server", "ros__parameters")
    planner_srv["expected_planner_frequency"] = 5.0

    controller = nested(cfg, "controller_server", "ros__parameters")
    controller["controller_frequency"] = 8.0
    progress = nested(controller, "progress_checker")
    progress["required_movement_radius"] = 0.02
    progress["movement_time_allowance"] = 45.0
    goal = nested(controller, "general_goal_checker")
    goal["xy_goal_tolerance"] = 0.18
    goal["yaw_goal_tolerance"] = 6.28
    follow = nested(controller, "FollowPath")
    follow["desired_linear_vel"] = 0.035
    follow["rotate_to_heading_angular_vel"] = 0.055
    follow["max_angular_accel"] = 0.10
    follow["use_rotate_to_heading"] = False

    local_costmap = nested(cfg, "local_costmap", "local_costmap", "ros__parameters")
    local_costmap["update_frequency"] = 3.0
    local_costmap["publish_frequency"] = 1.0
    local_costmap["always_send_full_costmap"] = False

    global_costmap = nested(cfg, "global_costmap", "global_costmap", "ros__parameters")
    global_costmap["update_frequency"] = 0.5
    global_costmap["publish_frequency"] = 0.5
    global_costmap["always_send_full_costmap"] = False

    smoother = nested(cfg, "velocity_smoother", "ros__parameters")
    smoother["max_velocity"] = [0.040, 0.0, 0.060]
    smoother["min_velocity"] = [-0.040, 0.0, -0.060]
    smoother["max_accel"] = [0.060, 0.0, 0.10]
    smoother["max_decel"] = [-0.060, 0.0, -0.10]
    smoother["velocity_timeout"] = 0.40

    behavior = nested(cfg, "behavior_server", "ros__parameters")
    behavior["max_rotational_vel"] = 0.060
    behavior["min_rotational_vel"] = 0.025
    behavior["rotational_acc_lim"] = 0.10

    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(dst)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
