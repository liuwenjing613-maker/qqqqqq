#!/usr/bin/env python3
"""Reliable lifecycle / Nav2 readiness probes via rclpy (avoid flaky ros2 CLI)."""

from __future__ import annotations

import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

PRIMARY_STATE_ACTIVE = 3


def _node_name(raw: str) -> str:
    return raw.strip().lstrip('/')


def get_lifecycle_state(node: Node, target: str, call_timeout: float = 8.0) -> tuple[int | None, str]:
    target = _node_name(target)
    client = node.create_client(GetState, f'/{target}/get_state')
    if not client.wait_for_service(timeout_sec=3.0):
        return None, 'no_service'
    req = GetState.Request()
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=call_timeout)
    if not fut.done() or fut.result() is None:
        return None, 'call_failed'
    st = fut.result().current_state
    return int(st.id), f'{st.label} [{st.id}]'


def wait_lifecycle_active(target: str, timeout_sec: float) -> int:
    target = _node_name(target)
    rclpy.init()
    node = Node('lifecycle_wait')
    deadline = time.time() + timeout_sec
    last = 'NO RESPONSE'
    try:
        while time.time() < deadline:
            state_id, label = get_lifecycle_state(node, target)
            if label:
                last = label
            if state_id == PRIMARY_STATE_ACTIVE:
                print(last)
                return 0
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(1.0)
        print(last)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def wait_nav_actions(timeout_sec: float) -> int:
    rclpy.init()
    node = Node('nav_action_wait')
    nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    plan = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    deadline = time.time() + timeout_sec
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if nav.server_is_ready() and plan.server_is_ready():
                print('nav_actions_ready')
                return 0
            time.sleep(0.8)
        print('nav_actions_not_ready')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    if len(sys.argv) < 2:
        print('usage: lifecycle_probe.py wait <node> [timeout_sec]', file=sys.stderr)
        print('       lifecycle_probe.py nav-actions [timeout_sec]', file=sys.stderr)
        return 2
    cmd = sys.argv[1]
    if cmd == 'wait':
        node = sys.argv[2]
        timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 120.0
        return wait_lifecycle_active(node, timeout)
    if cmd == 'nav-actions':
        timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 180.0
        return wait_nav_actions(timeout)
    print(f'unknown command: {cmd}', file=sys.stderr)
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
