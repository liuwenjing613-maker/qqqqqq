#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 navigation_goal_proposal.json，经 ComputePathToPose 硬门禁后发送 Nav2 目标。"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path as FilePath
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def _path_length_m(path: NavPath) -> float:
    total = 0.0
    poses = path.poses
    for i in range(1, len(poses)):
        p0 = poses[i - 1].pose.position
        p1 = poses[i].pose.position
        total += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total


def _atomic_write_json(path: FilePath, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, path)


class NavGoalSender(Node):
    def __init__(
        self,
        goal_json: FilePath,
        map_frame: str,
        timeout_s: float,
        wait_tf_s: float,
        compute_path_timeout_s: float = 45.0,
        skip_path_validation: bool = False,
        start_robot_tolerance_m: float = 0.45,
        goal_tolerance_m: float = 0.35,
        min_path_length_m: float = 0.05,
        max_path_length_m: float = 80.0,
    ) -> None:
        super().__init__("qwen_nav_goal_sender")
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._path_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self._path_pub = self.create_publisher(NavPath, "/qwen_session/planned_path", 10)
        self._goal_json = goal_json
        self._map_frame = map_frame
        self._timeout_s = timeout_s
        self._wait_tf_s = wait_tf_s
        self._compute_path_timeout_s = compute_path_timeout_s
        self._skip_path_validation = skip_path_validation
        self._start_tol = start_robot_tolerance_m
        self._goal_tol = goal_tolerance_m
        self._min_path_len = min_path_length_m
        self._max_path_len = max_path_length_m
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _wait_map_tf(self) -> Optional[Tuple[float, float]]:
        deadline = time.time() + self._wait_tf_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    "base_link",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.3),
                )
                t = tf.transform.translation
                self.get_logger().info("TF map -> base_link 可用")
                return float(t.x), float(t.y)
            except Exception:
                time.sleep(0.2)
        self.get_logger().error(
            f"等待 TF {self._map_frame} -> base_link 超时 ({self._wait_tf_s:.0f}s)"
        )
        return None

    def _validate_and_publish_path(
        self,
        pose: PoseStamped,
        robot_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
    ) -> Optional[NavPath]:
        if self._skip_path_validation:
            self.get_logger().warn("跳过 ComputePathToPose 硬门禁（仅调试用）")
            return None

        if not self._path_client.wait_for_server(timeout_sec=self._compute_path_timeout_s):
            self.get_logger().error("/compute_path_to_pose 不可用，终止导航")
            return None

        path_goal = ComputePathToPose.Goal()
        path_goal.goal = pose
        if hasattr(path_goal, "planner_id"):
            path_goal.planner_id = ""
        if hasattr(path_goal, "use_start"):
            path_goal.use_start = False

        send_future = self._path_client.send_goal_async(path_goal)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=self._compute_path_timeout_s
        )
        if not send_future.done():
            self.get_logger().error("ComputePathToPose 发送超时，终止导航")
            return None

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("ComputePathToPose 被拒绝，终止导航")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._compute_path_timeout_s
        )
        if not result_future.done():
            self.get_logger().error("ComputePathToPose 结果超时，终止导航")
            return None

        try:
            path = result_future.result().result.path
        except Exception as exc:
            self.get_logger().error(f"ComputePathToPose 结果异常: {exc}")
            return None

        if len(path.poses) == 0:
            self.get_logger().error("规划器返回空路径，终止导航")
            return None

        path_len = _path_length_m(path)
        if path_len < self._min_path_len:
            self.get_logger().error(
                f"路径过短 ({path_len:.3f}m < {self._min_path_len:.3f}m)，终止导航"
            )
            return None
        if path_len > self._max_path_len:
            self.get_logger().error(
                f"路径过长 ({path_len:.3f}m > {self._max_path_len:.1f}m)，终止导航"
            )
            return None

        start = path.poses[0].pose.position
        end = path.poses[-1].pose.position
        start_err = math.hypot(start.x - robot_xy[0], start.y - robot_xy[1])
        goal_err = math.hypot(end.x - goal_xy[0], end.y - goal_xy[1])
        if start_err > self._start_tol:
            self.get_logger().error(
                f"路径起点与机器人偏差过大 ({start_err:.2f}m > {self._start_tol:.2f}m)"
            )
            return None
        if goal_err > self._goal_tol:
            self.get_logger().error(
                f"路径终点与目标偏差过大 ({goal_err:.2f}m > {self._goal_tol:.2f}m)"
            )
            return None

        self._path_pub.publish(path)
        self.get_logger().info(
            f"ComputePathToPose OK: {len(path.poses)} poses, length={path_len:.2f}m"
        )
        return path

    def _update_goal_json_path_ok(self, path: NavPath) -> None:
        payload = json.loads(self._goal_json.read_text(encoding="utf-8"))
        safety = payload.setdefault("safety", {})
        safety["path_checked"] = True
        safety["reachability_validated"] = True
        safety["ready_for_nav2"] = True
        safety["path_pose_count"] = len(path.poses)
        safety["path_length_m"] = round(_path_length_m(path), 3)
        safety["note"] = "ComputePathToPose 成功，已允许 NavigateToPose。"
        _atomic_write_json(self._goal_json, payload)

    def _mark_goal_json_path_failed(self, reason: str) -> None:
        payload = json.loads(self._goal_json.read_text(encoding="utf-8"))
        safety = payload.setdefault("safety", {})
        safety["path_checked"] = False
        safety["reachability_validated"] = False
        safety["ready_for_nav2"] = False
        safety["note"] = reason
        _atomic_write_json(self._goal_json, payload)

    def run(self) -> int:
        payload = json.loads(self._goal_json.read_text(encoding="utf-8"))
        if payload.get("selection_status") != "REGION_PROPOSED":
            self.get_logger().error("navigation goal 无效或未选点")
            return 2
        goal = payload.get("goal_pose_map") or {}
        gx = float(goal.get("x", 0.0))
        gy = float(goal.get("y", 0.0))
        gyaw = float(goal.get("yaw_rad", 0.0))

        if not self._client.wait_for_server(timeout_sec=self._timeout_s):
            self.get_logger().error("/navigate_to_pose 不可用")
            return 3

        robot_xy = self._wait_map_tf()
        if robot_xy is None:
            return 3

        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = self._map_frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = gx
        msg.pose.pose.position.y = gy
        msg.pose.pose.orientation = yaw_to_quaternion(gyaw)

        path = self._validate_and_publish_path(msg.pose, robot_xy, (gx, gy))
        if path is None and not self._skip_path_validation:
            self._mark_goal_json_path_failed(
                "ComputePathToPose 失败或路径无效，未发送 NavigateToPose。"
            )
            return 6

        if path is not None:
            self._update_goal_json_path_ok(path)

        self.get_logger().info(
            f"发送 Nav2 目标 {self._map_frame} ({gx:.3f}, {gy:.3f}) yaw={math.degrees(gyaw):.1f}°"
        )
        send_future = self._client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self._timeout_s)
        if not send_future.done():
            self.get_logger().error("发送目标超时")
            return 4

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error("Nav2 未返回 goal handle")
            return 4
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 拒绝目标")
            return 4

        self.get_logger().info("Nav2 已接受目标，等待导航结果...")
        result_future = goal_handle.get_result_async()
        nav_wait_start = time.time()
        last_report = nav_wait_start
        deadline = nav_wait_start + max(600.0, self._timeout_s)
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            if result_future.done():
                break
            now = time.time()
            if now - last_report >= 15.0:
                elapsed = int(now - nav_wait_start)
                self.get_logger().info(f"导航进行中... 已等待 {elapsed}s")
                last_report = now

        if not result_future.done():
            self.get_logger().error("等待导航结果超时")
            return 5

        result = result_future.result()
        status = result.status if result is not None else -1
        if status == 4:
            self.get_logger().info("导航成功")
            return 0

        self.get_logger().warn(f"导航结束 status={status}（4=成功）")
        return 5


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-json", required=True)
    parser.add_argument("--pose-state-file", default="")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--wait-tf-s", type=float, default=30.0)
    parser.add_argument("--compute-path-timeout-s", type=float, default=45.0)
    parser.add_argument(
        "--skip-path-validation",
        action="store_true",
        help="跳过 ComputePathToPose 硬门禁（仅人工调试）",
    )
    args = parser.parse_args()

    rclpy.init()
    node = NavGoalSender(
        FilePath(args.goal_json).expanduser().resolve(),
        args.map_frame,
        args.timeout_s,
        args.wait_tf_s,
        compute_path_timeout_s=args.compute_path_timeout_s,
        skip_path_validation=args.skip_path_validation,
    )
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
