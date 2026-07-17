#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 navigation_goal_proposal.json，向 Nav2 /navigate_to_pose 发送目标。"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path as FilePath

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


class NavGoalSender(Node):
    def __init__(
        self,
        goal_json: FilePath,
        map_frame: str,
        timeout_s: float,
        wait_tf_s: float,
        publish_planned_path: bool = True,
        compute_path_timeout_s: float = 45.0,
    ) -> None:
        super().__init__("qwen_nav_goal_sender")
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._path_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self._path_pub = self.create_publisher(NavPath, "/qwen_session/planned_path", 10)
        self._goal_json = goal_json
        self._map_frame = map_frame
        self._timeout_s = timeout_s
        self._wait_tf_s = wait_tf_s
        self._publish_planned_path = publish_planned_path
        self._compute_path_timeout_s = compute_path_timeout_s
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _wait_map_tf(self) -> bool:
        deadline = time.time() + self._wait_tf_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                self._tf_buffer.lookup_transform(
                    self._map_frame,
                    "base_link",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.3),
                )
                self.get_logger().info("TF map -> base_link 可用")
                return True
            except Exception:
                time.sleep(0.2)
        self.get_logger().error(
            f"等待 TF {self._map_frame} -> base_link 超时 ({self._wait_tf_s:.0f}s)"
        )
        return False

    def _publish_planned_path_preview(self, pose: PoseStamped) -> None:
        if not self._publish_planned_path:
            return
        if not self._path_client.wait_for_server(timeout_sec=self._compute_path_timeout_s):
            self.get_logger().warn(
                "/compute_path_to_pose 不可用，跳过 /qwen_session/planned_path 预览（导航仍可继续）"
            )
            return

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
            self.get_logger().warn("ComputePathToPose 发送超时，跳过路径预览")
            return

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("ComputePathToPose 被拒绝，跳过路径预览")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._compute_path_timeout_s
        )
        if not result_future.done():
            self.get_logger().warn("ComputePathToPose 结果超时，跳过路径预览")
            return

        try:
            path = result_future.result().result.path
        except Exception as exc:
            self.get_logger().warn(f"ComputePathToPose 结果异常: {exc}")
            return

        if len(path.poses) == 0:
            self.get_logger().warn("规划器返回空路径，Foxglove 不显示 planned_path")
            return

        self._path_pub.publish(path)
        self.get_logger().info(
            f"已发布 /qwen_session/planned_path ({len(path.poses)} poses)"
        )

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

        if not self._wait_map_tf():
            return 3

        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = self._map_frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = gx
        msg.pose.pose.position.y = gy
        msg.pose.pose.orientation = yaw_to_quaternion(gyaw)

        self._publish_planned_path_preview(msg.pose)

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
            self.get_logger().error("Nav2 未返回 goal handle（可能定位/代价地图未就绪）")
            return 4
        if not goal_handle.accepted:
            self.get_logger().error(
                "Nav2 拒绝目标：常见原因是 AMCL 未定位、目标在障碍区、或 planner 未激活"
            )
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
                self.get_logger().info(f"导航进行中... 已等待 {elapsed}s（机器人应开始沿路径移动）")
                last_report = now

        if not result_future.done():
            self.get_logger().error("等待导航结果超时")
            return 5

        result = result_future.result()
        status = result.status if result is not None else -1
        # rclpy action status: 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info("导航成功")
            return 0

        self.get_logger().warn(f"导航结束 status={status}（4=成功）")
        return 5


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-json", required=True)
    parser.add_argument("--pose-state-file", default="", help="仅 shell 传递，本脚本不直接使用")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--wait-tf-s", type=float, default=30.0)
    parser.add_argument("--compute-path-timeout-s", type=float, default=45.0)
    parser.add_argument(
        "--no-planned-path",
        action="store_true",
        help="不调用 compute_path_to_pose，不发布 /qwen_session/planned_path",
    )
    args = parser.parse_args()

    rclpy.init()
    node = NavGoalSender(
        FilePath(args.goal_json).expanduser().resolve(),
        args.map_frame,
        args.timeout_s,
        args.wait_tf_s,
        publish_planned_path=not args.no_planned_path,
        compute_path_timeout_s=args.compute_path_timeout_s,
    )
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
