#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Foxglove click goal -> Nav2 NavigateToPose bridge.

Subscribe:
  /foxglove_goal_pose  geometry_msgs/msg/PoseStamped

Publish for visualization:
  /foxglove_click_planned_path  nav_msgs/msg/Path
  /foxglove_click_path_marker   visualization_msgs/msg/Marker
  /foxglove_click_accepted_goal geometry_msgs/msg/PoseStamped

Action clients:
  /compute_path_to_pose  nav2_msgs/action/ComputePathToPose
  /navigate_to_pose      nav2_msgs/action/NavigateToPose
"""

import argparse
import math
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from visualization_msgs.msg import Marker


def _norm_quaternion_in_place(pose: PoseStamped) -> None:
    q = pose.pose.orientation
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if n < 1e-9:
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0
        return
    q.x /= n
    q.y /= n
    q.z /= n
    q.w /= n


def _copy_goal_pose(msg: PoseStamped, frame_id: str) -> PoseStamped:
    goal = PoseStamped()
    goal.header.frame_id = msg.header.frame_id or frame_id
    goal.header.stamp = msg.header.stamp
    goal.pose = msg.pose
    _norm_quaternion_in_place(goal)
    return goal


class FoxgloveClickGoalBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('foxglove_click_goal_bridge')
        self.goal_topic = args.goal_topic
        self.goal_frame = args.goal_frame
        self.accept_any_frame = args.accept_any_frame
        self.auto_navigate = args.auto_navigate
        self.compute_path_timeout_sec = args.compute_path_timeout_sec

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_pub = self.create_publisher(PoseStamped, '/foxglove_click_accepted_goal', latched_qos)
        self.path_pub = self.create_publisher(Path, '/foxglove_click_planned_path', latched_qos)
        self.marker_pub = self.create_publisher(Marker, '/foxglove_click_path_marker', latched_qos)

        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self._on_goal_pose, 10)

        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self._nav_goal_handle = None
        self._goal_seq = 0
        self._last_feedback_log_sec = 0.0

        self.get_logger().info('Foxglove click goal bridge started.')
        self.get_logger().info(f'Subscribe goal topic: {self.goal_topic}')
        self.get_logger().info('Foxglove should publish a geometry_msgs/msg/PoseStamped in frame "map".')
        self.get_logger().info('Visual path topic: /foxglove_click_planned_path')

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        incoming_frame = msg.header.frame_id or self.goal_frame
        if (not self.accept_any_frame) and incoming_frame != self.goal_frame:
            self.get_logger().error(
                f'Reject goal in frame "{incoming_frame}". Expected "{self.goal_frame}". '
                f'Set Foxglove 3D fixed frame to {self.goal_frame}, or run with --accept-any-frame only if you know why.'
            )
            return

        goal = _copy_goal_pose(msg, self.goal_frame)
        if goal.header.stamp.sec == 0 and goal.header.stamp.nanosec == 0:
            goal.header.stamp = self.get_clock().now().to_msg()

        self._goal_seq += 1
        seq = self._goal_seq
        self.goal_pub.publish(goal)

        x = goal.pose.position.x
        y = goal.pose.position.y
        self.get_logger().info(f'[{seq}] Accepted clicked goal: frame={goal.header.frame_id}, x={x:.3f}, y={y:.3f}')

        self._request_path(goal, seq)
        if self.auto_navigate:
            self._send_navigation_goal(goal, seq)
        else:
            self.get_logger().warn(f'[{seq}] --no-auto-navigate is set, so only path planning was requested.')

    def _request_path(self, goal: PoseStamped, seq: int) -> None:
        if not self.path_client.wait_for_server(timeout_sec=self.compute_path_timeout_sec):
            self.get_logger().warn(
                f'[{seq}] /compute_path_to_pose action server not ready. '
                'Navigation can still run, but the pre-drawn path may not appear.'
            )
            return

        path_goal = ComputePathToPose.Goal()
        path_goal.goal = goal
        if hasattr(path_goal, 'planner_id'):
            path_goal.planner_id = ''
        if hasattr(path_goal, 'use_start'):
            path_goal.use_start = False

        future = self.path_client.send_goal_async(path_goal)
        future.add_done_callback(lambda f: self._on_path_goal_response(f, seq))

    def _on_path_goal_response(self, future, seq: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'[{seq}] ComputePathToPose send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f'[{seq}] ComputePathToPose goal rejected by planner server.')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_path_result(f, seq))

    def _on_path_result(self, future, seq: int) -> None:
        try:
            result_msg = future.result().result
            path = result_msg.path
        except Exception as exc:
            self.get_logger().error(f'[{seq}] ComputePathToPose result failed: {exc}')
            return

        if len(path.poses) == 0:
            self.get_logger().warn(f'[{seq}] Planner returned an empty path. Goal may be unreachable or costmap/localization is not ready.')
            return

        self.path_pub.publish(path)
        self.marker_pub.publish(self._make_path_marker(path, seq))
        self.get_logger().info(f'[{seq}] Planned path published: {len(path.poses)} poses -> /foxglove_click_planned_path')

    def _make_path_marker(self, path: Path, marker_id: int) -> Marker:
        marker = Marker()
        marker.header = path.header
        marker.ns = 'foxglove_click_goal'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.points = []
        for pose in path.poses:
            p = Point()
            p.x = pose.pose.position.x
            p.y = pose.pose.position.y
            p.z = 0.04
            marker.points.append(p)
        return marker

    def _send_navigation_goal(self, goal: PoseStamped, seq: int) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'[{seq}] /navigate_to_pose action server not ready. Start Nav2 first.')
            return

        if self._nav_goal_handle is not None:
            try:
                self.get_logger().warn(f'[{seq}] Canceling previous navigation goal before sending the new one.')
                self._nav_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f'[{seq}] Could not cancel previous goal cleanly: {exc}')

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        if hasattr(nav_goal, 'behavior_tree'):
            nav_goal.behavior_tree = ''

        future = self.nav_client.send_goal_async(nav_goal, feedback_callback=lambda fb: self._on_nav_feedback(fb, seq))
        future.add_done_callback(lambda f: self._on_nav_goal_response(f, seq))

    def _on_nav_goal_response(self, future, seq: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'[{seq}] NavigateToPose send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().error(f'[{seq}] NavigateToPose goal rejected. Check localization, costmap, and clicked goal position.')
            return

        self._nav_goal_handle = goal_handle
        self.get_logger().info(f'[{seq}] NavigateToPose goal accepted. Robot should start planning/following.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_nav_result(f, seq))

    def _on_nav_feedback(self, feedback_msg, seq: int) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self._last_feedback_log_sec < 2.0:
            return
        self._last_feedback_log_sec = now_sec
        fb = feedback_msg.feedback
        distance = getattr(fb, 'distance_remaining', None)
        recoveries = getattr(fb, 'number_of_recoveries', None)
        if distance is not None:
            self.get_logger().info(f'[{seq}] navigating: distance_remaining={distance:.3f} m, recoveries={recoveries}')

    def _on_nav_result(self, future, seq: int) -> None:
        try:
            wrapped = future.result()
            status = wrapped.status
            result = wrapped.result
            err_code = getattr(result, 'error_code', None)
            err_msg = getattr(result, 'error_msg', '')
        except Exception as exc:
            self.get_logger().error(f'[{seq}] NavigateToPose result failed: {exc}')
            return

        if status == GoalStatus.STATUS_SUCCEEDED or err_code == 0:
            self.get_logger().info(f'[{seq}] Navigation succeeded. status={status}')
        else:
            self.get_logger().error(f'[{seq}] Navigation ended. status={status}, error_code={err_code}, error_msg={err_msg}')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Bridge Foxglove clicked PoseStamped goals to Nav2 NavigateToPose.')
    parser.add_argument('--goal-topic', default='/foxglove_goal_pose', help='PoseStamped topic published by Foxglove 3D click-to-publish.')
    parser.add_argument('--goal-frame', default='map', help='Expected goal frame. Usually map.')
    parser.add_argument('--accept-any-frame', action='store_true', help='Accept non-map goal frames. Not recommended for normal use.')
    parser.add_argument('--no-auto-navigate', dest='auto_navigate', action='store_false', help='Only compute/publish path; do not send NavigateToPose.')
    parser.set_defaults(auto_navigate=True)
    parser.add_argument('--compute-path-timeout-sec', type=float, default=2.0, help='Wait time for /compute_path_to_pose server.')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = FoxgloveClickGoalBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
