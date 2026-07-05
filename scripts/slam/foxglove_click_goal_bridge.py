#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Foxglove click goal -> Nav2 NavigateToPose bridge.

Subscribe:
  /foxglove_goal_point  geometry_msgs/msg/PointStamped   (recommended: single click)
  /foxglove_goal_pose   geometry_msgs/msg/PoseStamped    (legacy: click + drag)

Publish for visualization:
  /foxglove_click_planned_path    nav_msgs/msg/Path
  /foxglove_click_path_marker     visualization_msgs/msg/Marker
  /foxglove_click_trajectory_dots visualization_msgs/msg/Marker  (red dots: actual motion)
  /foxglove_click_start_marker    visualization_msgs/msg/Marker  (start point)
  /foxglove_click_goal_marker     visualization_msgs/msg/Marker  (goal circle ring)
  /foxglove_click_goal_label      visualization_msgs/msg/Marker  (goal text label)
  /foxglove_click_accepted_goal   geometry_msgs/msg/PoseStamped
  /foxglove_map_viz               nav_msgs/msg/OccupancyGrid  (Foxglove-friendly map display)

Action clients:
  /compute_path_to_pose  nav2_msgs/action/ComputePathToPose
  /navigate_to_pose      nav2_msgs/action/NavigateToPose
"""

from __future__ import annotations

import argparse
import math
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_geometry_msgs
import tf2_ros
from tf2_ros import TransformException
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from visualization_msgs.msg import Marker

_SCRIPT_DIR = __import__('pathlib').Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in __import__('sys').path:
    __import__('sys').path.insert(0, str(_SCRIPT_DIR))
from map_goal_validate import (
    DEFAULT_ROBOT_RADIUS,
    is_footprint_known_free,
    is_nav_runtime_safe,
    path_stays_in_known_free,
)

GOAL_MARKER_NS = 'foxglove_click_goal_endpoint'
PATH_MARKER_NS = 'foxglove_click_goal'
TRAJECTORY_MARKER_NS = 'foxglove_click_trajectory'
START_MARKER_NS = 'foxglove_click_start'
TRAJECTORY_SAMPLE_SEC = 0.35
TRAJECTORY_MIN_STEP_M = 0.06
TRAJECTORY_DOT_RADIUS = 0.045
START_MARKER_RADIUS = 0.11
GOAL_CIRCLE_RADIUS = 0.13
GOAL_CIRCLE_LINE_WIDTH = 0.028
GOAL_FILL_HEIGHT = 0.018
GOAL_TEXT_HEIGHT = 0.09
GOAL_COLOR_OK = (1.0, 0.42, 0.08, 0.92)
GOAL_COLOR_PENDING = (1.0, 0.82, 0.12, 0.75)
GOAL_COLOR_REJECT = (0.95, 0.18, 0.18, 0.88)
MAP_VIZ_TOPIC = '/foxglove_map_viz'
MAP_VIZ_OCCUPIED_VALUE = 99  # Foxglove treats 100 as transparent in custom color mode


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


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


def _occupancy_grid_for_foxglove(msg: OccupancyGrid) -> OccupancyGrid:
    out = OccupancyGrid()
    out.header = msg.header
    out.info = msg.info
    remapped = []
    for raw in msg.data:
        val = int(raw)
        if val < 0:
            remapped.append(-1)
        elif val == 0:
            remapped.append(0)
        elif val >= 65:
            remapped.append(MAP_VIZ_OCCUPIED_VALUE)
        else:
            remapped.append(50)
    out.data = remapped
    return out


class FoxgloveClickGoalBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('foxglove_click_goal_bridge')
        self.goal_point_topic = args.goal_point_topic
        self.goal_pose_topic = args.goal_pose_topic
        self.goal_frame = args.goal_frame
        self.map_frame = args.map_frame
        self.base_frame = args.base_frame
        self.fallback_base_frame = args.fallback_base_frame
        self.prefer_current_yaw = args.prefer_current_yaw
        self.default_goal_yaw = args.default_goal_yaw
        self.accept_any_frame = args.accept_any_frame
        self.auto_navigate = args.auto_navigate
        self.compute_path_timeout_sec = args.compute_path_timeout_sec
        self.reject_unknown_goals = args.reject_unknown_goals
        self.robot_radius = float(args.robot_radius)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._map_grid: Optional[OccupancyGrid] = None

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self._map_viz_pub = self.create_publisher(OccupancyGrid, MAP_VIZ_TOPIC, map_qos)
        self.create_timer(2.0, self._republish_map_viz)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # foxglove_bridge publishes clicks with TRANSIENT_LOCAL; VOLATILE subs won't receive them.
        goal_sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_pub = self.create_publisher(PoseStamped, '/foxglove_click_accepted_goal', latched_qos)
        self.path_pub = self.create_publisher(Path, '/foxglove_click_planned_path', latched_qos)
        self.marker_pub = self.create_publisher(Marker, '/foxglove_click_path_marker', latched_qos)
        self.trajectory_pub = self.create_publisher(Marker, '/foxglove_click_trajectory_dots', latched_qos)
        self.start_marker_pub = self.create_publisher(Marker, '/foxglove_click_start_marker', latched_qos)
        self.goal_marker_pub = self.create_publisher(Marker, '/foxglove_click_goal_marker', latched_qos)
        self.goal_label_pub = self.create_publisher(Marker, '/foxglove_click_goal_label', latched_qos)

        self._pending_goal_marker_seq = 0
        self._last_goal_marker: Optional[tuple[float, float, int, str]] = None
        self._trajectory_points: list[tuple[float, float]] = []
        self._active_viz_seq = 0
        self.create_timer(1.0, self._republish_goal_marker)
        self.create_timer(TRAJECTORY_SAMPLE_SEC, self._sample_trajectory_pose)

        self.goal_point_sub = self.create_subscription(
            PointStamped,
            self.goal_point_topic,
            self._on_goal_point,
            goal_sub_qos,
        )
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self._on_goal_pose,
            goal_sub_qos,
        )

        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self._nav_goal_handle = None
        self._is_navigating = False
        self._pending_nav_goal: Optional[PoseStamped] = None
        self._goal_seq = 0
        self._last_feedback_log_sec = 0.0
        self._click_ready_announced = False

        self.create_timer(1.0, self._check_click_ready)
        self.create_timer(0.4, self._check_nav_map_safety)

        self.get_logger().info('Foxglove click goal bridge started.')
        self.get_logger().info('Listening point click topic:')
        self.get_logger().info(f'  {self.goal_point_topic}')
        self.get_logger().info('  geometry_msgs/msg/PointStamped')
        self.get_logger().info('Listening pose click topic:')
        self.get_logger().info(f'  {self.goal_pose_topic}')
        self.get_logger().info('  geometry_msgs/msg/PoseStamped')
        self.get_logger().info('Recommended Foxglove tool:')
        self.get_logger().info(f'  Publish -> 2D point -> {self.goal_point_topic}')
        self.get_logger().info('Visual path topic: /foxglove_click_planned_path')
        self.get_logger().info('Visual trajectory: /foxglove_click_trajectory_dots (red dots)')
        self.get_logger().info('Visual start marker: /foxglove_click_start_marker')
        self.get_logger().info('Visual goal marker: /foxglove_click_goal_marker + /foxglove_click_goal_label')
        self.get_logger().info(f'Foxglove map viz: {MAP_VIZ_TOPIC} (use in 3D panel instead of /map)')

    def _republish_map_viz(self) -> None:
        if self._map_grid is None:
            return
        self._map_viz_pub.publish(_occupancy_grid_for_foxglove(self._map_grid))

    def _delete_marker(self, ns: str, marker_id: int = 0) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _deleteall_marker(self, ns: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.action = Marker.DELETEALL
        return marker

    def _clear_previous_viz(self) -> None:
        """Remove last navigation path, trajectory dots, and endpoint markers."""
        self.marker_pub.publish(self._deleteall_marker(PATH_MARKER_NS))
        self.trajectory_pub.publish(self._deleteall_marker(TRAJECTORY_MARKER_NS))
        self.start_marker_pub.publish(self._deleteall_marker(START_MARKER_NS))
        self.goal_marker_pub.publish(self._deleteall_marker(GOAL_MARKER_NS))
        self.goal_label_pub.publish(self._delete_marker(GOAL_MARKER_NS, 0))

        empty_path = Path()
        empty_path.header.frame_id = self.map_frame
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)

        self._trajectory_points.clear()
        self._last_goal_marker = None

    def try_get_current_pose_xy(self) -> Optional[tuple[float, float]]:
        for base_frame in (self.base_frame, self.fallback_base_frame):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
                t = tf.transform.translation
                return (t.x, t.y)
            except Exception:
                continue
        return None

    def _publish_start_marker(self, x: float, y: float, seq: int) -> None:
        stamp = self.get_clock().now().to_msg()

        ring = Marker()
        ring.header.frame_id = self.map_frame
        ring.header.stamp = stamp
        ring.ns = START_MARKER_NS
        ring.id = 0
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.scale.x = 0.025
        ring.color.r = 0.15
        ring.color.g = 0.85
        ring.color.b = 0.35
        ring.color.a = 0.95
        ring.lifetime.sec = 0
        ring.points = self._circle_points(x, y, START_MARKER_RADIUS, z=0.05)

        text = Marker()
        text.header.frame_id = self.map_frame
        text.header.stamp = stamp
        text.ns = START_MARKER_NS
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = START_MARKER_RADIUS + 0.06
        text.pose.orientation.w = 1.0
        text.scale.z = 0.08
        text.color.r = 0.15
        text.color.g = 0.95
        text.color.b = 0.45
        text.color.a = 0.95
        text.text = f'起点#{seq}'
        text.lifetime.sec = 0

        self.start_marker_pub.publish(ring)
        self.start_marker_pub.publish(text)
        self.get_logger().info(f'[{seq}] Start marker at ({x:.3f}, {y:.3f}) -> /foxglove_click_start_marker')

    def _append_trajectory_point(self, x: float, y: float) -> None:
        if self._trajectory_points:
            lx, ly = self._trajectory_points[-1]
            if math.hypot(x - lx, y - ly) < TRAJECTORY_MIN_STEP_M:
                return
        self._trajectory_points.append((x, y))
        self._publish_trajectory_marker()

    def _publish_trajectory_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = TRAJECTORY_MARKER_NS
        marker.id = self._active_viz_seq
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = TRAJECTORY_DOT_RADIUS * 2.0
        marker.scale.y = TRAJECTORY_DOT_RADIUS * 2.0
        marker.scale.z = TRAJECTORY_DOT_RADIUS * 2.0
        marker.color.r = 0.95
        marker.color.g = 0.12
        marker.color.b = 0.12
        marker.color.a = 0.92
        marker.lifetime.sec = 0
        marker.points = []
        for x, y in self._trajectory_points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            marker.points.append(p)
        self.trajectory_pub.publish(marker)

    def _sample_trajectory_pose(self) -> None:
        if not self._is_navigating:
            return
        pose_xy = self.try_get_current_pose_xy()
        if pose_xy is None:
            return
        self._append_trajectory_point(pose_xy[0], pose_xy[1])

    def _republish_goal_marker(self) -> None:
        if self._last_goal_marker is None:
            return
        x, y, seq, status = self._last_goal_marker
        self._publish_goal_endpoint_marker(x, y, seq, status=status, log=False)

    def _circle_points(self, cx: float, cy: float, radius: float, z: float = 0.05, segments: int = 36) -> list[Point]:
        points: list[Point] = []
        for i in range(segments + 1):
            angle = 2.0 * math.pi * i / segments
            p = Point()
            p.x = cx + radius * math.cos(angle)
            p.y = cy + radius * math.sin(angle)
            p.z = z
            points.append(p)
        return points

    def _publish_goal_endpoint_marker(
        self,
        x: float,
        y: float,
        seq: int,
        *,
        status: str = 'ok',
        log: bool = True,
    ) -> None:
        """Publish circle + text at clicked goal immediately (map frame)."""
        self._last_goal_marker = (x, y, seq, status)
        if status == 'reject':
            color = GOAL_COLOR_REJECT
            label = '无效'
        elif status == 'pending':
            color = GOAL_COLOR_PENDING
            label = '终点'
        else:
            color = GOAL_COLOR_OK
            label = f'终点#{seq}'

        stamp = self.get_clock().now().to_msg()

        ring = Marker()
        ring.header.frame_id = self.map_frame
        ring.header.stamp = stamp
        ring.ns = GOAL_MARKER_NS
        ring.id = 0
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.scale.x = GOAL_CIRCLE_LINE_WIDTH
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = color
        ring.lifetime.sec = 0
        ring.points = self._circle_points(x, y, GOAL_CIRCLE_RADIUS, z=0.05)

        fill = Marker()
        fill.header.frame_id = self.map_frame
        fill.header.stamp = stamp
        fill.ns = GOAL_MARKER_NS
        fill.id = 1
        fill.type = Marker.CYLINDER
        fill.action = Marker.ADD
        fill.pose.position.x = x
        fill.pose.position.y = y
        fill.pose.position.z = GOAL_FILL_HEIGHT * 0.5
        fill.pose.orientation.w = 1.0
        fill.scale.x = GOAL_CIRCLE_RADIUS * 1.55
        fill.scale.y = GOAL_CIRCLE_RADIUS * 1.55
        fill.scale.z = GOAL_FILL_HEIGHT
        fill.color.r, fill.color.g, fill.color.b, fill.color.a = color[0], color[1], color[2], color[3] * 0.35
        fill.lifetime.sec = 0

        text = Marker()
        text.header.frame_id = self.map_frame
        text.header.stamp = stamp
        text.ns = GOAL_MARKER_NS
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = GOAL_CIRCLE_RADIUS + 0.06
        text.pose.orientation.w = 1.0
        text.scale.z = GOAL_TEXT_HEIGHT
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 0.95
        text.text = f'{label}\n({x:.1f},{y:.1f})'
        text.lifetime.sec = 0

        self.goal_marker_pub.publish(ring)
        self.goal_marker_pub.publish(fill)
        self.goal_label_pub.publish(text)
        if log:
            self.get_logger().info(
                f'Goal marker published at ({x:.3f}, {y:.3f}) -> '
                '/foxglove_click_goal_marker /foxglove_click_goal_label'
            )

    @property
    def is_navigating(self) -> bool:
        return self._is_navigating

    def _check_click_ready(self) -> None:
        if self._click_ready_announced:
            return
        if not self.nav_client.server_is_ready():
            return
        if self._map_grid is None:
            return
        try:
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return
        self._click_ready_announced = True
        self.get_logger().info(
            '>>> CLICK READY: Foxglove 可以单击地图白色区域发送导航目标。'
            f' Topic={self.goal_point_topic}, frame=map。'
            ' 若 scan 与地图错位，请先用 /initialpose 校正。导航进行中会忽略新点击。'
        )

    def try_get_current_yaw(self, default: float = 0.0) -> float:
        for base_frame in (self.base_frame, self.fallback_base_frame):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
                q = tf.transform.rotation
                return quaternion_to_yaw(q.x, q.y, q.z, q.w)
            except Exception:
                continue
        return default

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_grid = msg
        self._map_viz_pub.publish(_occupancy_grid_for_foxglove(msg))

    def _validate_goal_on_map(self, x: float, y: float) -> bool:
        if not self.reject_unknown_goals:
            return True
        if self._map_grid is None:
            self.get_logger().warn('Map not received yet; cannot validate goal cell. Rejecting goal.')
            return False
        ok, reason = is_footprint_known_free(
            self._map_grid, x, y, robot_radius=self.robot_radius
        )
        if not ok:
            self.get_logger().error(
                f'Reject goal ({x:.3f}, {y:.3f}): {reason}. '
                'Click only on scanned white (free) area; robot footprint must not touch gray unknown.'
            )
            return False
        self.get_logger().info(
            f'Goal footprint OK: ({x:.3f}, {y:.3f}) -> {reason} (radius={self.robot_radius:.2f}m)'
        )
        return True

    def _validate_robot_pose_on_map(self, x: float, y: float) -> bool:
        if not self.reject_unknown_goals or self._map_grid is None:
            return True
        ok, reason = is_footprint_known_free(
            self._map_grid, x, y, robot_radius=self.robot_radius
        )
        if not ok:
            self.get_logger().error(
                f'Reject navigation: robot at ({x:.3f}, {y:.3f}) is not on known-free map ({reason}). '
                'Use /initialpose in Foxglove to align on white area first.'
            )
            return False
        return True

    def _check_nav_map_safety(self) -> None:
        if not self._is_navigating or not self.reject_unknown_goals or self._map_grid is None:
            return
        pose_xy = self.try_get_current_pose_xy()
        if pose_xy is None:
            return
        ok, reason = is_nav_runtime_safe(
            self._map_grid, pose_xy[0], pose_xy[1], robot_radius=self.robot_radius
        )
        if ok:
            return
        self.get_logger().error(
            f'Robot left safe nav area ({reason}); canceling navigation.'
        )
        if self._nav_goal_handle is not None:
            try:
                self._nav_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f'Could not cancel nav goal after map safety trip: {exc}')
        self._is_navigating = False
        self._nav_goal_handle = None

    def _validate_frame(self, frame_id: str) -> bool:
        if self.accept_any_frame or frame_id == self.goal_frame:
            return True
        # base_link / odom clicks are accepted and transformed to map below.
        if frame_id in (self.base_frame, self.fallback_base_frame, 'odom'):
            return True
        self.get_logger().error(
            f'Reject goal in frame "{frame_id}". Expected "{self.goal_frame}" '
            f'(or {self.base_frame} which will be auto-transformed).'
        )
        return False

    def _point_to_map(self, msg: PointStamped) -> Optional[PointStamped]:
        frame_id = msg.header.frame_id or self.goal_frame
        if frame_id == self.map_frame:
            out = PointStamped()
            out.header.frame_id = self.map_frame
            out.header.stamp = self.get_clock().now().to_msg()
            out.point = msg.point
            return out
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            transformed = tf2_geometry_msgs.do_transform_point(msg, tf)
            transformed.header.frame_id = self.map_frame
            transformed.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(
                f'Transformed click from {frame_id} ({msg.point.x:.3f}, {msg.point.y:.3f}) '
                f'-> map ({transformed.point.x:.3f}, {transformed.point.y:.3f})'
            )
            return transformed
        except TransformException as exc:
            self.get_logger().error(
                f'Cannot transform click from "{frame_id}" to "{self.map_frame}": {exc}. '
                'Set Foxglove display frame to map, or wait for TF.'
            )
            return None

    def _on_goal_point(self, msg: PointStamped) -> None:
        if self.is_navigating:
            self.get_logger().warn('Navigation is running; ignore new clicked point.')
            return

        frame_id = msg.header.frame_id or self.goal_frame
        if not self._validate_frame(frame_id):
            return

        map_pt = self._point_to_map(msg)
        if map_pt is None:
            return

        x = map_pt.point.x
        y = map_pt.point.y
        z = map_pt.point.z

        self._pending_goal_marker_seq += 1
        self._publish_goal_endpoint_marker(x, y, self._pending_goal_marker_seq, status='pending')

        yaw = self.default_goal_yaw
        if self.prefer_current_yaw:
            yaw = self.try_get_current_yaw(default=self.default_goal_yaw)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = yaw_to_quaternion(yaw)

        self.get_logger().info(
            f'Received Foxglove clicked point: frame_id={frame_id}, '
            f'x={x:.3f}, y={y:.3f}, goal_yaw={yaw:.3f}'
        )
        self.handle_goal_pose(pose)

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if self.is_navigating:
            self.get_logger().warn('Navigation is running; ignore new clicked pose.')
            return

        incoming_frame = msg.header.frame_id or self.goal_frame
        if not self._validate_frame(incoming_frame):
            return

        goal = _copy_goal_pose(msg, self.goal_frame)
        if goal.header.stamp.sec == 0 and goal.header.stamp.nanosec == 0:
            goal.header.stamp = self.get_clock().now().to_msg()

        self._pending_goal_marker_seq += 1
        self._publish_goal_endpoint_marker(
            goal.pose.position.x,
            goal.pose.position.y,
            self._pending_goal_marker_seq,
            status='pending',
        )

        self.get_logger().info(
            f'Received Foxglove clicked pose: frame_id={goal.header.frame_id}, '
            f'x={goal.pose.position.x:.3f}, y={goal.pose.position.y:.3f}'
        )
        self.handle_goal_pose(goal)

    def handle_goal_pose(self, goal: PoseStamped) -> None:
        x = goal.pose.position.x
        y = goal.pose.position.y
        if not self._validate_goal_on_map(x, y):
            self._publish_goal_endpoint_marker(x, y, self._pending_goal_marker_seq, status='reject')
            return

        self._clear_previous_viz()

        self._goal_seq += 1
        seq = self._goal_seq
        self._active_viz_seq = seq

        start_xy = self.try_get_current_pose_xy()
        if start_xy is not None:
            if not self._validate_robot_pose_on_map(start_xy[0], start_xy[1]):
                self._publish_goal_endpoint_marker(x, y, self._pending_goal_marker_seq, status='reject')
                return
            self._publish_start_marker(start_xy[0], start_xy[1], seq)
            self._append_trajectory_point(start_xy[0], start_xy[1])

        self._publish_goal_endpoint_marker(x, y, seq, status='ok')
        self.goal_pub.publish(goal)
        self._pending_nav_goal = goal

        x = goal.pose.position.x
        y = goal.pose.position.y
        self.get_logger().info(f'[{seq}] Accepted goal: frame={goal.header.frame_id}, x={x:.3f}, y={y:.3f}')

        self._request_path(goal, seq)
        if not self.auto_navigate:
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

        if seq != self._goal_seq:
            self.get_logger().info(f'[{seq}] Ignoring stale planned path result (current goal={self._goal_seq}).')
            return

        if len(path.poses) == 0:
            self.get_logger().warn(
                f'[{seq}] Planner returned an empty path. Goal may be unreachable or localization is not ready.'
            )
            if self._pending_nav_goal is not None:
                gx = self._pending_nav_goal.pose.position.x
                gy = self._pending_nav_goal.pose.position.y
                self._publish_goal_endpoint_marker(gx, gy, seq, status='reject')
            return

        if self._map_grid is not None and self.reject_unknown_goals:
            ok, reason = path_stays_in_known_free(
                self._map_grid,
                path.poses,
                robot_radius=self.robot_radius,
            )
            if not ok:
                self.get_logger().error(
                    f'[{seq}] Reject planned path: crosses non-free area ({reason}). '
                    'Navigation canceled; click a goal fully inside white scanned area.'
                )
                if self._pending_nav_goal is not None:
                    gx = self._pending_nav_goal.pose.position.x
                    gy = self._pending_nav_goal.pose.position.y
                    self._publish_goal_endpoint_marker(gx, gy, seq, status='reject')
                self._pending_nav_goal = None
                return

        self.path_pub.publish(path)
        self.marker_pub.publish(self._make_path_marker(path, seq))
        self.get_logger().info(f'[{seq}] Planned path published: {len(path.poses)} poses -> /foxglove_click_planned_path')

        if self.auto_navigate and self._pending_nav_goal is not None:
            self._send_navigation_goal(self._pending_nav_goal, seq)
            self._pending_nav_goal = None

    def _make_path_marker(self, path: Path, marker_id: int) -> Marker:
        marker = Marker()
        marker.header = path.header
        marker.ns = PATH_MARKER_NS
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

        future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda fb: self._on_nav_feedback(fb, seq),
        )
        future.add_done_callback(lambda f: self._on_nav_goal_response(f, seq))

    def _on_nav_goal_response(self, future, seq: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'[{seq}] NavigateToPose send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                f'[{seq}] NavigateToPose goal rejected. Check localization, costmap, and clicked goal position.'
            )
            return

        self._nav_goal_handle = goal_handle
        self._is_navigating = True
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
        self._is_navigating = False
        self._nav_goal_handle = None
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
            pose_xy = self.try_get_current_pose_xy()
            if pose_xy is not None:
                self._append_trajectory_point(pose_xy[0], pose_xy[1])
            self.get_logger().info(f'[{seq}] Navigation succeeded. status={status}')
            self.get_logger().info(
                'Navigation succeeded. Current pose will be updated by pose_memory_node.'
            )
        else:
            self.get_logger().error(
                f'[{seq}] Navigation ended. status={status}, error_code={err_code}, error_msg={err_msg}'
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Bridge Foxglove clicked point/pose goals to Nav2 NavigateToPose.'
    )
    parser.add_argument('--goal-point-topic', default='/foxglove_goal_point')
    parser.add_argument('--goal-pose-topic', default='/foxglove_goal_pose')
    parser.add_argument(
        '--goal-topic',
        default=None,
        help='Deprecated alias for --goal-pose-topic.',
    )
    parser.add_argument('--goal-frame', default='map', help='Expected goal frame. Usually map.')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--fallback-base-frame', default='base_footprint')
    parser.add_argument('--prefer-current-yaw', dest='prefer_current_yaw', action='store_true')
    parser.add_argument('--no-prefer-current-yaw', dest='prefer_current_yaw', action='store_false')
    parser.set_defaults(prefer_current_yaw=True)
    parser.add_argument('--default-goal-yaw', type=float, default=0.0)
    parser.add_argument('--accept-any-frame', action='store_true')
    parser.add_argument('--no-auto-navigate', dest='auto_navigate', action='store_false')
    parser.set_defaults(auto_navigate=True)
    parser.add_argument('--compute-path-timeout-sec', type=float, default=2.0)
    parser.add_argument('--reject-unknown-goals', dest='reject_unknown_goals', action='store_true')
    parser.add_argument('--allow-unknown-goals', dest='reject_unknown_goals', action='store_false')
    parser.set_defaults(reject_unknown_goals=True)
    parser.add_argument(
        '--robot-radius',
        type=float,
        default=DEFAULT_ROBOT_RADIUS,
        help='Footprint radius (m) for white-area validation; match nav2_params robot_radius.',
    )
    args = parser.parse_args()
    if args.goal_topic:
        args.goal_pose_topic = args.goal_topic
    return args


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
