#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quat(yaw: float):
    z = math.sin(yaw * 0.5)
    w = math.cos(yaw * 0.5)
    return 0.0, 0.0, z, w


def make_pose(frame_id: str, x: float, y: float, yaw: float, node: Node) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quat(float(yaw))
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def path_length(path: Path) -> float:
    total = 0.0
    poses = path.poses
    for a, b in zip(poses[:-1], poses[1:]):
        dx = b.pose.position.x - a.pose.position.x
        dy = b.pose.position.y - a.pose.position.y
        total += math.hypot(dx, dy)
    return total


class PlanPreview(Node):
    def __init__(self, args):
        super().__init__("nav2_plan_preview")
        self.args = args

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.path_pub = self.create_publisher(Path, "/plan", qos)
        self.path_pub2 = self.create_publisher(Path, "/nav2_viz/global_plan", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/nav2_plan_markers", qos)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", qos)
        self.start_pub = self.create_publisher(PoseStamped, "/start_pose", qos)

        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

        self.path = None
        self.start_pose = None
        self.goal_pose = None
        self.markers = None

    def compute(self) -> bool:
        self.get_logger().info("waiting for /compute_path_to_pose action server...")
        if not self.client.wait_for_server(timeout_sec=self.args.action_timeout):
            self.get_logger().error("timeout waiting for /compute_path_to_pose")
            return False

        self.start_pose = make_pose("map", self.args.start_x, self.args.start_y, self.args.start_yaw, self)
        self.goal_pose = make_pose("map", self.args.goal_x, self.args.goal_y, self.args.goal_yaw, self)

        goal = ComputePathToPose.Goal()
        goal.start = self.start_pose
        goal.goal = self.goal_pose
        goal.planner_id = self.args.planner_id
        goal.use_start = True

        self.get_logger().info(
            f"request path: start=({self.args.start_x:.3f}, {self.args.start_y:.3f}, {self.args.start_yaw:.3f}), "
            f"goal=({self.args.goal_x:.3f}, {self.args.goal_y:.3f}, {self.args.goal_yaw:.3f}), "
            f"planner_id={self.args.planner_id!r}"
        )

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.action_timeout)

        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error("failed to send ComputePathToPose goal")
            return False
        if not goal_handle.accepted:
            self.get_logger().error("ComputePathToPose goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.action_timeout)

        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("timeout waiting for ComputePathToPose result")
            return False

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"ComputePathToPose failed, status={wrapped.status}")
            return False

        self.path = wrapped.result.path
        self.path.header.frame_id = "map"
        self.path.header.stamp = self.get_clock().now().to_msg()

        n = len(self.path.poses)
        length = path_length(self.path)

        if n == 0:
            self.get_logger().error("planner returned empty path")
            return False

        self.markers = self.make_markers()

        self.get_logger().info(f"path OK: poses={n}, length={length:.3f} m")
        self.publish_all()
        return True

    def make_markers(self) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = now
        line.ns = "nav2_plan_preview"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.045
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0
        line.pose.orientation.w = 1.0

        for ps in self.path.poses:
            p = Point()
            p.x = ps.pose.position.x
            p.y = ps.pose.position.y
            p.z = 0.04
            line.points.append(p)

        start = Marker()
        start.header.frame_id = "map"
        start.header.stamp = now
        start.ns = "nav2_plan_preview"
        start.id = 2
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        start.pose.position.x = self.args.start_x
        start.pose.position.y = self.args.start_y
        start.pose.position.z = 0.08
        start.pose.orientation.w = 1.0
        start.scale.x = 0.16
        start.scale.y = 0.16
        start.scale.z = 0.16
        start.color.r = 0.0
        start.color.g = 0.2
        start.color.b = 1.0
        start.color.a = 1.0

        goal = Marker()
        goal.header.frame_id = "map"
        goal.header.stamp = now
        goal.ns = "nav2_plan_preview"
        goal.id = 3
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.pose.position.x = self.args.goal_x
        goal.pose.position.y = self.args.goal_y
        goal.pose.position.z = 0.08
        goal.pose.orientation.w = 1.0
        goal.scale.x = 0.18
        goal.scale.y = 0.18
        goal.scale.z = 0.18
        goal.color.r = 1.0
        goal.color.g = 0.0
        goal.color.b = 0.0
        goal.color.a = 1.0

        arr.markers.extend([line, start, goal])
        return arr

    def publish_all(self):
        if self.path is None:
            return

        now = self.get_clock().now().to_msg()

        self.path.header.stamp = now
        for ps in self.path.poses:
            ps.header.stamp = now

        self.start_pose.header.stamp = now
        self.goal_pose.header.stamp = now

        for m in self.markers.markers:
            m.header.stamp = now

        self.path_pub.publish(self.path)
        self.path_pub2.publish(self.path)
        self.marker_pub.publish(self.markers)
        self.goal_pub.publish(self.goal_pose)
        self.start_pub.publish(self.start_pose)

    def keep_alive(self):
        keep_sec = float(self.args.keep_sec)
        start_time = time.time()

        self.get_logger().info(
            f"publishing preview topics for {keep_sec:.1f}s; open Foxglove and view /map + /plan + /nav2_plan_markers"
        )

        while rclpy.ok():
            self.publish_all()
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.5)

            if keep_sec > 0 and time.time() - start_time >= keep_sec:
                break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--start-yaw", type=float, default=0.0)
    parser.add_argument("--planner-id", type=str, default="GridBased")
    parser.add_argument("--keep-sec", type=float, default=600.0)
    parser.add_argument("--action-timeout", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = PlanPreview(args)

    try:
        ok = node.compute()
        if not ok:
            sys.exit(2)
        node.keep_alive()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
