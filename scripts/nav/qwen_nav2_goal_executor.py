#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Qwen navigation goal executor: path validation, Foxglove viz, NavigateToPose."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Optional, Tuple

_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_DIR = _SCRIPT_DIR.parents[1]
_SLAM_DIR = _SCRIPT_DIR.parent / "slam"
for p in (_SCRIPT_DIR, _SLAM_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

_DEFAULT_FASTDDS = str(_PROJECT_DIR / "configs" / "fastdds_no_shm.xml")
if not os.environ.get("FASTRTPS_DEFAULT_PROFILES"):
    os.environ["FASTRTPS_DEFAULT_PROFILES"] = _DEFAULT_FASTDDS

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from map_goal_validate import path_stays_in_known_free
from qwen_nav2_common import (
    NavPhase,
    ParsedGoal,
    ProgressWatchState,
    atomic_write_json,
    compute_max_path_length_m,
    normalize_yaw,
    path_length_m,
    progress_timed_out,
    realpath,
    time_now,
    update_progress_watch,
    validate_goal_inputs,
    validate_goal_on_map,
    validate_planned_path,
    write_nav2_state,
)

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return normalize_yaw(math.atan2(siny, cosy))


class QwenNav2GoalExecutor(Node):
    def __init__(self, args: argparse.Namespace, goal: ParsedGoal) -> None:
        super().__init__("qwen_nav2_goal_executor")
        self.args = args
        self.goal = goal
        self.raw_goal_x = goal.goal_x
        self.raw_goal_y = goal.goal_y
        self.effective_goal_x = goal.goal_x
        self.effective_goal_y = goal.goal_y
        self.effective_goal_yaw = goal.goal_yaw
        self.runtime_dir = Path(args.runtime_dir)
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.runtime_dir / "goal_executor.log"
        self._nav_launch_pid = int(args.nav_launch_pid) if args.nav_launch_pid else 0
        self._map_grid: Optional[OccupancyGrid] = None
        self._last_scan_ts = 0.0
        self._last_odom_ts = 0.0
        self._nav_goal_handle = None
        self._last_feedback_log = 0.0
        self._recoveries = 0
        self._progress = ProgressWatchState()
        self._cmd_pub: Optional[Any] = None  # type: ignore[name-defined]

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.path_pub = self.create_publisher(Path, "/qwen_session/planned_path", LATCHED_QOS)
        self.start_marker_pub = self.create_publisher(Marker, "/qwen_session/start_marker", LATCHED_QOS)
        self.goal_marker_pub = self.create_publisher(Marker, "/qwen_session/goal_marker", LATCHED_QOS)
        self.goal_label_pub = self.create_publisher(Marker, "/qwen_session/goal_label", LATCHED_QOS)
        self.trajectory_pub = self.create_publisher(Marker, "/qwen_session/trajectory", LATCHED_QOS)
        self.status_pub = self.create_publisher(String, "/qwen_session/nav_status", LATCHED_QOS)

        self.create_subscription(OccupancyGrid, "/map", self._on_map, LATCHED_QOS)
        self.create_subscription(LaserScan, "/scan_filtered", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_grid = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_ts = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom_ts = time.monotonic()

    def log(self, msg: str) -> None:
        line = f"[QWEN_NAV] {msg}"
        print(line, flush=True)
        with open(self.log_path, "a", encoding="utf-8") as fh:
            fh.write(line + "\n")

    def publish_zero_velocity(self, duration_s: float = 0.5, rate_hz: float = 10.0) -> None:
        """Short-lived /cmd_vel publisher; destroyed after stop pulse."""
        pub = self.create_publisher(Twist, "/cmd_vel", 10)
        twist = Twist()
        end = time.monotonic() + duration_s
        period = 1.0 / rate_hz
        while time.monotonic() < end:
            pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(period)
        self.destroy_publisher(pub)

    def publish_status(self, state: NavPhase | str) -> None:
        s = state.value if isinstance(state, NavPhase) else str(state)
        msg = String()
        msg.data = s
        self.status_pub.publish(msg)
        write_nav2_state(
            self.runtime_dir,
            session_id=self.goal.session_id,
            state=s,
            map_yaml=self.goal.map_yaml,
            candidate_id=self.goal.candidate_id,
            goal_x=self.effective_goal_x,
            goal_y=self.effective_goal_y,
            goal_yaw=self.effective_goal_yaw,
            extra={
                "raw_goal": {"x": self.raw_goal_x, "y": self.raw_goal_y},
                "effective_goal": {"x": self.effective_goal_x, "y": self.effective_goal_y},
            },
        )

    def get_robot_xy_yaw(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.3)
            )
            t = tf.transform.translation
            yaw = quaternion_to_yaw(tf.transform.rotation)
            return float(t.x), float(t.y), yaw
        except Exception:
            return None

    def _delete_all_markers(self) -> None:
        m = Marker()
        m.action = Marker.DELETEALL
        for pub in (
            self.start_marker_pub,
            self.goal_marker_pub,
            self.goal_label_pub,
            self.trajectory_pub,
        ):
            pub.publish(m)

    def _publish_ring_marker(
        self, pub, ns: str, x: float, y: float, color: Tuple[float, float, float, float], text: str = ""
    ) -> None:
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = color
        for i in range(33):
            ang = 2.0 * math.pi * i / 32.0
            p = Point()
            p.x = x + 0.13 * math.cos(ang)
            p.y = y + 0.13 * math.sin(ang)
            m.points.append(p)
        pub.publish(m)
        if text:
            t = Marker()
            t.header.frame_id = "map"
            t.header.stamp = m.header.stamp
            t.ns = ns + "_label"
            t.id = 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x
            t.pose.position.y = y + 0.2
            t.pose.position.z = 0.1
            t.scale.z = 0.12
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = text
            self.goal_label_pub.publish(t)

    def build_target_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def wait_action_servers(self) -> bool:
        if not self.path_client.wait_for_server(timeout_sec=15.0):
            self.log("ComputePathToPose unavailable")
            return False
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            self.log("NavigateToPose unavailable")
            return False
        return True

    def compute_path(self, target: PoseStamped, robot_xy: Tuple[float, float]) -> Optional[Path]:
        goal = ComputePathToPose.Goal()
        goal.goal = target
        if hasattr(goal, "use_start"):
            goal.use_start = False
        send_fut = self.path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=8.0)
        if not send_fut.done():
            self.log("ComputePathToPose send timeout")
            return None
        handle = send_fut.result()
        if handle is None or not handle.accepted:
            self.log("ComputePathToPose rejected")
            return None
        result_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut, timeout_sec=20.0)
        if not result_fut.done():
            self.log("ComputePathToPose result timeout")
            return None
        path = result_fut.result().result.path
        max_len = compute_max_path_length_m(
            robot_xy[0], robot_xy[1], self.effective_goal_x, self.effective_goal_y
        )
        ok, reason = validate_planned_path(
            path.poses,
            robot_x=robot_xy[0],
            robot_y=robot_xy[1],
            goal_x=self.effective_goal_x,
            goal_y=self.effective_goal_y,
            max_path_m=max_len,
            path_frame=path.header.frame_id,
        )
        if not ok:
            self.log(f"path validation FAIL: {reason}")
            return None
        if self._map_grid is not None:
            safe, why = path_stays_in_known_free(self._map_grid, path)
            if not safe:
                self.log(f"path safety FAIL: {why}")
                return None
        return path

    def _publish_path_viz(self, path: Path, robot_xy: Tuple[float, float]) -> None:
        self.path_pub.publish(path)
        line = Marker()
        line.header = path.header
        line.ns = "qwen_session_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.04
        line.color.r = 0.1
        line.color.g = 0.85
        line.color.b = 0.2
        line.color.a = 0.9
        for ps in path.poses:
            line.points.append(ps.pose.position)
        self.trajectory_pub.publish(line)
        self._publish_ring_marker(
            self.start_marker_pub, "start", robot_xy[0], robot_xy[1], (0.2, 0.9, 0.2, 0.9), "start"
        )
        self._publish_ring_marker(
            self.goal_marker_pub,
            "goal",
            self.effective_goal_x,
            self.effective_goal_y,
            (1.0, 0.45, 0.1, 0.9),
            f"goal c{self.goal.candidate_id}",
        )

    def navigate(self, target: PoseStamped, path: Path) -> NavPhase:
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = target
        send_fut = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self._on_nav_feedback,
        )
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
        if not send_fut.done():
            self.log("NavigateToPose send timeout")
            return NavPhase.FAILED
        self._nav_goal_handle = send_fut.result()
        if self._nav_goal_handle is None or not self._nav_goal_handle.accepted:
            self.log("NavigateToPose rejected")
            return NavPhase.FAILED
        self.publish_status(NavPhase.GOAL_ACCEPTED)
        self.publish_status(NavPhase.NAVIGATING)
        self.log("NAVIGATING")

        path_len = path_length_m(path.poses)
        nominal_speed = 0.04
        timeout_s = min(300.0, max(90.0, path_len / nominal_speed * 2.5))
        nav_start = time.monotonic()
        self._progress = ProgressWatchState(last_progress_time=nav_start)
        robot0 = self.get_robot_xy_yaw()
        if robot0:
            self._progress.last_progress_pose = (robot0[0], robot0[1])
        result_fut = self._nav_goal_handle.get_result_async()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.monotonic()
            if self._sensor_watchdog_fail():
                self._cancel_nav()
                return NavPhase.SENSOR_LOST
            if progress_timed_out(self._progress, now, timeout_s=30.0):
                self._cancel_nav()
                self.log("NO_PROGRESS: 30s without distance/pose progress")
                return NavPhase.NO_PROGRESS
            if self._recoveries > 4:
                self._cancel_nav()
                self.log("too many recoveries")
                return NavPhase.FAILED
            if now - nav_start > timeout_s:
                self._cancel_nav()
                self.log(f"navigation timeout {timeout_s:.0f}s")
                return NavPhase.FAILED
            if result_fut.done():
                break
            if self._nav_launch_pid and not _pid_alive(self._nav_launch_pid):
                self._cancel_nav()
                return NavPhase.SENSOR_LOST

        if not result_fut.done():
            self._cancel_nav()
            return NavPhase.FAILED

        status = result_fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return NavPhase.SUCCEEDED
        if status == GoalStatus.STATUS_CANCELED:
            return NavPhase.CANCELED
        return NavPhase.FAILED

    def _on_nav_feedback(self, fb) -> None:
        feedback = fb.feedback
        now = time.monotonic()
        dist = float(feedback.distance_remaining)
        self._recoveries = int(feedback.number_of_recoveries)
        robot = self.get_robot_xy_yaw()
        robot_xy = (robot[0], robot[1]) if robot else None
        update_progress_watch(
            self._progress,
            now=now,
            distance_remaining=dist,
            robot_xy=robot_xy,
        )
        if now - self._last_feedback_log < 1.0:
            return
        self._last_feedback_log = now
        pos = f"robot=({robot[0]:.2f},{robot[1]:.2f})" if robot else "robot=?"
        self.log(
            f"feedback dist={dist:.2f}m best={self._progress.best_distance_remaining} "
            f"recoveries={self._recoveries} {pos}"
        )

    def _sensor_watchdog_fail(self) -> bool:
        now = time.monotonic()
        if self._last_scan_ts and now - self._last_scan_ts > 2.0:
            return True
        if self._last_odom_ts and now - self._last_odom_ts > 1.0:
            return True
        try:
            self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            return True
        return False

    def _cancel_nav(self) -> None:
        if self._nav_goal_handle is not None:
            cancel_fut = self._nav_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_fut, timeout_sec=3.0)
            # brief wait for controller to stop before zeroing cmd_vel
            time.sleep(0.3)

    def run(self) -> int:
        try:
            self.log("TARGET_RECEIVED")
            self.log(f"candidate_id={self.goal.candidate_id}")
            self.log(
                f"goal map=({self.raw_goal_x:.3f}, {self.raw_goal_y:.3f}, "
                f"{math.degrees(self.goal.goal_yaw):.1f}°)"
            )
            self.log("session/map/bundle verification PASS")
            self.publish_status(NavPhase.TARGET_RECEIVED)
            self._delete_all_markers()

            if self.args.start_only:
                self.log("start-only: skip path and navigation")
                return 0

            if not self.wait_action_servers():
                self.publish_status(NavPhase.FAILED)
                return 3

            robot = None
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                robot = self.get_robot_xy_yaw()
                if robot:
                    break
            if robot is None:
                self.log("map->base_link unavailable for path start")
                self.publish_status(NavPhase.FAILED)
                return 3

            gx, gy = self.raw_goal_x, self.raw_goal_y
            if self._map_grid is not None:
                try:
                    gx, gy, why = validate_goal_on_map(
                        self._map_grid, gx, gy, robot[0], robot[1], map_yaml=self.goal.map_yaml
                    )
                    if why != "ok":
                        self.log(f"goal projection: {why}")
                except ValueError as exc:
                    self.log(f"goal map validation FAIL: {exc}")
                    self.publish_status(NavPhase.FAILED)
                    return 2
            self.effective_goal_x = gx
            self.effective_goal_y = gy
            self.log(
                f"effective_goal=({self.effective_goal_x:.3f},{self.effective_goal_y:.3f}) "
                f"raw=({self.raw_goal_x:.3f},{self.raw_goal_y:.3f})"
            )

            target = self.build_target_pose(
                self.effective_goal_x, self.effective_goal_y, self.effective_goal_yaw
            )
            path = self.compute_path(target, (robot[0], robot[1]))
            if path is None:
                self.publish_status(NavPhase.FAILED)
                return 6

            self._publish_path_viz(path, (robot[0], robot[1]))
            planned = {
                "session_id": self.goal.session_id,
                "candidate_id": self.goal.candidate_id,
                "path_length_m": path_length_m(path.poses),
                "pose_count": len(path.poses),
                "raw_goal": {"x": self.raw_goal_x, "y": self.raw_goal_y},
                "effective_goal": {
                    "x": self.effective_goal_x,
                    "y": self.effective_goal_y,
                    "yaw_deg": math.degrees(self.effective_goal_yaw),
                },
            }
            atomic_write_json(self.runtime_dir / "planned_path.json", planned)
            self.publish_status(NavPhase.PATH_VALIDATED)
            self.log(f"PATH_VALIDATED length={planned['path_length_m']:.2f}m")

            if self.args.compute_path_only or self.args.start_only:
                self.log("compute-path-only / start-only: skip NavigateToPose")
                return 0

            result = self.navigate(target, path)
            # fault path: cancel already done; then zero
            self.publish_zero_velocity(1.0 if result != NavPhase.SUCCEEDED else 0.5)
            self.publish_status(result)

            final = self.get_robot_xy_yaw()
            payload = {
                "session_id": self.goal.session_id,
                "state": result.value,
                "finished_epoch": time_now(),
                "goal_error_m": None,
                "final_pose": None,
                "raw_goal": {"x": self.raw_goal_x, "y": self.raw_goal_y},
                "effective_goal": {"x": self.effective_goal_x, "y": self.effective_goal_y},
            }
            if final:
                err = math.hypot(final[0] - self.effective_goal_x, final[1] - self.effective_goal_y)
                payload["goal_error_m"] = round(err, 3)
                payload["final_pose"] = {
                    "x": final[0],
                    "y": final[1],
                    "yaw_deg": math.degrees(final[2]),
                }
            atomic_write_json(self.runtime_dir / "nav2_result.json", payload)
            if result == NavPhase.SUCCEEDED:
                self.log("SUCCEEDED")
                return 0
            self.log(f"FAILED state={result.value}")
            return 5
        finally:
            # ensure stop on all exit paths; short pulse only
            try:
                self.publish_zero_velocity(0.5)
            except Exception:
                pass


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Qwen Nav2 goal executor")
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--pose-json", required=True)
    parser.add_argument("--goal-json", required=True)
    parser.add_argument("--candidate-bundle", required=True)
    parser.add_argument("--runtime-dir", required=True)
    parser.add_argument("--nav-launch-pid", default="")
    parser.add_argument("--start-only", action="store_true")
    parser.add_argument("--compute-path-only", action="store_true")
    args = parser.parse_args()

    try:
        goal, _ = validate_goal_inputs(
            session_id=args.session_id,
            map_yaml=realpath(Path(args.map_yaml)),
            goal_json=realpath(Path(args.goal_json)),
            candidate_bundle=realpath(Path(args.candidate_bundle)),
            pose_json=realpath(Path(args.pose_json)),
        )
    except ValueError as exc:
        print(f"[QWEN_NAV] validation FAIL: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = QwenNav2GoalExecutor(args, goal)
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
