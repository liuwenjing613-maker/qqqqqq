"""Foxglove markers for online MAP/Qwen candidate visualization."""
from __future__ import annotations

import math
from typing import Optional, Sequence

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from fusion.live_frontier_backend_core_v2 import FrontierCandidate


def _stamp_msg(stamp) -> object:
    return stamp


def delete_all_marker(frame_id: str, stamp, namespace: str) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = 0
    marker.action = Marker.DELETEALL
    return marker


def build_candidate_markers(
    candidates: Sequence[FrontierCandidate],
    *,
    frame_id: str,
    stamp,
    selected_id: Optional[str] = None,
) -> MarkerArray:
    """Yellow numbered spheres for geometry candidates; red highlight for Qwen pick."""
    array = MarkerArray()
    array.markers.append(delete_all_marker(frame_id, stamp, "map_qwen_candidates"))

    for index, candidate in enumerate(candidates, start=1):
        selected = selected_id is not None and candidate.candidate_id == selected_id
        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = stamp
        sphere.ns = "map_qwen_candidates"
        sphere.id = index
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(candidate.x)
        sphere.pose.position.y = float(candidate.y)
        sphere.pose.position.z = 0.14 if not selected else 0.20
        diameter = 0.30 if not selected else 0.38
        sphere.scale.x = sphere.scale.y = sphere.scale.z = diameter
        if selected:
            sphere.color = ColorRGBA(r=1.0, g=0.12, b=0.12, a=0.98)
        else:
            sphere.color = ColorRGBA(r=1.0, g=0.88, b=0.10, a=0.92)
        array.markers.append(sphere)

        label = Marker()
        label.header.frame_id = frame_id
        label.header.stamp = stamp
        label.ns = "map_qwen_candidates"
        label.id = 1000 + index
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(candidate.x)
        label.pose.position.y = float(candidate.y)
        label.pose.position.z = 0.42 if selected else 0.36
        label.scale.z = 0.18 if selected else 0.16
        if selected:
            label.color = ColorRGBA(r=1.0, g=0.85, b=0.85, a=1.0)
            label.text = f"QWEN {index}:{candidate.candidate_id}"
        else:
            label.color = ColorRGBA(r=1.0, g=0.95, b=0.55, a=1.0)
            label.text = f"C{index}"
        array.markers.append(label)

    return array


def build_selected_goal_markers(
    candidate: FrontierCandidate,
    *,
    frame_id: str,
    stamp,
) -> MarkerArray:
    """Red goal sphere + yaw arrow for the final navigation target."""
    array = MarkerArray()
    array.markers.append(delete_all_marker(frame_id, stamp, "map_qwen_goal"))

    sphere = Marker()
    sphere.header.frame_id = frame_id
    sphere.header.stamp = stamp
    sphere.ns = "map_qwen_goal"
    sphere.id = 1
    sphere.type = Marker.SPHERE
    sphere.action = Marker.ADD
    sphere.pose.position.x = float(candidate.x)
    sphere.pose.position.y = float(candidate.y)
    sphere.pose.position.z = 0.22
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.34
    sphere.color = ColorRGBA(r=1.0, g=0.10, b=0.10, a=0.98)
    array.markers.append(sphere)

    arrow = Marker()
    arrow.header.frame_id = frame_id
    arrow.header.stamp = stamp
    arrow.ns = "map_qwen_goal"
    arrow.id = 2
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    yaw = float(candidate.yaw)
    arrow.points.append(
        Point(x=float(candidate.x), y=float(candidate.y), z=0.22)
    )
    arrow.points.append(
        Point(
            x=float(candidate.x) + 0.55 * math.cos(yaw),
            y=float(candidate.y) + 0.55 * math.sin(yaw),
            z=0.22,
        )
    )
    arrow.scale.x = 0.08
    arrow.scale.y = 0.14
    arrow.scale.z = 0.18
    arrow.color = ColorRGBA(r=1.0, g=0.25, b=0.20, a=0.98)
    array.markers.append(arrow)

    label = Marker()
    label.header.frame_id = frame_id
    label.header.stamp = stamp
    label.ns = "map_qwen_goal"
    label.id = 3
    label.type = Marker.TEXT_VIEW_FACING
    label.action = Marker.ADD
    label.pose.position.x = float(candidate.x)
    label.pose.position.y = float(candidate.y)
    label.pose.position.z = 0.52
    label.scale.z = 0.20
    label.color = ColorRGBA(r=1.0, g=0.90, b=0.90, a=1.0)
    label.text = f"GOAL {candidate.candidate_id}"
    array.markers.append(label)

    return array


def build_clear_markers(frame_id: str, stamp) -> MarkerArray:
    array = MarkerArray()
    array.markers.append(delete_all_marker(frame_id, stamp, "map_qwen_candidates"))
    array.markers.append(delete_all_marker(frame_id, stamp, "map_qwen_goal"))
    return array
