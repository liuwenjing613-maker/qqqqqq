#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Inspect / optionally clear unowned Nav2 overlay processes.

Never kills sensor-base PGID members (lidar/filter/chassis/foxglove/slam/static_tf/mapping).
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_DIR = _SCRIPT_DIR.parents[1]
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from qwen_nav2_common import (  # noqa: E402
    atomic_write_json,
    pid_alive,
    read_proc_cmdline,
    read_proc_start_ticks,
    time_now,
)

NAV2_ROLE_PATTERNS: Dict[str, Tuple[str, ...]] = {
    "map_server": ("map_server",),
    "amcl": ("amcl",),
    "planner_server": ("planner_server",),
    "controller_server": ("controller_server",),
    "bt_navigator": ("bt_navigator",),
    "behavior_server": ("behavior_server",),
    "velocity_smoother": ("velocity_smoother",),
    "smoother_server": ("smoother_server",),
    "waypoint_follower": ("waypoint_follower",),
    "lifecycle_manager": ("lifecycle_manager",),
    "nav2_launch": (
        "nav2_click_nav_bringup",
        "nav2_bringup",
        "run_nav2_saved_map",
        "run_qwen_target_nav2_reuse",
    ),
}

PROTECTED_SUBSTRINGS = (
    "ydlidar",
    "simple_scan_filter",
    "m1_pwm_cmd_vel_bridge",
    "slam_toolbox",
    "foxglove_bridge",
    "static_transform_publisher",
    "run_joy_mapping",
    "run_corridor_mapping",
)


@dataclass
class ProcInfo:
    pid: int
    pgid: Optional[int]
    exe: str
    cmdline: str
    start_ticks: Optional[int]
    role: str


def _read_exe(pid: int) -> str:
    try:
        return os.readlink(f"/proc/{pid}/exe")
    except OSError:
        return ""


def _read_pgid(pid: int) -> Optional[int]:
    try:
        # /proc/PID/stat field 5 is pgrp
        fields = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()
        return int(fields[4])
    except (OSError, IndexError, ValueError):
        return None


def _classify_role(cmdline: str) -> Optional[str]:
    low = cmdline.lower()
    for role, pats in NAV2_ROLE_PATTERNS.items():
        for pat in pats:
            if pat.lower() in low:
                return role
    return None


def _is_protected(cmdline: str, exe: str) -> bool:
    blob = f"{cmdline} {exe}".lower()
    return any(s.lower() in blob for s in PROTECTED_SUBSTRINGS)


def list_nav2_procs() -> List[ProcInfo]:
    try:
        out = subprocess.check_output(["ps", "-eo", "pid=", "cmd="], text=True)
    except subprocess.CalledProcessError:
        return []
    found: List[ProcInfo] = []
    seen: Set[int] = set()
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) < 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        cmd = parts[1]
        role = _classify_role(cmd)
        if role is None:
            continue
        if pid in seen:
            continue
        seen.add(pid)
        found.append(
            ProcInfo(
                pid=pid,
                pgid=_read_pgid(pid),
                exe=_read_exe(pid),
                cmdline=cmd,
                start_ticks=read_proc_start_ticks(pid),
                role=role,
            )
        )
    return found


def group_by_pgid(procs: List[ProcInfo]) -> Dict[int, List[ProcInfo]]:
    groups: Dict[int, List[ProcInfo]] = {}
    for p in procs:
        key = int(p.pgid) if p.pgid is not None else int(p.pid)
        groups.setdefault(key, []).append(p)
    return groups


def pgid_has_protected(pgid: int) -> Tuple[bool, List[str]]:
    """Scan all processes in PGID for protected sensor/mapping roles."""
    protected: List[str] = []
    try:
        out = subprocess.check_output(["ps", "-eo", "pid=,pgid=,cmd="], text=True)
    except subprocess.CalledProcessError:
        return False, protected
    for line in out.splitlines():
        parts = line.split(None, 2)
        if len(parts) < 3:
            continue
        try:
            pid = int(parts[0])
            g = int(parts[1])
        except ValueError:
            continue
        if g != pgid:
            continue
        cmd = parts[2]
        exe = _read_exe(pid)
        if _is_protected(cmd, exe):
            protected.append(f"{pid}:{cmd[:120]}")
    return bool(protected), protected


def archive_stale_owner(owner_path: Path, runtime_dir: Path) -> Optional[Dict[str, Any]]:
    if not owner_path.is_file():
        return None
    try:
        owner = json.loads(owner_path.read_text(encoding="utf-8"))
    except Exception:
        return None
    launch_pid = owner.get("launch_pid")
    if launch_pid is not None and pid_alive(int(launch_pid)):
        ticks = owner.get("start_ticks")
        cur = read_proc_start_ticks(int(launch_pid))
        if ticks is None or cur is None or int(ticks) == int(cur):
            return None  # still valid
    stale_dir = runtime_dir / "stale_owners"
    stale_dir.mkdir(parents=True, exist_ok=True)
    stamp = int(time_now())
    dest = stale_dir / f"nav2_owner_{stamp}.json"
    owner["archived_epoch"] = time_now()
    owner["archive_reason"] = "pid_missing_or_reused"
    atomic_write_json(dest, owner)
    owner_path.unlink(missing_ok=True)
    return {"archived_to": str(dest), "owner": owner}


def cancel_nav_best_effort() -> None:
    try:
        import rclpy
        from action_msgs.srv import CancelGoal
        from rclpy.node import Node
        from unique_identifier_msgs.msg import UUID

        if not rclpy.ok():
            rclpy.init()
        node = Node("nav2_preflight_cancel")
        cli = node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        if cli.wait_for_service(timeout_sec=1.5):
            req = CancelGoal.Request()
            req.goal_info.goal_id = UUID(uuid=[0] * 16)
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


def clear_pgid(pgid: int) -> Dict[str, Any]:
    result = {"pgid": pgid, "term": False, "kill": False, "remaining": []}
    try:
        os.killpg(pgid, signal.SIGTERM)
        result["term"] = True
    except ProcessLookupError:
        pass
    except PermissionError as exc:
        result["error"] = str(exc)
        return result
    deadline = time.time() + 5.0
    while time.time() < deadline:
        alive, _ = pgid_members_alive(pgid)
        if not alive:
            break
        time.sleep(0.2)
    alive, members = pgid_members_alive(pgid)
    if alive:
        try:
            os.killpg(pgid, signal.SIGKILL)
            result["kill"] = True
        except ProcessLookupError:
            pass
        time.sleep(0.3)
        _, members = pgid_members_alive(pgid)
    result["remaining"] = members
    return result


def pgid_members_alive(pgid: int) -> Tuple[bool, List[str]]:
    members: List[str] = []
    try:
        out = subprocess.check_output(["ps", "-eo", "pid=,pgid=,cmd="], text=True)
    except subprocess.CalledProcessError:
        return False, members
    for line in out.splitlines():
        parts = line.split(None, 2)
        if len(parts) < 3:
            continue
        try:
            pid = int(parts[0])
            g = int(parts[1])
        except ValueError:
            continue
        if g == pgid and pid_alive(pid):
            members.append(f"{pid} {parts[2][:100]}")
    return bool(members), members


def inspect(runtime_dir: Optional[Path] = None) -> Dict[str, Any]:
    procs = list_nav2_procs()
    groups = group_by_pgid(procs)
    report_groups = []
    clearable = []
    blocked = []
    for pgid, members in groups.items():
        protected, protected_list = pgid_has_protected(pgid)
        entry = {
            "pgid": pgid,
            "members": [asdict(m) for m in members],
            "protected": protected,
            "protected_members": protected_list,
        }
        report_groups.append(entry)
        if protected:
            blocked.append(pgid)
        else:
            clearable.append(pgid)
    stale = None
    if runtime_dir is not None:
        stale = archive_stale_owner(runtime_dir / "nav2_owner.json", runtime_dir)
    return {
        "checked_epoch": time_now(),
        "nav2_proc_count": len(procs),
        "groups": report_groups,
        "clearable_pgids": clearable,
        "blocked_pgids": blocked,
        "stale_owner": stale,
        "clear_command": (
            f"python3 {_SCRIPT_DIR / 'nav2_overlay_preflight.py'} --clear-unowned"
        ),
    }


def clear_unowned(runtime_dir: Optional[Path] = None) -> Dict[str, Any]:
    report = inspect(runtime_dir)
    if report["blocked_pgids"]:
        report["status"] = "REFUSED_PROTECTED"
        report["message"] = "Refusing to clear PGID that contains sensor/mapping processes"
        return report
    if not report["clearable_pgids"] and report["nav2_proc_count"] == 0:
        report["status"] = "CLEAN"
        return report
    cancel_nav_best_effort()
    cleared = []
    for pgid in report["clearable_pgids"]:
        cleared.append(clear_pgid(int(pgid)))
    # re-scan
    after = inspect(runtime_dir)
    report["cleared"] = cleared
    report["after"] = after
    report["status"] = "CLEARED" if after["nav2_proc_count"] == 0 else "RESIDUAL"
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Nav2 overlay preflight")
    parser.add_argument("--inspect", action="store_true")
    parser.add_argument("--clear-unowned", action="store_true")
    parser.add_argument("--runtime-dir", default="")
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()
    if not args.inspect and not args.clear_unowned:
        args.inspect = True
    runtime_dir = Path(args.runtime_dir) if args.runtime_dir else None
    if args.clear_unowned:
        payload = clear_unowned(runtime_dir)
    else:
        payload = inspect(runtime_dir)
        payload["status"] = "CLEAN" if payload["nav2_proc_count"] == 0 else "FOUND"
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    print(text)
    if args.json_out:
        atomic_write_json(Path(args.json_out), payload)
    if args.clear_unowned:
        return 0 if payload.get("status") in ("CLEAN", "CLEARED") else 1
    if payload.get("nav2_proc_count", 0) > 0:
        print(
            "\n[PREFLIGHT] Unowned/ leftover Nav2 detected. "
            f"Run: {payload.get('clear_command')}",
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
