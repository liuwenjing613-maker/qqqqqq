#!/usr/bin/env python3
"""Idempotently add REUSE_BASE_STACK=1 support to the proven exp2 launcher.

The patch is intentionally tiny: it only skips exp2's own SLAM startup when a
complete shared /map,/odom,/scan_filtered stack is already provided by the hub.
Camera, detector, semantic mapper and explore nodes remain the original code.
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

MARKER = "# VOICE_DEMO_HUB_V1_REUSE_BASE_PATCH"
DDS_MARKER = "# VOICE_DEMO_HUB_V1_REUSE_DDS_ATTACH"


def _ensure_dds_attach(text: str) -> tuple[str, bool]:
    """When REUSE_BASE_STACK=1, attach DDS only — never wipe /dev/shm under live hub SLAM."""
    if DDS_MARKER in text:
        return text, False
    old = (
        "# shellcheck source=scripts/lib/ros_dds_env.sh\n"
        'source "${PWD}/scripts/lib/ros_dds_env.sh"\n'
        "prepare_ros_dds_env\n"
    )
    new = (
        "# shellcheck source=scripts/lib/ros_dds_env.sh\n"
        'source "${PWD}/scripts/lib/ros_dds_env.sh"\n'
        f"# Hub shared base is already live under REUSE_BASE_STACK=1; never wipe /dev/shm. {DDS_MARKER}\n"
        'if [ "${REUSE_BASE_STACK:-0}" = "1" ]; then\n'
        "  attach_ros_dds_env\n"
        "else\n"
        "  prepare_ros_dds_env\n"
        "fi\n"
    )
    if old not in text:
        # Already adapted manually (or source drifted); do not fail install.
        if "attach_ros_dds_env" in text and "REUSE_BASE_STACK" in text:
            return text, False
        raise RuntimeError("cannot find exp2 prepare_ros_dds_env block for DDS attach patch")
    return text.replace(old, new, 1), True


def patch(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    changed = False

    text, dds_changed = _ensure_dds_attach(text)
    changed = changed or dds_changed

    if MARKER not in text:
        var_needle = 'NAV_ONLY="${NAV_ONLY:-0}"\n'
        if var_needle not in text:
            raise RuntimeError("cannot find NAV_ONLY declaration; source version changed")
        text = text.replace(
            var_needle,
            var_needle + 'REUSE_BASE_STACK="${REUSE_BASE_STACK:-0}"  # ' + MARKER + "\n",
            1,
        )

        # Accept optional blank line between stop_stale_live_stack and shellcheck.
        start_needles = (
            "stop_stale_live_stack\n\n# shellcheck source=scripts/lib/slam_calibrated_env.sh\n",
            "stop_stale_live_stack\n# shellcheck source=scripts/lib/slam_calibrated_env.sh\n",
        )
        start_needle = next((n for n in start_needles if n in text), None)
        if start_needle is None:
            raise RuntimeError("cannot find exp2 SLAM start block")
        wrapper = '''if [ "$REUSE_BASE_STACK" = "1" ]; then
  echo "[semantic_explore] REUSE_BASE_STACK=1: reuse shared calibrated SLAM/sensors"
  wait_topic_exists /scan 30 || exit 1
  wait_topic_exists /scan_filtered 30 || exit 1
  wait_topic_exists /odom 30 || exit 1
  wait_topic_exists /map 30 || exit 1
  wait_topic_exists /tf 30 || exit 1
  wait_tf_before_explore_nodes || exit 1
else
'''
        text = text.replace(start_needle, wrapper + start_needle, 1)

        end_needle = "wait_tf_before_explore_nodes || exit 1\n\nstart_joy_control_stack_async\n"
        if end_needle not in text:
            raise RuntimeError("cannot find end of exp2 SLAM start block")
        text = text.replace(
            end_needle,
            "wait_tf_before_explore_nodes || exit 1\nfi\n\nstart_joy_control_stack_async\n",
            1,
        )
        changed = True

    if changed:
        path.write_text(text, encoding="utf-8")
    return changed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    path = args.path.resolve()
    if not path.is_file():
        raise SystemExit(f"missing launcher: {path}")
    if args.check:
        text = path.read_text(encoding="utf-8")
        if MARKER not in text:
            raise SystemExit("patch is not installed")
        print("[check] exp2 shared-base patch installed")
        return 0
    backup = path.with_name(path.name + ".before_voice_demo_hub_v1")
    if not backup.exists():
        shutil.copy2(path, backup)
    changed = patch(path)
    print("[patch] exp2 launcher patched" if changed else "[patch] already installed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
