#!/usr/bin/env python3
"""Wait until explore-nav TF chain is ready (map<-odom, odom<-base_link, map<-base_link)."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def _lookup_ok(buffer: Buffer, target: str, source: str, timeout_sec: float) -> bool:
    try:
        buffer.lookup_transform(
            target,
            source,
            Time(),
            timeout=Duration(seconds=timeout_sec),
        )
        return True
    except Exception:
        return False


def wait_link(
    buffer: Buffer,
    target: str,
    source: str,
    timeout_sec: float,
    need_ok: int,
    poll_sec: float,
) -> bool:
    label = f"{target} <- {source}"
    print(f"[WAIT] TF {label}, need {need_ok} consecutive OK", flush=True)
    ok_count = 0
    start = time.time()
    while time.time() - start < timeout_sec:
        if _lookup_ok(buffer, target, source, 0.35):
            ok_count += 1
            print(f"[WAIT] TF {label}: OK {ok_count}/{need_ok}", flush=True)
            if ok_count >= need_ok:
                print(f"[OK] TF {label} stable", flush=True)
                return True
        else:
            ok_count = 0
            print(f"[WAIT] TF {label}: not ready", flush=True)
        time.sleep(poll_sec)
    print(f"[ERROR] TF {label} not stable after {timeout_sec:.0f}s (last lookup failed)", flush=True)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Wait for semantic explore TF chain")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--need-ok", type=int, default=3)
    parser.add_argument("--poll", type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("wait_tf_chain")
    buffer = Buffer(cache_time=Duration(seconds=30.0))
    listener = TransformListener(buffer, node)

    # Let tf2 buffer fill after slam / chassis come up.
    warmup_deadline = time.time() + min(8.0, args.timeout * 0.15)
    while time.time() < warmup_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    per_link_timeout = max(30.0, args.timeout * 0.45)
    if not wait_link(
        buffer, args.map_frame, args.odom_frame, per_link_timeout, 2, args.poll
    ):
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not wait_link(
        buffer, args.odom_frame, args.base_frame, per_link_timeout, 2, args.poll
    ):
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Direct map<-base_link often fails with tf2_echo extrapolation even when the
    # chain is valid; use latest-time lookup here.
    if wait_link(
        buffer, args.map_frame, args.base_frame, per_link_timeout, args.need_ok, args.poll
    ):
        node.destroy_node()
        rclpy.shutdown()
        return 0

    chain_ok = (
        _lookup_ok(buffer, args.map_frame, args.odom_frame, 0.5)
        and _lookup_ok(buffer, args.odom_frame, args.base_frame, 0.5)
    )
    if chain_ok:
        print(
            f"[semantic_explore] WARN: direct {args.map_frame}<-{args.base_frame} "
            f"unstable; chain {args.map_frame}<-{args.odom_frame} + "
            f"{args.odom_frame}<-{args.base_frame} OK — continuing",
            flush=True,
        )
        node.destroy_node()
        rclpy.shutdown()
        return 0

    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    sys.exit(main())
