#!/usr/bin/env python3
"""
Mapping-oriented LaserScan filter.

Purpose:
  /scan -> /scan_filtered for SLAM mapping.

It filters:
  1. invalid measurements: 0.0 / NaN / inf
  2. too near points: likely robot body / cable / near-field noise
  3. too far points: unstable far returns in indoor mapping
  4. isolated single-beam outliers: points not supported by nearby beams

Important:
  This is intended for SLAM mapping.
  Do not assume this is the final filter for Nav2 local obstacle avoidance.
"""

import math
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SimpleScanFilter(Node):
    def __init__(
        self,
        in_topic: str,
        out_topic: str,
        min_range: float,
        max_range: float,
        isolated_window: int,
        isolated_delta: float,
        min_support_neighbors: int,
        stats_every: int,
    ):
        super().__init__("simple_scan_filter")

        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.isolated_window = int(isolated_window)
        self.isolated_delta = float(isolated_delta)
        self.min_support_neighbors = int(min_support_neighbors)
        self.stats_every = int(stats_every)
        self.frame_count = 0

        self.pub = self.create_publisher(
            LaserScan,
            out_topic,
            qos_profile_sensor_data,
        )

        self.sub = self.create_subscription(
            LaserScan,
            in_topic,
            self.cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "filter "
            f"{in_topic} -> {out_topic}, "
            f"keep=[{self.min_range:.2f}, {self.max_range:.2f}], "
            f"isolated_window={self.isolated_window}, "
            f"isolated_delta={self.isolated_delta:.2f}, "
            f"min_support_neighbors={self.min_support_neighbors}"
        )

    def basic_valid(self, r: float) -> bool:
        if r == 0.0:
            return False
        if math.isnan(r):
            return False
        if math.isinf(r):
            return False
        if r < self.min_range:
            return False
        if r > self.max_range:
            return False
        return True

    def cb(self, msg: LaserScan) -> None:
        raw = list(msg.ranges)
        n = len(raw)

        # Layer 1: basic validity filtering.
        basic = []
        for r in raw:
            if self.basic_valid(float(r)):
                basic.append(float(r))
            else:
                basic.append(float("inf"))

        # Layer 2: isolated outlier rejection.
        # A valid point is kept if at least N neighbor beams within +/- window
        # have similar range. This removes single-beam spikes but keeps real clusters.
        filtered = list(basic)
        removed_isolated = 0

        if self.min_support_neighbors > 0 and self.isolated_window > 0:
            for i, r in enumerate(basic):
                if not math.isfinite(r):
                    continue

                support = 0

                for offset in range(-self.isolated_window, self.isolated_window + 1):
                    if offset == 0:
                        continue

                    j = i + offset
                    if j < 0:
                        j += n
                    elif j >= n:
                        j -= n

                    nr = basic[j]
                    if not math.isfinite(nr):
                        continue

                    if abs(nr - r) <= self.isolated_delta:
                        support += 1

                if support < self.min_support_neighbors:
                    filtered[i] = float("inf")
                    removed_isolated += 1

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = self.min_range
        out.range_max = self.max_range
        out.ranges = filtered
        out.intensities = msg.intensities

        self.pub.publish(out)

        self.frame_count += 1
        if self.stats_every > 0 and self.frame_count % self.stats_every == 0:
            zeros = sum(1 for v in raw if v == 0.0)
            nan = sum(1 for v in raw if math.isnan(float(v)))
            inf = sum(1 for v in raw if math.isinf(float(v)))
            finite_raw = sum(1 for v in raw if math.isfinite(float(v)) and v != 0.0)
            finite_out = sum(1 for v in filtered if math.isfinite(v))
            self.get_logger().info(
                f"scan stats: n={n}, zeros={zeros}, nan={nan}, inf={inf}, "
                f"finite_raw={finite_raw}, finite_out={finite_out}, "
                f"removed_isolated={removed_isolated}"
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-topic", default="/scan")
    parser.add_argument("--out-topic", default="/scan_filtered")

    # Conservative defaults for indoor SLAM mapping.
    # If too much wall is lost, increase max_range to 5.0 or reduce min_range to 0.18.
    parser.add_argument("--min-range", type=float, default=0.18)
    parser.add_argument("--max-range", type=float, default=4.0)

    # Isolated point filtering.
    # min_support_neighbors=1 means one nearby supporting beam is enough to keep the point.
    # This is intentionally not aggressive, to avoid deleting small real objects too easily.
    parser.add_argument("--isolated-window", type=int, default=2)
    parser.add_argument("--isolated-delta", type=float, default=0.25)
    parser.add_argument("--min-support-neighbors", type=int, default=1)

    parser.add_argument("--stats-every", type=int, default=50)

    args = parser.parse_args()

    rclpy.init()
    node = SimpleScanFilter(
        in_topic=args.in_topic,
        out_topic=args.out_topic,
        min_range=args.min_range,
        max_range=args.max_range,
        isolated_window=args.isolated_window,
        isolated_delta=args.isolated_delta,
        min_support_neighbors=args.min_support_neighbors,
        stats_every=args.stats_every,
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
