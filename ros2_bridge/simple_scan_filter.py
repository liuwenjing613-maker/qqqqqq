#!/usr/bin/env python3
import math
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SimpleScanFilter(Node):
    def __init__(self, in_topic, out_topic, min_range, max_range):
        super().__init__("simple_scan_filter")

        self.min_range = float(min_range)
        self.max_range = float(max_range)

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
            f"filter {in_topic} -> {out_topic}, keep [{self.min_range}, {self.max_range}]"
        )

    def cb(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = self.min_range
        out.range_max = self.max_range
        out.intensities = msg.intensities

        filtered = []

        for r in msg.ranges:
            if r == 0.0:
                filtered.append(float("inf"))
            elif math.isnan(r):
                filtered.append(float("inf"))
            elif r < self.min_range:
                filtered.append(float("inf"))
            elif r > self.max_range:
                filtered.append(float("inf"))
            else:
                filtered.append(r)

        out.ranges = filtered
        self.pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-topic", default="/scan")
    parser.add_argument("--out-topic", default="/scan_filtered")
    parser.add_argument("--min-range", type=float, default=0.18)
    parser.add_argument("--max-range", type=float, default=4.0)
    args = parser.parse_args()

    rclpy.init()
    node = SimpleScanFilter(
        args.in_topic,
        args.out_topic,
        args.min_range,
        args.max_range,
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
