#!/usr/bin/env bash
# Stop stale LiDAR/SLAM/Nav helper processes before starting a fresh stack.

cleanup_lidar_slam_nav_processes() {
  pkill -f "slam_toolbox" 2>/dev/null || true
  pkill -f "static_transform_publisher.*base_link.*laser" 2>/dev/null || true
  pkill -f "simple_scan_filter.py" 2>/dev/null || true
  pkill -f "foxglove_bridge" 2>/dev/null || true
}
