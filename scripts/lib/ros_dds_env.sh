#!/usr/bin/env bash
# Fast DDS: disable shared-memory transport and clear stale /dev/shm segments.
# Hobot camera uses its own shm_fastdds.xml; other nodes should use UDP-only here.

# shellcheck source=scripts/lib/cleanup_lidar_slam_nav.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/cleanup_lidar_slam_nav.sh"

export_ros_dds_env() {
  local project_dir="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
  export FASTRTPS_DEFAULT_PROFILES="${project_dir}/configs/fastdds_no_shm.xml"
}

prepare_ros_dds_env() {
  export_ros_dds_env
  cleanup_ros2_fastrtps_shm
}
