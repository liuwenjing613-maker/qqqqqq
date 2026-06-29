#!/usr/bin/env python3
"""Launch YDLidar T-MINI PLUS via ydlidar_ros2_driver.

Static TF base_link -> laser is NOT started here.
Use scripts/lib/lidar_frame_config.sh from shell startup scripts instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = os.path.expanduser("~/rdk_x5_vln_robot")
    default_params = os.path.join(project_dir, "lidar", "config", "tmini_plus.yaml")

    params_file = LaunchConfiguration("params_file")

    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to T-MINI PLUS ROS2 parameters file.",
    )

    driver_node = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
        namespace="/",
    )

    return LaunchDescription([params_declare, driver_node])
