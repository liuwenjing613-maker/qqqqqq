#!/usr/bin/env python3
"""Launch YDLidar T-MINI PLUS via ydlidar_ros2_driver."""

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

    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser",
        arguments=["0", "0", "0.02", "0", "0", "0", "1", "base_link", "laser"],
    )

    return LaunchDescription([params_declare, driver_node, tf_node])
