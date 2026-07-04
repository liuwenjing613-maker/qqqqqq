# Launch wrapper for Microdia USB 2.0 Camera on RDK X5.
# Stock hobot_usb_cam defaults (960x480) cause "Select timeout" on this device.
# Verified stable profile: 1280x720 MJPEG @ 20fps.

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    calibration_file = os.path.join(
        project_dir, "config", "usb_camera_calibration.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "usb_video_device",
            default_value="/dev/video0",
            description="USB camera device path",
        ),
        DeclareLaunchArgument("usb_image_width", default_value="1280"),
        DeclareLaunchArgument("usb_image_height", default_value="720"),
        DeclareLaunchArgument("usb_framerate", default_value="20"),
        DeclareLaunchArgument("usb_pixel_format", default_value="mjpeg"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("hobot_usb_cam"),
                    "launch/hobot_usb_cam.launch.py",
                )
            ),
            launch_arguments={
                "usb_video_device": LaunchConfiguration("usb_video_device"),
                "usb_camera_calibration_file_path": calibration_file,
                "usb_image_width": LaunchConfiguration("usb_image_width"),
                "usb_image_height": LaunchConfiguration("usb_image_height"),
                "usb_framerate": LaunchConfiguration("usb_framerate"),
                "usb_pixel_format": LaunchConfiguration("usb_pixel_format"),
            }.items(),
        ),
    ])
