# Launch wrapper for Microdia USB 2.0 Camera on RDK X5.
# The stock hobot_usb_cam defaults (960x480 @ 30fps) are unsupported by this
# device and cause "Select timeout". Use 1280x720 @ 60fps (MJPEG) instead.

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
                "usb_image_width": "1280",
                "usb_image_height": "720",
                "usb_framerate": "60",
                "usb_pixel_format": "mjpeg",
            }.items(),
        ),
    ])
