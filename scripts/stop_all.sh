#!/usr/bin/env bash

echo "[STOP] stopping all MVP processes..."

pkill -f run_mvp_task.py
pkill -f red_target_servo_ros.py
pkill -f yolo_world_servo_ros.py
pkill -f keyboard_cmd_vel.py
pkill -f cmd_vel_to_rosmaster.py
pkill -f hobot_usb_cam
pkill -f websocket
pkill -f hobot_yolo_world

source /opt/tros/humble/setup.bash

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null

echo "[STOP] done."
