#!/usr/bin/env bash

source /opt/tros/humble/setup.bash

if [ -f ~/rdk_x5_vln_robot/source_stage10.sh ]; then
  source ~/rdk_x5_vln_robot/source_stage10.sh
fi

echo "============================================================"
echo " ROS Topic Check"
echo "============================================================"

echo
echo "[Topic list]"
ros2 topic list

echo
echo "[/image]"
ros2 topic info /image || true

echo
echo "[/image_raw]"
ros2 topic info /image_raw || true

echo
echo "[/cmd_vel]"
ros2 topic info /cmd_vel || true

echo
echo "[/target_words]"
ros2 topic info /target_words || true

echo
echo "[/hobot_yolo_world]"
ros2 topic info /hobot_yolo_world || true

echo
echo "============================================================"
echo " If camera and bridge are running, expected:"
echo " /image       : sensor_msgs/msg/CompressedImage"
echo " /image_raw   : sensor_msgs/msg/Image"
echo " /cmd_vel     : geometry_msgs/msg/Twist"
echo "============================================================"
