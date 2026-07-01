# Arrow Calibration Tools for RDK X5 VLN Robot

## Purpose

These files add a **non-destructive visualization calibration layer** for Foxglove/RViz arrows.

They do **not** modify:

- `/odom`
- `odom -> base_link`
- `base_link -> laser`
- `map -> odom`
- `lidar/config/*.yaml`
- `ros2_bridge/m1_pwm_cmd_vel_bridge.py`
- `scripts/lib/run_chassis_bridge.sh`

They add only:

- `/arrow_calibration_markers`
- `base_forward_calibrated` TF child frame
- `laser_forward_calibrated` TF child frame
- `configs/arrow_calibration.env`

This is deliberate: if the map is already correct, changing the core TF can break SLAM. These tools keep the mapping chain intact and correct only the displayed arrows.

## Files

```text
ros2_bridge/arrow_calibration_probe.py
ros2_bridge/arrow_display_markers.py
scripts/slam/run_arrow_calibrated_joy_mapping.sh
scripts/slam/run_arrow_display_only.sh
install_arrow_tools.sh
docs/README_ARROW_CALIBRATION.md
```

## Install

Unzip this package at the project root:

```bash
cd /root/rdk_x5_vln_robot
unzip /path/to/arrow_calibration_pack.zip
bash install_arrow_tools.sh
```

## One-command calibrated mapping

```bash
cd /root/rdk_x5_vln_robot
bash scripts/slam/run_arrow_calibrated_joy_mapping.sh
```

Default behavior:

1. Starts the existing SLAM live stack.
2. Waits for `/scan`, `/odom`, `/tf`.
3. Commands a slow forward movement until `/odom` displacement reaches about 1 m.
4. Computes `BASE_ARROW_YAW_OFFSET = atan2(dy, dx) - odom_yaw`.
5. Saves the result to `configs/arrow_calibration.env`.
6. Starts calibrated visualization arrows.
7. Starts joystick mapping.
8. Pressing `Ctrl+C` saves the map and stops the robot.

## Foxglove

Connect to:

```text
ws://<robot-ip>:8765
```

Enable:

```text
/map
/scan_filtered
/odom
/tf
/tf_static
/arrow_calibration_markers
```

Useful display frames:

```text
base_forward_calibrated
laser_forward_calibrated
```

## Optional parameters

```bash
TARGET_DISTANCE=0.6 CALIB_SPEED=0.035 bash scripts/slam/run_arrow_calibrated_joy_mapping.sh
```

Use a different base SLAM script:

```bash
SLAM_BASE_SCRIPT=/root/rdk_x5_vln_robot/scripts/slam/run_joy_mapping_calibrated.sh \
  bash scripts/slam/run_arrow_calibrated_joy_mapping.sh
```

## Important limitation

The probe can infer **the direction of positive forward command in `/odom` coordinates**.
It cannot magically know the physical front of the chassis without a trusted external reference. It therefore solves the safe part automatically: it aligns visualization arrows with the measured forward displacement while leaving the working SLAM TF chain untouched.
