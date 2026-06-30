# SLAM calibration helper scripts

These two ROS2 scripts are for RDK X5 + Yahboom Rosmaster M1 SLAM calibration.

## Files

- `scripts/calib/rotate_sweep_test.py`
  - Publishes increasing `/cmd_vel.angular.z` values.
  - Logs `/odom`, `/cmd_vel_sent`, and `/chassis_bridge_state`.
  - You type `ok` when the robot rotation looks mechanically correct, especially when all four wheels participate.

- `scripts/calib/odom_calibration_wizard.py`
  - Does not drive the robot.
  - You manually drive the robot and tell it when a real test is completed.
  - Computes recommended deadzones and scale factors.

## Install on robot

Upload the zip to `~/rdk_x5_vln_robot/`, then run:

```bash
cd ~/rdk_x5_vln_robot
unzip -o slam_calib_files.zip
chmod +x scripts/calib/*.py
python3 -m py_compile scripts/calib/rotate_sweep_test.py
python3 -m py_compile scripts/calib/odom_calibration_wizard.py
```

## Start chassis bridge first

```bash
cd ~/rdk_x5_vln_robot
source /opt/tros/humble/setup.bash

pkill -f "m1_pwm_cmd_vel_bridge.py" 2>/dev/null || true

export PROJECT_DIR=~/rdk_x5_vln_robot
export CHASSIS_DEV=/dev/rosmaster

export CHASSIS_MAX_VX=0.04
export CHASSIS_MAX_WZ=0.20
export CHASSIS_WZ_PWM_DEADBAND=10.0
export CHASSIS_WZ_PWM_GAIN=150.0
export CHASSIS_PWM_MAX=40.0
export CHASSIS_MAX_PWM_DELTA=5.0
export CHASSIS_PWM_SMOOTH_ALPHA=0.25

source scripts/lib/run_chassis_bridge.sh
run_chassis_bridge logs/calib/chassis_bridge_calib.log
```

If `/dev/rosmaster` does not exist, use your actual serial port, for example `/dev/ttyUSB0`.

## Script 1: rotation sweep

```bash
cd ~/rdk_x5_vln_robot
source /opt/tros/humble/setup.bash

python3 scripts/calib/rotate_sweep_test.py \
  --stage-mode seconds \
  --seconds-per-stage 8 \
  --wz-list "0.06,0.08,0.10,0.12,0.14,0.16,0.18"
```

Interactive commands:

- `ok`: mark current stage as good
- `n`: next stage
- `p`: pause/resume
- `q`: stop and save

## Script 2: odom calibration wizard

```bash
cd ~/rdk_x5_vln_robot
source /opt/tros/humble/setup.bash

python3 scripts/calib/odom_calibration_wizard.py
```

Interactive tests:

```text
static 30
start rot
done rot 360
start line
done line 1.0
save
q
```

Results are saved under:

```text
logs/calib/
```

## Recommended order

1. Run `rotate_sweep_test.py` to find reliable rotation command/PWM parameters.
2. Run `odom_calibration_wizard.py` with `static 30`.
3. Run `start rot` / `done rot 360`.
4. Run `start line` / `done line 1.0`.
5. Apply the recommended environment variables to your chassis bridge or patch the bridge to accept scale/deadzone variables.
