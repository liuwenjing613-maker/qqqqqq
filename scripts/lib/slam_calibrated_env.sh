#!/usr/bin/env bash
# SLAM 建图专用校准参数（motor_trims + odom scale + 运动限速）
# 用法: source scripts/lib/slam_calibrated_env.sh

export PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
export CHASSIS_DEV="${CHASSIS_DEV:-/dev/rosmaster}"

# ===== 底盘控制参数（影响真实小车运动）=====
export CHASSIS_MAX_VX=0.04
export CHASSIS_MAX_WZ=0.10

export CHASSIS_VX_PWM_DEADBAND=10.0
export CHASSIS_VX_PWM_GAIN=200.0
export CHASSIS_WZ_PWM_DEADBAND=10.0
export CHASSIS_WZ_PWM_GAIN=150.0
export CHASSIS_PWM_MAX=40.0
export CHASSIS_MAX_PWM_DELTA=5.0
export CHASSIS_PWM_SMOOTH_ALPHA=0.25

# 已验证：左侧 M1/M2 需要增强，直线明显改善
export CHASSIS_MOTOR_TRIMS="1.15,1.15,1.0,1.0"

# ===== odom 校准参数（不改变真实运动，只修正 /odom）=====
export CHASSIS_ODOM_VXY_DEADZONE=0.003
export CHASSIS_ODOM_WZ_DEADZONE=0.015
export CHASSIS_ODOM_VY_SCALE=1.0

export CHASSIS_ODOM_VX_SCALE=0.68
export CHASSIS_ODOM_WZ_SCALE=-0.58
export CHASSIS_BASE_YAW_OFFSET=3.141592653589793
export CHASSIS_ODOM_XY_YAW_OFFSET=3.141592653589793
export CHASSIS_ODOM_USE_VY=0

# 手柄遥操作限速，与 CHASSIS_MAX_VX/WZ 对齐
export JOY_SCALE_LINEAR_X="${JOY_SCALE_LINEAR_X:-0.04}"
export JOY_SCALE_ANGULAR_YAW="${JOY_SCALE_ANGULAR_YAW:-0.10}"
