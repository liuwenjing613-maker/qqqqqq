#!/usr/bin/env bash
# Unified LiDAR mount parameters: base_link -> laser static TF only.
# Physical LiDAR forward mount: LASER_YAW should normally be 0.0.

export LASER_X="${LASER_X:-0.10}"
export LASER_Y="${LASER_Y:-0.0}"
export LASER_Z="${LASER_Z:-0.12}"
export LASER_ROLL="${LASER_ROLL:-0.0}"
export LASER_PITCH="${LASER_PITCH:-0.0}"
export LASER_YAW="${LASER_YAW:-0.0}"
export LASER_FRAME="${LASER_FRAME:-laser}"

echo "[lidar_frame_config] base_link -> ${LASER_FRAME}"
echo "[lidar_frame_config] xyz=(${LASER_X}, ${LASER_Y}, ${LASER_Z}) rpy=(${LASER_ROLL}, ${LASER_PITCH}, ${LASER_YAW})"
if awk -v y="${LASER_YAW}" 'BEGIN { exit !(y == 0 || y == 0.0) }'; then
  echo "[lidar_frame_config] EXPECTED: physical LiDAR is mounted forward; LASER_YAW=0.0"
else
  echo "[lidar_frame_config] WARN: LASER_YAW=${LASER_YAW} (non-zero compensation enabled)"
fi
