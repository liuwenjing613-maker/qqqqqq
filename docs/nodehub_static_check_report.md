# NodeHub Static Check Report

- Root: /root/rdk_x5_vln_robot
- Time: 2026-07-09 13:19:37

- OK: `README.md`
- OK: `.env.example`
- OK: `configs`
- OK: `scripts`
- OK: `src`
- OK: `ros2_bridge`
- OK: `scripts/lib/project_dir.sh`
- OK: `scripts/lib/run_chassis_bridge.sh`
- OK: `scripts/system/stop_all_safe.sh`
- OK: `ros2_bridge/m1_pwm_cmd_vel_bridge.py`
- OK: `ros2_bridge/simple_scan_filter.py`
- ENTRY_OK: `scripts/nav/start_yolo_lidar_semantic_explore_nav.sh`
- ENTRY_MISSING: `scripts/nav/start_yolo_lidar_failsafe_nav.sh`
- ENTRY_OK: `scripts/nav/start_yolo_lidar_stable_nav.sh`
- ENTRY_MISSING: `scripts/nav/start_qwen_lidar_nav.sh`
- ENTRY_OK: `scripts/slam/run_joy_mapping_calibrated.sh`
- ENTRY_MISSING: `scripts/slam/run_joy_mapping_all.sh`
- ENTRY_OK: `scripts/slam/run_nav2_saved_map.sh`
- ENTRY_OK: `scripts/slam/run_nav2_foxglove_click_goal.sh`
- ENTRY_OK: `rdk_x5_qwen_vln_robot/scripts/nav/start_qwen_api_lidar_nav.sh`

## Summary

- MISSING=0
- BASH_ERROR=0
- PY_ERROR=0
- YAML_ERROR=0
- BIG_FILE=5
- BAD_FILE=1
- REAL_SECRET=0
- SIZE=280M
NODEHUB_STATIC_RESULT=PASS
