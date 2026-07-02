# Semantic Explore Navigation Design

## Overview

Semantic explore navigation extends the stable YOLO + LiDAR visual servo stack with:

- Live SLAM (`run_slam_calibrated.sh`)
- Real-time semantic mapping (`semantic_mapper_node.py`)
- Active exploration goal selection (`explore_goal_selector.py`)
- Semantic-guided SEARCH in `run_shared_nav_semantic_explore.py`

## Priority Order

1. Lidar safety (BLOCKED)
2. Visible target (CANDIDATE_LOCK / TRACK)
3. Lost recovery scan
4. `/explore_goal_hint` semantic explore
5. Free-space / spin fallback

## Key Topics

| Topic | Producer | Consumer |
|-------|----------|----------|
| `/semantic_map_json` | semantic_mapper | explore_goal_selector |
| `/explore_goal_hint` | explore_goal_selector | run_shared_nav_semantic_explore |
| `/explore_state_json` | explore_goal_selector | Foxglove |
| `/nav_state` | nav app | monitoring |

## Launch

```bash
bash scripts/nav/start_yolo_lidar_semantic_explore_nav.sh \
  configs/nav_yolo_lidar_semantic_explore.yaml \
  "find the bottle"
```

## Config

Unified config: `configs/nav_yolo_lidar_semantic_explore.yaml`

Switches:

- `semantic_explore.enabled`
- `qwen_text.enabled` (default false)
- `planner.mode` (`bearing_first` default, `astar` optional)

## Protected Files

Do not modify stable nav / joy semantic mapping scripts. All explore logic lives in new `_semantic_explore` files under `src/planning/`, `src/apps/run_shared_nav_semantic_explore.py`, and `scripts/nav/start_yolo_lidar_semantic_explore_nav.sh`.
