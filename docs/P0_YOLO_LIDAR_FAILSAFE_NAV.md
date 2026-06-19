# P0 YOLO + LiDAR Failsafe Navigation

YOLO-World bbox center visual servo + LiDAR free-space explore + emergency stop/rotate safety layer.

Qwen/Ollama is **not** used in this control loop.

## Quick start (full stack)

```bash
cd ~/rdk_x5_vln_robot
bash scripts/start_yolo_lidar_failsafe_nav.sh
```

Optional args:

```bash
bash scripts/start_yolo_lidar_failsafe_nav.sh configs/yolo_lidar_failsafe_nav.yaml bottle
```

Environment variables (aligned with `start_yolo_live_preview.sh`):

- `TARGET_WORDS=bottle,water bottle,cup`
- `TARGET_CLASSES=bottle,cup`
- `NAV_ONLY=1` — only start nav node (sensors/YOLO already running)
- `CHASSIS_PORT=/dev/ttyUSB0`
- `CAMERA_DEV=/dev/video0`

## Nav only (step-by-step debug)

```bash
# Terminal 1: sensors + YOLO (or use existing live preview chain)
# Terminal 2:
NAV_ONLY=1 bash scripts/start_yolo_lidar_failsafe_nav.sh
```

## Unit tests

```bash
python3 tests/test_free_space_waypoint.py
python3 tests/test_target_bbox_parser.py
```

## Fake bbox test (no YOLO)

```bash
python3 scripts/pub_fake_bbox.py --bbox 250 140 390 420 --class-name bottle
```

Left / right bias:

```bash
python3 scripts/pub_fake_bbox.py --bbox 80 140 220 420
python3 scripts/pub_fake_bbox.py --bbox 420 140 560 420
```

## Debug topics

| Topic | Purpose |
|-------|---------|
| `/failsafe_nav_state` | JSON state machine + front_distance + cmd |
| `/failsafe_nav_point` | Active target/free-space pixel u/v |
| `/target_bbox_json` | YOLO bridge output |
| `/cmd_vel` | Chassis velocity commands |

## Architecture

```
/image_raw + /hobot_yolo_world -> yolo_world_to_bbox_json -> /target_bbox_json
/scan -> free-space waypoint (target not visible)
/target_bbox_json -> bbox center servo (target visible)
All cmd_vel -> LiDAR emergency safety override
```

## Key files

- Config: `configs/yolo_lidar_failsafe_nav.yaml`
- Main node: `src/apps/run_yolo_lidar_failsafe_nav.py`
- Free-space: `src/perception/free_space_waypoint.py`
- Bbox parser: `src/perception/target_bbox_parser.py`
- YOLO bridge: `src/perception/yolo_world_to_bbox_json.py`

## Tuning

See `P0_YOLO_LIDAR_FAILSAFE_NAV_IMPLEMENTATION.md` section 13.

Common fixes:

- Won't move: raise `target_mid_vx`, `explore_vx`
- Won't turn: raise `target_max_wz`, `explore_max_wz`
- Always rotating: lower `free_space_min_clearance`, check `/scan` data
- Wrong turn direction: adjust sign in servo or `camera_lidar_yaw_offset_deg`
