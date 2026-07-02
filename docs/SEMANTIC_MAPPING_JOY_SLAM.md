# Joystick SLAM + Semantic Mapping

Calibrated joystick SLAM with a **semantic overlay** sidecar. Geometric mapping stays in `slam_toolbox`; semantic landmarks are stored separately and published as ROS topics.

## Quick start

```bash
cd /root/rdk_x5_vln_robot
MAP_NAME=joy_semantic_test \
SEMANTIC_CLASSES='bottle,cup,backpack,chair,dining table,book,potted plant,cell phone,couch' \
bash scripts/slam/run_joy_semantic_mapping_calibrated.sh
```

Drive with the joystick. Press **Ctrl+C** to save:

1. Semantic map (`logs/semantic_mapping/session_*/semantic_map.json`)
2. SLAM occupancy grid (`maps/${MAP_NAME}.pgm/.yaml`)

## Architecture

```
SLAM stack (unchanged)  →  /map /tf /odom /scan_filtered
Camera + YOLO BPU       →  /target_bbox_json (boxes[])
semantic_mapper_node    →  /semantic_map_json /semantic_landmarks /semantic_viewpoints ...
```

`semantic_mapper` does **not** publish `/cmd_vel`, `/map`, or core TF.

## Foxglove

1. Connect: `ws://<robot-ip>:8765`
2. **Layout → Import layout** (not just open file): `configs/foxglove_joy_semantic_mapping.layout.json`
3. 3D panel **Fixed Frame = map** (layout sets this automatically after re-import)
4. Topics:
   - `/map`, `/scan_filtered`, `/tf`
   - `/semantic_landmarks`, `/semantic_viewpoints`
   - `/semantic_loop_error`, `/semantic_status`
5. Image panel: `/yolov5s_bpu/annotated/compressed`
6. YOLO web preview: `http://<robot-ip>:8088/`

### Red exclamation on topics

| Symptom | Cause | Fix |
|---------|-------|-----|
| Image panel red | Old layout used deprecated `cameraTopic` | Re-import updated layout |
| `/semantic_*` red before step 5/7 | Semantic stack not started yet | Wait until script prints `[5/7]` and `[7/7] Running` |
| `/map` red briefly at connect | SLAM map not published yet | Wait ~10s after `[1/7]`; map uses latched QoS |
| `/yolov5s_bpu/annotated/compressed` red before step 4 | YOLO not started | Wait until `[4/7]` completes |
| Topics red when stack stopped | No ROS publishers | Restart `run_joy_semantic_mapping_calibrated.sh` |

## Driving strategy (loop closure)

1. Stop at start 3–5 s for TF/map to stabilize
2. Slow in-place scan (~360°) for initial viewpoints
3. Move slowly along walls/corridor; avoid fast spins
4. Pause 1–2 s near objects for multi-frame YOLO votes
5. Close the loop back to start
6. Stop 5 s at start, then Ctrl+C

## Configuration

Main config: `configs/semantic_mapping.yaml`

Key tunables:

| Block | Purpose |
|-------|---------|
| `classes.whitelist` | Objects stored as landmarks |
| `classes.dynamic` | Observation only (person/dog/cat) |
| `detection_filter` | Score/area/edge filters |
| `temporal_vote` | Multi-frame confirmation |
| `landmark_fusion` | Merge radius, confirm count |
| `camera.yaw_offset_deg` | Fix left/right projection without touching laser TF |
| `loop_quality` | Return-to-start error thresholds |

Environment variables for launch script:

| Variable | Default |
|----------|---------|
| `MAP_NAME` | `joy_semantic_calibrated_map` |
| `SEMANTIC_CONFIG` | `configs/semantic_mapping.yaml` |
| `SEMANTIC_CLASSES` | bottle,cup,... |
| `CAMERA_DEV` | `/dev/video0` |
| `JOY_DEV` | `/dev/input/js0` |

## Output files

Each session under `logs/semantic_mapping/session_YYYYMMDD_HHMMSS/`:

```
semantic_map.json
observations.jsonl
landmarks.json
viewpoints.jsonl
loop_quality.json
keyframes/vp_*.jpg
```

## Acceptance tests

### 1. Old SLAM unaffected

```bash
bash scripts/slam/run_joy_mapping_calibrated.sh
```

Verify `/map`, `/odom`, `/scan_filtered`, joystick mapping, Ctrl+C save.

### 2. Semantic pipeline

```bash
bash scripts/slam/run_joy_semantic_mapping_calibrated.sh
ros2 topic list | egrep 'semantic|target_bbox|map|scan_filtered|odom'
ros2 topic echo /semantic_status
ros2 topic echo /semantic_loop_error
```

### 3. Quality criteria

- `/map` walls continuous; scan aligns after loop closure
- Landmarks not heavily duplicated
- bottle/cup → candidate or confirmed
- chair/table roughly in correct map regions
- `loop_xy_error_m < 0.20`, `loop_yaw_error_deg < 12°`
- Both SLAM map and `semantic_map.json` saved on Ctrl+C

## FAQ

### Return pose drift at start

Check `/odom` and `/scan_filtered` rates, drive slower (`JOY_SCALE_LINEAR_X=0.03`), close loop path. See `loop_quality` in saved JSON.

### Semantic markers left/right reversed

Adjust `camera.yaw_offset_deg` in `semantic_mapping.yaml` only.

### Many duplicate bottles

Increase `landmark_fusion.merge_radius_small_m` or `confirm_seen_count`.

### YOLO dropouts

Lower `min_score_small_object` slightly; keep `min_votes_confirmed: 3`; do not single-frame confirm.

## Files added (do not modify SLAM core)

```
configs/semantic_mapping.yaml
configs/foxglove_joy_semantic_mapping.layout.json
src/mapping/
scripts/slam/run_joy_semantic_mapping_calibrated.sh
docs/SEMANTIC_MAPPING_JOY_SLAM.md
```

## Presentation note

> We keep the existing SLAM pipeline for 2D occupancy and TF. A sidecar semantic mapper subscribes to YOLO boxes, LiDAR, and TF, applies temporal voting and range projection, and builds object-level landmarks and viewpoint nodes as an overlay for later semantic frontier planning.
