# OpenVLN-RDKX5 — Open-Vocabulary Vision-Language Navigation Robot

**OpenVLN-RDKX5** is a full on-robot engineering repository for open-vocabulary vision-language navigation on the **D-Robotics RDK X5** (地瓜机器人). It ships scripts, configs, ROS 2 nodes, maps, and launchers for real hardware—**not** a minimal core-code RAR submission package.

> Suitable as the GitHub homepage for a [D-Robotics NodeHub](https://developer.d-robotics.cc/) bound repository.

---

## Supported Platform

| Item | Requirement |
|------|-------------|
| Board | D-Robotics **RDK X5** |
| OS | **Ubuntu 22.04** |
| Middleware | **ROS 2 Humble** (or TROS Humble on board image) |
| Chassis | Yahboom **Rosmaster M1** (PWM / serial bridge) |
| LiDAR | YDLidar (e.g. T-Mini Plus) |
| Camera | **USB / RGB** camera (`/dev/video0`) |
| Optional | Foxglove Studio, joystick (`/dev/input/js*`) |

---

## Three Main Workflows

| Flow | Description |
|------|-------------|
| **A** | **YOLO + LiDAR + semantic exploration / failsafe navigation** — onboard YOLO detection, LiDAR safety, live SLAM, semantic mapping, and active exploration |
| **B** | **SLAM mapping + Nav2 + Foxglove click navigation** — joystick mapping, saved-map Nav2, mouse-click goals |
| **C** | **Qwen API vision-language navigation** — cloud multimodal model + LiDAR servo (sub-project `rdk_x5_qwen_vln_robot/`) |

---

## Safety Before Running

1. **Confirm hardware:** lidar, USB camera, chassis serial port, ROS 2 environment.
2. **API keys are not in this repo.** Copy `.env.example` → `.env` and fill in credentials (Flow C only).
3. **Do not run motion scripts blindly on first try.** Use `RUN_CHASSIS=0` (Qwen) or YOLO preview-only scripts first.
4. **Run one stack at a time.** Flows share `/cmd_vel`, camera, lidar, and Foxglove port `8765`.

---

## Repository Layout

```
rdk_x5_vln_robot/
├── configs/                 # Nav, SLAM, Nav2, Foxglove layouts, mvp_tune
├── docs/                    # Design notes and run guides
├── lidar/                   # YDLidar launch and udev rules
├── maps/                    # Saved occupancy maps (.yaml / .pgm)
├── perception/              # USB camera launch (used by camera_stack)
├── ros2_bridge/             # Chassis bridge, scan filter, helpers
├── scripts/
│   ├── nav/                 # Navigation launchers
│   ├── slam/                # SLAM mapping and Nav2 click-nav
│   ├── lidar/               # Lidar-only and Foxglove helpers
│   ├── lib/                 # Shared shell (camera, chassis, cleanup)
│   └── yolo/                # YOLO-only preview / bbox diagnostics
├── src/                     # Python nodes (perception, planning, nav FSM)
├── state/                   # Last pose for AMCL bootstrap (runtime)
├── rdk_x5_qwen_vln_robot/   # Qwen API VLN sub-project (Flow C)
├── .env.example             # API key template (copy to .env, never commit)
├── README_cn.md             # Chinese documentation
└── README.md                # This file
```

---

## Quick Start

### 1. Clone

```bash
git clone <your-nodehub-repo-url> rdk_x5_vln_robot
cd rdk_x5_vln_robot
```

### 2. Source ROS 2 and lidar workspace

```bash
source /opt/ros/humble/setup.bash    # or /opt/tros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash
```

### 3. Configure API keys (Flow C only)

```bash
cp .env.example .env
# Edit .env — set DASHSCOPE_API_KEY, QWEN_BASE_URL, QWEN_MODEL
set -a && source .env && set +a
```

Never commit `.env` or real API keys.

### 4. Hardware check

```bash
ls -l /dev/video0 /dev/ydlidar /dev/ttyUSB* /dev/rosmaster /dev/input/js0
bash scripts/lidar/check_lidar.sh    # optional
```

Update `chassis.port` in the relevant YAML if your serial device differs.

---

## How to Run

Recommended commands below match **scripts that exist in this repository**.

### Flow A — YOLO + LiDAR + semantic exploration / failsafe navigation

**Primary launcher (semantic explore v1):**

```bash
cd ~/rdk_x5_vln_robot
source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash

bash scripts/nav/start_yolo_lidar_semantic_explore_nav.sh \
  configs/nav_yolo_lidar_semantic_explore.yaml \
  "find the bottle"
```

**Enhanced launcher (semantic explore exp2, SLAM + YOLO BPU + explore + joystick mux):**

```bash
bash scripts/nav/start_yolo_lidar_semantic_explore_nav_exp2.sh \
  configs/nav_yolo_lidar_semantic_explore_exp2.yaml \
  "find the bottle"
```

Nav-only when sensors and SLAM are already up:

```bash
NAV_ONLY=1 bash scripts/nav/start_yolo_lidar_semantic_explore_nav_exp2.sh \
  configs/nav_yolo_lidar_semantic_explore_exp2.yaml \
  "find the bottle"
```

LiDAR-safe point navigation (YOLO + LiDAR, no semantic explore):

```bash
bash scripts/nav/start_yolo_lidar_nav.sh configs/nav_yolo_lidar.yaml "find the bottle"
```

YOLO preview only (no `/cmd_vel`):

```bash
bash scripts/yolo/start_yolo_diag_raw.sh
```

Config: `configs/nav_yolo_lidar_semantic_explore.yaml` / `configs/nav_yolo_lidar_semantic_explore_exp2.yaml`  
Foxglove layout: `configs/foxglove_semantic_explore_nav.layout.json`  
Design doc: `docs/SEMANTIC_EXPLORE_NAV_DESIGN.md`

---

### Flow B — SLAM mapping + Nav2 + Foxglove click navigation

**Step 1 — Joystick SLAM mapping**

```bash
cd ~/rdk_x5_vln_robot
source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash

bash scripts/slam/run_joy_mapping_calibrated.sh
# Drive with joystick; Ctrl+C saves maps/joy_calibrated_corridor_map.yaml / .pgm
```

**Step 2 — Nav2 on saved map (standalone, optional)**

```bash
MAP_YAML=maps/joy_calibrated_corridor_map.yaml \
  bash scripts/slam/run_nav2_saved_map.sh
```

**Step 3 — Foxglove click navigation (recommended)**

```bash
bash scripts/slam/run_nav2_foxglove_click_goal.sh
# Layout: configs/foxglove_click_goal_nav.layout.json
# Foxglove: ws://<board-ip>:8765
# Use /initialpose if laser and map are misaligned
```

Guide: `docs/foxglove_click_goal_nav2_guide.md`

---

### Flow C — Qwen API vision-language navigation

```bash
cd ~/rdk_x5_vln_robot/rdk_x5_qwen_vln_robot
cp ../.env.example .env
set -a && source .env && set +a
source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash

# Safe first run — no chassis motion
RUN_CHASSIS=0 bash scripts/nav/start_qwen_api_lidar_nav.sh "find bottle"

# Full run — publishes /cmd_vel
bash scripts/nav/start_qwen_api_lidar_nav.sh "find bottle"
```

Stop: `bash scripts/nav/stop_qwen_api_lidar_nav.sh`  
Config: `rdk_x5_qwen_vln_robot/configs/qwen_api_lidar_nav.yaml`

---

## FAQ

**Which flow should I use?**  
- Semantic search with onboard YOLO: **Flow A**  
- Map once, then click goals: **Flow B**  
- Natural-language targets via cloud VLM: **Flow C**

**Can I run two flows together?**  
No. Stop the previous stack first (`/cmd_vel`, camera, lidar, and port `8765` conflict).

**Click-nav says map not found.**  
Run `scripts/slam/run_joy_mapping_calibrated.sh` first, or set `MAP_YAML` to your map.

**Robot does not move.**  
Check `chassis.port` in YAML, `/dev/ttyUSB*` permissions, and competing `/cmd_vel` publishers. For Qwen, verify with `RUN_CHASSIS=0` first.

**Laser does not match map in click-nav.**  
Foxglove → Publish → 2D Pose Estimate → `/initialpose`.

**Where is the API key?**  
In local `.env` only (see `.env.example`). Not committed to Git.

**YOLO BPU model not found.**  
Install RDK Model Zoo samples or update `yolov5s_bpu.model` in the yaml (default under `/root/rdk_model_zoo/`).

---

## More Documentation

| File | Topic |
|------|--------|
| [README_cn.md](README_cn.md) | 中文说明 |
| [docs/SEMANTIC_EXPLORE_NAV_DESIGN.md](docs/SEMANTIC_EXPLORE_NAV_DESIGN.md) | Semantic explore design |
| [docs/foxglove_click_goal_nav2_guide.md](docs/foxglove_click_goal_nav2_guide.md) | Click navigation |
| [rdk_x5_qwen_vln_robot/README.md](rdk_x5_qwen_vln_robot/README.md) | Qwen sub-project |

---

## License

Released under the **MIT License**. See [LICENSE](LICENSE) in the repository root.

---

## Disclaimer

This software can command a physical mobile robot. Clear the workspace, keep emergency stop ready, verify all sensors and serial ports, and use dry-run modes before enabling chassis motion.
