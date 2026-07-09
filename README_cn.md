# OpenVLN-RDKX5 开放词汇视觉语言导航机器人

**OpenVLN-RDKX5** 是面向 **地瓜机器人 RDK X5** 平台的开放词汇视觉语言导航（VLN）**完整实机工程仓库**，包含脚本、配置、ROS 2 节点、地图与启动器，**不是**精简版核心代码 RAR 提交包。

> 适合作为绑定 [地瓜机器人 NodeHub](https://developer.d-robotics.cc/) 的 GitHub 仓库首页。

---

## 支持平台

| 项目 | 要求 |
|------|------|
| 开发板 | 地瓜机器人 **RDK X5** |
| 系统 | **Ubuntu 22.04** |
| 中间件 | **ROS 2 Humble**（或板载 TROS Humble） |
| 底盘 | 亚博智能 **Rosmaster M1**（PWM / 串口桥） |
| 雷达 | YDLidar（如 T-Mini Plus） |
| 相机 | **USB / RGB** 相机（`/dev/video0`） |
| 可选 | Foxglove Studio、手柄（`/dev/input/js*`） |

---

## 三条主流程

| 流程 | 说明 |
|------|------|
| **A** | **YOLO + LiDAR + 语义探索 / failsafe 导航** — 板载 YOLO 检测、LiDAR 安全、实时 SLAM、语义建图与主动探索 |
| **B** | **SLAM 建图 + Nav2 + Foxglove 点击导航** — 手柄建图、加载地图 Nav2、鼠标点击目标 |
| **C** | **Qwen API 视觉语言导航** — 云端多模态模型 + LiDAR 伺服（子项目 `rdk_x5_qwen_vln_robot/`） |

---

## 运行前安全须知

1. **确认硬件：** 雷达、USB 相机、底盘串口、ROS 2 环境均已就绪。
2. **API Key 不随仓库提交。** 请复制 `.env.example` 为 `.env` 后本地配置（仅流程 C 需要）。
3. **首次勿直接运行会动车体的脚本。** 建议 `RUN_CHASSIS=0`（Qwen）或仅运行 YOLO 可视化脚本。
4. **同一时间只跑一套栈。** 各流程共用 `/cmd_vel`、相机、雷达及 Foxglove 端口 `8765`。

---

## 目录结构

```
rdk_x5_vln_robot/
├── configs/                 # 导航、SLAM、Nav2、Foxglove 布局、mvp_tune
├── docs/                    # 设计说明与运行指南
├── lidar/                   # YDLidar launch 与 udev 规则
├── maps/                    # 已保存占据栅格地图（.yaml / .pgm）
├── perception/              # USB 相机 launch（camera_stack 调用）
├── ros2_bridge/             # 底盘桥、scan 滤波等
├── scripts/
│   ├── nav/                 # 导航启动脚本
│   ├── slam/                # SLAM 建图与 Nav2 点击导航
│   ├── lidar/               # 仅雷达与 Foxglove 辅助
│   ├── lib/                 # 公共 shell（相机、底盘、清理）
│   └── yolo/                # 仅 YOLO 预览 / bbox 诊断
├── src/                     # Python 节点（感知、规划、导航 FSM）
├── state/                   # AMCL 位姿引导文件（运行时生成）
├── rdk_x5_qwen_vln_robot/   # Qwen API VLN 子项目（流程 C）
├── .env.example             # API Key 模板（复制为 .env，勿提交）
├── README_cn.md             # 本文件
└── README.md                # 英文说明
```

---

## 快速开始

### 1. 克隆仓库

```bash
git clone <你的-nodehub-仓库地址> rdk_x5_vln_robot
cd rdk_x5_vln_robot
```

### 2. 加载 ROS 2 与雷达工作空间

```bash
source /opt/ros/humble/setup.bash    # 或 /opt/tros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash
```

### 3. 配置 API Key（仅流程 C）

```bash
cp .env.example .env
# 编辑 .env，填入 DASHSCOPE_API_KEY、QWEN_BASE_URL、QWEN_MODEL
set -a && source .env && set +a
```

切勿将 `.env` 或真实 API Key 提交到 Git。

### 4. 硬件检查

```bash
ls -l /dev/video0 /dev/ydlidar /dev/ttyUSB* /dev/rosmaster /dev/input/js0
bash scripts/lidar/check_lidar.sh    # 可选
```

若串口与默认不一致，请在对应 YAML 的 `chassis.port` 中修改。

---

## 运行方式

以下推荐命令均对应当前仓库中**真实存在**的脚本。

### 流程 A — YOLO + LiDAR + 语义探索 / failsafe 导航

**主启动器（语义探索 v1）：**

```bash
cd ~/rdk_x5_vln_robot
source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash

bash scripts/nav/start_yolo_lidar_semantic_explore_nav.sh \
  configs/nav_yolo_lidar_semantic_explore.yaml \
  "find the bottle"
```

**增强版启动器（语义探索 exp2，含 SLAM + YOLO BPU + 探索 + 手柄 mux）：**

```bash
bash scripts/nav/start_yolo_lidar_semantic_explore_nav_exp2.sh \
  configs/nav_yolo_lidar_semantic_explore_exp2.yaml \
  "find the bottle"
```

传感器与 SLAM 已就绪时，仅启动导航：

```bash
NAV_ONLY=1 bash scripts/nav/start_yolo_lidar_semantic_explore_nav_exp2.sh \
  configs/nav_yolo_lidar_semantic_explore_exp2.yaml \
  "find the bottle"
```

LiDAR 安全点对点导航（YOLO + LiDAR，无语义探索）：

```bash
bash scripts/nav/start_yolo_lidar_nav.sh configs/nav_yolo_lidar.yaml "find the bottle"
```

仅 YOLO 可视化（不发布 `/cmd_vel`）：

```bash
bash scripts/yolo/start_yolo_diag_raw.sh
```

配置：`configs/nav_yolo_lidar_semantic_explore.yaml` / `configs/nav_yolo_lidar_semantic_explore_exp2.yaml`  
Foxglove 布局：`configs/foxglove_semantic_explore_nav.layout.json`  
设计文档：`docs/SEMANTIC_EXPLORE_NAV_DESIGN.md`

---

### 流程 B — SLAM 建图 + Nav2 + Foxglove 点击导航

**步骤 1 — 手柄 SLAM 建图**

```bash
cd ~/rdk_x5_vln_robot
source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash

bash scripts/slam/run_joy_mapping_calibrated.sh
# 手柄驾驶；Ctrl+C 保存至 maps/joy_calibrated_corridor_map.yaml / .pgm
```

**步骤 2 — 加载地图启动 Nav2（可选单独运行）**

```bash
MAP_YAML=maps/joy_calibrated_corridor_map.yaml \
  bash scripts/slam/run_nav2_saved_map.sh
```

**步骤 3 — Foxglove 点击导航（推荐）**

```bash
bash scripts/slam/run_nav2_foxglove_click_goal.sh
# 布局：configs/foxglove_click_goal_nav.layout.json
# Foxglove：ws://<板子IP>:8765
# 激光与地图错位时用 /initialpose 对齐
```

指南：`docs/foxglove_click_goal_nav2_guide.md`

---

### 流程 C — Qwen API 视觉语言导航

```bash
cd ~/rdk_x5_vln_robot/rdk_x5_qwen_vln_robot
cp ../.env.example .env
set -a && source .env && set +a
source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash

# 安全首次运行 — 底盘不动
RUN_CHASSIS=0 bash scripts/nav/start_qwen_api_lidar_nav.sh "find bottle"

# 完整运行 — 发布 /cmd_vel
bash scripts/nav/start_qwen_api_lidar_nav.sh "find bottle"
```

停止：`bash scripts/nav/stop_qwen_api_lidar_nav.sh`  
配置：`rdk_x5_qwen_vln_robot/configs/qwen_api_lidar_nav.yaml`

---

## 常见问题

**该用哪条流程？**  
- 板载 YOLO 语义搜索：**流程 A**  
- 先建图再点目标：**流程 B**  
- 云端自然语言找目标：**流程 C**

**能同时跑两套吗？**  
不能。先停掉上一套（`/cmd_vel`、相机、雷达、端口 `8765` 会冲突）。

**点击导航报地图不存在？**  
先运行 `scripts/slam/run_joy_mapping_calibrated.sh`，或设置 `MAP_YAML`。

**车不动但话题正常？**  
检查 YAML 中 `chassis.port`、`/dev/ttyUSB*` 权限及 `/cmd_vel` 是否被占用。Qwen 建议先用 `RUN_CHASSIS=0` 验证。

**点击导航激光与地图错位？**  
Foxglove → Publish → 2D Pose Estimate → `/initialpose`。

**API Key 在哪？**  
仅本地 `.env`（参考 `.env.example`），仓库不提交密钥。

**YOLO BPU 模型找不到？**  
安装 RDK Model Zoo 样例，或修改 yaml 中 `yolov5s_bpu.model`（默认在 `/root/rdk_model_zoo/`）。

---

## 更多文档

| 文件 | 内容 |
|------|------|
| [README.md](README.md) | English |
| [docs/SEMANTIC_EXPLORE_NAV_DESIGN.md](docs/SEMANTIC_EXPLORE_NAV_DESIGN.md) | 语义探索设计 |
| [docs/foxglove_click_goal_nav2_guide.md](docs/foxglove_click_goal_nav2_guide.md) | 点击导航 |
| [rdk_x5_qwen_vln_robot/README.md](rdk_x5_qwen_vln_robot/README.md) | Qwen 子项目 |

---

## 许可证

本项目采用 **MIT License**。详见仓库根目录 [LICENSE](LICENSE) 文件。

---

## 免责声明

本软件可驱动物理移动机器人。请清空工作区域、保持急停可用、确认传感器与串口正常，并在启用底盘运动前使用干跑模式。
