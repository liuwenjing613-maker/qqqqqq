# Foxglove 可视化：P0 Failsafe 导航

## 1. 启动可视化 bridge

全栈启动已自动包含；若只跑 nav，可单独起：

```bash
cd ~/rdk_x5_vln_robot
python3 src/apps/failsafe_nav_foxglove_viz.py \
  --config configs/yolo_lidar_failsafe_nav.yaml
```

## 2. 发布的可视化 topic

| Topic | 类型 | Foxglove 面板 | 内容 |
|-------|------|---------------|------|
| `/scan` | LaserScan | **3D** | 原始雷达点云 |
| `/failsafe_nav/markers` | MarkerArray | **3D** | 安全圈、前方距离、free-space 射线、目标方向、状态文字 |
| `/failsafe_nav/debug_image` | Image | **Image** | 相机图 + bbox + 导航点十字 + **u/v 坐标** + mode 文字 |
| `/failsafe_nav_state` | String(JSON) | Raw Messages / Plot | 完整状态 JSON |
| `/failsafe_nav_point` | String(JSON) | Raw Messages | 当前跟踪/探索像素点 |
| `/target_bbox_json` | String(JSON) | Raw Messages | YOLO 检测框 |
| `/cmd_vel` | Twist | Plot | 底盘速度 |

## 3. Foxglove 3D 面板 Marker 含义

| 颜色 | 含义 |
|------|------|
| 红色弧 | emergency_stop 距离 (0.22m) |
| 橙色弧 | hard_stop 距离 (0.32m) |
| 黄色弧 | slow 距离 (0.55m) |
| 绿色箭头 | 正前方测距 `front_distance` |
| 蓝色箭头+球 | LiDAR free-space 探索方向与 clearance |
| 粉色箭头 | YOLO 目标 bbox 中心方向（近似） |
| 白色文字 | mode / front / vx,wz / reason |

**Fixed Frame** 设为：`laser`

## 4. 推荐 Foxglove 布局（手动添加 4 个面板）

### 面板 A：3D
- Add panel → 3D
- Fixed frame: `laser`
- 勾选 Topics：
  - `/scan` → LaserScan
  - `/failsafe_nav/markers` → Markers

### 面板 B：Image（推荐）
- Add panel → Image
- **Topic 必须选** `/failsafe_nav/debug_image`（不是 `/image_raw`）
- 叠加了：YOLO **绿色检测框**、框旁 **u/v**、导航点十字（橙=target / 蓝=free_space）

备选原始相机：`/image_raw`（无框、无叠加）

### 面板 C：State JSON
- Add panel → Raw Messages
- Topic: `/failsafe_nav_state`

### 面板 D：Plot（可选）
- Add panel → Plot
- 若用 JSON path 插件可画 `front_distance`、`cmd_vx`
- 或直接看 Raw Messages

## 5. 连接方式

板端已跑 `foxglove_bridge` 时，Foxglove Studio 连接：

```
ws://<板子IP>:8765
```

本地 ROS2 直连：

```bash
# Foxglove Studio → Open connection → Rosbridge / Foxglove WebSocket
```

## 6. 配置项（yaml）

[`configs/yolo_lidar_failsafe_nav.yaml`](../configs/yolo_lidar_failsafe_nav.yaml)：

```yaml
viz_frame_id: "laser"
viz_markers_topic: "/failsafe_nav/markers"
viz_debug_image_topic: "/failsafe_nav/debug_image"
viz_rate_hz: 5.0
viz_image_source: "compressed"
viz_image_topic: "/image"
viz_image_max_fps: 15.0
viz_image_scale: 0.5
```

## 7. 降低摄像头延迟

默认 viz **直接订阅 `/image`（相机 MJPEG）**，收到帧即发布 `/failsafe_nav/debug_image`（最高 15fps），**不再走 `/image_raw`（算法桥接仅 2fps）**。

可调参数（[`configs/yolo_lidar_failsafe_nav.yaml`](../configs/yolo_lidar_failsafe_nav.yaml)）：

| 参数 | 默认 | 说明 |
|------|------|------|
| `viz_image_source` | `compressed` | `compressed`=低延迟；`raw`=用 `/image_raw` |
| `viz_image_max_fps` | `15` | Foxglove 图像最高帧率 |
| `viz_image_scale` | `0.5` | 缩小后再发（减带宽）；要更清晰设 `1.0` |

Foxglove Image 面板仍订阅 **`/failsafe_nav/debug_image`**（带检测框叠加）。不要直接看 `/image_raw`。

修改后重启 viz：

```bash
pkill -f failsafe_nav_foxglove_viz
python3 src/apps/failsafe_nav_foxglove_viz.py --config configs/yolo_lidar_failsafe_nav.yaml
```

## 7. 已有 JSON topic 直接看（无需 viz bridge）

```bash
ros2 topic echo /failsafe_nav_state
ros2 topic echo /failsafe_nav_point
ros2 topic echo /target_bbox_json
```

但 Foxglove 对 String JSON 不直观，**推荐用 `/failsafe_nav/markers` + `/failsafe_nav/debug_image`**。
