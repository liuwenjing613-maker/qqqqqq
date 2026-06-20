# scripts 目录说明

脚本按功能分子目录存放，请使用完整路径调用。

## 目录结构

| 目录 | 用途 | 常用脚本 |
|------|------|----------|
| `nav/` | 导航主控（P0 failsafe、Qwen LiDAR、像素任务） | `start_yolo_lidar_failsafe_nav.sh`, `start_qwen_lidar_nav.sh`, `pub_fake_bbox.py` |
| `lidar/` | 雷达驱动与检测 | `start_lidar_only.sh`, `check_lidar.sh`, `source_ydlidar.sh` |
| `yolo/` | YOLO 启动、bbox 检测与坐标校验 | `start_yolo_live_preview.sh`, `start_yolo_diag_raw.sh`, `check_bbox_once.sh` |
| `qwen/` | Ollama / Qwen 内存与推理辅助 | `ollama_prep_infer.sh`, `ollama_recover.sh`, `setup_ollama_memory.sh` |
| `mvp/` | 早期 red / MVP 视觉伺服线 | `start_red_servo.sh`, `start_red_mvp.sh` |
| `chassis/` | 底盘与云台调试 | `test_chassis_ground.sh`, `gimbal_tune.sh` |
| `system/` | 进程清理与系统检查 | `stop_all_safe.sh`, `stop_all.sh`, `check_system.sh` |
| `debug/` | 录像与抓帧 | `capture_navigation_video.sh`, `save_debug_frame.sh` |
| `lib/` | 公共 shell 片段 | `project_dir.sh`, `load_mvp_tune.sh`, `run_chassis_bridge.sh` |

## 路径约定

子目录内脚本统一通过 `lib/project_dir.sh` 解析仓库根目录：

```bash
bash scripts/nav/start_yolo_lidar_failsafe_nav.sh
bash scripts/lidar/check_lidar.sh
```

## 常用命令

### P0 YOLO + LiDAR Failsafe 导航

```bash
# 全栈（雷达 + YOLO + bbox bridge + nav）
bash scripts/nav/start_yolo_lidar_failsafe_nav.sh

# 仅 nav（需自行保证 /scan 与 /target_bbox_json 有数据）
NAV_ONLY=1 bash scripts/nav/start_yolo_lidar_failsafe_nav.sh

# 假 bbox 联调
python3 scripts/nav/pub_fake_bbox.py --bbox 250 140 390 420 --class-name bottle
```

### Qwen LiDAR 导航

```bash
bash scripts/nav/start_qwen_lidar_nav.sh
bash scripts/nav/stop_qwen_lidar_nav.sh
```

### 雷达

```bash
bash scripts/lidar/check_lidar.sh
bash scripts/lidar/start_lidar_only.sh
```

### YOLO 预览 / 诊断

```bash
bash scripts/yolo/start_yolo_live_preview.sh
bash scripts/yolo/start_yolo_diag_raw.sh
```

### 清理与检查

```bash
bash scripts/system/stop_all_safe.sh
bash scripts/system/check_system.sh
```

## 新增脚本

1. 放入对应功能子目录。
2. 在脚本开头引用 `project_dir.sh`：

```bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
```

3. 脚本间相互调用时使用 `"$PROJECT_DIR/scripts/<subdir>/..."` 路径。
