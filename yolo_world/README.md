# YOLO-World 检测链路说明

## 架构（务必理解）

```
hobot_usb_cam          compressed_to_raw_image          hobot_yolo_world
     |                          |                              |
  /image (CompressedImage)  /image_raw (Image)          /hobot_yolo_world
                                                                    |
                                                         yolo_world_bbox_preview.py
                                                         yolo_world_servo_ros.py
                                                         yolo_world_diag_preview.py
                                                         run_mvp_task.py
```

- **`hobot_yolo_world`**：真正的 YOLO-World **推理节点**（BPU 推理），发布 `ai_msgs/msg/PerceptionTargets`。
- **`yolo_world_bbox_preview.py` / `yolo_world_servo_ros.py`**：只是**后处理脚本**，订阅 `/hobot_yolo_world`，做类别过滤、红色校验、框选与伺服。**它们不会运行模型**。
- **`--target-classes` / `target_classes`**：仅在后处理阶段过滤已检测到的类别；**不会改变模型检测哪些词**。
- **模型检测类别**必须由推理节点设置：
  - 启动参数 `-p texts:="backpack,handbag,suitcase"`
  - 或向 `/target_words` 话题发布 `std_msgs/msg/String`

## 图像话题对应关系

| 话题 | 类型 | 来源 |
|------|------|------|
| `/image` | `sensor_msgs/msg/CompressedImage` | `hobot_usb_cam` 默认输出 |
| `/image_raw` | `sensor_msgs/msg/Image` (bgr8) | `compressed_to_raw_image.py` 桥接 |

**`hobot_yolo_world` 的 `ros_img_sub_topic_name` 必须与实际发布的 Image 话题一致：**

| 场景 | 相机输出 | bridge | YOLO 应订阅 |
|------|----------|--------|-------------|
| 推荐（本项目脚本默认） | `/image` | `/image` → `/image_raw` | **`/image_raw`** |
| 直接订阅压缩流 | `/image` | 无 | `/image`（需确认 feed_type 与编码） |
| OpenCV 直出原图 | `/image_raw` | 无 | `/image_raw` |

本项目启动脚本（`scripts/yolo/start_yolo_diag_raw.sh`、`scripts/yolo/start_yolo_mvp_raw.sh`）统一使用：

```bash
COMPRESSED_IMAGE_TOPIC=/image
RAW_IMAGE_TOPIC=/image_raw
YOLO_IMAGE_TOPIC=/image_raw   # hobot_yolo_world 订阅此话题
```

若 YOLO 订阅 `/image` 而 bridge 只发布 `/image_raw`，推理节点将收不到图像。

## 启动顺序（完整示例）

### 1. 环境

```bash
cd ~/rdk_x5_vln_robot
source /opt/tros/humble/setup.bash
# 若存在 stage10 环境脚本：
[ -f source_stage10.sh ] && source source_stage10.sh
```

### 2. 相机 + 原图桥接

```bash
# 相机 -> /image (CompressedImage)
ros2 launch ~/rdk_x5_vln_robot/perception/launch/usb_cam.launch.py usb_video_device:=/dev/video0

# /image -> /image_raw (bgr8)
python3 ~/rdk_x5_vln_robot/src/perception/compressed_to_raw_image.py \
  --in-topic /image --out-topic /image_raw
```

### 3. 发布检测词表（必须在 YOLO 启动前或同时）

```bash
ros2 topic pub -r 1 /target_words std_msgs/msg/String \
  "{data: 'backpack,handbag,suitcase'}"
```

### 4. 启动 YOLO-World 推理节点

```bash
ros2 run hobot_yolo_world hobot_yolo_world --ros-args \
  -p feed_type:=1 \
  -p ros_img_sub_topic_name:=/image_raw \
  -p ros_string_sub_topic_name:=/target_words \
  -p ai_msg_pub_topic_name:=/hobot_yolo_world \
  -p texts:="backpack,handbag,suitcase" \
  -p score_threshold:=0.01 \
  -p iou_threshold:=0.45
```

若相机未做 bridge、YOLO 直接吃压缩图：

```bash
ros2 run hobot_yolo_world hobot_yolo_world --ros-args \
  -p feed_type:=1 \
  -p ros_img_sub_topic_name:=/image \
  -p texts:="backpack,handbag,suitcase" \
  -p score_threshold:=0.01
```

### 5. 启动后处理预览（打印全部 ROI + MVP 过滤）

```bash
# 打印所有 target.type / confidence / bbox，不过滤类别
# min-score 默认读取 configs/mvp_tune.yaml 的 yolo_min_score
python3 ~/rdk_x5_vln_robot/yolo_world/yolo_world_bbox_preview.py \
  --target-classes ""

# 仅后处理过滤书包相关类（不改变模型词表）
python3 ~/rdk_x5_vln_robot/yolo_world/yolo_world_bbox_preview.py \
  --target-classes "backpack,handbag,suitcase"
```

### 6. 一键诊断（相机 + YOLO + 可视化快照）

```bash
cd ~/rdk_x5_vln_robot
SHOW_ALL_BOXES=1 bash scripts/yolo/start_yolo_diag_raw.sh
```

## 统一调参（推荐）

日常只改 **`configs/mvp_tune.yaml`**，以下会自动同步：

| 配置项 | 同步到 |
|--------|--------|
| `yolo_min_score` | `hobot_yolo_world` 的 `score_threshold`、MVP `--min-score`、YOLO 工具默认阈值 |
| `max_vx` / `max_wz` / FSM 参数 | `run_mvp_task.py`、底盘 `min_drive_vx` |
| `chassis_port` | 各 `start_*_mvp_raw.sh` |

```bash
python3 ~/rdk_x5_vln_robot/src/config/mvp_tune.py   # 查看当前生效值
python3 -m unittest discover -s tests -v            # 跑 YOLO + tune 单元测试
```

## 离线词表限制

当前离线词表文件：`yolo_world/config/offline_vocabulary_embeddings.json`

**已确认包含：** `backpack`、`handbag`、`suitcase`

**不包含（勿直接写入 texts，否则无效）：** `bag`、`school bag`、`red backpack` 等

若需使用 `bag` 等词，需：

1. 更换/扩展离线词表并重新生成 text embedding；或
2. 在后处理用同义词匹配（`target_backend_yolo.py` 已支持 `bag` ↔ `backpack/handbag/suitcase` 模糊匹配）；或
3. **临时改用颜色检测后端**：`run_mvp_task.py --backend red` 或 `debug_tools/check_bbox_once.py`（纯 HSV 红色书包，不依赖 YOLO 词表）

## 后处理参数说明

| 参数 | 作用 | 影响推理？ |
|------|------|-----------|
| `configs/mvp_tune.yaml` → `yolo_min_score` | 统一阈值（节点 + 后处理） | **是/否** |
| `-p texts:=...` / `/target_words` | 模型开放词汇检测类别 | **是** |
| `--target-classes` | 后处理类别过滤；空字符串=不过滤 | 否 |
| `--min-score` | 后处理置信度阈值（默认读 mvp_tune） | 否 |
| `-p score_threshold:=...` | 推理节点内部阈值（启动脚本从 mvp_tune 读取） | **是** |

## 调试命令

```bash
# 检查话题
bash ~/rdk_x5_vln_robot/scripts/system/check_topics.sh

# 打印原始 ROI（蓝/黄两种坐标解释，不做后处理）
python3 ~/rdk_x5_vln_robot/check_bbox/yolo_raw_bbox_debug.py

# 后处理预览：打印全部检测 + MVP 结果
python3 ~/rdk_x5_vln_robot/yolo_world/yolo_world_bbox_preview.py --target-classes ""
```
