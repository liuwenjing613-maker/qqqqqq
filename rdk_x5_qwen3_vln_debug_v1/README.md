# RDK X5 Qwen3-VL 导航感知调试版 V1

这是复赛重构的第一版最小系统，目标不是马上驱动车辆，而是先把 **Qwen3-VL-Flash 的视觉判断、像素点输出、状态切换和 Foxglove 可视化**验证清楚。

当前版本只完成四件事：

1. 调用 `qwen3-vl-flash` 分析真车相机画面；
2. 按状态选择独立提示词；
3. 严格解析 JSON；模型输出 `[0, 1000]` 相对坐标，本地换算为像素后绘制/发布；
4. 把模型点绘制在**本次 API 实际输入的那一帧图像**上，并发布给 Foxglove。

本版本**不发布 `/cmd_vel`，不控制底盘，不包含 SLAM、Frontier、记忆、路径规划或 YOLO-World 融合**。先判断模型到底点中了什么，再允许机器人运动，多少算是给墙壁一点基本尊重。

---

## 1. 状态机

状态结构参考大创中熟悉的 `Target Locked / Target Inferred / Searching` 思路，但 V1 只切换视觉提示词，不处理运动。

```text
WAIT_IMAGE
    │ 收到图像且已有任务
    ▼
OBSERVE
    ├── TARGET_VISIBLE (高置信) ────> TARGET_LOCKED
    ├── TARGET_INFERRED (高置信) ───> TARGET_INFERRED
    └── 低置信探索点 ───────────────> SEARCHING

TARGET_LOCKED
    ├── TARGET_VISIBLE ─────────────> TARGET_LOCKED
    └── TARGET_INFERRED / 低置信 ───> TARGET_INFERRED / SEARCHING

SEARCHING / TARGET_INFERRED
    ├── TARGET_VISIBLE ─────────────> TARGET_LOCKED
    ├── TARGET_INFERRED (高置信) ───> TARGET_INFERRED
    └── 低置信 ─────────────────────> SEARCHING

任意工作状态 -- verify --> VERIFY
VERIFY
    ├── VERIFY_SUCCESS ─────────────> SUCCESS
    ├── VERIFY_FAILED (高置信) ─────> TARGET_INFERRED
    └── VERIFY_FAILED (低置信) ─────> SEARCHING

任意工作状态 -- pause --> PAUSED
API 或解析异常 ─────────────────────> ERROR
ERROR 冷却后 ───────────────────────> OBSERVE
```

状态与提示词模式：

| 状态 | 提示词 | 当前职责 |
|---|---|---|
| `OBSERVE` | `observe.txt` | 判断准确目标是否可见，输出目标中心点 |
| `TARGET_LOCKED` | `track.txt` | 在当前帧重新定位同一目标，不盲目复用旧点 |
| `SEARCHING` | `search.txt` | 目标不可见时寻找可见语义线索 |
| `TARGET_INFERRED` | `search.txt` | 已有语义线索，继续更新或发现真实目标 |
| `VERIFY` | `verify.txt` | 验证候选是否完整符合指令 |

`TARGET_INFERRED` 中的黄色点只是**视觉注意区域**，不是地面路径点，也不会被发送给底盘。

---

## 2. 目录结构

```text
rdk_x5_qwen3_vln_debug_v1/
├── configs/
│   └── qwen3_vln_debug.yaml
├── prompts/
│   ├── common.txt
│   ├── observe.txt
│   ├── track.txt
│   ├── search.txt
│   └── verify.txt
├── docs/
│   ├── FOXGLOVE.md
│   └── MODULES.md
├── scripts/
│   ├── start_debug_node.sh
│   ├── test_single_image.sh
│   └── run_tests.sh
├── src/
│   ├── apps/
│   │   ├── qwen_vln_debug_node.py
│   │   └── test_single_image.py
│   └── qwen_vln/
│       ├── prompt_manager.py
│       ├── qwen_client.py
│       ├── state_machine.py
│       ├── types.py
│       └── visualizer.py
├── tests/test_core.py
├── env.example
└── requirements.txt
```

提示词全部放在 `prompts/`，不需要进入主程序改字符串。第一阶段调提示词时，主要修改这里。

---

## 3. 环境配置

```bash
cd /root/rdk_x5_qwen3_vln_debug_v1
cp env.example .env.local
vim .env.local
source .env.local
python3 -m pip install -r requirements.txt
```

至少设置：

```bash
export DASHSCOPE_API_KEY="你的 API Key"
export QWEN_MODEL="qwen3-vl-flash"
```

`QWEN_BASE_URL` 必须与你的百炼 API Key 和业务空间地域一致。`env.example` 已留出北京和新加坡业务空间地址示例。

依赖检查：

```bash
python3 -c "import cv2,numpy,yaml,openai; print('dependencies ok')"
```

---

## 4. 先测试单张图片

```bash
bash scripts/test_single_image.sh \
  /root/test_images/bottle.jpg \
  "find the bottle" \
  observe
```

模式可选：

```text
observe / track / search / verify
```

输出：

```text
outputs/latest_result.json
outputs/latest_annotated.jpg
```

这一阶段先测试几十张真实相机图片，确认模型是否稳定输出目标中心点，而不是凭三张成功截图宣布人工智能取得全面胜利。

---

## 5. 启动 ROS2 节点

先启动原项目中已经稳定的相机链路：

```bash
ros2 topic list | grep image
ros2 topic hz /image_raw
```

默认订阅原始图像：

```bash
bash scripts/start_debug_node.sh "find the bottle"
```

修改原始图像话题：

```bash
IMAGE_TOPIC=/image \
IMAGE_TRANSPORT=raw \
bash scripts/start_debug_node.sh "find the red cup"
```

订阅压缩图像：

```bash
IMAGE_TOPIC=/image_raw/compressed \
IMAGE_TRANSPORT=compressed \
bash scripts/start_debug_node.sh "find the bottle"
```

原始图像支持：

```text
bgr8 / rgb8 / bgra8 / rgba8 / mono8 / YUYV / UYVY / NV12 / NV21
```

---

## 6. 运行中修改任务和状态

修改任务：

```bash
ros2 topic pub --once /qwen_vln/instruction std_msgs/msg/String \
  "{data: 'find the red cup near the table'}"
```

手动切状态：

```bash
ros2 topic pub --once /qwen_vln/command std_msgs/msg/String "{data: 'observe'}"
ros2 topic pub --once /qwen_vln/command std_msgs/msg/String "{data: 'search'}"
ros2 topic pub --once /qwen_vln/command std_msgs/msg/String "{data: 'inferred'}"
ros2 topic pub --once /qwen_vln/command std_msgs/msg/String "{data: 'track'}"
ros2 topic pub --once /qwen_vln/command std_msgs/msg/String "{data: 'verify'}"
ros2 topic pub --once /qwen_vln/command std_msgs/msg/String "{data: 'pause'}"
ros2 topic pub --once /qwen_vln/command std_msgs/msg/String "{data: 'resume'}"
ros2 topic pub --once /qwen_vln/command std_msgs/msg/String "{data: 'reset'}"
```

---

## 7. Foxglove 话题

主要图像：

```text
/qwen_vln/annotated_image/compressed
```

调试话题：

```text
/qwen_vln/state
/qwen_vln/result_json
/qwen_vln/latency_ms
/qwen_vln/pixel_point
/qwen_vln/prompt_text
```

图像颜色：

- 绿色：真实目标点 `target`
- 黄色：语义搜索提示点 `search`
- 浅蓝：验证通过点 `verify`
- 灰色竖线：图像中心

默认只绘制当前结果，不叠加不同时间帧的历史点。否则相机一动，历史点画在新图上会非常热闹，也非常错误。

更重要的是：节点保存每次请求时的图像副本，API 返回后将像素点绘制在**对应请求帧**上，而不是绘制在几秒后的最新画面上。

---

## 8. JSON 输出协议

目标可见：

```json
{
  "result": "TARGET_VISIBLE",
  "point": {"x": 612, "y": 287},
  "point_role": "target",
  "label": "bottle",
  "reason_code": "exact_target_visible"
}
```

目标不可见但仍返回探索路点（新协议要求 `point` 永不为 null）：

```json
{
  "result": "TARGET_INFERRED",
  "point": {"x": 500, "y": 720},
  "point_role": "search",
  "label": "open corridor toward desk",
  "reason_code": "free_space_toward_desk"
}
```

`point` 使用 Qwen3-VL 相对坐标网格 `[0, 1000]`（不是像素，也不是 0～1）。
解析器会换算为送入 API 的缩放图像像素：

`pixel = round(coord / 1000 * size)` clamped to `[0, size-1]`

其中 `size` 为图像宽或高。0～1 归一化坐标会被拒绝。ROS 话题 `/qwen_vln/pixel_point` 与标注图上的点均为换算后的像素。

---

## 9. 测试

```bash
bash scripts/run_tests.sh
```

测试覆盖：

- JSON 解析；
- 坐标越界拒绝；
- 状态与结果类型匹配；
- `TARGET_INFERRED` 转移；
- 请求间隔；
- 手动状态切换使旧请求失效；
- 外部提示词文件加载。

---

## 10. V1 验收标准

建议进入底盘控制前至少达到：

- JSON 解析成功率 100%；
- 目标存在时可见判断率不低于 90%；
- 目标不存在时误锁率不高于 5%；
- 输出点落在目标主体上的比例不低于 90%；
- 同一场景重复调用，横向点位波动可接受；
- 连续运行 20 分钟，无请求堆积、无旧状态结果覆盖新任务；
- Foxglove 中点位始终画在对应 API 输入帧上。

下一版再接入目标可见时的本地视觉伺服和雷达安全层。
