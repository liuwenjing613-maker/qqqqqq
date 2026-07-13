# 模块与功能说明

## 1. `prompts/`

所有提示词均为外部文本文件，便于第一阶段独立调试，不需要修改程序逻辑。

- `common.txt`：统一 JSON、像素坐标和安全约束；
- `observe.txt`：判断目标是否可见；
- `track.txt`：重新定位已锁定目标；
- `search.txt`：生成语义搜索提示点或明确无提示；
- `verify.txt`：验证候选是否完整符合任务。

V1 不再沿用一个巨大提示词同时处理 `TARGET/PATH/NONE`。每次请求只做一类视觉任务，解析器也按当前状态限制合法结果。

## 2. `prompt_manager.py`

读取外部提示词，填入：

- 当前任务；
- API 图像宽高；
- 合法像素范围；
- 上一次目标点；
- JSON 输出结构。

也可用环境变量 `QWEN_PROMPT_DIR` 指向另一套提示词目录，方便 A/B 测试。

## 3. `qwen_client.py`

负责：

- OpenAI 兼容接口；
- `qwen3-vl-flash` 模型调用；
- `response_format={"type":"json_object"}`；
- 通过 `extra_body` 关闭思考模式；
- JPEG 编码和 Data URL；
- 严格 JSON、状态结果和像素坐标校验；
- 在解析失败时保留原始输出预览，便于定位问题。

解析器不会自动转换归一化坐标，因为第一阶段的目的就是确认模型能否按提示词稳定返回真实像素。

## 4. `state_machine.py`

状态参考大创中熟悉的结构：

```text
TARGET_LOCKED / TARGET_INFERRED / SEARCHING
```

并加入工程所需的：

```text
WAIT_IMAGE / OBSERVE / VERIFY / SUCCESS / PAUSED / ERROR
```

状态机当前只负责：

- 选择提示词模式；
- 控制请求间隔；
- 根据结果和置信度切状态；
- 手动状态切换；
- 使过期请求失效；
- API 异常冷却恢复。

它完全不发布速度。

## 5. `qwen_vln_debug_node.py`

ROS2 主节点负责：

- 订阅原始或压缩相机图像；
- 支持常见 RGB、灰度、YUV 和 NV12/NV21 编码；
- 缩放 API 图像；
- 单线程异步请求，避免阻塞 ROS 回调；
- 新任务或手动切状态后丢弃过期响应；
- 保存每次 API 输入帧；
- 发布结果、状态、延迟、提示词、像素点和标注图。

只允许一个 API 请求同时进行，因此不会出现请求越积越多、模型还在看三秒前画面的荒诞场景。

## 6. `visualizer.py`

绘制：

- 当前模型输出点；
- 图像中心线；
- 当前状态；
- 任务指令；
- 结果、置信度、延迟和请求编号；
- `reason_code`；
- API 请求状态；
- 错误信息；
- 当前显示帧来源。

## 7. `test_single_image.py`

用于脱离 ROS2 测试单张真车图片。它会生成：

```text
outputs/latest_result.json
outputs/latest_annotated.jpg
```

建议先用它完成提示词 A/B 测试，再启动实时相机节点。

## 8. 后续模块顺序

```text
V1：Qwen 状态机 + 严格像素点 + Foxglove
V2：目标可见时的视觉伺服 + 雷达安全覆盖
V3：出生环视 + 按 yaw 保存观察结果
V4：轻量短期记忆和失败点记录
V5：实时地图上的安全候选观察点
V6：YOLO-World 短时锚定和断网降级
```
