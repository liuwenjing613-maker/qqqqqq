# 项目范围说明

## 为什么新建这个项目

`/root/rdk_x5_vln_robot` 中几乎所有导航模式都依赖 YOLO 检测：

- `yolov5s_bpu_web_node.py` → `/target_bbox_json`
- `hobot_yolo_world` + `yolo_world_to_bbox_json.py`
- 语义建图、语义探索、P0 failsafe 导航均以 bbox 为输入

原项目里的 Qwen（`src/vlm/`）只做：

- 指令文本解析（关键词 → target_classes）
- 探索候选点重排（可选，默认关闭）
- 到达后 Mock 验证

**不做图像级目标检测。**

## 本项目的定位

用 Qwen 多模态能力替代 YOLO：

1. 输入：相机图像 + 自然语言指令（如 "find the bottle"）
2. 输出：目标 bbox / 中心点 / 可见性 / 语义描述
3. 下游：复用类似的 FSM + 视觉伺服 + LiDAR 安全层（后续实现）

## 暂不包含

- 不修改 `/root/rdk_x5_vln_robot` 任何文件
- 不在此阶段复制整套 SLAM/Nav2/语义探索（可后续按需引入）
