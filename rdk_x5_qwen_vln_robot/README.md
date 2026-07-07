# RDK X5 Qwen VLN Robot

独立于 `/root/rdk_x5_vln_robot` 的千问视觉导航项目，**不修改原仓库任何文件**。

## 环境变量

```bash
export DASHSCOPE_API_KEY="你的 API Key"
export QWEN_BASE_URL="https://<WorkspaceId>.cn-beijing.maas.aliyuncs.com/compatible-mode/v1"
export QWEN_MODEL="视觉模型名"
```

## 离线测试

```bash
cd /root/rdk_x5_qwen_vln_robot
python3 src/apps/test_qwen_dashscope_client_image.py \
  --image angle1.jpg --instruction "find bottle"
```

## 启动（默认直接驱动底盘，发布 /cmd_vel）

```bash
bash scripts/nav/start_qwen_api_lidar_nav.sh "find bottle"
```

默认会启动底盘桥（`RUN_CHASSIS=1`），速度发到 `/cmd_vel`。若只想观察不动车：

```bash
RUN_CHASSIS=0 bash scripts/nav/start_qwen_api_lidar_nav.sh "find bottle"
```

启动脚本会**只读调用**原仓库的相机桥、雷达 launch 和底盘桥（`RDK_ORIGINAL_ROOT=/root/rdk_x5_vln_robot`），不修改原仓库。

## 文件结构

```
src/vlm/qwen_dashscope_client.py      # 云端千问客户端
src/apps/run_qwen_api_lidar_nav.py    # ROS 导航节点
src/apps/test_qwen_dashscope_client_image.py
src/perception/lidar_depth.py
src/control/qwen_lidar_point_servo.py
configs/qwen_api_lidar_nav.yaml
scripts/nav/start_qwen_api_lidar_nav.sh
```

## 串口与速度

与 `/root/rdk_x5_vln_robot/configs/nav_yolo_lidar_semantic_explore.yaml` 对齐：

- 底盘串口默认 `/dev/ttyUSB0`（`chassis.port`）
- 伺服：`max_vx=0.06`, `max_wz=0.05`, `kp_turn=0.1`
- 底盘桥 PWM 参数来自 yaml `chassis` 块 + 原项目 `load_mvp_tune.sh`

临时覆盖串口：

```bash
CHASSIS_PORT=/dev/ttyUSB1 bash scripts/nav/start_qwen_api_lidar_nav.sh "find bottle"
```
