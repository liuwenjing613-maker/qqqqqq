# V1 + 在线地图候选 + Qwen + Nav2 全流程 V2

适配目录：`/root/rdk_x5_vln_robot/rdk_x5_qwen3_vln_debug_v1`

## 1. 这次解决的真实缺口

仓库中原有 `start_v1_online_map_plan_fusion.sh` 已完成协议桥、介入管理器与速度仲裁，但默认不启动实时地图栈，也没有可执行的真实在线后端。它要求外部已经发布：

- `/map_qwen_plan/candidate_summary`
- `/map_qwen_plan/status`
- `/map_qwen_plan/cmd_vel`

因此它是“接口连通”，还不是“一条命令完成候选、Qwen、规划和行驶”。本 V2 新增真实后端和统一总启动器。

## 2. 一条命令启动

首次配置密钥：

```bash
cd /root/rdk_x5_vln_robot
cp -n .env.example .env
# 编辑 .env，填写新的 DASHSCOPE_API_KEY
```

先离线检查：

```bash
cd /root/rdk_x5_vln_robot
bash rdk_x5_qwen3_vln_debug_v1/scripts/fusion/run_fullflow_v2_tests.sh
```

抬轮或禁用运动启动全栈：

```bash
cd /root/rdk_x5_vln_robot
MAP_QWEN_DRY_RUN=1 \
MOTION_ENABLED=0 \
bash rdk_x5_qwen3_vln_debug_v1/scripts/fusion/start_v1_map_qwen_fullflow_v2.sh \
  --task 'find the bottle'
```

真车启动：

```bash
cd /root/rdk_x5_vln_robot
set -a && source .env && set +a
MOTION_ENABLED=1 \
bash rdk_x5_qwen3_vln_debug_v1/scripts/fusion/start_v1_map_qwen_fullflow_v2.sh \
  --task 'find the bottle'
```

保持该终端开启。`Ctrl+C` 会停止该启动器创建的 V1、后端、Nav2，以及在本次运行中创建的 SLAM。

## 3. 启动顺序

总启动器自动完成：

1. 加载 TROS/ROS2 与 DDS 环境；
2. 检查配置、脚本和节点文件；
3. 如 `/map`、`/odom`、`/scan_filtered` 已存在，则复用实时 SLAM；
4. 否则运行 `scripts/slam/run_slam_calibrated.sh`；
5. 等待地图、里程计、雷达和 `map -> base_link` TF；
6. 生成临时 Nav2 参数，启用 A*，降低速度；
7. 只启动 Nav2 导航组件，不启动第二个 SLAM、map_server 或 AMCL；
8. 将 Nav2 最终速度改发 `/map_qwen_plan/cmd_vel_raw`，禁止绕过融合仲裁；
9. 启动真实在线后端；
10. 启动现有 bridge、介入管理器、EGO/MAP/HOLD mux 和 V1；
11. 持续监控关键进程，任一安全关键进程退出则停止全栈。

## 4. 运行中的数据流

```text
/map + map->base_link
  -> online_map_qwen_nav_backend_v2
  -> 安全 Frontier 候选
  -> /map_qwen_plan/candidate_summary
  -> existing bridge
  -> /third_view/candidate_summary
  -> existing intervention manager

需要介入：
intervention request
  -> existing bridge
  -> backend request
  -> 候选复验
  -> 单候选直接选 / 多候选 Qwen 选 ID
  -> NavigateToPose
  -> /map_qwen_plan/cmd_vel_raw
  -> backend request-id 门控与限速
  -> /map_qwen_plan/cmd_vel
  -> existing bridge
  -> /cmd_vel_map
  -> existing EGO/MAP/HOLD mux
  -> /cmd_vel_autonomy
  -> existing joystick priority mux
  -> /cmd_vel
```

## 5. 候选点规则

V2 后端直接读取实时 `OccupancyGrid`：

- `-1` 为未知；
- `0~20` 为自由区；
- `>=65` 为障碍；
- 障碍膨胀半径默认 `0.30m`；
- 候选必须位于膨胀后的已知自由区；
- 候选必须与机器人处于同一自由空间连通分量；
- 距离默认限制 `0.50~2.60m`；
- 一个大型连续 Frontier 会按机器人相对方位拆分，防止 T 字路口被错误压成单一候选；
- 候选最终仍由 Nav2 做路径与碰撞检查。

本版不使用长期轨迹和视觉覆盖，不会伪造“未重复探索”的结论。

## 6. Qwen 规则

- 只有多个候选时调用地图 Qwen；
- 只允许输出现有候选 ID；
- 禁止自由生成世界坐标；
- 输出无效、超时或没有密钥时，可退回几何最高分候选；
- `MAP_QWEN_DRY_RUN=1` 可强制使用几何回退测试完整控制链。

## 7. 路径与最终朝向

- Nav2 使用实时 `/map` 的 StaticLayer；
- planner 的 `use_astar` 临时改为 `true`；
- Nav2 只负责到达目标 `x/y`；
- 到达后后端根据 `map -> base_link` 低速旋转到候选 `yaw`；
- 连续 `0.40s` 保持在 `8°` 内，才上报 `COMPLETED + final_orientation_done=true`；
- 然后现有介入管理器 HOLD、请求一帧新的第一视角结果，再恢复 EGO。

## 8. 状态检查

```bash
bash rdk_x5_qwen3_vln_debug_v1/scripts/fusion/check_v1_map_qwen_fullflow_v2.sh
```

重点话题：

```bash
ros2 topic echo /map_qwen_plan/backend_debug
ros2 topic echo /map_qwen_plan/bridge_status
ros2 topic echo /third_view/intervention/status
ros2 topic echo /third_view/intervention/decision
ros2 topic echo /map_qwen_plan/candidate_summary
```

速度发布者检查：

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_autonomy -v
ros2 topic info /cmd_vel_map -v
ros2 topic info /map_qwen_plan/cmd_vel_raw -v
```

Nav2 不应直接成为 `/cmd_vel` 发布者。

## 9. 日志

```text
rdk_x5_qwen3_vln_debug_v1/logs/fullflow_v2/slam.log
rdk_x5_qwen3_vln_debug_v1/logs/fullflow_v2/nav2.log
rdk_x5_qwen3_vln_debug_v1/logs/fullflow_v2/backend.log
rdk_x5_qwen3_vln_debug_v1/logs/fullflow_v2/fusion_v1.log
```

每次地图 Qwen 请求还会保存候选图、提示词和原始响应。

## 10. 真车测试顺序

1. `run_fullflow_v2_tests.sh`；
2. `MOTION_ENABLED=0 MAP_QWEN_DRY_RUN=1` 启动；
3. 确认所有话题、Action 和 TF；
4. 抬轮，设置 `MOTION_ENABLED=1 MAP_QWEN_DRY_RUN=1`；
5. 手动触发多候选，确认 EGO -> HOLD -> MAP -> HOLD -> EGO；
6. 地面低速测试几何回退；
7. 启用真实地图 Qwen；
8. 测试后端超时、错误 ID、Nav2 无路径、目标中途出现和手柄覆盖；
9. 最后测试完整 T 字路口和目标搜索。

## 11. 停止

前台正常使用 `Ctrl+C`。异常遗留时：

```bash
bash rdk_x5_qwen3_vln_debug_v1/scripts/fusion/stop_v1_map_qwen_fullflow_v2.sh
```

## 12. 当前边界

已经连通：

- 实时地图；
- 在线安全候选；
- 单候选直达；
- 多候选 Qwen 选择；
- A* / Nav2；
- 最终朝向；
- 控制权安全切换；
- 目标出现后取消地图导航；
- 返回第一视角。

尚未加入：

- 已走轨迹；
- 相机视野覆盖；
- 拓扑分岔记忆；
- 长期候选黑名单。

这些内容后续应在候选生成和介入判定之前增加，不需要推翻本 V2 的在线协议与速度仲裁。
