# V1 第一视角与实时地图 Qwen/A* 在线融合说明

## 1. 版本定位

本补丁面向仓库：

```text
liuwenjing613-maker/qqqqqq
branch: fix/semantic-explore-direction-lock
```

依赖当前 `rdk_x5_qwen3_vln_debug_v1` 中已经存在并基本通过测试的：

- `src/intervention/core.py`
- `src/intervention/mux_logic.py`
- `src/apps/third_view_intervention_node.py`
- `src/apps/cmd_vel_intervention_mux.py`
- `scripts/intervention/start_live_servo_with_intervention.sh`
- `configs/qwen3_vln_servo.yaml` 中的 `third_view_intervention`

本阶段只接通以下闭环：

```text
V1 第一视角导航
  -> 介入条件成立
  -> 实时地图提取候选点
  -> 单候选直接选 / 多候选 Qwen 选择
  -> 在线路径规划并执行
  -> 到达或失败
  -> 切回第一视角
```

暂不依赖轨迹记忆、视觉覆盖记忆和历史分岔图。配置中保持：

```yaml
integration:
  use_memory: false
```

## 2. 为什么不能直接嵌套旧会话脚本

`run_joy_map_qwen_plan_session.sh` 是一个“人工建图调试会话”，不是运行时可重入的导航服务。它的主要阶段是：

1. 启动或附着手柄建图；
2. 启动路径/Frontier 标注；
3. 等待终端输入 `OK`；
4. 保存地图、轨迹、位姿和标注图片；
5. 调用 `qwen_live_session_planner.py` 生成 `goal_pose_map`；
6. 自动导航时停止 SLAM、手柄、雷达桥和 Foxglove，再冷启动 Nav2。

把整个脚本从正在运行的 V1 中调用会导致：

- 等待人工 `OK`，无法事件触发；
- 每次介入都写文件和保存地图，延迟与故障点过多；
- 可能杀掉当前实时 SLAM、雷达和控制栈；
- 旧流程直接使用 `/cmd_vel`，会与 V1 控制器争抢；
- 导航完成后没有与 V1 的安全恢复握手；
- 不具备请求 ID，旧状态可能误确认新任务。

因此，本补丁保留它真正有价值的逻辑：

```text
实时地图 -> 程序候选 -> Qwen 二阶段选择 -> 完整目标位姿 -> 路径规划
```

同时把“文件会话边界”替换成“带 request_id 的在线 ROS 消息边界”。

## 3. 最终架构

```text
                         /qwen_vln/servo/status
                                   |
                                   v
V1 Qwen+雷达 -> /cmd_vel_ego -> Intervention Manager
                                      |
                  /third_view/intervention/request
                                      |
                                      v
                           Online Map Plan Bridge
                         /                 |       \
        candidate probe /                  |        \ status
                       v                   v         v
            实时地图候选提取        Qwen 候选选择     A*/路径执行
                       \                   |         /
                        \                  v        /
                         +---- teammate backend ---+
                                      |
                         /map_qwen_plan/cmd_vel
                                      |
                                      v
                        bridge 限速/超时/请求门控
                                      |
                              /cmd_vel_map
                                      |
                  EGO/MAP/HOLD velocity mux
                                      |
                         /cmd_vel_autonomy
                                      |
                           原有 joy 优先级 mux
                                      |
                                  /cmd_vel
```

### 控制权优先关系

```text
手柄人工覆盖
  > 第一视角雷达紧急后退
  > 地图路径控制
  > 普通第一视角控制
  > 零速度 HOLD
```

地图后端禁止直接发布：

```text
/cmd_vel
/cmd_vel_autonomy
/cmd_vel_ego
```

它只能发布：

```text
/map_qwen_plan/cmd_vel
```

## 4. 新增模块

### 4.1 `src/fusion/online_map_protocol.py`

无 ROS 依赖的确定性协议层，负责：

- 兼容队友候选字段别名；
- 规范化候选点摘要；
- 将 `MAP_DIRECT/MAP_QWEN` 翻译为后端操作；
- 规范化后端状态；
- request_id 过滤；
- 最终朝向门控；
- 地图速度限幅；
- 后端命令和心跳超时；
- 取消后立即清零。

后端操作分三类：

```text
NAVIGATE_CANDIDATE
  MAP_DIRECT 且恰好一个候选，跳过 Qwen。

SELECT_AND_NAVIGATE
  已有多个候选，Qwen 只在候选 ID 中选择。

EXTRACT_SELECT_AND_NAVIGATE
  行为失败触发，但当前候选摘要缺失或过期，先提候选再选择。
```

### 4.2 `src/apps/online_map_plan_bridge_node.py`

ROS2 在线桥接节点。它不是第二个规划器，只做边界控制：

- 转发介入请求；
- 自动补充当前任务、地图/里程计/雷达话题；
- 附带最近候选快照和 `map_version`；
- 转发取消；
- 规范化后端状态给现有介入管理器；
- 候选摘要转发给 `InterventionCore`；
- 后端未确认时持续输出新鲜零速度；
- 后端状态未匹配当前 request_id 时全部忽略；
- 只有后端先发匹配状态后，速度才获得授权；
- 对速度进行 `0.06 m/s`、`0.06 rad/s` 限幅；
- 速度消息过期 `0.40 s` 自动变零；
- 后端心跳过期自动失败并回第一视角。

### 4.3 `configs/online_map_plan_fusion.yaml`

独立配置覆盖层。默认：

```yaml
online_map_plan_fusion:
  enabled: false
```

关闭时新桥、新后端、新速度通道都不会启动。

### 4.4 `scripts/fusion/start_v1_online_map_plan_fusion.sh`

统一入口：

- 关闭时直接执行原 V1；
- 开启时生成临时运行配置；
- 启动在线桥；
- 调用现有 Intervention Manager + EGO/MAP/HOLD mux；
- 可选择附着现有建图/后端，或由配置启动它们；
- 任一安全关键进程退出，整体自动清理并停车。

### 4.5 `scripts/qwen_servo/make_intervention_servo_config.py`

当前分支的 Intervention 包装脚本调用的是：

```text
scripts/qwen_servo/make_intervention_servo_config.py
```

而仓库中的同名实现位于另一个目录。本补丁在包装脚本实际调用的位置增加兼容实现，生成临时配置并将 V1 输出改到 `/cmd_vel_ego`。它不修改原 YAML。

### 4.6 `src/apps/mock_online_map_plan_backend.py`

用于不接队友真模块时验证：

- 候选探测；
- 多候选；
- 单候选；
- ACK；
- 规划；
- 导航；
- 完成；
- 失败；
- 目标中途出现；
- 取消。

默认永远发布零速度。只有显式加入 `--motion-test` 才输出测试速度。

## 5. 后端接口契约

### 5.1 候选探测请求

桥在 EGO 阶段按 1 Hz 发布：

```text
/map_qwen_plan/candidate_probe
```

```json
{
  "protocol_version": 1,
  "probe_id": "probe-00000001",
  "operation": "EXTRACT_CANDIDATES_ONLY",
  "instruction": "find the bottle",
  "live_inputs": {
    "map_topic": "/map",
    "odom_topic": "/odom",
    "scan_topic": "/scan_filtered"
  },
  "options": {
    "call_qwen": false,
    "execute_navigation": false,
    "use_memory": false
  }
}
```

这个阶段只能做便宜的几何候选提取，不能调用 Qwen，不能导航。

### 5.2 候选摘要

队友模块发布：

```text
/map_qwen_plan/candidate_summary
```

推荐格式：

```json
{
  "stamp": 1784300000.0,
  "map_seq": 88,
  "distance_to_decision_m": 0.74,
  "candidate_points": [
    {
      "candidate_id": "F_LEFT",
      "relative_heading_deg": 65.0,
      "final_score": 0.73,
      "is_reachable": true,
      "visited": false,
      "status": "UNSEEN",
      "path_length": 1.8,
      "goal_pose": {"x": 1.0, "y": 0.8, "yaw": 1.1}
    }
  ]
}
```

桥接受以下常见别名：

```text
id / candidate_id / frontier_id / region_id
heading_deg / relative_heading_deg / yaw_rel_deg / angle_deg
score / final_score / candidate_score / utility
pose / goal_pose / world_pose
```

### 5.3 正式请求

桥发布：

```text
/map_qwen_plan/request
```

```json
{
  "protocol_version": 1,
  "request_id": "intervention-000001",
  "operation": "SELECT_AND_NAVIGATE",
  "decision": "MAP_QWEN",
  "reason_code": "BRANCH_AMBIGUOUS",
  "instruction": "find the bottle",
  "candidate_ids": ["F_LEFT", "F_RIGHT"],
  "candidate_snapshot": [
    {"id": "F_LEFT", "pose": {"x": 1.0, "y": 0.8, "yaw": 1.1}}
  ],
  "map_version": "88",
  "robot_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
  "options": {
    "use_live_map": true,
    "use_memory": false,
    "keep_mapping_alive": true,
    "return_full_goal_pose": true,
    "execute_navigation": true
  }
}
```

后端必须重新验证：

- 候选 ID 仍存在；
- 候选仍在自由空间；
- 地图版本变化后目标仍有效；
- 路径可达；
- 目标位姿包含最终 yaw；
- A* 失败时不得自行改发无关旧目标。

### 5.4 后端状态

发布：

```text
/map_qwen_plan/status
```

每条状态都必须带当前 `request_id`：

```json
{"request_id":"intervention-000001","state":"ACCEPTED"}
{"request_id":"intervention-000001","state":"QWEN_SELECTING"}
{"request_id":"intervention-000001","state":"PLANNING"}
{"request_id":"intervention-000001","state":"NAVIGATING"}
{"request_id":"intervention-000001","state":"ARRIVED_ALIGNED","final_orientation_done":true}
```

失败：

```json
{
  "request_id":"intervention-000001",
  "state":"PLAN_FAILED",
  "reason":"candidate became unreachable"
}
```

目标中途出现：

```json
{"request_id":"intervention-000001","state":"TARGET_VISIBLE"}
```

### 5.5 地图速度

发布：

```text
/map_qwen_plan/cmd_vel
```

类型：

```text
geometry_msgs/msg/Twist
```

桥只在收到匹配 request_id 的有效后端状态后转发速度，并进行限幅和新鲜度检查。

### 5.6 取消

后端订阅：

```text
/map_qwen_plan/cancel
```

```json
{
  "protocol_version": 1,
  "request_id": "intervention-000001",
  "reason": "fresh_first_person_target_visible"
}
```

收到后必须立即：

1. 取消路径规划和当前 goal；
2. 停止速度发布或只发零速度；
3. 清理该 request_id；
4. 不允许旧异步 Qwen 回调重新发 goal。

## 6. 开关和启动

### 6.1 安装

ZIP 在 `/root/rdk_x5_vln_robot` 解压：

```bash
cd /root/rdk_x5_vln_robot
unzip -o rdk_x5_vln_v1_online_map_fusion_patch.zip
```

### 6.2 关闭模式回归

保持：

```yaml
online_map_plan_fusion:
  enabled: false
```

运行：

```bash
bash rdk_x5_qwen3_vln_debug_v1/scripts/fusion/start_v1_online_map_plan_fusion.sh \
  "find the bottle"
```

此时直接进入原 `start_live_servo.sh`，不启动新节点。

### 6.3 开启融合

修改：

```yaml
online_map_plan_fusion:
  enabled: true
```

如果实时建图和队友后端已经由其他终端启动，保持：

```yaml
startup:
  start_map_stack: false
  start_backend: false
```

再启动：

```bash
MOTION_ENABLED=1 \
FUSION_CONFIG=/root/rdk_x5_vln_robot/rdk_x5_qwen3_vln_debug_v1/configs/online_map_plan_fusion.yaml \
bash /root/rdk_x5_vln_robot/rdk_x5_qwen3_vln_debug_v1/scripts/fusion/start_v1_online_map_plan_fusion.sh \
  "find the bottle"
```

## 7. 推荐测试顺序

### 阶段 0：静态和单元测试

```bash
cd /root/rdk_x5_vln_robot/rdk_x5_qwen3_vln_debug_v1
bash scripts/fusion/run_all_fusion_tests.sh
```

要求：

- 原介入测试全部通过；
- 新协议 9 个测试通过；
- `py_compile` 通过；
- shell 语法通过；
- 全流程模拟显示 `SIMULATION PASS`。

### 阶段 1：关闭开关回归

- `enabled=false`；
- 运行原任务 3 次；
- 确认没有 `/map_qwen_plan/bridge_status`；
- 确认第一视角速度和原版本一致；
- 确认出生环视、TURN、紧急后退、成功停止均不变。

### 阶段 2：Mock 后端，底盘禁用

终端 A：

```bash
bash scripts/fusion/start_mock_backend.sh multi_success
```

终端 B：

```bash
MOTION_ENABLED=0 bash scripts/fusion/start_v1_online_map_plan_fusion.sh "find the bottle"
```

检查：

```bash
ros2 topic echo /map_qwen_plan/bridge_status
ros2 topic echo /third_view/intervention/status
ros2 topic echo /third_view/navigation_status
ros2 topic echo /cmd_vel_map
```

`MOTION_ENABLED=0` 时介入核心的硬屏蔽必须阻止正式 A* 请求。

### 阶段 3：手工注入介入请求，底盘抬轮

直接向桥发请求，验证不依赖介入条件：

```bash
ros2 topic pub --once /third_view/intervention/request std_msgs/msg/String \
  "{data: '{\"request_id\":\"manual-1\",\"decision\":\"MAP_QWEN\",\"reason_code\":\"MANUAL_TEST\",\"candidate_ids\":[\"F_LEFT\",\"F_RIGHT\"]}'}"
```

要求：

- 桥立即发送零 `/cmd_vel_map`；
- Mock 状态匹配后才允许速度；
- `--motion-test` 前不能出现非零地图速度；
- 旧 request_id 状态不能影响当前任务。

### 阶段 4：轮子抬起，测试控制源互斥

Mock 使用：

```bash
bash scripts/fusion/start_mock_backend.sh multi_success --motion-test
```

检查：

- EGO 时只有 `/cmd_vel_ego` 生效；
- STOPPING/WAIT_ACK 时 `/cmd_vel_autonomy=0`；
- MAP 时只转发 `/cmd_vel_map`；
- 地图完成后先 HOLD，再等新第一视角结果，最后恢复 EGO；
- 地图模式中触发雷达紧急后退时，紧急后退覆盖地图速度。

### 阶段 5：真候选，禁用真路径速度

让队友模块：

- 读取真实 `/map`；
- 响应候选 probe；
- 发布真实候选点；
- Qwen 选择候选；
- 发布规划路径和目标，但 `/map_qwen_plan/cmd_vel` 固定为零。

验证：

- T 字路口产生两个方向分离候选；
- 连续两次候选摘要后触发 `BRANCH_AMBIGUOUS`；
- Qwen 只能选候选 ID；
- 目标点在膨胀自由空间；
- A* 路径可达；
- `goal_pose` 含 yaw。

### 阶段 6：单候选 MAP_DIRECT

构造只有一个合法候选：

- 不调用 Qwen；
- request operation 为 `NAVIGATE_CANDIDATE`；
- 直接规划该候选；
- 到达后按握手切回第一视角。

### 阶段 7：多候选 MAP_QWEN 真车低速

首次地面测试参数保持：

```yaml
max_linear_x: 0.04
max_angular_z: 0.04
```

稳定后再恢复补丁默认 `0.06/0.06`。

### 阶段 8：故障注入

必须逐项验证：

1. 后端不回复：8 秒失败并恢复 EGO；
2. 后端只 ACK 不发速度：保持零速度；
3. 速度停止更新：0.40 秒自动归零；
4. 错 request_id：忽略；
5. 候选过期：后端拒绝并返回 FAILED；
6. A* 无路径：FAILED 后恢复；
7. 地图导航中目标出现：取消地图并切回视觉伺服；
8. 桥进程退出：启动脚本清理整个自动栈；
9. Intervention Manager 退出：原 wrapper 清理整个自动栈；
10. 手柄接管：始终能覆盖自动控制。

## 8. 第一阶段明确不做的内容

- 已走路径重合检测；
- 相机视野投影覆盖；
- 语义拓扑节点；
- 失败候选长期黑名单；
- 回到历史路口后的未探索分支记忆；
- 周期全局审计；
- 旧会话地图文件与当前在线地图的混合定位。

这些内容后续应接在候选生成与介入判断之前，不应改动本补丁已经稳定的控制握手。

## 9. 验收标准

第一阶段融合完成应满足：

- 关闭开关时原 V1 完全可用；
- 正常第一视角导航不被地图模块打断；
- 分岔或第一视角持续失败时能发唯一 request_id；
- 后端候选、Qwen 选择、路径规划完整走通；
- 任意时刻只有一个自动速度源生效；
- 无 ACK、无速度、旧状态、旧异步回调均不能让底盘运动；
- 目标中途可见时 A* 被取消；
- 到达后停止、观察、获得新第一视角结果后再恢复；
- 不停止实时 SLAM，不冷启动另一套占据同一控制话题的 Nav2 栈。
