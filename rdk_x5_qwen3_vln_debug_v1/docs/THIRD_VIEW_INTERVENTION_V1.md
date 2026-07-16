# 第三视角介入转移模块 V1

适配基线：`fix/semantic-explore-direction-lock`，提交 `85739b0`  
适配目录：`rdk_x5_qwen3_vln_debug_v1`

## 1. 模块目标

该模块不替代现有第一视角 Qwen + 雷达导航，也不负责生成地图候选点。它只负责：

1. 持续读取现有伺服状态、里程计和队友候选点摘要；
2. 判断第一视角导航是否仍应继续；
3. 在真正的分岔或局部导航失败时，请求第三视角候选分析；
4. 以 `HOLD → MAP → HOLD → EGO` 的顺序安全切换速度控制权；
5. 第三视角无响应、失败或超时后，自动恢复现有第一视角流程。

V1 输出四种决策：

| 决策 | 含义 |
|---|---|
| `KEEP_EGO` | 保持现有第一视角 VLM + 雷达导航 |
| `RECOVERY` | 先使用现有 TURN / 紧急后退等局部恢复，不调用地图 Qwen |
| `MAP_DIRECT` | 只有一个合法候选，队友模块直接规划，不调用地图 Qwen |
| `MAP_QWEN` | 存在多个合理候选，调用第三视角 Qwen 排序后规划 |

## 2. 总体控制链

关闭模块时：

```text
qwen_visual_servo_node
  └─ /cmd_vel_autonomy
       └─ 原有 joy 优先级 mux
            └─ /cmd_vel
```

开启模块时，启动脚本动态生成运行时配置，不修改原配置中的 `topics.cmd_output`：

```text
qwen_visual_servo_node
  └─ /cmd_vel_ego ─┐
                    ├─ cmd_vel_intervention_mux ─ /cmd_vel_autonomy
地图 A*/控制器      │
  └─ /cmd_vel_map ─┘
                              └─ 原有 joy 优先级 mux ─ /cmd_vel
```

`cmd_vel_intervention_mux` 只有三种请求模式：

- `EGO`：仅转发 `/cmd_vel_ego`；
- `MAP`：仅转发 `/cmd_vel_map`；
- `HOLD`：持续发布零速度。

此外，MAP 模式下仍订阅现有伺服节点的雷达安全状态：紧急后退激活时临时转发 `/cmd_vel_ego` 的后退命令；安全状态超过 0.8 秒未更新时，MAP 输出直接降级为 HOLD。这样 A* 不能绕过当前已经验证过的前向雷达安全层。

原有手柄优先级 mux 保持在下游，所以手柄覆盖能力不受影响。

## 3. 开关与兼容性

配置文件：`configs/qwen3_vln_servo.yaml`

```yaml
third_view_intervention:
  enabled: false
```

- `false`：包装脚本立即回到原始 `start_live_servo.sh`，原有 Qwen 节点、伺服节点、话题和速度链路全部保持不变；
- `true`：才启动介入判断节点与速度 mux，并把第一视角速度动态重映射到 `/cmd_vel_ego`。

没有修改现有提示词、Qwen 输出协议、伺服控制算法、出生环视逻辑和 TURN 逻辑。

## 4. V1 触发条件

### 4.1 硬屏蔽条件

以下任意条件成立时均返回 `KEEP_EGO`：

- `MOTION_ENABLED=0`；
- 启动保护期未结束；
- `/qwen_vln/servo/status` 或 `/odom` 过期；
- 状态为 `WAIT_IMAGE / SPAWN_SCAN / PAUSED / SUCCESS / ERROR`；
- 出生环视仍在转动、停稳、评分或返回最佳方向；
- 显式 TURN 正在等待、转动或稳定画面；
- 紧急后退正在执行；
- 当前结果已确认目标可见；
- 第三视角流程已经处于 `STOPPING / WAIT_ACK / MAP_NAV / RESUMING`；
- 冷却时间未结束。

这些屏蔽条件的目的，是防止正常 TURN、出生环视或安全后退被误判为全局导航失败。

### 4.2 `BRANCH_AMBIGUOUS`

持续两次候选更新同时满足：

- 至少两个合法、可达、未访问候选；
- 最大方向夹角至少 `50°`；
- 必须提供 `decision_distance_m`，且决策区域距离机器人不超过 `1.30 m`；
- 几何评分前两名差值不超过 `0.12`，或比值不超过 `1.25`。

输出：`MAP_QWEN`。

候选数量很多但几何第一名明显占优时，不调用 Qwen。地图大模型不需要参加一眼就能看懂的选择题。

### 4.3 `RETURNED_JUNCTION_*`

队友记忆模块连续两次报告机器人已回到历史分岔节点：

- 无合法未探索候选：`RECOVERY / RETURNED_JUNCTION_EMPTY`；
- 恰好一个合法候选：`MAP_DIRECT / RETURNED_JUNCTION_SINGLE`；
- 两个及以上合法候选：`MAP_QWEN / RETURNED_JUNCTION_MULTI`。

如果摘要声称“只有一个未探索分支”，但实际候选列表里仍有多个合法项，V1 不会随便取第一个，而是按多候选处理。

### 4.4 `NO_PROGRESS_AFTER_RECOVERY`

第一阶段判断窗口为 `6 s`：

- 正向速度命令至少在 55% 时间内大于 `0.025 m/s`；
- 积分命令距离至少 `0.14 m`；
- 实际净位移不超过 `0.08 m`。

第一次满足时只输出：

```text
RECOVERY / NO_PROGRESS_LOCAL_RECOVERY
```

此时仍让现有 TURN、雷达停止和紧急后退逻辑先处理。经过 `4 s` 恢复宽限后，再观察一个完整 `6 s` 窗口；第二次仍无进展，才升级到地图模块：

- 一个合法候选：`MAP_DIRECT`；
- 多个合法候选：`MAP_QWEN`；
- 候选摘要暂时缺失：请求地图模块先提取候选再分析；
- 有摘要但零合法候选：`RECOVERY`。

恢复期间只要实际移动超过 `0.18 m`，升级流程立即取消。

### 4.5 `ACTION_OSCILLATION`

只记录新的 Qwen `request_id`，最近 8 个结果至少跨越 `4 s`，且机器人位移不超过 `0.22 m`，再检查：

- `TURN_LEFT / TURN_RIGHT` 至少翻转 3 次；或
- 横向误差越过 `±0.15` 后，左右符号至少翻转 4 次。

这样正常 S 形通道不会因为像素点左右变化就触发，只有“反复改方向但没走出去”才升级。

### 4.6 `REPEATED_EMERGENCY_REVERSE`

按紧急后退的上升沿计数，`15 s` 内达到 2 次后触发。单次临时障碍或雷达毛刺不会立刻请求第三视角。

## 5. 参数为何这样设置

当前第一视角控制最大线速度约为 `0.07 m/s`，Qwen 请求间隔约为一秒量级，出生环视使用 6 个 `60°` 扇区。因此：

- 分岔方向阈值使用 `50°`，既排除同一 Frontier 簇的重复候选，也能识别 T 字口和明显岔路；
- 6 秒无进展窗口足够覆盖多次 20 Hz 控制输出，并避开单次 API 等待；
- `0.14 m` 命令积分距离约等于持续 `0.04 m/s` 前进 3.5 秒，不会把大量停等时间当成卡住；
- `0.08 m` 实际位移高于普通里程计微小噪声，又明显低于一次有效推进；
- 8 个 Qwen 结果能够覆盖约 6 至 8 秒，避免单帧错误触发；
- 12 秒冷却用于防止同一地图结构重复请求；
- 候选摘要允许 2.5 秒新鲜度，适配 0.5 至 1 Hz 候选提取频率。

这些是首轮真车参数，不应在未记录日志的情况下同时改动多项。调参不留证据，是机器人开发里一种很传统的民俗活动。

## 6. 与队友模块的接口合同

所有接口均使用 ROS2 `std_msgs/String` 承载 JSON，便于当前工程快速联调。

### 6.1 候选点摘要

发布话题：`/third_view/candidate_summary`

建议频率：`0.5 ~ 1 Hz`。`heading_deg` 必须是以机器人当前朝向为 0° 的相对方向，`score` 建议统一归一化到 0~1。

```json
{
  "map_version": "map-28",
  "decision_distance_m": 0.82,
  "returned_to_junction": false,
  "junction_id": null,
  "unseen_candidate_count": 2,
  "candidates": [
    {
      "id": "F12",
      "heading_deg": -68.0,
      "score": 0.72,
      "reachable": true,
      "visited": false,
      "status": "UNSEEN",
      "path_length": 1.9
    },
    {
      "id": "F18",
      "heading_deg": 57.0,
      "score": 0.68,
      "reachable": true,
      "visited": false,
      "status": "UNSEEN",
      "path_length": 2.2
    }
  ]
}
```

`status` 为以下值时，该候选不会参与选择：

```text
BLOCKED / FAILED / EXHAUSTED / BLACKLISTED
```

### 6.2 介入请求

订阅话题：`/third_view/intervention/request`

```json
{
  "request_id": "intervention-000001",
  "decision": "MAP_QWEN",
  "reason_code": "BRANCH_AMBIGUOUS",
  "candidate_ids": ["F12", "F18"],
  "evidence": {},
  "robot_pose": {"x": 1.2, "y": 2.4, "yaw": 0.31},
  "expected_navigation_status_topic": "/third_view/navigation_status",
  "map_cmd_topic": "/cmd_vel_map"
}
```

处理规则：

- `MAP_DIRECT`：直接采用请求中的唯一 `candidate_id`；
- `MAP_QWEN`：在请求候选中进行地图 Qwen 排序；
- `candidate_ids=[]`：表示行为失败已确认，但候选摘要缺失，队友模块应先运行候选提取，再调用 Qwen；
- 最终送给 A* 的候选仍必须重新验证可达性。

### 6.3 导航状态

发布话题：`/third_view/navigation_status`

所有状态必须带回相同 `request_id`：

```json
{"request_id":"intervention-000001","status":"ACCEPTED"}
{"request_id":"intervention-000001","status":"NAVIGATING"}
{"request_id":"intervention-000001","status":"COMPLETED"}
```

支持状态：

- 接收/执行：`ACCEPTED / PLANNING / NAVIGATING / RUNNING / ACTIVE`；
- 成功：`COMPLETED / REACHED / SUCCEEDED / DONE`；
- 失败：`FAILED / REJECTED / CANCELLED / ABORTED / TIMEOUT`；
- 目标事件：`TARGET_VISIBLE / TARGET_LOCKED`。

### 6.4 地图速度与取消

- 地图控制器发布：`/cmd_vel_map`；
- 订阅取消：`/third_view/intervention/cancel`；
- 介入管理器在 6 秒内未收到 ACK，会恢复 EGO；
- 地图导航超过 120 秒，会发布取消并恢复 EGO；
- 地图导航期间第一视角 Qwen 仍继续观察；若出现新的 `TARGET_VISIBLE` 结果，会立即取消 A*。

## 7. 安全切换状态机

```text
EGO
 ├─ MAP_DIRECT / MAP_QWEN
 ▼
STOPPING       发布 HOLD，等待 0.30 s
 ▼
WAIT_ACK       发布请求，最长等待 6 s
 ├─ ACK
 ▼
MAP_NAV        发布 MAP，最长 120 s
 ├─ 完成 / 失败 / 目标出现 / 超时
 ▼
RESUMING       发布 HOLD，发送 Qwen search
               至少等待 0.30 s，并等待新的第一视角 request_id
               新结果最长等待 3.0 s
 ▼
EGO            进入 12 s 冷却
```

恢复时等待新的 `request_id`，用于避免地图导航前遗留的旧 `POINT` 在重新接管的一瞬间继续向前。

## 8. 安装与启动

把补丁 ZIP 解压到仓库父目录，使文件覆盖到：

```text
/root/rdk_x5_vln_robot/rdk_x5_qwen3_vln_debug_v1
```

执行一次安装钩子：

```bash
cd /root/rdk_x5_vln_robot/rdk_x5_qwen3_vln_debug_v1
bash scripts/qwen_servo/install_intervention_v1.sh
```

安装脚本会：

1. 为原 `start_live_servo.sh` 创建带时间戳备份；
2. 在 `set -euo pipefail` 后插入一个幂等包装钩子；
3. 保留原脚本其余内容不变。

关闭状态下仍使用原启动命令：

```bash
MOTION_ENABLED=1 bash scripts/qwen_servo/start_live_servo.sh "find the bottle"
```

开启步骤：

```yaml
third_view_intervention:
  enabled: true
```

随后仍使用同一个启动命令。

## 9. 调试话题

```text
/third_view/intervention/decision
/third_view/intervention/status
/third_view/intervention/control_mode
/third_view/intervention/cmd_mux_status
/third_view/intervention/request
/third_view/intervention/cancel
```

常用检查：

```bash
ros2 topic echo /third_view/intervention/decision
ros2 topic echo /third_view/intervention/status
ros2 topic echo /third_view/intervention/cmd_mux_status
```

日志：

```text
logs/third_view_intervention.log
logs/cmd_vel_intervention_mux.log
```

## 10. 离线测试

```bash
bash scripts/qwen_servo/run_intervention_tests.sh
```

测试内容：

- 19 个纯策略与速度仲裁单元测试；
- 正常走廊、持续分岔、历史路口、出生环视、TURN 和目标接管模拟；
- Python 与 Shell 语法检查；
- 运行时配置是否正确把第一视角输出改到 `/cmd_vel_ego`。

离线测试不需要 ROS2、相机、雷达或底盘。

## 11. 真车验证顺序

1. 保持 `enabled: false`，确认原流程行为与 `85739b0` 一致；
2. 开启模块但暂不启动队友模块，人工发布双候选摘要，确认进入 HOLD，6 秒后自动回到 EGO；
3. 启动队友 Mock，只发布 `ACCEPTED → COMPLETED`，地图速度保持零，检查控制权状态机；
4. 抬轮测试 `/cmd_vel_ego` 与 `/cmd_vel_map` 互斥；
5. 低速地面测试 `MAP_DIRECT`；
6. 最后测试 `MAP_QWEN` 与 A*。

## 12. 当前 V1 边界

V1 尚未实现轨迹重合、视觉视野覆盖和完整拓扑记忆。这些属于第二版记忆模块。V1 只依赖队友模块给出的 `returned_to_junction` 和候选状态完成历史路口接入，但接口已经为后续记忆层预留。
