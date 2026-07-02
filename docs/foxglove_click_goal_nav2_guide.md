# Foxglove 点选地图目标 → Nav2 自动规划与导航方案

## 0. 结论

可行。最稳妥的实现方式不是改你已经成功的建图脚本，而是新增一个桥接节点：

```text
Foxglove 3D Panel 点选目标
        ↓ 发布 geometry_msgs/PoseStamped
/foxglove_goal_pose
        ↓ 新增桥接节点 foxglove_click_goal_bridge.py
Nav2 /compute_path_to_pose 先算路径并发布可视化 Path
        ↓
Nav2 /navigate_to_pose 执行导航
        ↓
/cmd_vel → 你现有底盘桥 → 小车运动
```

你原来终端输入坐标的 `nav2_oneclick_goal.sh` 本质上是“把目标坐标送给 Nav2”。现在只是把“坐标来源”从终端手输，换成 Foxglove 鼠标点击。

---

## 1. 推荐工作流：保存地图后再导航

强烈建议：

```text
先建图 → Ctrl+C 保存地图 → 停掉建图链路 → 启动 saved-map Nav2 → Foxglove 设置初始位姿 → 鼠标点目标导航
```

原因：

1. 你仓库现有 `run_nav2_saved_map.sh` 就是按 saved-map 模式写的，会加载 `maps/joy_corridor_map.yaml`。
2. 该脚本会主动停止 `slam_toolbox`、手柄节点和建图脚本，避免 SLAM 与 Nav2 同时抢 `/cmd_vel` 或 `map->odom`。
3. saved-map 模式下，地图是静态的，Nav2 的全局路径不会因为地图继续变形、闭环修正而突然漂移。
4. Foxglove 里仍然会有小车当前位置。这个位置来自 Nav2/AMCL + `/odom` + TF，不需要建图脚本继续运行。

---

## 2. 为什么不建议“差不多建完但不退出不保存就直接点导航”

理论上可以做“边 SLAM 边 Nav2 导航”，但对你当前阶段不高效。

主要风险：

1. `slam_toolbox` 和 AMCL/Nav2 可能同时处理 `map->odom` 关系，TF 责任容易混乱。
2. 建图时地图还在变，刚画出的路径可能下一秒就因为地图更新、障碍变化、局部闭环而变得不一致。
3. 手柄/建图控制和 Nav2 控制都可能发布 `/cmd_vel`，导致底盘到底听谁的不明确。
4. 你当前最重要的是先证明“保存地图 → 点选目标 → 自动导航”闭环成功，而不是把两个复杂系统强行叠在一起。

一句话：现在别贪“边建边跑”。先把 saved-map 导航打通，这才是最高效路线。

---

## 3. 新增文件

把本包安装到项目后，会新增：

```text
scripts/slam/foxglove_click_goal_bridge.py
scripts/slam/run_nav2_foxglove_click_goal.sh
scripts/slam/check_foxglove_click_nav_ready.sh
docs/foxglove_click_goal_nav2_guide.md
```

不覆盖你当前成功的建图脚本。

---

## 4. 安装步骤

假设你把这个文件包放到了小车，例如：

```bash
/root/nav2_foxglove_click_goal_pack
```

执行：

```bash
cd /root/nav2_foxglove_click_goal_pack
bash install_foxglove_click_goal_nav2.sh /root/rdk_x5_vln_robot
```

然后检查：

```bash
cd /root/rdk_x5_vln_robot
bash scripts/slam/check_foxglove_click_nav_ready.sh
```

---

## 5. 建图并保存地图

保持你原来成功建图方式不变：

```bash
cd /root/rdk_x5_vln_robot
bash scripts/slam/run_joy_mapping_all.sh
```

操作：

1. Foxglove 连接 `ws://小车IP:8765`。
2. 慢速手柄建图。
3. 地图满意后，在运行建图脚本的终端按 `Ctrl+C`。
4. 脚本会保存：

```text
/root/rdk_x5_vln_robot/maps/joy_corridor_map.yaml
/root/rdk_x5_vln_robot/maps/joy_corridor_map.pgm
```

---

## 6. 启动 Foxglove 点选导航

```bash
cd /root/rdk_x5_vln_robot
MAP_YAML=/root/rdk_x5_vln_robot/maps/joy_corridor_map.yaml \
  bash scripts/slam/run_nav2_foxglove_click_goal.sh
```

这个脚本会：

1. 调用你原来的 `scripts/slam/run_nav2_saved_map.sh` 启动 saved-map Nav2。
2. 等待 `/map`、`/odom`、`/tf`、`/navigate_to_pose`。
3. 启动 `foxglove_click_goal_bridge.py`。
4. 保持 Foxglove Bridge 端口 `8765`。

---

## 7. Foxglove 设置

连接：

```text
ws://小车IP:8765
```

3D Panel 设置：

```text
Fixed frame / 固定参考系：map
```

建议显示这些 topic：

```text
/map
/tf
/tf_static
/odom
/scan
/global_costmap/costmap
/local_costmap/costmap
/foxglove_click_planned_path
/foxglove_click_path_marker
/foxglove_click_accepted_goal
```

### 7.1 先设置小车初始位姿

saved-map Nav2 通常需要 AMCL 初始位姿。

在 Foxglove 3D Panel 的 Publish 工具中配置：

```text
工具：2D pose estimate
Topic：/initialpose
```

在地图上点小车实际所在位置，并拖出车头方向。

### 7.2 再点目标

配置：

```text
工具：2D pose
Topic：/foxglove_goal_pose
```

在地图可通行区域点目标位置，并拖一下方向。

点完后：

1. `/foxglove_click_planned_path` 会显示预规划路线。
2. `/foxglove_click_path_marker` 会显示一条更明显的路径线。
3. 小车会开始按照 Nav2 路径运动。

---

## 8. 验收标准

成功状态应该是：

1. 终端出现：

```text
Accepted clicked goal
Planned path published
NavigateToPose goal accepted
```

2. Foxglove 上能看到 `/foxglove_click_planned_path`。
3. `/cmd_vel` 有 Nav2 输出。
4. 小车朝路径移动。
5. 到达后终端出现 Navigation succeeded 或至少距离明显变小。

检查命令：

```bash
ros2 topic echo /foxglove_goal_pose --once
ros2 topic echo /foxglove_click_planned_path --once
ros2 action list | grep navigate_to_pose
ros2 topic info /cmd_vel -v
```

`/cmd_vel` 最理想状态：只有 Nav2/velocity_smoother 一类导航节点在发布，不要同时有手柄 teleop 或其他避障脚本抢控制。

---

## 9. 常见问题

### 问题 A：点击后没有反应

检查 Foxglove Publish topic 是否写成：

```text
/foxglove_goal_pose
```

再检查：

```bash
ros2 topic echo /foxglove_goal_pose --once
```

如果 echo 不到，说明 Foxglove 没有真正发布目标。

### 问题 B：路径不显示，但小车可能会动

检查：

```bash
ros2 action list | grep compute_path_to_pose
ros2 topic echo /foxglove_click_planned_path --once
```

如果 `/compute_path_to_pose` 没起来，预画路径不会出现，但 `/navigate_to_pose` 仍可能执行。

### 问题 C：路径乱飞或目标位置偏

通常是 Foxglove 的 Fixed frame 不是 `map`，或者 AMCL 初始位姿没有设置好。

处理：

1. 3D Panel 固定参考系设为 `map`。
2. 用 `/initialpose` 重新设置小车当前位置和车头方向。
3. 确认 `/map`、`/odom`、`base_link` 的 TF 链存在。

### 问题 D：小车不动，但路径有了

检查：

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel
```

如果 `/cmd_vel` 有输出但车不动，问题在底盘桥/串口/运动参数。
如果 `/cmd_vel` 没输出，问题在 Nav2 控制器、costmap、localization 或目标不可达。

### 问题 E：目标点在黑色障碍物或灰色未知区

不要点障碍物内部。优先点白色可通行区域。

如果 `allow_unknown: true`，Nav2 可能会尝试穿过未知区；如果现场安全性优先，后续可以把全局规划器改为不允许未知区。但这属于 Nav2 参数优化，不是本次点选功能的必要修改。

---

## 10. 给 Cursor 的一次性执行指令

把下面整段发给 Cursor Agent：

```text
请在 /root/rdk_x5_vln_robot 中做增量修改，不要改动现有成功建图脚本。

目标：新增 Foxglove 点击地图目标后，Nav2 自动规划路径并执行导航的功能。

请完成：
1. 新增 scripts/slam/foxglove_click_goal_bridge.py：订阅 /foxglove_goal_pose 的 geometry_msgs/msg/PoseStamped；调用 Nav2 /compute_path_to_pose 预规划路径并发布 /foxglove_click_planned_path 和 /foxglove_click_path_marker；再调用 /navigate_to_pose 执行导航。
2. 新增 scripts/slam/run_nav2_foxglove_click_goal.sh：包装现有 scripts/slam/run_nav2_saved_map.sh，等待 /map、/odom、/tf、/navigate_to_pose，然后启动 foxglove_click_goal_bridge.py。
3. 新增 scripts/slam/check_foxglove_click_nav_ready.sh：检查地图文件、脚本语法、ROS 包、Nav2 action 是否存在。
4. 新增 docs/foxglove_click_goal_nav2_guide.md：写明保存地图后导航的操作步骤、Foxglove 里 /initialpose 与 /foxglove_goal_pose 的设置方法。
5. chmod +x 三个脚本。
6. 执行 bash -n 和 python3 -m py_compile 做语法检查。

严禁修改 scripts/slam/run_joy_mapping_all.sh、scripts/slam/run_corridor_mapping_live_foxglove.sh、configs/slam_toolbox.yaml、底盘桥和雷达脚本。
```

---

## 11. 最小操作命令总表

```bash
# 1. 安装新增文件
cd /root/nav2_foxglove_click_goal_pack
bash install_foxglove_click_goal_nav2.sh /root/rdk_x5_vln_robot

# 2. 检查
cd /root/rdk_x5_vln_robot
bash scripts/slam/check_foxglove_click_nav_ready.sh

# 3. 先按原方式建图并保存
bash scripts/slam/run_joy_mapping_all.sh

# 4. 启动 saved-map 点选导航
MAP_YAML=/root/rdk_x5_vln_robot/maps/joy_corridor_map.yaml \
  bash scripts/slam/run_nav2_foxglove_click_goal.sh
```

Foxglove：

```text
连接 ws://小车IP:8765
固定参考系 map
先用 /initialpose 设置当前位姿
再用 /foxglove_goal_pose 点目标
显示 /foxglove_click_planned_path
```
