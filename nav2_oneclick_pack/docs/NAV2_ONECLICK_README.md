# RDK X5 ROSMASTER M1：保存地图一键 Nav2 定点导航说明

## 1. 这个压缩包解决什么

这个包提供一个独立脚本：

```bash
scripts/slam/nav2_oneclick_goal.sh
```

它会一键完成：

1. 清理会抢 `/cmd_vel` 的旧节点，例如 `shared_nav`、YOLO、手柄、旧 Nav2、旧 SLAM。
2. 修复地图 YAML 里 `image:` 指向不存在 `.tmp_*.pgm` 的问题。
3. 修复 `configs/nav2_params.yaml` 的关键 Nav2 参数。
4. 启动雷达 `/scan`。
5. 自动读取 `/scan.header.frame_id`，并发布正确的 `base_link -> 雷达frame` 静态 TF。
6. 启动 `m1_pwm_cmd_vel_bridge`，发布 `/odom`，订阅 `/cmd_vel`。
7. 启动 Nav2：`map_server`、`amcl`、`planner_server`、`controller_server`、`bt_navigator` 等。
8. 如果 lifecycle 没自动激活，会尝试手动激活 `/map_server` 和 `/amcl`。
9. 发布 AMCL 初始位姿。
10. 检查 `/amcl_pose` 和 `map -> base_link`。
11. 发送一个目标点，让小车定点导航。

---

## 2. 安装方式

把压缩包解压后，在板端项目根目录执行：

```bash
cd /root/rdk_x5_vln_robot
cp -r /你的解压目录/scripts/slam/* scripts/slam/
chmod +x scripts/slam/nav2_oneclick_goal.sh
chmod +x scripts/slam/cmd_vel_burst.py
```

如果是在电脑下载后上传，最终文件应该在：

```text
/root/rdk_x5_vln_robot/scripts/slam/nav2_oneclick_goal.sh
/root/rdk_x5_vln_robot/scripts/slam/cmd_vel_burst.py
```

---

## 3. 最常用启动命令

终端 A 执行：

```bash
cd /root/rdk_x5_vln_robot
bash scripts/slam/nav2_oneclick_goal.sh 0.30 0.00 0.00
```

参数含义：

```text
0.30  目标点 x，单位 m，map 坐标系
0.00  目标点 y，单位 m，map 坐标系
0.00  目标终点 yaw，单位 rad
```

也就是说：

```bash
bash scripts/slam/nav2_oneclick_goal.sh GOAL_X GOAL_Y GOAL_YAW
```

例如：

```bash
bash scripts/slam/nav2_oneclick_goal.sh 0.60 0.00 0.00
bash scripts/slam/nav2_oneclick_goal.sh 0.60 0.30 1.5708
bash scripts/slam/nav2_oneclick_goal.sh 0.00 0.00 3.1416
```

---

## 4. 只修改脚本里的默认目标点

如果你不想每次命令行输入目标点，就打开：

```bash
nano scripts/slam/nav2_oneclick_goal.sh
```

修改顶部：

```bash
DEFAULT_GOAL_X="0.30"
DEFAULT_GOAL_Y="0.00"
DEFAULT_GOAL_YAW="0.00"
```

以后直接运行：

```bash
bash scripts/slam/nav2_oneclick_goal.sh
```

---

## 5. 初始位姿怎么设置

默认初始位姿是：

```bash
DEFAULT_INIT_X="0.00"
DEFAULT_INIT_Y="0.00"
DEFAULT_INIT_YAW="0.00"
```

它表示小车一开始在地图原点附近，车头朝 `map` 的 x 正方向。

如果实际小车不在地图原点，需要在脚本顶部修改：

```bash
DEFAULT_INIT_X="你的初始x"
DEFAULT_INIT_Y="你的初始y"
DEFAULT_INIT_YAW="你的初始朝向rad"
```

---

## 6. 关键检查命令

另开终端 B：

```bash
source /opt/ros/humble/setup.bash
[ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash

ros2 topic list | grep -E "^/map$|^/scan$|^/odom$|^/tf$|^/cmd_vel$|^/amcl_pose$"
ros2 action list | grep navigate
ros2 run tf2_ros tf2_echo map base_link
```

成功状态应该有：

```text
/map
/scan
/odom
/tf
/cmd_vel
/amcl_pose
/navigate_to_pose
```

并且：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

能持续输出小车在地图里的位置。

---

## 7. 如果小车不动，先测底盘直发

先不要急着改 Nav2。测试 `/cmd_vel -> 底盘桥 -> 电机` 链路：

```bash
cd /root/rdk_x5_vln_robot
python3 scripts/slam/cmd_vel_burst.py 0.15 0.0 2.0
```

如果不动，试：

```bash
python3 scripts/slam/cmd_vel_burst.py 0.25 0.0 1.5
python3 scripts/slam/cmd_vel_burst.py 0.0 0.6 1.5
```

判断：

| 现象 | 说明 |
|---|---|
| 直发能动，Nav2 不动 | Nav2 速度参数或代价地图问题 |
| 直发也不动 | 底盘桥、串口、电机使能、PWM 参数问题 |
| 直发动了但 `/odom` 不变 | 里程计发布或积分有问题 |

---

## 8. 为什么不能同时开 YOLO / shared_nav / 手柄

定点导航时，`/cmd_vel` 应该只有 Nav2 负责发速度，`m1_pwm_cmd_vel_bridge` 负责接收速度。

不要同时运行：

```text
shared_nav
yolov5s_bpu_web_node
teleop_twist_joy
joy_node
run_mvp_task.py
```

否则就是多个节点同时抢 `/cmd_vel`，相当于多个人同时抢方向盘。

---

## 9. 脚本改了哪些关键点

### 9.1 地图 YAML

修复：

```yaml
image: joy_calibrated_corridor_map.tmp_xxx.pgm
```

为实际存在的：

```yaml
image: joy_calibrated_corridor_map.pgm
```

### 9.2 local_costmap

修复：

```yaml
global_frame: map
```

为：

```yaml
global_frame: odom
```

原因：局部控制应该基于连续平滑的 `odom`，全局规划才用 `map`。

### 9.3 map_server

把 `yaml_filename` 写成绝对路径，避免 launch 参数覆盖失败导致 `/map_server unconfigured`。

### 9.4 DWB 速度

原配置速度上限大约是 `0.06 m/s`，对小车可能太小。脚本改为保守可动范围：

```text
max_vel_x: 0.18
max_vel_theta: 0.80
```

### 9.5 progress_checker

原来要求 10 秒内移动 `0.5m`，对于 0.3m 小目标和慢速小车不合理。脚本改为：

```text
required_movement_radius: 0.05
movement_time_allowance: 20.0
```

---

## 10. 推荐调试顺序

1. 跑：

```bash
bash scripts/slam/nav2_oneclick_goal.sh 0.30 0.00 0.00
```

2. 看有没有 `/amcl_pose`。
3. 看 `map -> base_link` 是否正常。
4. 看 `/cmd_vel` 是否有非零输出。
5. 如果 `/cmd_vel` 有输出但车不动，跑 `cmd_vel_burst.py`。
6. 如果直发能动，再微调 Nav2 速度和代价地图。

---

## 11. 紧急停止

按脚本所在终端的：

```text
Ctrl+C
```

脚本会尝试发送 0 速度，并清理 Nav2、底盘桥、TF、雷达相关进程。

实车旁边仍然要有人看着，第一次只测 0.3m 小目标。
