# 稳定版 YOLO + LiDAR 导航测试流程

目标：小车可以慢，但必须稳定导航。验证 `/cmd_vel` 与 `/cmd_vel_sent` 一致，底盘能持续运动。

配置：`configs/yolo_lidar_failsafe_nav.yaml`  
启动：`bash scripts/nav/start_yolo_lidar_stable_nav.sh`

---

## 1. 底盘桥通路测试

先不启动导航，只启动桥：

```bash
cd ~/rdk_x5_vln_robot
source /opt/tros/humble/setup.bash

pkill -f m1_pwm_cmd_vel_bridge.py || true
pkill -f cmd_vel_to_rosmaster.py || true

source scripts/lib/load_mvp_tune.sh
source scripts/lib/run_chassis_bridge.sh
run_chassis_bridge logs/chassis_test.log
# 或手动：
# python3 ros2_bridge/m1_pwm_cmd_vel_bridge.py \
#   --port /dev/ttyUSB1 \
#   --max-vx 0.06 \
#   --max-wz 0.06 \
#   --wheel-layout fl-rl-fr-rr \
#   --debug
```

另开终端测试前进（车轮架空）：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.06, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

再测旋转：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.24}}"
```

验收：

- 车轮架空必须能动
- `ros2 topic echo /cmd_vel_sent` 应接近 0.06 / 0.24
- `ros2 topic echo /chassis_bridge_state` 显示 `last_sent_vx/wz`

---

## 2. 只跑导航（桥保持运行）

```bash
NAV_ONLY=1 bash scripts/nav/start_yolo_lidar_failsafe_nav.sh \
  configs/yolo_lidar_failsafe_nav.yaml \
  "bottle"
```

观察：

```bash
ros2 topic echo /failsafe_nav_state
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_sent
ros2 topic echo /chassis_bridge_state
```

---

## 3. 全栈稳定版

```bash
bash scripts/nav/start_yolo_lidar_stable_nav.sh configs/yolo_lidar_failsafe_nav.yaml bottle
```

---

## 4. 真车低速测试（落地前架空 30 秒）

| 状态 | 期望 |
|------|------|
| FREE_SPACE_EXPLORE | 轮子持续缓慢转，不再每 0.3s 清零 |
| TARGET_TRACK | 轮子持续缓慢转/转向 |
| BLOCKED_ROTATE | 同向旋转至少 1.2s，不左右抽搐 |
| TARGET_REACQUIRE | 丢目标时缓慢原地扫描，不立即全停 |
| EMERGENCY_STOP | `front_distance <= 0.35m` 立即停 |

对比 `/cmd_vel` 与 `/cmd_vel_sent`：

- 两者接近 → 导航与桥正常
- `/cmd_vel` 有值但 `/cmd_vel_sent` 很小 → 问题在底盘桥/平滑/kick
- `/cmd_vel_sent` 有值但车不动 → 端口/供电/死区/机械

---

## 5. Foxglove 布局建议

| Panel | Topic |
|-------|-------|
| 3D | `/scan` |
| 3D | `/failsafe_nav/markers` |
| Image | `/failsafe_nav/debug_image` |
| Raw | `/failsafe_nav_state` |
| Raw | `/chassis_bridge_state` |
| Plot | `/cmd_vel.linear.x` |
| Plot | `/cmd_vel.angular.z` |
| Plot | `/cmd_vel_sent.linear.x` |
| Plot | `/cmd_vel_sent.angular.z` |

---

## 6. 最终验收标准

| 测试 | 通过标准 |
|------|----------|
| 手动 `/cmd_vel` | `vx=0.06` 能前进，`wz=0.24` 能转 |
| 全流程 `/cmd_vel` | FREE_SPACE 连续非零，不再频繁清零 |
| `/cmd_vel_sent` | 与 `/cmd_vel` 接近 |
| 雷达探索 | 前方空时持续慢进；侧方空时稳定转向 |
| 目标跟踪 | YOLO 出现后进入 TARGET_TRACK 并靠近 |
| 丢目标 | 短暂 TARGET_REACQUIRE，不立即全停 |
| blocked | 同向旋转 ≥ 1.2s 再换向 |
| 安全 | 紧急距离必须立即停 |

---

## 7. 端口说明

默认 `CHASSIS_PORT=/dev/ttyUSB1`（yaml `chassis.port`）。若单独测试用 `/dev/myserial`：

```bash
CHASSIS_PORT=/dev/myserial bash scripts/nav/start_yolo_lidar_stable_nav.sh
```

不要用 `/dev/ttyUSB0` 除非确认该端口接 M1 底盘。
