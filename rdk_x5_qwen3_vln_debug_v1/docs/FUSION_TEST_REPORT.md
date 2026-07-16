# 在线地图融合补丁离线测试报告

## 已执行

1. `python -m unittest`：9/9 通过。
2. Python 语法检查：通过。
3. Shell 语法检查：通过。
4. 确定性流程模拟：通过。

模拟轨迹：

```text
候选摘要归一化
  -> MAP_QWEN 请求翻译为 SELECT_AND_NAVIGATE
  -> WAIT_BACKEND，输出零速度
  -> 后端 ACK，仍无命令则输出零速度
  -> 匹配 request_id 的速度获得授权
  -> 0.08/-0.09 被限幅为 0.06/-0.06
  -> ARRIVED_ALIGNED
  -> COMPLETED
  -> IDLE / 返回第一视角
  -> 旧 request_id 状态被忽略
```

目标中途出现模拟：

```text
ACTIVE MAP
  -> fresh_first_person_target_visible
  -> CANCELLING
  -> 地图速度立即变零
```

## 未冒充完成的测试

当前生成环境没有 RDK X5 真机、ROS2 图、雷达、M1 底盘及队友实际后端，因此以下内容必须在板端按主文档顺序验证：

- ROS QoS 实际连接；
- 真 `/map` 候选坐标正确性；
- A* 控制速度方向；
- 雷达紧急后退对地图速度的覆盖；
- 最终 yaw；
- 真实目标中途出现的取消时序；
- 真车刹停距离。
