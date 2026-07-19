# 语音统一功能演示总控 V1

## 1. 这版解决什么

一次启动后，常驻两类基础服务：

1. `run_slam_calibrated.sh` 提供雷达、底盘、里程计、实时 SLAM、TF 和 Foxglove。
2. KWS 功能选择服务只加载一次，持续等待唤醒，不会每轮重新加载关键词模型。

联网探索和断网探索复用同一套 `/map`、`/odom`、`/scan_filtered`，切换时只停止功能特有节点。再次说出唤醒词时，语音服务会先写入 `wake` 事件，总控立即发送零速度并停止当前功能，然后才继续录制 5 秒功能命令。

## 2. 共享栈与例外

开机后常驻并复用：

- 校准实时 SLAM（雷达/底盘/里程计/TF/Foxglove）
- 语音 KWS
- 相机 + Qwen 预热（空闲 pause；联网复用；断网独占相机时临时停）

模式行为：

- 联网探索 ↔ 断网探索 ↔ 遥控建图：共享 SLAM **不重启**。建图只在其上挂手柄 teleop，结束时 `map_saver` 存盘。
- 点击导航：保存地图后的 Nav2 定位与实时 `slam_toolbox` 不能同时占 `/map`，因此会短暂停共享 SLAM，离开后自动恢复。
- 说“完成建图”：先存图，再进入点击导航。

## 3. 安装

把压缩包解压到任意目录，然后执行：

```bash
cd voice_demo_hub_v1_overlay
bash apply_voice_demo_hub_v1.sh /root/rdk_x5_vln_robot
```

安装器只做三件事：复制新增文件、给 exp2 增加一个很小的 `REUSE_BASE_STACK=1` 分支、运行静态检查。原 exp2 会备份为：

```text
scripts/nav/start_yolo_lidar_semantic_explore_nav_exp2.sh.before_voice_demo_hub_v1
```

## 4. 启动

```bash
cd /root/rdk_x5_vln_robot
VOICE_FUNCTION_RECORD_SECONDS=5 \
  bash scripts/demo/start_voice_demo_hub_v1.sh
```

等终端出现总 `READY` 后再唤醒。

## 5. 固定演示命令

| 语音 | 行为 |
|---|---|
| 开始联网探索 | 调用现有双视角联网脚本，固定任务默认为 `find the bottle` |
| 开始断网探索 | 复用共享 SLAM，启动 exp2 的相机、YOLO、语义地图与雷达探索模块 |
| 开始建图 | 复用共享 SLAM，只启动手柄遥控；结束时保存地图 |
| 完成建图 | 停车、保存地图、切换到 Foxglove 点击目标 Nav2 |
| 点击导航 | 使用已经保存的演示地图进入点击导航 |
| 停止机器人 | 停止当前功能，恢复共享 SLAM 并待机 |
| 重置地图 | 停止当前功能并重新启动一张空的实时地图 |

命令和固定目标都可以在 `configs/voice_demo_hub_v1.yaml` 修改。

## 6. 推荐现场流程

1. 启动总控并等 `READY`。
2. “小车你好” → “开始联网探索”。
3. 需要打断时再次“小车你好”。唤醒一被检测到，小车先停车，再录制新命令。
4. “开始建图”，用手柄慢速建图。
5. “小车你好” → “完成建图”，脚本自动保存并进入点击导航。
6. 在 Foxglove 里点击目标位置。
7. 任何时候再次唤醒即可停止当前模式并选择其他功能。

## 7. 重要限制

- “断网探索”表示导航决策不调用 Qwen。功能选择本身仍沿用你们现有云端 ASR，因此现场不要真的断开网络；真断网时应使用键盘备用入口或后续增加离线命令词 KWS。
- 相机没有设为全局常驻。现有联网脚本和 exp2 都把相机进程视为自己管理的资源，强行共享反而容易出现 `/dev/video0` 重复占用。基础 SLAM、雷达、底盘、TF、Foxglove 和语音 KWS 才是安全常驻集合。
- 所有模式互斥，避免多个 `/cmd_vel` 发布者争夺底盘。

## 8. 检查与日志

```bash
bash scripts/demo/check_voice_demo_hub_v1.sh
tail -f rdk_x5_qwen3_vln_debug_v1/logs/voice_demo_hub_v1/latest/main.log
```

每个功能都有独立日志：`online.log`、`offline.log`、`mapping.log`、`click_nav.log`、`voice.log`、`base_slam.log`。

## 9. 现场键盘兜底

语音识别是演示链路中最容易被现场噪声羞辱的一环，因此保留同一状态机的键盘入口。它不会另起一套控制逻辑，只是向总控注入与语音相同的 `wake + command` 事件：

```bash
bash scripts/demo/control_voice_demo_hub_v1.sh online
bash scripts/demo/control_voice_demo_hub_v1.sh offline
bash scripts/demo/control_voice_demo_hub_v1.sh mapping
bash scripts/demo/control_voice_demo_hub_v1.sh finish
bash scripts/demo/control_voice_demo_hub_v1.sh click
bash scripts/demo/control_voice_demo_hub_v1.sh stop
bash scripts/demo/control_voice_demo_hub_v1.sh reset
bash scripts/demo/control_voice_demo_hub_v1.sh status
```

正式演示仍然用语音；后台队员只在 ASR 没识别出来或现场太吵时使用这个入口。由于两种入口走同一套打断与清理流程，不会产生“语音状态”和“终端状态”各活在一个平行宇宙的问题。
