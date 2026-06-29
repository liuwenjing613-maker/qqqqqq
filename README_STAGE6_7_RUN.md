# 阶段 6 + 阶段 7 运行说明

本文件用于记录阶段 6 相机测试、阶段 7 红色目标视觉伺服的启动顺序。

---

## 一、阶段 6：相机测试

### 1. OpenCV 直接测试相机

```bash
cd ~/rdk_x5_vln_robot/perception
python3 opencv_camera_check.py --camera /dev/video0

### 2. 启动 ROS2 USB 摄像头节点

终端 1 执行：

```bash
cd ~/rdk_x5_vln_robot/perception
source /opt/tros/humble/setup.bash
ros2 launch ~/rdk_x5_vln_robot/perception/launch/usb_cam.launch.py usb_video_device:=/dev/video0

### 3.检查图像话题

新开终端 2 执行：
```bash
source /opt/tros/humble/setup.bash
ros2 topic list
ros2 topic info /image
ros2 topic hz /image

### 4.浏览器查看相机画面

新开终端 3 执行：
```bash
source /opt/tros/humble/setup.bash
ros2 launch websocket websocket.launch.py websocket_image_topic:=/image websocket_only_show_image:=true

然后在电脑浏览器打开：

http://你的RDK_IP:8000

例如：

http://192.168.1.88:8000

## 二、阶段 7：红色目标视觉伺服

### 终端1：启动相机
```bash
cd ~/rdk_x5_vln_robot/perception
source /opt/tros/humble/setup.bash
ros2 launch ~/rdk_x5_vln_robot/perception/launch/usb_cam.launch.py usb_video_device:=/dev/video0

### 终端2：启动底盘桥
```bash
cd ~/rdk_x5_vln_robot/ros2_bridge
source /opt/tros/humble/setup.bash
python3 m1_pwm_cmd_vel_bridge.py --port /dev/ttyUSB0 --max-vx 0.06 --max-wz 0.06 --wheel-layout fl-rl-fr-rr --debug

### 终端3：启动红色视觉伺服
```bash
cd ~/rdk_x5_vln_robot/visual_servo
source /opt/tros/humble/setup.bash
python3 red_target_servo_auto_ros.py \
  --image-topic /image \
  --max-vx 0.06 \
  --max-wz 0.35 \
  --kp-turn 0.9 \
  --center-threshold 0.22 \
  --arrive-area-ratio 0.30 \
  --save-debug

### 终端4：观察/cmd_vel
```bash
source /opt/tros/humble/setup.bash
ros2 topic echo /cmd_vel

### 终端5：观察视觉伺服状态（可选）
```bash
source /opt/tros/humble/setup.bash
ros2 topic echo /red_servo_state


