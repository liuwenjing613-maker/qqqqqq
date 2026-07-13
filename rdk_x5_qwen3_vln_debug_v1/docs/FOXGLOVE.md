# Foxglove 可视化配置

推荐布局：

1. **Image**：`/qwen_vln/annotated_image/compressed`
2. **Raw Messages**：`/qwen_vln/state`
3. **Raw Messages**：`/qwen_vln/result_json`
4. **Raw Messages**：`/qwen_vln/prompt_text`
5. **Plot**：`/qwen_vln/latency_ms`

图像标记：

- 绿色：准确目标点 `target`
- 黄色：语义搜索提示点 `search`
- 浅蓝：验证通过点 `verify`
- 中央灰色竖线：相机画面中心

默认 `history_length: 1`，因此不把不同请求帧的历史像素叠加到当前图像。需要在固定相机测试时观察波动，可以临时增大配置，但真车运动时应保持 1。

## 像素与图像严格对应

API 调用有延迟。若直接把返回点画到最新相机画面上，车辆或云台一动，点位就会看起来完全错误。

本节点在发起请求时保存 API 输入帧，并在返回后把点画在对应的请求帧上。画面顶部会显示：

```text
FRAME: exact API input image for request N
```

## 检查话题

```bash
ros2 topic hz /qwen_vln/annotated_image/compressed
ros2 topic echo /qwen_vln/state
ros2 topic echo /qwen_vln/result_json
ros2 topic echo /qwen_vln/prompt_text
```

`/qwen_vln/pixel_point` 使用 `geometry_msgs/PointStamped`：

```text
point.x = 像素 x
point.y = 像素 y
point.z = confidence
header.frame_id = camera_pixels_target/search/verify
```
