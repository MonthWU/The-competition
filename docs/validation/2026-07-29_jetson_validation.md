# 2026-07-29 Jetson 部署与无图像验证

## 部署范围

- Windows 权威源：`D:\CodexFolder\JetsonNano\Copyfiles\26E_vision`
- Jetson 镜像：`~/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision`
- 未修改标准项目 `Copyfiles/ros2_ws`。
- 本次发现的 `192.168.137.161` 仅作为会话临时地址，没有写入项目或 SSH 配置。

## 设备身份

- 架构：`aarch64`
- 主机名：`monthwu-jetson`
- 用户：`jetson`
- ROS 2：`/opt/ros/humble/setup.bash` 存在
- 视频设备：`/dev/video0`、`/dev/video1`

## 构建修正

首次 ARM64 构建发现 Jetson 未安装 `cv_bridge`。检查确认两个新节点只需要在 BGR OpenCV 矩阵和 `sensor_msgs/Image` 之间转换，因此没有安装新系统依赖，而是在 Windows 权威源码中实现了直接转换并移除 `cv_bridge` 依赖。

修正后以下 6 个包在 Jetson 上构建成功：

- `vision_interfaces`
- `puzzle_perception_node`
- `puzzle_solver_node`
- `puzzle_coordinator_node`
- `web_tuner_node`
- `vision_bringup`

## 测试结果

- `ros2 interface show vision_interfaces/msg/PuzzlePlan` 可正确解析接口。
- `puzzle_solver_node`：4 个测试记录，0 错误、0 失败、0 跳过。
- `mode:=work` 只启动：

```text
/puzzle_coordinator_node
/puzzle_perception_node
/puzzle_solver_node
```

## 无图像场景

启动任务并确认工具离开后，感知节点没有找到有效 A4 图像，发布：

```text
valid: false
workspace_mapping_valid: false
status: A4_NOT_FOUND
pieces: []
```

协调节点按配置执行有限次数重试，最终状态为：

```text
{"state":"FAILED","reason":"PLAN_FAILED:SCENE_INVALID:A4_NOT_FOUND","active_piece_id":0}
```

系统没有输出猜测坐标或放置请求，符合无有效图像时拒绝执行的安全要求。

## 一致性与清理

- 部署完成后比较 Windows 与 Jetson 的源码、配置和文档：81 个文件，0 缺失、0 多余、0 哈希不一致。
- 本次 launch 使用独立进程组，结束后三个节点均干净退出。
- 清理后本次进程组为空，`ros2 node list` 为空。
- Jetson smoke log：`~/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision/jetson_work_smoke.log`

## 尚待真实图像验证

- A4 四角识别和透视校正。
- 拼图块轮廓、面积、顶点与抓取点提取。
- A4 局部坐标到工作区绝对坐标的实测映射。
- 真实拼图的唯一解、放置顺序、耗时和重复性。
