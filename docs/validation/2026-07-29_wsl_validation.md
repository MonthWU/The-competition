# 2026-07-29 WSL 验证记录

## 环境

- Ubuntu 22.04 WSL，x86_64。
- ROS 2 Humble。
- OpenCV 4.5.4。
- 本记录用于接口、C++、配置和 launch 检查，不替代 Jetson ARM64 实机验证。

## 构建与接口

以下包在干净的 WSL 原生 Linux 工作区完成 `colcon build`：

- `vision_interfaces`
- `puzzle_perception_node`
- `puzzle_solver_node`
- `puzzle_coordinator_node`
- `web_tuner_node`
- `vision_bringup`

构建结果为 6 个包全部成功。验证过程中发现并修复：

- ROS 参数整数返回类型与 `std::max(int, int64_t)` 不匹配。
- `vision_interfaces/package.xml` 缺少 `ament_cmake` 构建类型声明，造成接口虽可编译，但不能被 ROS 运行时包索引发现。

修复后，`ros2 pkg list` 可发现全部新包，`ros2 interface show vision_interfaces/msg/PuzzlePlan` 可完整解析拼图规划消息。

## 单元测试

`puzzle_solver_node` 的测试全部通过：

1. 四块等尺寸矩形可合成为允许范围内的目标矩形。
2. 空拼图块集合会被拒绝。
3. 一条长边由两条短边分段匹配的 T 形接缝可以求解。

最终 `colcon test-result --verbose`：4 个测试记录，0 错误、0 失败、0 跳过。

四块对称矩形求解测试约 5.3 秒，T 形接缝测试约 0.1 秒。这是 WSL x86_64 上的未调优合成数据结果，不作为 Jetson 性能结论。

## Launch smoke test

`mode:=work` 启动时只出现：

```text
/puzzle_coordinator_node
/puzzle_perception_node
/puzzle_solver_node
```

测试结束后节点列表为空，三个节点均响应 SIGINT 并干净退出。默认工作模式未启动 YOLO、TensorRT、Kalman、串口或 HMI 节点。

`mode:=debug web_enabled:=false` 同样完成核心节点启动与干净退出。完整网页模式在该 WSL 环境缺少 `python3-flask`，因此网页节点未通过运行验证；项目包已经声明 `python3-flask` 运行依赖，本轮未安装或修改系统依赖。

## 配置静态检查

- `vision_system.yaml` 与 `competition_tuning.yaml` 可被 YAML 解析器加载。
- launch 文件通过 Python 语法检查。
- 新节点声明的可调参数在项目 YAML 中均有对应项。
- 新核心节点无 YOLO 或 TensorRT 依赖。

## 尚未完成的实机验证

- Jetson SSH 别名出现代理横幅超时，临时热点地址 `192.168.137.161` 也不可达。
- 尚未完成 Jetson aarch64 编译、真实相机、A4 识别、拼图块分割、绝对坐标映射和性能测试。
- `workspace_mapping_valid` 保持为 `false`；实测固定映射完成前，系统不会把局部坐标冒充绝对坐标输出。
- 现场阈值、磁铁颜色范围、图案接缝权重和最优解差距仍需真实样本调参。
