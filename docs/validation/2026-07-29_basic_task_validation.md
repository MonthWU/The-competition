# 2026-07-29 基础题实现与验证

## 范围

- Windows 权威源：`D:\CodexFolder\JetsonNano\Copyfiles\26E_vision`
- Jetson 部署镜像：`~/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision`
- 未访问或修改 `webapp/`、`Model/`、`Monthwu.md` 和标准项目 `Copyfiles/ros2_ws/`。
- 本次实现任务 1；任务 2、3 的业务流程仍未定义并会显式拒绝。

## 实现结果

- 新增通用 `puzzle_geometry` 包，提供单应性点变换、相机中心像素坐标、半周角
  归一化、旋转差和最长边方向角，不依赖任务编号。
- 绿色 A4 按 `region_split_ratio=0.5` 划分上部源区域和下部目标区域。
- 目标矩形中心使用下半区中心 `(105.0, 222.75) mm`，仍由 YAML 配置。
- `PuzzleScene` 增加 A4 毫米到原始相机像素的单应性和图像尺寸。
- `PuzzlePlacement` 同时携带相机中心像素、A4 毫米、工作区毫米和源/目标/旋转差角度。
- 任务 1 状态机接通扫描、求解、逐片下发、放置后复核、完成后回到
  `IDLE/NO_TASK`。
- `serial_bridge_node` 订阅 `puzzle/placement_request`，单次发送
  `[x0,y0,x1,y1,angle]`；基础题默认坐标系为 `camera_center_px`，角度语义为
  `source`。
- debug 图像增加上下区分界、区域名称、碎片轮廓、编号、中心点及相机中心像素坐标；
  网页状态改为显示场景、碎片数、求解耗时和最近串口帧。

## 备份与部署

- Windows 备份：
  `D:\CodexFolder\JetsonNano\Copyfiles\backups\26E_vision\20260729-160859_basic_task`
- Jetson 备份：
  `~/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729-160859_basic_task`
- Jetson 身份：`aarch64`、`monthwu-jetson`、`jetson`。
- 本次临时直连地址 `192.168.137.161` 未写入项目或 SSH 配置。
- 8 个关键源码、接口和配置文件在 Windows 与 Jetson 上 SHA256 一致。

## 构建与测试

Jetson 完成以下 8 个包的选择性构建：

```text
puzzle_geometry
vision_interfaces
puzzle_perception_node
puzzle_solver_node
puzzle_coordinator_node
serial_bridge_node
web_tuner_node
vision_bringup
```

结果：`8 packages finished`。

单元测试结果：

```text
Summary: 7 tests, 0 errors, 0 failures, 0 skipped
```

覆盖：通用平面几何、绿色 A4 检测、三项拼图求解器测试、旧视觉帧和新五字段帧格式。

协调节点话题级集成结果：

```text
IDLE
-> WAIT_GREEN_A4
-> TASK_SELECTED
-> WAIT_PLAN
-> WAIT_PLACEMENT_DONE
-> WAIT_PLAN
-> COMPLETE
-> IDLE
```

共观察到 3 次扫描请求和 1 次指定碎片放置请求，最终状态为
`active_task=0, reason=NO_TASK`。

串口伪终端集成结果：

```text
[12.35,-6.50,30.00,40.25,-15.00]
```

实际从 Linux PTY 主端只读到上述一个完整帧，未发现周期性重复旧位姿。

## Debug 冒烟

- 启动节点：`puzzle_perception_node`、`puzzle_solver_node`、
  `puzzle_coordinator_node`、`serial_bridge_node`、`web_tuner_node`。
- 为避免影响真实 MCU/陀螺仪，本次冒烟强制 `serial_enabled=false`。
- 网页 `/api/status` 返回 HTTP 200。
- debug 图像可用，分辨率 1920×1080，抽查图像流约 29 FPS。
- 串口调度观察值 200 Hz、`frequency_ok=true`。
- 冒烟日志未发现 `BUG_POINT`、`ERROR` 或 `FATAL`。
- 清理后无本次项目进程、无 ROS 节点、`/dev/video0` 无占用。

## 尚未验证

- 用户尚未给出基础题确定碎片的具体形状，因此没有登记形状模板或完成真实模板匹配。
- 未在本次物理场景中确认绿色 A4、真实碎片轮廓、中心点和完整布局求解正确性。
- 固定工作区绝对毫米映射仍未实测，`workspace_mapping_valid=false`。
- PTY 证明 Jetson 生成并完整写出五字段帧，不证明 MSPM0 已解析、执行或返回确认。
- 真实 UART1、UART0 陀螺仪转发和 MCU 端到端链路未在本次冒烟中启用。

## 回滚

1. 停止 `26E_vision` 运行实例。
2. 从 Windows 备份目录按原相对路径恢复现有文件。
3. 删除备份清单中列出的新增文件和 `src/puzzle_geometry/`。
4. 如 Jetson 已部署，从 Jetson 备份目录恢复覆盖文件，并删除同一新增文件清单。
5. 重新构建上述 8 个包并运行 `colcon test-result --verbose`。
