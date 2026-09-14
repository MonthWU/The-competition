# 2026-07-29 发挥题一 Jetson 验证记录

## 验证范围

- Windows 权威副本：`D:/CodexFolder/JetsonNano/Copyfiles/26E_vision`
- Jetson 部署镜像：`/home/jetson/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision`
- 目标身份：`aarch64 / monthwu-jetson / jetson`
- 验证内容：任务2几何求解、自由锚定落位、任务队列、五字段帧、`[move,ok]` 映射、构建、测试和清理。
- 未覆盖：真实白色拼图、现场照明、真实绿框、真实 MCU 和机构运动。

## 构建

以下包在目标 Jetson ROS 2 Humble 环境中构建成功：

```text
puzzle_geometry
vision_interfaces
puzzle_perception_node
puzzle_solver_node
puzzle_coordinator_node
serial_bridge_node
vision_bringup
```

构建结果：`7 packages finished`。

## 单元测试

最终回归：

```text
Summary: 10 tests, 0 errors, 0 failures, 0 skipped
```

覆盖内容包括：

- 基础题4片模板求解和模板不匹配拒绝；
- 通用4片矩形与 T 形接缝；
- 发挥题一保持可行锚定片、目标方向不强制水平；
- 绿色框检测；
- 相机中心坐标与角度换算；
- 五字段串口格式；
- `[task,1|2|3]` 解析；
- `[move,ok]` 只能确认已完整发送且唯一等待中的拼图块，提前和重复确认拒绝。

最终测试日志：`/tmp/26e_task2_final_tests_20260729.log`。

## 伪终端端到端烟测

使用 `socat` 临时伪终端代替 MCU，未打开真实控制串口。实际启动：

```text
/puzzle_coordinator_node
/puzzle_solver_node
/serial_bridge_node
```

输入任务命令和3片合成场景后，求解器两次输出：

```text
{"status":"SOLVED","active_task":2,"piece_count":3,"solve_ms":317.735}
{"status":"SOLVED","active_task":2,"piece_count":3,"solve_ms":280.418}
```

一片被选为不移动锚定片，串口桥实际写出另外两片：

```text
[-195.00,-205.50,45.00,-160.50,-90.00]
[75.00,4.50,45.00,-55.50,0.00]
```

伪 MCU 每次回写 `[move,ok]`，串口桥依次记录：

```text
MCU move acknowledgement accepted: piece=1
MCU move acknowledgement accepted: piece=3
```

协调器状态顺序：

```text
WAIT_GREEN_A4
TASK_SELECTED
WAIT_PLAN
WAIT_PLACEMENT_DONE queued_placements=2 active_piece_id=1
WAIT_PLACEMENT_DONE queued_placements=1 active_piece_id=3
COMPLETE reason=TASK_2_ALL_PIECES_COMPLETE
IDLE reason=NO_TASK
```

烟测日志：

- `/tmp/26e_task2_smoke_20260729_1926.log`
- `/tmp/26e_task2_status_20260729_1926.log`
- `/tmp/26e_task2_solver_20260729_1926.log`

## 性能与超时

- 3片合成场景求解：约 0.28--0.32 s。
- 4片自由锚定单元测试：约 16.8 s。
- 因此任务2计划超时独立设为 30 s；基础题仍为 5 s。

这些是合成几何输入下的 Jetson 数据，不代表真实图像端到端时延。

## 部署与回滚

- Windows 修改前备份：`D:/CodexFolder/JetsonNano/Copyfiles/backups/26E_vision/20260729_184820`
- Jetson 修改前备份：`/home/jetson/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729_190427`
- Jetson 备份包 SHA256：`2a623836b865738a1ab3b9bcf2e0815a2a3d36ef60d8cc3efe12743cfd0e1fda`

Windows 回滚按备份目录中的 `backup_manifest.md` 恢复对应相对路径，并删除清单列出的新增文件。Jetson 回滚时在部署目录外解包 `pre_task2_source.tar.gz` 到 `26E_vision`，然后重新构建受影响包。

## 清理结果

- 烟测独立进程组：已终止，无残留进程。
- 最终 `ros2 node list`：为空。
- 未停止或修改任何测试前已存在的 ROS 节点；测试前本来就为空。

## 尚待真实硬件验证

真实图像下仍需测量：绿框定位误差、白色轮廓 IoU、顶点毫米误差 P50/P95、20 mm 边长门控误差、1--4片完整布局正确率、拒绝率、真实 UART 丢包/重复确认、机构放置误差。取得这些证据前，不宣称现场精度或成功率已经达标。
