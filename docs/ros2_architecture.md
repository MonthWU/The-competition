# 26E_vision ROS2架构

## 数据流

```text
puzzle/start
  -> puzzle_coordinator_node
  -> puzzle/scan_request
  -> puzzle_perception_node
  -> puzzle/scene
  -> puzzle_solver_node
  -> puzzle/plan
  -> puzzle_coordinator_node
  -> puzzle/placement_request
  -> serial_bridge_node
  -> MCU UART [x0,y0,x1,y1,angle] (当前默认 /dev/ttyTHS3, 115200bps)
  -> 外部机械控制侧
  -> MCU UART [move,ok]
  -> serial_bridge_node 映射为 puzzle/placement_done(piece_id)
  -> puzzle_coordinator_node 下发缓存队列中的下一条
```

## 求解状态

`puzzle_solver_node` 为每个碎片集合维护 `DP[mask]`。叶节点是单片碎片，内部节点
由两个不相交子集二叉合并。合并时枚举外露边，允许整边反向对齐，也允许短边与
长边端点对齐并保留剩余边段，因此可处理T形接缝。

最终候选必须同时满足：

- 发挥题一、二的全部初始碎片轮廓完整位于纵向 A4 上半区；
- 长边90至120 mm、短边50至90 mm；
- 长短边比例符合非正方形要求；
- 多边形重叠面积不超过阈值；
- 总碎片面积与最小外接矩形面积之比达到矩形度阈值；
- 最优解与不同得分的次优解保持足够差距；
- 图案模式下，完整边接缝的灰度条带相关性通过评分。
- 完整目标多边形位于 A4 下半区安全边界内，而不只是中心点落在下半区。

## 质量门控

- `A4_NOT_FOUND`：未检测到固定角点或运行时四边形。
- `PIECE_COUNT_INVALID`：有效碎片数量不是1至4片。
- `INITIAL_PIECE_OUTSIDE_UPPER_REGION`：发挥题初始碎片越过上半区有效边界。
- `SCENE_VALID_LOCAL_ONLY`：轮廓有效，但绝对坐标映射尚未确认。
- `WORKSPACE_MAPPING_REQUIRED`：禁止把A4局部坐标发送给控制侧。
- `NO_VALID_RECTANGLE`：没有满足整体矩形约束的组合。
- `AMBIGUOUS_LAYOUT`：最优解与次优解差距不足。
- `SOLVED`：可以生成完整目标位姿。

## 串口与 HMI

`vision_inference_node`、`opencv_roi_node` 和 `kalman_filter_node` 保留在源码树中用于
参考和回溯，但默认 launch 不启动。

当前 26E 硬件实测 UART1(`/dev/ttyTHS1`) 本地回环失败；默认配置优先保证
MCU 通信：MCU 连接 UART0(`/dev/ttyTHS3`)，使用 115200bps 8N1。串口屏直连
Jetson 时仍按 HMI 角色使用 9600bps 8N1，当前配置保留在 UART1；若 UART1 未修复，
直连串口屏不可作为可靠链路。后续如物理接线再次互换，可同步互换 `mcu_port` 与
`hmi_port`，波特率仍按角色分别使用 `baud_rate` 与 `hmi_baud_rate`。
若串口屏连接单片机，由单片机转发两端数据，则保持 MCU 口为实际接线口并设置
`hmi_over_mcu=true`，此时 `serial_bridge_node` 不再单独打开 `hmi_port`。

默认 launch 启动 `serial_bridge_node` 与 `hmi_control_node`；除非用户明确要求禁用串口，
后续调试和运行均保持串口节点启用。`serial_bridge_node` 是 Jetson 侧 UART 唯一拥有者：
任务 1、2、3 均使用五字段帧。协调器在首次有效求解后缓存本轮全部待执行
`PuzzlePlacement`；串口桥仍以单一未完成请求把 `[move,ok]` 映射成对应
`piece_id` 的完成事件，协调器收到确认后发送队列下一条。任务 3 先复用任务 2
的批量队列结构，再在求解和验收层扩展花纹接缝评分。坐标与角度变换由通用 `puzzle_geometry` 包提供，基础题协议见
`docs/basic_task_protocol.md`，发挥题一见 `docs/challenge_task_one.md`，发挥题二见
`docs/challenge_task_two.md`。

`hmi_control_node` 对所有串口屏 `[cmd,val]` 修改先写入 `vision_system.yaml` 的
`hmi_value_*` 参数，再发送 ACK，并同时发布 `vision/hmi/command` 供可实时生效的节点订阅。
`debug` 属于启动模式参数，默认 `mode:=auto` 下重启后读取 `hmi_value_debug` 生效。
