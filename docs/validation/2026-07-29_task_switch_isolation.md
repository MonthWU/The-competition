# 任务切换与中断隔离验证

日期：2026-07-29

## 问题

协调器原先只允许在 `IDLE`、`COMPLETE`、`FAILED` 接收 `[task,1|2|3]`。因此 Web 页面显示
`当前任务：1 (WAIT_TOOL_CLEAR)` 时，新任务命令会被 `TASK_REENTRY` 拒绝。

仅删除拒绝判断无法隔离旧扫描、旧求解结果和迟到的 `[move,ok]`，存在旧任务异步消息推进新任务的风险。

## 实现

- 协调器在任何状态接收合法任务命令；每次选择（包括再次选择同一任务）递增 `generation`。
- `TaskSession` 立即通知感知、求解和串口节点取消或隔离旧代次工作。
- `ScanRequest`、`PuzzleScene`、`PuzzlePlan`、`PuzzlePlacement`、`PlacementDone` 全链路携带
  `task_id + generation`。
- 协调器只接受当前 `task_id + generation` 的场景、方案和放置完成回执；旧消息只记录 Bug 点并丢弃，
  不消耗当前任务重试次数。
- 求解器使用独立任务会话回调组；切换可在旧求解运行时更新代次，旧求解结束后在发布前再次检查并丢弃结果。
- 串口桥保留已经发出的旧动作及其原代次回执；未发送的旧放置帧会被删除。新任务放置不得复用旧任务的
  无标签 `[move,ok]`，旧动作仍在等待回执时只保留最新任务的延后放置请求。
- 发布放置命令后，协调器把工具状态按“不安全”处理；若此时切换任务，新任务必须等待新的工具离场信号再扫描。

## 验证结果

Jetson：Orin Nano Super 8G，ARM64，ROS 2 Humble。

- `colcon build`：12 个包成功。
- `colcon test`：16 个测试，0 error，0 failure，0 skipped。
- 新接口 `TaskSession`、`ScanRequest`、`PlacementDone` 可由 `ros2 interface show` 正确发现。
- 隔离探针：在 `WAIT_TOOL_CLEAR` 连续执行任务 `1 -> 2 -> 3`，状态依次为
  `generation 1 -> 2 -> 3`，三次切换全部接受。
- 向任务 3 / generation 3 注入任务 1 / generation 1 的 `PuzzleScene`、`PuzzlePlan`、
  `PlacementDone`，协调器分别记录 `TASK_SESSION_SCENE`、`TASK_PLAN_RACE`、
  `TASK_SESSION_ACK` 并保持当前状态；匹配 generation 3 的消息才能推进到 `COMPLETE`。
- Web API：实际调用 `/api/task` 先选任务 1、再选任务 2，`/api/status` 返回
  `active_task=2`、`generation=5`、`state=WAIT_TOOL_CLEAR`。
- Windows 与 Jetson 选择性部署的 19 个文件 SHA256 全部一致。
- 清理后 `ros2 node list` 为空，26E 项目保持停止。

## 边界

当前下位机 `[move,ok]` 不携带任务号、代次或动作号，也没有已确认的“取消正在执行运动”协议。因此：

- 视觉软件可以保证旧场景、旧方案、旧放置回执不会推进当前任务；
- 未发送的旧放置帧可以取消；
- 已经完整发送给下位机的物理运动不能由本次视觉改动撤回。切换后必须等待该动作结束和工具离场，再开始当前任务扫描；
- 未进行真实串口和机构运动测试，不能把“下位机动作可被取消”列为已验证能力。

## 备份与回滚

- Windows 修改前备份：
  `D:\CodexFolder\JetsonNano\Copyfiles\backups\26E_vision\20260729-224408`
- Jetson 覆盖前备份：
  `/home/jetson/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729-225900-task-switch`
- Windows 备份中的 `CHANGE_MANIFEST.md` 记录原文件 SHA256、新增文件清单和恢复步骤。
- 回滚时先停止 26E 项目，恢复备份中的既有文件，删除清单列出的新增消息和本文档，选择性同步到 Jetson，
  重新 `colcon build` 并在测试会话清理后确认 `ros2 node list` 为空。
