# 发挥题二实现与 Jetson 验证记录

## 变更范围

- `PuzzleScene` 增加任务编号和扩展校正画布原点。
- 感知节点增加任务三实测标定门、A4 外侧 Lab 动态背景采样、完整外轮廓和直线交点细化。
- 求解节点增加任务三纹理评分、绝对质量上限、唯一性门和 Debug 目标布局画布。
- 协调节点支持任务三冻结计划、逐片 `[move,ok]` 确认以及完成后直接回到无任务。
- 配置、单元测试和任务三流程文档同步更新。

## Jetson 验证结果

目标：Jetson Orin Nano Super 8G，ARM64，ROS 2 Humble。

1. 选择性构建：
   - `vision_interfaces`
   - `puzzle_geometry`
   - `puzzle_perception_node`
   - `puzzle_solver_node`
   - `puzzle_coordinator_node`
   - `vision_bringup`
   - 结果：6 个包全部构建成功。
2. 单元测试：
   - 结果：16 项，0 error，0 failure，0 skipped。
   - 新增覆盖：Lab 背景重采样和不稳定拒绝、扑克牌内部花纹不影响外侧前景、
     `CHAIN_APPROX_NONE` 轮廓直线拟合与交点、求解绝对质量拒绝，以及 Lab 颜色差、
     灰度 ZNCC、梯度 ZNCC、SSIM 四项组合接缝评分执行路径。
3. 安全冒烟：
   - 仅启动感知、求解、协调三个核心节点；未启动串口桥。
   - 发送 `[task,3]` 后得到 `task_id: 3`、`solved: false`、
     `SCENE_INVALID:CALIBRATION_REQUIRED`、`placements: []`。
   - 证明默认未确认标定状态不会生成五字段移动坐标。
   - 测试清理后 `ros2 node list` 为空。
4. 部署一致性：21 个实现文件 Windows/Jetson SHA256 全部一致；本验证文档随后单独同步。

## 日志与回滚

- Windows 备份：
  `D:\CodexFolder\JetsonNano\Copyfiles\backups\26E_vision\20260729-204026`
- Jetson 备份：
  `/home/jetson/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729-211500-task3`
- 构建日志：
  `/home/jetson/ProjectsByMonthWU/VisionJetson/logs/26E_vision_task3_build_20260729.log`
- 测试日志：
  `/home/jetson/ProjectsByMonthWU/VisionJetson/logs/26E_vision_task3_final_texture_test_20260729.log`
- 冒烟日志：
  `/home/jetson/ProjectsByMonthWU/VisionJetson/logs/26E_vision_task3_smoke_20260729.log`

## 尚未验证

- `camera_calibration_valid` 仍为 `false`；当前相机矩阵是禁用的参考值，不是现场实测标定。
- 尚无真实浅绿 A4、黑色现场背景和扑克牌碎片图像，因此未验证轮廓 IoU、顶点毫米误差、
  完整拼图正确率、唯一性拒绝率、任务三 P50/P95 延迟或串口端到端动作。
- 启用任务三前必须录入最终相机模式的实测内参/畸变，采集真实标注集并按
  `docs/challenge_task_two.md` 的指标完成验收。
