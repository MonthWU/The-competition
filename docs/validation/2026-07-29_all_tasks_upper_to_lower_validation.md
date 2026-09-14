# 三题统一上半区初始、下半区拼图规则验证

## 1. 变更结论

2026-07-29 用户确认三题共用同一初始场景：拼图区域为纵向 A4 纸，全部碎片随机摆放在
纸张上半区域，装置把碎片移动到下半区域完成拼图。本记录取代此前发挥题二“碎片位于
A4 外侧黑色背景”的输入假设；旧验证记录只作为历史保留。

本次实现包括：

- 三题题面和项目文档统一写明纵向 A4、上半区初始、下半区目标。
- 发挥题一、二在求解前检查每个初始碎片的完整多边形，越过上半区有效边界时返回
  `INITIAL_PIECE_OUTSIDE_UPPER_REGION`。
- 发挥题二取消 A4 外扩画布和纸外背景路径，改为 `210 mm × 297 mm` A4 校正平面内的
  动态浅绿色纸面 Lab 背景采样；扑克牌图案仍通过外轮廓完整保留。
- 发挥题一、二的自由目标位姿被限制在 A4 下半区：配置原点 `y=148.5 mm`、高度
  `148.5 mm`，并对完整目标多边形而非仅中心点施加 3 mm 安全边距。

## 2. 方案选择

- 基础题和发挥题一继续使用独立 `white_piece_*` HSV，因为对象是白色几何碎片且场景受控。
- 发挥题二使用当前 A4 纸面 Lab 中位数与颜色距离，不使用仅白色 HSV，避免红黑牌面图案
  切断碎片轮廓。
- “纸外黑背景分割”因不再符合输入条件而删除；YOLO/YOLO-seg 仍不作为默认路径，因为
  当前任务需要可解释的精确轮廓、毫米边长和目标边界硬约束。

## 3. Windows 静态检查

- `competition_tuning.yaml` 使用 Python `yaml.safe_load` 解析通过。
- 已搜索当前源码和非历史文档，未残留 `challenge_two_a4_exclusion_margin_mm`、
  `challenge_two_max_canvas_extent_mm` 或 `TASK3_PLANE_EXTENT_INVALID`。
- Windows 未使用 WSL，也未将 Windows 结果作为 ROS/ARM64 验收证据。

## 4. Jetson ARM64 验证

目标身份：

```text
aarch64
monthwu-jetson
jetson
ROS_HUMBLE_OK
```

构建命令涉及以下 6 个包，结果全部完成：

```text
vision_interfaces
puzzle_geometry
puzzle_perception_node
puzzle_solver_node
puzzle_coordinator_node
vision_bringup
```

测试结果：

```text
Summary: 3 packages finished
Summary: 16 tests, 0 errors, 0 failures, 0 skipped
```

关键覆盖：

- `lab_background_mask`：浅绿色 A4 纸面为背景、带红黑图案的浅色碎片保持为完整前景，
  A4 外区域不成为候选，不稳定背景继续失败关闭。
- `ChallengePlacesCompleteRectangleInsideLowerHalf`：四片初始中心均在上半区，求解后的每个
  目标多边形顶点均位于 `x=[3,207] mm`、`y=[151.5,294] mm` 安全边界内。

安全冒烟只启动感知、求解和协调节点，使用 GStreamer 测试源，不启动串口桥、不发送控制帧。
任务 3 在实测标定仍未启用时按预期返回：

```text
task_id: 3
solved: false
status: SCENE_INVALID:CALIBRATION_REQUIRED
placements: []
```

冒烟结束后 ROS 节点列表为空。日志：

```text
/home/jetson/ProjectsByMonthWU/VisionJetson/logs/26E_vision_task3_smoke_20260729.log
```

17 个选择性部署文件的 Windows/Jetson SHA256 全部一致。

## 5. 回滚

- Windows 备份：`D:\CodexFolder\JetsonNano\Copyfiles\backups\26E_vision\20260729-212448`
- Jetson 备份：`/home/jetson/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729-213732-upper-lower/prechange.tar`
- Jetson 备份归档 SHA256：
  `e3871330dc68fe665bd71125cd93db9865c95afbba3708832a8bb96737436150`

Windows 回滚按备份 `MANIFEST.md` 恢复并删除本验证文件。Jetson 回滚时在项目目录执行：

```bash
tar -C /home/jetson/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision \
  -xf /home/jetson/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729-213732-upper-lower/prechange.tar
```

随后重新构建相同 6 个包并复查节点为空。

## 6. 未验证边界

- 当前 `camera_calibration_valid=false`，发挥题二真实图像正向感知仍被安全门阻止；本次没有
  伪造相机内参。
- 尚未用真实纵向浅绿色 A4、上半区随机扑克牌碎片和最终照明验证背景误分率、计数准确率、
  轮廓 IoU、顶点 P50/P95、完整拼图正确率、唯一性拒绝率和端到端时延。
- 未连接或驱动机械机构，未打开串口，未验证实际移动、放置精度或 `[move,ok]` 硬件握手。

