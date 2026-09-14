# 2026-07-29 基础题四模板与旋转角验证

## 变更目标

- 将用户提供的基础题尺寸图登记为 4 块固定模板；
- 完成图固定为 `100 mm × 60 mm`；
- 模板不匹配时拒绝退回通用矩形猜测；
- 串口 `angle` 改为从当前姿态到目标姿态的完整旋转差，顶视图逆时针为正、
  顺时针为负，范围 `[-180°, 180°)`。

## Windows 侧验证结果

1. 参考图已复制到 `docs/images/basic_task_puzzle_geometry.png`，大小 35484 字节，
   SHA256 为 `31D199AD17817E25022AB6DD3B229560796DFC54E731C30B0E5BFBCEECCF8465`。
2. 从代码模板重新计算得到四块面积 `2400、480、1080、2040 mm²`，总面积
   `6000 mm²`；对角线三段长度为 `20、50、30 mm`。
3. `puzzle_geometry` 测试使用 Windows MinGW `g++ -std=c++17 -Wall -Wextra
   -Wpedantic` 实际编译并运行通过，覆盖逆时针正、顺时针负及全周归一化。
4. YAML 解析通过；`placement_angle_mode` 为 `delta`，模板 RMS、单点最大误差和面积
   容差均能由比赛调参文件加载。
5. 等价数值回归使用四块随机旋转、部分反向顶点顺序、约 0.35 mm 顶点扰动及
   ±4% 面积扰动，唯一匹配顺序为四个正确模板；示例输出角为
   `+20.264°、-41.057°、+62.556°、-83.163°`，符号与定义一致。
6. 面积被改为 `100 mm²` 的错误四边形被模板门控拒绝。
7. 修改文件差异空白检查、备份文件 SHA256 和新增文档尾随空白检查通过。

## Jetson 部署与断电恢复验证

1. 通过会话级 `HostKeyAlias=monthwu-jetson` 连接并确认目标身份为
   `aarch64 / monthwu-jetson / jetson`。验证期间 Jetson 曾意外断电，重新开机后的
   启动时间为 `2026-07-29 17:59:02`。
2. 覆盖部署前备份位于
   `~/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729-174454`，清单共 22 项。
   重启后重新核对 22 个选择性部署文件，Jetson SHA256 与 Windows 权威副本全部一致。
3. ARM64/ROS2 全量构建通过：
   `puzzle_geometry`、`vision_interfaces`、`puzzle_perception_node`、
   `puzzle_solver_node`、`puzzle_coordinator_node`、`serial_bridge_node` 和
   `vision_bringup` 共 7 个包完成；绿色框感知文件补充同步后的 2 包重建也通过。
4. 断电重启后重新执行 4 个包的测试，`colcon test-result --verbose` 返回
   `9 tests, 0 errors, 0 failures, 0 skipped`。其中基础题测试覆盖给定四模板求解、
   模板不匹配拒绝、逆时针为正和顺时针为负。
5. 禁用串口发送后启动四节点冒烟测试，在线节点为
   `puzzle_perception_node`、`puzzle_solver_node`、`puzzle_coordinator_node` 和
   `serial_bridge_node`。运行时参数确认如下：
   - `basic_task_template_enabled=true`；
   - `basic_template_max_vertex_rms_mm=3.0`；
   - `basic_template_max_vertex_error_mm=5.0`；
   - `green_a4_s_min=25`；
   - `green_frame_min_short_long_ratio=0.3`；
   - `placement_angle_mode=delta`；
   - `serial_enabled=false`、`placement_tx_enabled=false`。
6. 向 `puzzle/scan_request` 发布一次 `std_msgs/Bool(data=true)` 后，相机成功产生
   `1920 × 1080` 的 `puzzle/scene` 消息。当前实景没有绿色框，系统按设计返回
   `a4_detected=false`、`valid=false`、`status=GREEN_A4_NOT_FOUND`，没有输出拼图块或
   错误映射。
7. 测试期间 `/dev/ttyTHS1` 与 `/dev/ttyTHS3` 均未被占用；结束后测试进程组、
   四个 ROS 节点及 `/dev/video0`、两路串口占用均清空。

关键日志：

- `logs/20260729_basic_template_build.log`
- `logs/20260729_green_frame_rebuild.log`
- `logs/20260729_post_reboot_tests.log`
- `logs/20260729_post_reboot_test_results.log`
- `logs/20260729_post_reboot_camera_smoke.log`

## 仍需实物闭环验收

本次已经验证构建、算法单元测试、参数加载、真实 CSI 相机采集、ROS 消息链路和安全
退出，但当前相机画面没有用户提供的绿色框和四块实物。因此尚未验证浅色至一般绿色
实物框的召回率、四块轮廓在真实透视和光照下的模板匹配、机械放置闭环以及下位机对
串口角度字段的解析。后续实物验收应覆盖随机旋转、顶点顺序、偏暗/偏亮和每次放置后
复核，并逐帧核对 `rotation_delta_deg` 与顶视图实际旋转方向。
