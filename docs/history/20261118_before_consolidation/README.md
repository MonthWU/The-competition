# CAR26 新项目

2026-11-18本地新增有/无雷达分项目，入口和配置见[分项目索引](projects/PROJECTS.md)，规划接口见[TODO1说明](docs/TODO1_NAVIGATION.md)。按用户后续澄清，`bash start.sh without-radar work`直接从二维码开始，不进行上位机规划；`bash start.sh with-radar debug`保留动态规划与网页展示。省略分项目时默认无雷达，独立物块入口保持不变。两版共用视觉、通信与模型。只有有雷达版需要填写道路、起点、任务点与障碍配置。本地分项目尚未部署；已授权的独立二维码测试实测识别到134+213+314+321，不等于完整应用或MCU验收。

按 `D:/AIworkplace/RDKx5/26GongChuan_vision/command.md` 重写的四模块应用：摄像头、雷达、通信、人机交互。业务由一个应用统一管理，不再通过多个旧 ROS 节点、话题和子 launch 串接。ROS2 包仅提供安装和可选启动包装。

本地项目：`D:/AIworkplace/RDKx5/26GongChuan_vision/car26_new`。现有板端 `/root/Monthwu/car26_ws` 仍是运行权威；本次没有向板端写入文件、启动或停止设备链路。重写前完整本地备份在 `../artifacts/RDK_CJLU_V01`。

先读 [架构](docs/architecture.md)，执行时遵守 [权限](docs/database.md)，交付按 [构建与部署](docs/deployment.md)。接口变化及证据见 [迁移说明](docs/MIGRATION.md) 和 [验证记录](docs/VALIDATION.md)。

## 已实现

- 摄像头：OpenCV V4L2、二维码原文识别、现有 YOLO11 BPU 模型加载与 NV12/DFL 解码、面积/中心位置过滤、标靶冲突消解、可选录像、JPEG 预览。
- 雷达：C1 查询与健康检查、转速设置、Standard 5 字节点解析、圆周拼接、距离/质量过滤、障碍物聚类、可配置外参与地图匹配。
- 通信：继续 `/dev/ttyS1`、115200、8N1；握手和三次 `[shot]`；小写 `[ack]`；地图 XOR 校验；二维码四帧发送；物块/标靶小端像素坐标。部分写入补齐，失败中止相机切换。
- 人机交互：Web 状态、最近通信事件、按角色相机预览、车体坐标雷达障碍物显示。
- `work` 默认，`debug` 共用完整业务链路，仅增加日志。

## 配置与启动

新设备默认配置在 [config/system.json](config/system.json)。两路新相机及雷达设备路径暂空；图像尺寸、帧率是待实物核对的初始配置。雷达地图坐标和朝向标定也暂空。配置检查会报出缺失项，不能把默认外参当作已标定值。填写方法见 [配置说明](docs/CONFIGURATION.md)。

以下命令是交付材料，运行硬件链路仍由用户确认和执行：

```bash
# 板端新项目目录内；此检查不打开设备
PYTHONPATH=src/car26_system python3 -m car26 --config config/system.json --check

# 补齐设备及标定后
bash scripts/start.sh config/system.json work
bash scripts/start.sh config/system.json debug
```

`config/legacy_camera.json` 提供现有三相机、相机 ROI 预扫描参数，用于迁移对照。它运行新应用，也不启动旧节点；其 ROI 仅对起点 4 和原相机安装有效。该配置还未在硬件上验证。

在 Windows 本地执行已获用户授权的纯算法测试：

```powershell
& 'D:\AIworkplace\RDKx5\26GongChuan_vision\car26_new\scripts\check.ps1'
```

本项目不自动安装服务或设置开机自启。直接启动使用 Python 应用；需要 ROS2 包时按部署文档构建单个 `car26_system` 包。
# 2026-11-18 实机部署状态

两个公开入口：`bash start.sh [work|debug]`启动二维码→物块；`bash start_coloritems.sh [work|debug]`只启动物块检测及标靶精定位。省略模式默认为work。work运行日志保存在`log/runtime/`且不逐帧打印，debug直接显示终端诊断；业务链路相同。两者追加`--check`或`--check-files`可只做检查，不打开硬件。

本四模块版已部署于`/root/Monthwu/GongChuang_vision_26`，旧项目归档在`/root/Monthwu_archive/20261118_todo2`。原生OpenCV库、ROS2包与51项人工/模拟测试通过；当前保持待启动，实机效果待验收。

用户手动启动：`bash start.sh work`为二维码→物块；`bash start_coloritems.sh debug`为直接物块与标靶二精定位。TODO2的3°门控、1080p圆心差值、0x54报文及结束命令见`docs/TODO2_ALIGNMENT.md`。当前board配置不启用未标定的雷达动态规划。下文早期未部署描述为历史记录，以此日期及本轮交付记录为准。
