# 项目工具

所有命令均从 `/root/dev_ws/appli` 项目根目录执行。

| 路径 | 用途 |
|---|---|
| `build_package.sh` | 构建指定 ROS 包，例如 `bash tools/build_package.sh obj_detect` |
| `verify_full_flow.py` | 板端完整链路联调，使用真实相机、PTY 和临时测试配置 |
| `verify_flow_guards.py` | 板端检查预扫描失败阻断及节点故障清理 |
| `diagnostics/camera_fps.py` | 历史 `/dev/video0`、`/dev/video2` 帧率诊断；相机身份以正式入口 by-id 为准 |
| `diagnostics/qr_topics.py` | 查询二维码话题是否存在 |
| `legacy/` | 归档的 tmux 启动、环境恢复、旧二维码实验及原 `other/` 工具 |

赛前运行使用根目录 `start_all.sh` 或 `start_simple.sh`，参数与设备要求见 [赛前运行说明](../docs/SCHOOL_RUNBOOK.md)。
`legacy/` 中保存早期工具，包含旧设备编号与部署假设，不参与当前启动链路。
