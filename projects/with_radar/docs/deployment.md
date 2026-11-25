# 构建与交付

与父项目共同交付src、assets、native、scripts、config和projects，避免只复制本目录导致共享引用失效。ROS2包及原生库在父项目构建，详见../../../docs/deployment.md。

入口：start.sh [work|debug] [--check|--check-files]，默认work。配置检查不打开设备。首次运行需补充雷达device、extrinsic、extrinsic_calibrated和navigation.pose_file。规划结果不向MCU发送，物块和标靶协议继续沿用公共实现。本轮已部署并通过构建与人工/模拟检查，真实路径和MCU运动尚未验收。
