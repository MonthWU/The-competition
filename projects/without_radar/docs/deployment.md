# 构建与交付

与父项目共同交付src、assets、native、scripts、config和projects，避免只复制本目录导致共享引用失效。ROS2包及原生库在父项目构建，详见../../../docs/deployment.md。

入口：start.sh [work|debug] [--check|--check-files]，默认work。配置检查不打开设备。确认mission.entry=qr、radar.enabled=false、navigation.enabled=false，不需要route.json。二维码转发、物块和标靶协议沿用公共实现。本轮已部署，ARM构建及板端90项人工/模拟测试通过；独立QR测试仍不代表完整应用或MCU验收。
