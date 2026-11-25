# 有雷达架构

C1新扫描 + 同步有效定位快照 + 道路图及固定障碍 -> 累积阻断 -> 动态重规划 -> Telemetry -> HMI。无定位、外参未确认或数据过期均撤下路线。

算法和接口详见../../../docs/TODO1_NAVIGATION.md。配置为config/board.json与config/route.json，模型和原生库从父项目只读引用。
