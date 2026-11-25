# 分项目索引

| 分项目 | 入口 | 配置 | 行为 |
| --- | --- | --- | --- |
| 无雷达 | without_radar/start.sh | without_radar/config/board.json | 直接二维码→物块，无上位机规划 |
| 有雷达 | with_radar/start.sh | with_radar/config/board.json与route.json | 定位及雷达驱动更新路线 |

父项目统一维护../src、../assets、../native与构建脚本。有雷达版场地数据、设备与定位生产者待接入，规划仅显示，不下发运动指令。无雷达版不需要路线配置。完整接口见../docs/TODO1_NAVIGATION.md。
