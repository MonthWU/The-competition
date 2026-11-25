# 有雷达分项目

本项目使用实时雷达观测和已确认定位，更新理论路线。共享camera、communication、HMI和模型，配置与启动入口独立。

从本目录运行：

```bash
bash start.sh work
bash start.sh debug --check
```

起点、道路、任务点和固定障碍全部通过config/route.json修改，格式与接口见[共享规划说明](../../docs/TODO1_NAVIGATION.md)。当前场地数据未填写，网页显示configuration_required；这不是已有实机路线。

雷达串口、伸出后的外参及定位快照路径需在config/board.json补充；雷达设备路径为空时检查失败。
