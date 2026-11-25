# 无雷达分项目

本项目直接从二维码阶段开始，识别并转发后关闭二维码相机，再打开物块相机。没有上位机路径规划，不启动雷达或预扫描。共享camera、communication、HMI和模型，配置与启动入口独立。

从本目录运行：

```bash
bash start.sh work
bash start.sh debug --check
```

配置在config/board.json，mission.entry=qr、radar.enabled=false、navigation.enabled=false。不需要场地路线、固定障碍或车辆定位输入。流程边界见[共享说明](../../docs/TODO1_NAVIGATION.md)。

雷达强制关闭，不依赖雷达设备、预扫描相机或定位快照。
