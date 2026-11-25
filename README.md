# GongChuan_vision2026 工创视觉项目

2026-11-24本地启动入口新增补光参数：`bash start.sh work light_able` / `light_unable`，独立物块用`bash start_coloritems.sh work light_able` / `light_unable`。默认light_able（曝光50/增益16/亮度128），light_unable使用旧项目补光前参数150/48/128。追加`--check`可查看所选参数，不打开设备。此接口仅本地完成，详见[补光参数选择](docs/LIGHTING_PROFILES.md)。

2026-11-22部署完成：新类别、分辨率框架及新串口协议已同步至板端/root/Monthwu/GongChuang_vision_26，并恢复部署前正在运行的car26-object.service。板端132项测试全通过、ROS2构建及--check-files通过，新版UART已观察到0x71物块包。MCU接收和物理效果待联调；下文旧“仅本地/尚未部署”描述为此前过程记录。

2026-11-22最终同步：二维码状态0x60/业务0x70，物块状态0x61/业务0x71，标靶状态0x62/业务0x72。类别以assets/models/classes.name的用户给定顺序为准，lightblueblock为浅蓝、darkblueblock为深蓝；合法二维码不含]，接收端在0x70类型后读至首个0x5D结束，正文中的[不是新帧头。本轮仅本地，尚未部署。

下位机开发交接使用[完整串口协议](docs/MCU_SERIAL_PROTOCOL.md)：标靶0x72、物块0x71、二维码0x70，以及双向状态、控制命令和兼容预扫描格式。

2026-11-22用户指定当前类型码：标靶0x72、物块合并包0x71、二维码0x70；状态通知0x60/0x61/0x62。仅本地修改，尚未部署；后续历史实测段落中的旧类型码仍代表当时版本。

2026-11-22后续物块串口改为全类别单包：`5B 71 num (class dx_lo dx_hi dy_lo dy_hi)×num 5D`。num表示当前发送了几种物块（0～6），每种仅发送类别及框中心相对画面中心的dx/dy，不发送整个框。无物块时发送num=0，标靶姿态仍独立。126项本地测试中124通过、2跳过，尚未部署；详见[SERIAL_PROTOCOL](docs/SERIAL_PROTOCOL.md)。

2026-11-22本地依根command.md完成双功能分辨率框架及串口TODO。所有上位机报文（含二维码）统一`[`/`]`，状态通知`5B Flagnum 2C BOOL 5D`使用0x61物料转盘、0x62标靶和原始00/01，只交流状态，不控制任务。双端状态分别记录；协议与二维码不含]的分帧规则见[STATE_EXCHANGE](docs/STATE_EXCHANGE.md)。119项本地测试中117通过、2板端专用跳过，尚未部署本轮修改。

2026-11-22项目改名为GongChuan_vision2026；本地工作区根目录同步改名，car26_new源码目录名、car26_system包名及板端部署路径保持现状。本次改名未操作板端设备。

2026-11-18后续：独立扫码预览已增加可见识别结果和重置键，新版本板端97项人工/模拟测试通过。依用户指令当前保留扫码预览运行，正式任务应用未启动；重置键仅清空预览结果，不是整场任务复位。详情见docs/VALIDATION.md最新节。

本地源项目：D:/AIworkplace/RDKx5/GongChuan_vision2026/car26_new。板端项目：/root/Monthwu/GongChuang_vision_26，主机RDKx5-Monthwu-01 / aarch64。2026-11-18本轮业务修改已部署，ARM原生库和ROS2构建通过，板端90项人工/模拟测试全部通过，应用保持停止。

| 板端项目目录内的入口 | 行为 |
| --- | --- |
| `bash start.sh [work\|debug]` | 默认无雷达，直接二维码→物块，不创建规划器 |
| `bash start.sh without-radar [work\|debug]` | 显式选择无雷达 |
| `bash start.sh with-radar [work\|debug]` | 有雷达理论规划；设备、标定、场地与定位来源待填写 |
| `bash start_coloritems.sh [work\|debug]` | 跳过二维码，独立物块及标靶精定位 |

默认work，诊断留档；debug显示终端。追加--check或--check-files只检查，不打开设备。有雷达版当前因device为空而预期报错，不能填假值通过。

共享摄像头、雷达、通信、HMI四模块及模型，各分项目维护自己的配置和入口。二维码默认roi=null整帧扫描，保留非空UTF-8原文，四次发送及flush成功后关闭QR相机再打开物块相机。当前标靶使用0x72角度、像素差及来源报文，颜色物块协议保持。

先读[架构](docs/architecture.md)与[权限和步骤](docs/database.md)，交付见[部署](docs/deployment.md)，接口见[配置](docs/CONFIGURATION.md)、[协议](docs/SERIAL_PROTOCOL.md)、[分项目](projects/PROJECTS.md)，结果见[验证](docs/VALIDATION.md)。

本轮部署未启动真实设备或服务。此前独立扫码曾识别134+213+314+321；旧0x54物块实测属于历史版本，不能作为当前0x72的MCU闭环验收。历史文档保存在docs/history/20261118_before_consolidation，不作为当前入口。
