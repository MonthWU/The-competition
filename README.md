# appli — RDK X5 智能搬运视觉系统

ROS 2 / TROS Humble 视觉项目，板端工作区为 `/root/dev_ws/appli`，当前分支为 `26GongChuan_vision`。
正式配置启用障碍预扫描、二维码原文发送和物块识别；下位机负责云台及运动，视觉负责采图、识别与串口输出。

## 启动与停止

两个入口二选一。当前 `framework/school_profile.json` 的 `object_scan_enabled` 为 `true`，扫码发送后继续识别物块。

| 入口 | 流程 | 启动要求 |
| --- | --- | --- |
| `start_all.sh` | 障碍预扫描 → 二维码 → 物块 | 下位机先发 `[4]` 或 `[24]`，再按 0° / 45° / 90° 各发一次 `[shot]` |
| `start_simple.sh` | 二维码 → 物块 | 直接扫码，不等待启停帧或 `[shot]`，不发送障碍地图 |

```bash
# 完整启动：30 是每条预扫描指令的等待超时秒数，缺省为 30
bash /root/dev_ws/appli/start_all.sh --check
bash /root/dev_ws/appli/start_all.sh 30

# 简化启动：直接从二维码开始
bash /root/dev_ws/appli/start_simple.sh --check
bash /root/dev_ws/appli/start_simple.sh
```

`--check` 只检查该流程的相机、模型、串口和环境脚本，不启动节点；`--help` 查看参数。
初始等待启停区时，下位机可发送 `[skip]`：跳过三角度扫描和障碍定位，释放串口并直接进入二维码阶段；不返回预扫描 `[ack]` 或地图，无需后续 `[shot]`。此指令只在初始起点等待阶段识别。
两个入口共用 `scripts/start_common.sh`，自动加载 `/opt/tros/humble/setup.bash` 与工作区 `install/setup.bash`，默认 `ROS_DOMAIN_ID=42`。
前台任务使用 `Ctrl+C` 停止。两个入口启动时均先自动停止当前项目（包括开机服务和旧的前台任务），确认退出后再启动所选流程，无需先手动停止。停止失败则不启动新任务。开机服务调用入口时保留自身服务；`--check` / `--help` 不停止任务。服务管理命令：

```bash
systemctl stop appli.service
# 通过服务运行完整流程
systemctl start appli.service
systemctl status appli.service --no-pager
journalctl -u appli.service -n 100 --no-pager
```

开机服务持续等待下位机 `[4]`、`[24]` 或 `[skip]`，收到起点后每次 `[shot]` 仍限时 30 秒；失败时 5 秒后重试。服务直接执行 `start_all.sh`，`/usr/local/bin/appli.sh` 是同一入口的兼容包装。
板端升级后需同步已安装 unit 与包装脚本，步骤见 [赛前运行说明](docs/SCHOOL_RUNBOOK.md#两个启动入口与服务安装)。

## 配置与设备

| 配置 | 默认值 / 行为 |
| --- | --- |
| `APPLI_SERIAL_DEVICE` | `/dev/ttyS1`，115200、8N1；三个阶段共用，预扫描释放后扫码阶段接管 |
| `APPLI_START_TIMEOUT` | 初始启停帧等待时间（秒）；`0` 持续等待，仅开机服务默认设置；未设置时沿用启动参数，`[shot]` 超时仍由启动参数决定 |
| `APPLI_SCHOOL_PROFILE` | `framework/school_profile.json` 的绝对路径，可覆盖为临时配置 |
| `object_scan_enabled` | `true`；改为 `false` 时两个入口扫码发送后结束，不要求物块相机或模型 |
| `fixed_obstacle_id` | `null`；公布固定位置后填入实际候选地图 ID，不填测试 ID |
| `roi_start_ids` | `[4,24]`；两个启停区共用同一份 `roi_file`，不复制或变换 ROI |
| `DNN_ENGINE` | `workaround`；当前已通过板端验证的正式推理路径 |

```bash
# 示例：同一串口覆盖贯通全部阶段
APPLI_SERIAL_DEVICE=/dev/ttyS1 bash /root/dev_ws/appli/start_all.sh 30
```

相机按 by-id 识别，设备编号 `/dev/videoN` 不作为身份依据。

| 阶段 | 相机 | 固定路径 |
| --- | --- | --- |
| 障碍预扫描 | DCXIN | `/dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0` |
| 二维码 | KINGSEN KS1A293 | `/dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0` |
| 物块 | LRCP AR0234 | `/dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0` |

完整入口的 `--check` 核对三路相机和两套模型；运行时先检查二维码/物块资源，收到数字起点后再检查障碍相机和模型，以支持 `[skip]`。简化入口要求扫码、物块两路相机和物块模型。
启停区 `[4]` 和 `[24]` 共用 `framework/dnn/obstacle_roi/merged_runtime.json` 的 0° / 45° / 90° 标定数据。两个起点使用相同候选点映射；扫描基线方向仍按各自起点定义。缺少或损坏 ROI 文件时仍报 `CALIBRATION_REQUIRED`。
当前共享三角度 ROI 覆盖候选点 1–13；候选点序号、地图 ID 及复核要求见 [赛前运行说明](docs/SCHOOL_RUNBOOK.md#障碍-roi-覆盖与复核)。

## 通信与阶段切换

预扫描使用 ASCII `[...]` 帧，无换行；启停区 1 为 `[4]`，启停区 2 为 `[24]`。
有效起点通过配置检查后回复 `[ack]`；每次 `[shot]` 完成采图后再回复 `[ack]`；三次结束只发送一帧地图。

```text
下位机 → 视觉：[4] 或 [24]
视觉 → 下位机：[ack]
下位机 → 视觉：[shot] × 3（0°、45°、90°，每次等待回执）
视觉 → 下位机：[ack] × 3
视觉 → 下位机：[1 <障碍ID> <校验>]
```

地图 ID 为 `row * 5 + col`，取值 0–24；校验为障碍数量 XOR 各障碍 ID，编码为两位大写十六进制 ASCII。
校赛要求一个障碍，定位失败、配置缺失或等待超时都会停止完整流程。协议实现见 [serial_protocol.py](framework/serial_protocol.py)。

| 二维码 / 物块输出 | 帧格式 | 行为 |
| --- | --- | --- |
| 二维码 | `FF 37 <UTF-8原文> FE` | 非空 UTF-8 文本原样连发四帧；不裁剪、不校验任务码排列；无码不发送占位帧 |
| 物块 / 标识 | `FF CLASS XL XH YL YH FE` | 检测框中心的图像坐标，x/y 各 16 位，低字节在前；实际检测图像为 640×480 |

| 类别 | CLASS |
| --- | --- |
| `red1` / `black1` / `green1` / `yellow1` / `blue1` / `blue2` | `41` / `42` / `43` / `44` / `45` / `46`（十六进制） |
| `targetOne` / `targetTwo` / `targetThree` | `51` / `52` / `53`（十六进制） |

串口四帧写入并刷新成功后发布 `/qrc_forwarded`，`qrc_cam_killer` 随后发布 `/kill_qrc`。
`obj_camd` 等待扫码相机释放，再启动物块相机；重复切换信号不重复拉起相机。物块每帧发送距离参考点 `(320,240)` 最近的一个，放置标识全部发送。
源码保留旧模型类别兼容映射，见 [obj_serial.py](obj_detect/obj_detect/obj_serial.py)。

## 检测、预览与录像

正式物块推理路径为 `/image` → `obj_dnn` → `/hobot_dnn_detection_raw` → `obj_target_area_filter` → `/hobot_dnn_detection` → 串口与网页。
原图检测框面积小于 `2000` 像素²的目标剔除，等于阈值保留；全部类别共用门限。`enable_roi_filter` 默认关闭，现场标定后再启用。
在线面积参数调整方法见 [面积过滤说明](obj_detect/AREA_FILTER.md)。备用 `native` 存在既有实拍 `exit -11`，当前通过的是默认 `workaround` 路径。

物块图像预览使用板端 HTTP `8000` 与 WebSocket `8080`。录像仅在收到实际图像后创建，默认 MJPG、640×480、20 fps，写入 `_tmp_videos/`。
轮转、打包和退出均关闭 AVI；打包失败保留原文件。完整 launch 的关键节点异常退出会停止任务并清理其他节点，结束任务时也关闭物块相机子进程。

## 构建与验证

在已安装 TROS Humble 的 RDK 上，从工作区根目录构建：

```bash
cd /root/dev_ws/appli
source /opt/tros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 不使用真实下位机的功能测试
python3 -m pytest -q obj_detect/test/test_target_area_filter.py \
  obj_detect/test/test_flow_lifecycle.py qrc_skandier/test/test_qr_forwarding.py \
  framework/test/test_start_timeout.py framework/test/test_skip_prescan.py \
  framework/test/test_shared_obstacle_roi.py

# 真实相机 + PTY + 临时障碍配置/测试图；运行前停止现有任务
export ROS_DOMAIN_ID=42
python3 tools/verify_full_flow.py
python3 tools/verify_flow_guards.py
```

2026-10-04 板端验收：完整入口 19 项、简化入口 8 项、失败分支 3 项全部通过；部署文件 SHA256 一致，测试结束后无项目进程、相机或串口占用。
结果、简化入口测试边界和当前服务状态见 [部署验证记录](docs/verification/2026-10-04-startup.md)。
上述验证使用真实相机、PTY 模拟下位机与测试图，不替代赛场实际障碍定位、现场二维码解码及 MCU 电气/动作验收。

## 目录与文档

```text
appli/
├── start_all.sh / start_simple.sh   两个正式启动入口
├── scripts/                        共用启动逻辑
├── docs/                           赛前说明、待办、历史记录、验证记录
├── framework/                      障碍预扫描、协议、配置、模型与 ROI
├── launch/                         二维码/物块阶段 launch
├── obj_detect/                     物块推理、过滤、相机守护、串口、录像
├── qrc_skandier/                    二维码相机、解码与结束握手
├── qrc_hobot_usb_cam/               C++ USB 相机 ROS 包
├── dnn/                            物块模型与任务配置
├── service/                        systemd unit 与兼容包装
├── gpio_shutdown/                  GPIO 关机服务
├── tools/                          构建、验证、diagnostics/ 与 legacy/
└── webapp/                         OpenCV 参数调节网页
```

`build/`、`install/`、`log/`、`_tmp_scan_imgs/`、`_tmp_videos/` 为生成或运行产物，不提交。
历史入口 `start_new.sh` / `start_old.sh` 已合并为两个正式入口；旧工具移至 `tools/legacy/`。

- [共享障碍标定验证](docs/verification/2026-10-04-shared-roi.md)
- [串口 skip 验证](docs/verification/2026-10-04-skip.md)
- [开机自启动验证](docs/verification/2026-10-04-autostart.md)
- [赛前运行说明](docs/SCHOOL_RUNBOOK.md)
- [当前问题与待验收](docs/CURRENT_ISSUES.md)
- [当前待办](docs/TODO.md)
- [项目工具](tools/README.md)
- [预扫描设计](framework/DESIGN.md)
- [历史开发记录](docs/DEVELOPMENT_HISTORY.md)
- [模型导出问题记录](docs/MODEL_EXPORT_FORMAT_ISSUE.md)
