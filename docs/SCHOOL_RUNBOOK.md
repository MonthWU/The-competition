# 校赛视觉链路（2026-10-04）

当前操作与协议入口见 [项目 README](../README.md)，最新板端结果见 [2026-10-04 部署验证记录](verification/2026-10-04-startup.md)。

## 规则与入口

- 抽签起点为启停区 1（内部 ID 4）或启停区 2（内部 ID 24）。校赛只有一个固定位置的障碍物，但具体位置尚未公布。
- 下位机先发 `[4]` 或 `[24]`，视觉确认配置后回 `[ack]`；随后按 0°、45°、90° 各发一次 `[shot]`，视觉每次采图后回 `[ack]`。三次结束时只发送一帧 `[1 <障碍ID> <校验>]`。
- 然后启动二维码阶段。解码得到非空 UTF-8 文本后直接发送，不检查任务码格式、数字范围或颜色/位置排列；`123+231`、`156+123+516+231` 及其他文本均原样转发。串口沿用 `FF 37 <UTF-8文本> FE`，连续发送四帧。未识别到二维码时不发布占位结果，也不发送占位串口帧；实际解码为 `0000000` 的二维码仍作为普通文本发送。当前 `object_scan_enabled: true`；四帧写入并刷新 UART 后发布 `/qrc_forwarded`，杀码节点再发布 `/kill_qrc`，等待 KS1A293 释放后启动 LRCP 物块相机。
- 物块模型所有类别共用检测框最小面积 `min_target_area_px=2000`，按原图 `width * height` 过滤，小于阈值剔除。默认 workaround 推理通过原始话题 → 面积过滤 → 检测话题 → 串口与网页。物块每帧发送最近的一个，放置标识全部发送。
- 完整启动：`bash /root/dev_ws/appli/start_all.sh --check` 检查设备，`bash /root/dev_ws/appli/start_all.sh 30` 开始障碍 → 二维码 → 物块流程；`appli.service` 指向同一入口。
- 简化启动：`bash /root/dev_ws/appli/start_simple.sh --check` 检查二维码/物块设备，`bash /root/dev_ws/appli/start_simple.sh` 直接从二维码开始，扫码后继续物块识别；不等待 `[4]`、`[24]` 或 `[shot]`。
- 两个入口均先自动停止现有项目，再启动所选流程；自动停止开机服务和旧的前台任务，停止失败则不启动新任务。开机服务自身调用时保留当前服务。`--check` / `--help` 不停止任务。两者均读取 `framework/school_profile.json`，当前 `object_scan_enabled: true`。完整启动要求三路相机和两套模型；简化启动要求扫码与物块两路相机和物块模型。
- `APPLI_SERIAL_DEVICE`（默认 `/dev/ttyS1`）贯通障碍、二维码和物块阶段；预扫描关闭串口后主任务再打开。启动入口显式设置 ROS 日志目录，支持 systemd 后台运行。
- 停止任务时关闭物块相机子进程和当前录像文件。等待二维码时不生成空 AVI；录像打包前关闭文件，打包失败保留原文件。关键进程异常退出时，完整 launch 报错退出并清理其余进程。

## 串口跳过障碍扫描

完整入口等待启停区时，下位机发送小写 ASCII `[skip]` 即可直接进入二维码阶段。
视觉不返回预扫描 `[ack]`、不等待 `[shot]`、不采集障碍照片、不创建障碍定位器，也不发送障碍地图；预扫描阶段关闭 UART 后，二维码阶段接管同一串口。
此指令只在初始握手阶段有效，已经接受 `[4]` 或 `[24]` 后仍按原有三次拍摄流程运行。
`[skip]` 不要求障碍相机、模型或起点 ROI；二维码/物块资源按当前 `object_scan_enabled` 配置检查。`start_all.sh --check` 仍检查完整流程所需设备。

## 两个启动入口与服务安装

```bash
# 完整流程，30 秒等待每条预扫描指令
bash /root/dev_ws/appli/start_all.sh --check
bash /root/dev_ws/appli/start_all.sh 30

# 从二维码开始；与完整流程二选一
bash /root/dev_ws/appli/start_simple.sh --check
bash /root/dev_ws/appli/start_simple.sh
```

两个脚本共用 `scripts/start_common.sh`，自动加载环境并传递同一串口配置。完整入口在预扫描失败时立即退出，简化入口不发送障碍地图。`object_scan_enabled: false` 时，两者均使用仅扫码的 launch，不要求物块相机或模型。

升级板端项目后，已有服务需要同步新的 unit 与兼容包装；复制项目源码不会自动更新 `/etc/systemd/system` 或 `/usr/local/bin`。板端停止当前任务后执行：

```bash
systemctl stop appli.service
install -m 644 /root/dev_ws/appli/service/appli.service /etc/systemd/system/appli.service
install -m 755 /root/dev_ws/appli/service/appli.sh /usr/local/bin/appli.sh
systemctl daemon-reload
systemctl enable appli.service
# 需要服务运行完整流程时执行
systemctl start appli.service
```

项目说明集中在 `docs/`；调试与历史工具见 [tools/README.md](../tools/README.md)。模型、ROI、ROS 包与 launch 仍使用原有运行路径。

开机服务设置 `APPLI_START_TIMEOUT=0`，因此上电后持续等待 `[4]`、`[24]` 或 `[skip]`；初始等待不会在 30 秒后退出。
收到有效起点后，各次 `[shot]` 仍保留 30 秒超时。设备尚未就绪、配置错误或节点异常退出时，systemd 每 5 秒重试；手动 `systemctl stop` 不触发重试。
普通前台 `start_all.sh 30` 未设置该环境变量时，初始起点等待仍为 30 秒。`journalctl -u appli.service -f` 查看等待和运行日志。

## 校赛障碍物配置

`framework/school_profile.json` 是唯一校赛位置配置入口。组委会公布障碍位置后，将 `fixed_obstacle_id` 从 `null` 改成 5×5 网格的候选点 ID（`1, 3, 5, 7, 9, 11, 12, 13, 15, 17, 19, 21, 23`）即可；下次启动生效。固定模式仍完成三角度采图和留档，但地图直接采用固定 ID，不要求视角里检出障碍。下位机也可按同一 ID 固定位置。

用户已确认 `[4]` 和 `[24]` 使用同一组标定。正式配置统一为 `roi_start_ids: [4,24]`，两者读取相同 `roi_file`，不再因 `[24]` 单独缺少标定而拒绝扫描。原始点列与运行 ROI 保持原内容；0°、45°、90° 的候选映射完全共用。缺少或损坏 ROI 文件仍返回 `CALIBRATION_REQUIRED`。

`framework/dnn/obstacle_roi/merged_runtime.json` 是当前两个启停区共用的三角度 ROI。只有在实际赛场相机角度、安装位置和分辨率（DCXIN 1280×720）与标注照片相同时才可使用；现场应核对留档 `_tmp_scan_imgs/scan_*.jpg`。检测到多个候选位置且没有足够一致票数时，预扫描失败，不会把空地图或占位位置发给下位机。

### 障碍 ROI 覆盖与复核

候选点序号与地图 ID 是两套编号：`map_model.py` 中第 13 个候选点为零基网格坐标 `(4,3)`，地图 ID 为 `4*5+3=23`，位于底行右侧路段；地图 ID 13 则是第 8 个候选点 `(2,3)`。候选点序号也不是模型类别；障碍模型只识别 `block`。

旧的第一组 0°/45° 标注及旧运行 ROI 有误，已由 2026-09-29 六张照片的整套重标数据替换。原始点列见 `framework/dnn/obstacle_roi/source_annotations.json`，生成脚本为同目录 `fit_obstacle_rois.py`；`merged_runtime.json` 是运行文件，`fit_summary.json` 记录审计数据。带原图的叠加总览可用脚本和板端照片生成。旧 `g1_0.json`、`g1_45.json` 已移除，历史内容可从 Git 记录查看。

| 角度 | 新版 ROI 候选点序号 |
| --- | --- |
| 0° | 2、4、5、8、11 |
| 45° | 1、2、3、4、6、7、8、9、10、11、12、13 |
| 90° | 9、10、12、13 |

并集覆盖 1–13，三角度内均无重叠。候选点 13 在 45°、90° 可映射到地图 ID 23。0° 的 label 4 是实际三角形；45° 的 label 12 在两组照片中分别为三角形与短顶边四边形，融合结果标为中等置信度。原始 `w` 是用户给出的点列面积；确认的三角形在运行文件中按外侧三点重新计算面积。重新拟合和验收方法见 [obstacle_roi/README.md](../framework/dnn/obstacle_roi/README.md)。

这些 ROI 已合并为 `[4]`、`[24]` 共用像素模板；按用户确认直接复用原数据。实际赛场障碍仍需逐点核对，相机位置、云台角度或分辨率变化时须重新核对。组委会未公布固定位置，`fixed_obstacle_id` 保持 `null`；只有公布后才能填入对应的地图 ID。

## 相机分配与验收

| 阶段 | 身份 | 板端设备 |
| --- | --- | --- |
| 障碍物 | DCXIN | `/dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0` |
| 二维码 | KS1A293 | `/dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0` |
| 物块 | LRCP AR0234 | `/dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0` |

当前完整模式启动检查要求三路相机和两套模型。不按 `/dev/videoN` 猜测身份。完整流程验收需依次看到：三角度留档与唯一障碍地图帧、二维码经串口原样发送四帧、扫码相机退出、物块相机出图及过滤后坐标输出。没有赛场画面时，不把软件检查当作现场验收。

物块串口节点默认关闭 `enable_roi_filter`。旧参数 `roi_x_min=380` 会漏掉历史实拍画面中位于 x=250/348 的真实浅蓝/红色物块；在当前相机视角下完成现场 ROI 标定后，可通过 ROS 参数重新开启并调整边界。面积过滤能消除小框，较大机械结构误检仍需现场复核。

## 完整流程自动联调

板端空闲、三路相机接齐时执行：

```bash
export ROS_DOMAIN_ID=42
source /opt/tros/humble/setup.bash
source /root/dev_ws/appli/install/setup.bash
python3 /root/dev_ws/appli/tools/verify_full_flow.py
```

脚本从正式 `start_all.sh` 入口运行，使用同一 PTY 模拟下位机，发送 `[4]` 与三次 `[shot]`。
正式物块开关必须为 true；障碍 ID 19 只写入临时测试配置。三路相机均真实打开，二维码与已知物块图仅在测试时注入。
验证四次 ACK、一帧测试地图、四帧二维码原始 UART 字节、过滤后的已知物块坐标、网页 HTTP、WebSocket 二进制画面和录像。
停止后复核每个新 AVI 的可读性及索引。结果及图像保存在 `/tmp/appli_continuity`，退出码 0 表示全部检查通过。
这项验证不驱动真实云台，也不代替实际障碍定位、二维码现场解码和 MCU 电气/动作验收。

默认 workaround 是当前正式入口。备用 native 在修改前后的实拍对照中均出现 `exit -11`；
本次保留其入口，但失败会终止完整流程，不将 native 实拍推理列为已通过。

2026-10-03 板端验证：两包构建及 39 项检查通过；完整流程 19 项检查全部通过，
同一 PTY 收到四次 ACK、测试地图、四帧二维码以及已知红色物块坐标。
三路相机真实出图，过滤后的物块结果无小于 2000 像素²的目标，WebSocket 收到 46447 字节二进制数据。
停止后三个新 AVI 的索引为 101 / 195 / 72 帧，均可读取首帧；相机、串口无占用，项目节点无残留。

失败分支可复现检查：

```bash
# 同样需要 source TROS 和项目 install/setup.bash
python3 /root/dev_ws/appli/tools/verify_flow_guards.py
```

失败分支现检查：缺少启停指令不进入主任务、共享 ROI 文件缺失时拒绝启动、推理节点故障时完整 launch 非零退出。历史记录中的“起点 24 无 ROI”已被两个起点共用标定的规则替代。

2026-09-29 软件联调：用真实 DCXIN、KS1A293，临时固定障碍 ID 19、虚拟下位机串口及合成二维码，从 `start_new.sh` 完整走通障碍扫描到扫码结束；收到四次 `[ack]`、`[1 19 12]` 和四帧二维码数据，进程退出且未残留。测试用固定 ID 和虚拟串口均未写入正式配置。赛场障碍和实际二维码的现场验收按用户要求暂缓。

2026-09-29 重标后板端静态验收：板端重新拟合得到的运行 JSON 与仓库文件一致；读取器确认 13 个候选点、21 个凸多边形、角度内零重叠，起点 24 缺少 ROI 时拒绝启动，临时固定 ID 23 可映射到 `(4,3)`；`start_new.sh --check` 通过。此项不包含真实障碍的现场识别验收。
