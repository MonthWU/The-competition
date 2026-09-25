# appli —— RDK X5 智能搬运视觉系统

基于 ROS2 Humble 的智能搬运机器人视觉子系统，运行于地瓜 RDK X5（`/root/dev_ws/appli`），
配套 2027 浙江省工创大赛智能搬运赛项。双 USB 相机分时复用：**二维码扫描 + 目标检测（YOLOv5s）**，
检测结果经串口（`ttyS1@115200`）下发下位机执行。

## 1. 快速开始

```bash
# 开机自启（服务已安装，当前 disabled）
systemctl enable appli.service      # 启用自启
systemctl start appli               # 手动启动

# 或直接运行启动脚本（等价）
bash /usr/local/bin/appli.sh

# 一键启动入口（launch 文件）
ros2 launch /root/dev_ws/appli/launch/run_all.launch.py
```

启动链路：`appli.service` → `/usr/local/bin/appli.sh` → `ros2 launch launch/run_all.launch.py`

## 2. 系统架构（两阶段任务流）

**阶段 1 · 二维码扫描**：相机采集 → pyzbar 解码 → 扫到有效码后**杀掉二维码链路并启动检测相机**
**阶段 2 · 目标检测**：检测相机 → hobot_codec 解码 → DNN 推理（YOLOv5s-672）→ 串口下发 + Web 展示 + 录像

```
launch/run_all.launch.py
 ├─ obj_detect.launch.py（完整版 271 行，install→build）
 │    ├─ obj_camd          : 守护节点，收到 /kill_qrc 后启动检测相机（hobot_usb_cam）
 │    ├─ hobot_codec_decode: /image → 共享内存 /hbmem_img
 │    ├─ websocket         : Web 展示检测叠加画面
 │    ├─ dnn_node_example  : YOLOv5s-672（dnn/task_obj.json）→ /hobot_dnn_detection
 │    ├─ hobot_shm         : 共享内存服务
 │    ├─ obj_serial        : 订阅检测+二维码结果 → 串口 ttyS1@115200
 │    └─ obj_video_dumper  : 录像到 _tmp_videos/（MJPG，10s 分片）
 └─ qrc_skandier.launch.py
      ├─ qrc_cam           : 抓帧(640×400@240) → /qrc_image
      ├─ qrc_scanner       : pyzbar 解码 → /qrc_result（无码发 "0000000"）
      └─ qrc_cam_killer    : 有效码 → 广播 /kill_qrc → 自杀（一次性）
```

关键话题：`/qrc_image` · `/qrc_result` · `/kill_qrc` · `/image` · `/hbmem_img` ·
`/hobot_dnn_detection`（PerceptionTargets）· `/serial_send`

## 3. 目录结构

```
appli/
├── launch/run_all.launch.py     # 一键启动（obj_detect + qrc_skandier）
├── obj_detect/                  # 目标检测包（obj_camd / obj_serial / obj_video_dumper）
├── qrc_skandier/                # 二维码包（qrc_cam / qrc_scanner / qrc_cam_killer / flaskr）
├── qrc_hobot_usb_cam/           # USB 相机 ROS2 包装（C++）
├── dnn/                         # 模型资产：原任务 yolov5s_v5s_672 + yolov11 9 类物块/标识（task_obj_v11.json）
├── framework/dnn/               # 【新增】全局扫描 1 类 ball yolo + task_obj_obstacle.json
├── service/                     # appli.service + appli.sh（自启链路）
├── gpio_shutdown/               # GPIO 关机键服务
├── other/                       # 辅助 launch/脚本
├── framework/                   # 【新增】避障预扫描框架（通信完成、识别待实现；见 §6）
└── _tmp_videos/                 # 检测录像输出
```

## 4. 串口协议（上位机 → 下位机，ttyS1 @ 115200）

帧格式：`0xFF(头) + CLASS(1B) + [数据] + 0xFE(尾)`

| 帧 | 字节流 | 说明 |
|---|---|---|
| 二维码 | `FF 37 <UTF-8内容> FE` | 有效码连发 4 次 |
| 检测目标 | `FF CLASS XL XH YL YH FE` | 坐标低 8 位在前；图像系 960×544 |

CLASS 映射（v11 9 类新模型，`obj_serial.py` 已实现）：
`red1=0x41 · black1=0x42 · green1=0x43 · yellow1=0x44 · blue1=0x45 · blue2=0x46`（圆台物块，每帧只发离参考点最近 1 个）·
`targetOne=0x51 · targetTwo=0x52 · targetThree=0x53`（放置区标识，全部发送）。
旧模型（250720_v5s）兼容保留：`rcf=0x31 红圆环 · gcf=0x32 绿圆环 · bcf=0x33 蓝圆环 · rof=0x34 红目标 · gof=0x35 绿目标 · bof=0x36 蓝目标`

行为规则：默认 mode=2（二维码+检测都发）→ 扫到有效码切 mode=1；只发离画面中心最近的
目标，且满足区域过滤（x∈[140,500]、y<420，圆环不受限）；无二维码时每 50 帧发心跳
`0000000`。调试可 `ros2 topic echo /serial_send`。

## 5. 相机分配（2026-09-23 三路相机固化，by-id 路径 + 物理接口固定；2026-09-23 二次修正对调）

板端实接 3 个 USB 相机，**全部挂 USB Bus01（480M）同一 Hub 下**——3 路不并发常开、每阶段用完
立即 `kill` 释放带宽（实测带宽争抢会丢帧）。任务映射（by-id 路径固定，不受 `/dev/video*`
编号漂移影响；**物理 USB 接口长期不拔，拓扑稳定**）。

> **2026-09-23 二次固化确认**：用户明确摄像头不会拔下来；物理 USB 接口（Port 2/3/4）
> 视为不变量。因此**双保险**已就位：① by-id 路径（基于设备 VID:PID+序列号，不随插拔顺序变）；
> ② 物理端口拓扑（Hub 1 Port 2/3/4 → KS1A293/LRCP/DCXIN）。两者任一变化都会触发回归测试。

| 任务 | 相机（USB Port）| 出图 by-id 节点 | max fps | 备注 |
|---|---|---|---|---|
| 扫码 qrc_skandier | **KS1A293**（Port 2）| `/dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0` | 240 | 唯一支持 240fps@640×400，兼容黑白二维码高速抓拍 |
| 检测 obj_detect | **LRCP AR0234**（Port 3）| `/dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0` | 90 | yolov11 9 类（圆台物块 + 放置区标识）；实测拍到物块区画面 |
| 全局扫描 framework/map_scanner | **DCXIN Camera**（Port 4）| `/dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0` | 90 | 1920×1080 MJPG，开局斜视看 2400×2400 全场 |
> **⚠️ 2026-09-23 二次修正**：通过实机画面确认，检测 ↔ 全局扫描 之前物理接线与代码
> 假设反了——LRCP AR0234 实际装在检测位（拍到物块区），DCXIN Camera 实际装在车顶
> 全局扫描位。已对调 `obj_detect_v11.launch.py` / `prescan.launch.py` /
> `framework/map_scanner.py` 三处 by-id。

> **⚠️ DCXIN 固件设计问题（2026-09-23 实测）**：DCXIN 出厂默认 `auto_exposure=1 (Manual Mode)` +
> `brightness=50` + `exposure_time_absolute=78` —— **Linux uvcvideo 驱动下默认输出全黑**
> （亮度 28.6/255）。需要每次上电/USB 复位后**手动执行**：
> ```
> v4l2-ctl -d /dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0 \
>   --set-ctrl=auto_exposure=3 --set-ctrl=brightness=128 \
>   --set-ctrl=exposure_time_absolute=156 --set-ctrl=gain=0
> ```
> **修复后亮度 158.4/255（正常）**。另外 dmesg 报 `Failed to query UVC control 5/7/17` 警告
> （-32 EPIPE），是 vendor 固件的控制查询失败，**Linux uvcvideo 安全忽略导致默认参数错误**。
> 下次 DCXIN 不出图 → 先 v4l2-ctl --get-ctrl=auto_exposure 看是不是 1。

v11 链路 `obj_detect_v11.launch.py` 的 `cap_qrc_devnode` / `cap_objdet_devnode` 默认按上表
写死；`framework/map_scanner.py` 默认设备同上。原 `obj_detect.launch.py` 的 `find_camera()`
启发式仅在两相机场景下兼容，**未做 by-id 化**（按边界规则保留原文件）。

**当前观测点（v11 链路必读）**：检测相机默认 960×544，但 YOLOv11 9 类模型内部要求 640×640
NV12 输入；当下用 `cap_objdet=/dev/video0`（KS1A293）做单相机冒烟时已规避分段错误，全链路
端到端验收待回。

> **✅ 2026-09-23 已修复**：原 `usb_video_device` 参数未生效的根因是 Node() 的 parameters key
> 写成了 launch argument 名（带 `usb_` 前缀），但 `hobot_usb_cam` 节点接收的参数名是
> `video_device` / `image_width` / `image_height` / `framerate`（无前缀）。节点接收不到
> 参数 → fallback 到默认 `/dev/video8` → 打开失败 → fallback 到 `video0`（KS1A293）。
> 修复见 commit `5444dda`。修正后 `prescan_usb_cam` 正确打开 LRCP（`/dev/video2`），
> DNN 节点加载模型 `yolo11_detect_bayese_640x640_nv12` 成功，三节点（`prescan_usb_cam` /
> `hobot_codec` / `prescan_dnn_example`）全部正常启动。下一步可做端到端实机验证。

## 6. 避障增量（2027 新增，framework/）

今年仅新增避障能力（只增不改，原代码零改动）。**架构约束（2026-09-05 确认）**：
本仓库（RDK 上位机）只负责**视觉感知与串口通信**；云台转角、路径规划、行走等
**一切控制由下位机完成**，故 `camera_pan.py` / `path_planner.py` 不在本仓库实现。

预扫描流程：最开始时下位机发 `[num]`（启停位置，如 `[4]`=右上角 (0,4) 启停区）→ RDK
记录并回 `[ack]`；
随后下位机**分三次发 `[shot]`**（云台 0°/45°/90° 各转到位后触发一次，发一次拍一次；
**0° 基线方向由启停位置决定**（已确认 2026-09-08：从 4 出发 0° 沿右列向下
`4→9→14→19→24`；从 24 出发 0° 向左沿底行 `24→23→22→21→20`；0~90° 顺时针）→
视觉每次回 `[ack]`（障碍识别 `obstacle_detector` 待实现）→ 三次完成后判定
（误差圈就近命中 13 候选点 `road_judge` → `MapModel` 记录障碍）→ 下发地图帧
`[<count> <障碍ID>... <校验>]`（如 `[2 1 3 00]`，仅一次）→ 再进入原任务。
详见 `framework/DESIGN.md`（部分章节已过时，以本 README 为准）。

### 6.1 预扫描通信协议（2026-09-08 v3，[] 帧；ttyS1@115200 不变）

帧格式：ASCII 纯文本，**帧头 `[` + 载荷 + 帧尾 `]`**，无换行。

| 方向 | 帧 | 说明 |
|---|---|---|
| 单片机 → 视觉 | `[4]` | 启停位置（5×5 row-major 0~24；右上=4、右下=24 为启停区），时序最前发一次 |
| 视觉 → 单片机 | `[ack]` | 对启停帧的回执 |
| 单片机 → 视觉 | `[shot]` | 触发一次拍摄（一个角度）；三个角度发三次 |
| 视觉 → 单片机 | `[ack]` | 每次拍摄完成回传一次 |
| 视觉 → 单片机 | `[2 1 3 00]` | 三次拍摄完成后（仅一次）：`[count 障碍ID… 校验]`。count=0~3；ID=5×5 网格 row-major 十进制 0~24（如 (0,1)→1、(0,3)→3）；校验=count XOR 各 ID，两位大写 HEX |

实现：`framework/serial_protocol.py`（build_trigger / build_ack / build_obstacle_frame /
parse_trigger / parse_ack / parse_obstacle_frame / 启停帧 build_start_frame / parse_start_frame /
is_start_frame）+ `framework/mission_dispatcher.py`
（run_prescan：等 `[num]` 启停 → `[ack]` → 三次 `[shot]` → `scan_angle` 钩子 → 三次 `[ack]`
→ 地图帧 `[count ID… CHK]`；`START_DIR_MAP` 记录 0° 基线方向草案，**待人工确认固化**，
→ 地图帧 `[count ID… CHK]`；`START_DIR_MAP` 已确认（4→右列向下 / 24→底行向左），
串口读写已完成，
视觉环节保留 `scan_angle` 钩子）。无下位机联调：
`cd framework && python3 /tmp/test_comm.py`（os.openpty 虚拟串口对，`[4]`→ack→3×shot→3×ack→`[2 1 3 00]` 闭环）。

### 6.2 framework/ 新增节点状态（2026-09-23 更新）

| 文件 | 状态 | 说明 |
|---|---|---|
| `map_model.py` | ✅ 完成 | 5×5 通行矩阵 + 13 候选点 / 8 固定节点 |
| `road_judge.py` | ⚠️ 部分 | `nearest_candidate` 已实现；`judge()` 误差圈判定待实现 |
| `serial_protocol.py` | ✅ 完成 | 文本协议编码/解码（shot / ack / 25 位地图） |
| `mission_dispatcher.py` | ✅ 通信完成 | `run_prescan` 时序 + 串口读写；`scan_angle` 视觉钩子待接 |
| `obstacle_detector.py` | 🟡 模型已就位 | 加载 `framework/dnn/yolo11_x5_obstacle.bin`（1 类 `ball`，md5 6fd337ab…） + `task_obj_obstacle.json`（yolov8 parser, 640×640 NV12）；`detect()` / `pixel_to_map()` 占位（推理由 `dnn_node_example` 完成，话题 `/hobot_dnn_detection`）|
| `camera_pan.py` | ⏸ 不实现 | 云台控制归下位机 |
| `path_planner.py` | ⏸ 不实现 | 路径规划归下位机 |
| `DESIGN.md` | ⚠️ 部分过时 | 协议/状态以本 README 与 `serial_protocol.py` 为准 |

### 6.3 全局扫描阶段的 yolo 模型（2026-09-23 固化）

`framework/dnn/` 与 `obj_detect/dnn/` 是**两个独立链路、两个不同模型**——不要混用：

| 资产 | 路径 | 用途 |
|---|---|---|
| 物块/标识 9 类模型 | `dnn/yolo11_x5.bin` + `dnn/classes.names` + `dnn/task_obj_v11.json` | 原任务检测（圆台物块 6 色 + 放置区标识 3 种）|
| 障碍 1 类模型 | `framework/dnn/yolo11_x5_obstacle.bin` + `framework/dnn/classes_obstacle.names` + `framework/dnn/task_obj_obstacle.json` | 全局扫描开局 3 帧识别障碍（**仅 1 类 `ball`**，md5 6fd337ab…与物块模型不同）|

两个模型的 `.bin` 内部模型名都是 `yolo11_detect_bayese_640x640_nv12`，但量化参数不同（前者
9 类 65KB 更大），**不可互相替代**。

半成品：**13 候选点照片区间标定**——0°/45° 照片人工框选斜四边形 ROI（标注器 skill
`map-quad-annotator`，板端 :8888 页面）。当前 0° 已框 6 个（待补 label 与剩余），45° 未框。

## 8. Git 与回滚

- 项目为 git 仓库（main 分支，origin 已配置），历史提交见 `git log`
- 新增内容（framework/、README 等）独立提交，回滚方式：
  ```bash
  git log --oneline -10        # 查看提交
  git revert <commit>           # 反向回滚（推荐，保留历史）
  git reset --hard <commit>     # 硬回退（慎用，丢失之后改动）
  ```
- 当前未推送本地提交时：`git status -sb` 会显示 `[ahead N]`，推送用 `git push origin main`。
- **推送前置**：仓库 `origin` 当前指向 `http://127.0.0.1:3000/neolux/GongzongAppli.git`（本机端口转发），3000 端口未启动时 `git push` 会失败；请先确认正确的远程地址。