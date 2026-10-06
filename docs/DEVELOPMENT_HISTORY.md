# 历史开发记录

以下内容归档自旧 README，包含已过时的架构、参数、相机状态与启动命令。当前使用方式以 [项目 README](../README.md) 和 [赛前运行说明](SCHOOL_RUNBOOK.md) 为准。

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
> **物块识别双链路（2026-09-29 落地）**：
> - **workaround（默认）**：`dnn/yolo11_x5.bin`（md5 `d2ec3e70…`，NCHW）+ `obj_dnn.py` 自写解码绕开 TROS parser_yolov8 的 NHWC 假设（首帧 SIGSEGV）。
> - **native**：`dnn/yolo11_x5_nhwc.bin`（md5 `d31741bb…`，NHWC）+ TROS `dnn_node_example` 直接吃 `dnn/task_obj_v11_nhwc.json`。
> - 切换：`export DNN_ENGINE=native && ros2 launch launch/run_all.launch.py`，或一次性 `ros2 launch ... dnn_engine:=native`。
> - 两链路**对外接口完全一致**（同一 topic `/hobot_dnn_detection`，同一 launch 参数集），下游 `obj_serial` / websocket / 录像无需任何改动；删其中一条不影响其他文件。


## 3. 目录结构

```
appli/
├── start_all.sh                 # 完整流程：障碍预扫描 → 二维码 → 物块
├── start_simple.sh              # 简化流程：二维码 → 物块
├── scripts/start_common.sh      # 共用环境、参数和设备检查
├── docs/                        # 赛前运行说明、待办、模型导出问题
├── launch/                      # 二维码/物块阶段的 ROS launch
├── obj_detect/                  # 目标检测包（obj_camd / obj_serial / obj_video_dumper）
├── qrc_skandier/                # 二维码包（qrc_cam / qrc_scanner / qrc_cam_killer / flaskr）
├── qrc_hobot_usb_cam/           # USB 相机 ROS2 包装（C++）
├── dnn/                         # 模型资产：原任务 yolov5s_v5s_672 + yolov11 9 类物块/标识（task_obj_v11.json）
│                                # 物块双链路：yolo11_x5.bin（NCHW/workaround）+ yolo11_x5_nhwc.bin（NHWC/native）+ task_obj_v11{,_nhwc}.json
├── framework/                   # 障碍预扫描、通信、校赛配置；dnn/ 存障碍模型与 ROI
├── service/                     # systemd 完整启动与兼容包装
├── gpio_shutdown/               # GPIO 关机键服务
├── tools/                       # 构建、板端验证；diagnostics/ 调试，legacy/ 历史工具
├── webapp/                      # OpenCV 参数调节网页
├── _tmp_scan_imgs/              # 运行时预扫描照片（不入版本管理）
└── _tmp_videos/                 # 运行时检测录像（不入版本管理）
```

## 4. 串口协议（上位机 → 下位机，ttyS1 @ 115200）

帧格式：`0xFF(头) + CLASS(1B) + [数据] + 0xFE(尾)`

| 帧 | 字节流 | 说明 |
|---|---|---|
| 二维码 | `FF 37 <UTF-8内容> FE` | 有效码连发 4 次 |
| 检测目标 | `FF CLASS XL XH YL YH FE` | 坐标低 8 位在前；图像系 **640×480**（检测链路实际分辨率，2026-09-25 修正） |

CLASS 映射（v11 9 类新模型，`obj_serial.py` 已实现）：
`red1=0x41 · black1=0x42 · green1=0x43 · yellow1=0x44 · blue1=0x45 · blue2=0x46`（圆台物块，每帧只发离参考点最近 1 个）·
`targetOne=0x51 · targetTwo=0x52 · targetThree=0x53`（放置区标识，全部发送）。
旧模型（250720_v5s）兼容保留：`rcf=0x31 红圆环 · gcf=0x32 绿圆环 · bcf=0x33 蓝圆环 · rof=0x34 红目标 · gof=0x35 绿目标 · bof=0x36 蓝目标`

行为规则：默认 mode=2（二维码+检测都发）→ 扫到有效码切 mode=1；只发**离参考点最近**
的物块（1 个/帧）；区域过滤可在现场标定后启用。**放置区标识全部发送**；无二维码时每 50 帧
发心跳 `0000000`。调试可 `ros2 topic echo /serial_send`。

> **2026-09-25 修正**：区域过滤此前**只存在于文档、代码并未实现**（`obj_serial.py` 中
> `self.xin=180 / self.yin=420` 定义了却从未被引用）。现已补实现并**参数化**：
> `enable_roi_filter` / `roi_x_min` / `roi_x_max` / `roi_y_max`，参考点
> `ref_pt_x` / `ref_pt_y`（默认画面中心 320,240）。
> 历史阈值 `x∈[380,640]、y≤480` 会漏掉另一组实拍画面中的真实物块，校赛默认关闭过滤；现场标定后可重新启用。
> **现场需复核标定** —— 旧文档的 960×544 规格（x∈[140,500]）与当前 640×480 相机
> 视野完全错开，直接套用会把目标全部滤除（已实测）。

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
> ** 2026-09-23 二次修正**：通过实机画面确认，检测 ↔ 全局扫描 之前物理接线与代码
> 假设反了——LRCP AR0234 实际装在检测位（拍到物块区），DCXIN Camera 实际装在车顶
> 全局扫描位。已对调 `obj_detect_v11.launch.py` / `prescan.launch.py` /
> `framework/map_scanner.py` 三处 by-id。

> ** DCXIN 亮度问题（2026-09-25 重新定位，修正此前记录）**
>
> 该机固件 `auto_exposure` **只接受 1(Manual) / 3(Aperture Priority)**，**没有真正的
> Auto(0)**（实测设 0 报 `Invalid argument`）。而出厂默认的 3（光圈优先）在 UVC 摄像头上
> 是**空转**的 —— 没有可变光圈可调、`exposure_time_absolute` 被标记 `flags=inactive`（只读），
> 所以这个"自动曝光"从未真正工作，画面亮度实际只由 `brightness` / `gain` 决定。
> 出厂值 `brightness=50 / gain=0` 明显偏暗（实测画面均值 **68.6**，中央区仅 28.8）。
>
> **关键：必须通过 `hobot_usb_cam` 节点参数设置**，而非 launch 之前的 v4l2-ctl 预设 ——
> 节点启动时会写入自己的 `brightness` 默认值 (50)，会覆盖预设（已实测被覆盖）。
> 正确做法见 `framework/launch/prescan.launch.py`：
> ```python
> {"brightness": 128}, {"gain": 48},
> ```
> 修正后画面均值 **138.9**（提升约 2 倍）。非 ROS 场景可手动执行
> `bash framework/setup_dcxin.sh`（写入同样的值）。
>
> 另外 dmesg 报 `Failed to query UVC control 5/7/17`（-32 EPIPE）是 vendor 固件的控制
> 查询失败、被 uvcvideo 安全忽略，属正常现象，与亮度问题无关。

v11 链路 `obj_detect_v11.launch.py` 的 `cap_qrc_devnode` / `cap_objdet_devnode` 默认按上表
写死；`framework/map_scanner.py` 默认设备同上。原 `obj_detect.launch.py` 的 `find_camera()`
启发式仅在两相机场景下兼容，**未做 by-id 化**（按边界规则保留原文件）。

**当前观测点（v11 链路必读）**：检测相机默认 960×544，但 YOLOv11 9 类模型内部要求 640×640
NV12 输入；当下用 `cap_objdet=/dev/video0`（KS1A293）做单相机冒烟时已规避分段错误，全链路
端到端验收待回。

> ** 2026-09-23 已修复**：原 `usb_video_device` 参数未生效的根因是 Node() 的 parameters key
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
视觉每次回 `[ack]`（障碍识别 `obstacle_detector` + `road_judge` 已实现）→ 三次完成后判定
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

### 6.2 framework/ 新增节点状态（2026-09-25 更新）

| 文件 | 状态 | 说明 |
|---|---|---|
| `map_model.py` |  完成 | 5×5 通行矩阵 + 13 候选点 / 8 固定节点 |
| `road_judge.py` |  可用 | `nearest_candidate` / `judge()`（误差圈）+ `judge_from_hits()`（置信度过滤）均已实现。2026-09-25 修：`__init__` 原先直接 `raise NotImplementedError` 导致类**无法实例化**；并把混用的 `hit_threshold` 拆为 `hit_threshold`（网格距离，1.5）与 `conf_threshold`（置信度，0.5） |
| `serial_protocol.py` |  完成 | 文本协议编码/解码（shot / ack / 25 位地图） |
| `mission_dispatcher.py` |  可用 | `run_prescan` 时序 + 串口读写 + `scan_angle` 视觉钩子已接通。2026-09-25 修：`__init__` 补齐 `obstacle_detector` / `road_judge` / `_launch_proc` / `_rclpy_init`（此前一进 `scan_angle` 就 AttributeError）；`_grid_id` → `grid_id` |
| `obstacle_detector.py` |  可用 | 模型已换为 `zaw.bin`（1 类 **`block`**、md5 `90f15717…`、box 输出 NHWC）；`detect_from_perception()` 解析 `/hobot_dnn_detection` 的 `PerceptionTargets`。2026-09-25 修：错误的 `from map_model import grid_id`（实际定义在 `serial_protocol.py`）导致本模块 **import 即 ImportError**；`pixel_to_grid()` 仍需现场单应性标定 |
| `camera_pan.py` | ⏸ 不实现 | 云台控制归下位机 |
| `path_planner.py` | ⏸ 不实现 | 路径规划归下位机 |
| `DESIGN.md` |  部分过时 | 协议/状态以本 README 与 `serial_protocol.py` 为准 |

### 6.3 全局扫描阶段的 yolo 模型（2026-09-23 固化）

`framework/dnn/` 与 `obj_detect/dnn/` 是**两个独立链路、两个不同模型**——不要混用：

| 资产 | 路径 | 用途 |
|---|---|---|
| 物块/标识 9 类模型（NCHW · 现役） | `dnn/yolo11_x5.bin`（md5 `d2ec3e70…`，NCHW，3.19 MB）+ `dnn/classes.names` + `dnn/task_obj_v11.json` | 原任务检测（圆台物块 6 色 + 放置区标识 3 种），需 `obj_dnn.py` 绕行层（`dnn_engine:=workaround`，默认） |
| 物块/标识 9 类模型（NHWC · 新版） | `dnn/yolo11_x5_nhwc.bin`（md5 `d31741bb…`，NHWC，10.34 MB）+ `dnn/task_obj_v11_nhwc.json` | 同 9 类，但导出时已 layout 转置，TROS `dnn_node_example` 直吃（`dnn_engine:=native`，无需 `obj_dnn.py`） |
| 障碍 1 类模型 | `framework/dnn/yolo11_x5_obstacle.bin`（md5 `4b36aed6…`，NCHW cls + NHWC box） + `framework/dnn/classes_obstacle.names` + `framework/dnn/task_obj_obstacle.json` | 全局扫描开局 3 帧识别障碍（**仅 1 类 `block`**，需 `obs_dnn.py` 绕 `dnn_node_example` 段错误） |

两个模型的 `.bin` 内部模型名都是 `yolo11_detect_bayese_640x640_nv12`，但量化参数不同（前者
9 类 65KB 更大），**不可互相替代**。
**三个 `.bin` 的 `properties.layout` 都不一定与实际内存一致**——`obs_dnn.py` 与 `obj_dnn.py` 已用「**实测 shape** 而不是 layout 字段」做解码适配；TROS `dnn_node_example` 只信任 layout 字段，所以 native 链仅在「layout 标签对」时能跑通，否则需要绕行层。

物块任务推荐先试 `dnn/yolo11_x5_nhwc.bin`（native 链）——TROS 自带 `dnn_node_example` 能直吃，无须额外节点；现场先跑通 native，确认精度后再决定是否下线 workaround。


当前校赛 ROI 已根据六张照片的重标数据更新；实际覆盖与复核要求见 [SCHOOL_RUNBOOK.md](SCHOOL_RUNBOOK.md#障碍-roi-覆盖与复核)。本节其他早期标注进度不代表运行文件。


## 7. 物块双链路切换 & 单删指南（2026-09-29）

`obj_detect_v11.launch.py` 内置两套可替换的推理实现，通过 `dnn_engine` 启动参数切换：

| `dnn_engine` | 模型 | 推理节点 | 默认状态 |
|---|---|---|---|
| `workaround` | `dnn/yolo11_x5.bin`（NCHW，3.19 MB） | `obj_dnn`（自写 pyeasy_dnn + DFL 解码，绕开 TROS 段错误） | **默认** |
| `native`     | `dnn/yolo11_x5_nhwc.bin`（NHWC，10.34 MB） | `dnn_node_example`（TROS 自带，`task_obj_v11_nhwc.json` 直吃） | 备选（精度待现场回验）|

两条链路**对外接口完全一致**：
- 同一话题 `/hobot_dnn_detection`（`ai_msgs/msg/PerceptionTargets`）
- 同一参数集（`cap_objdet` / `dnn_example_image_{width,height}` / `msg_pub_topic_name` / `score_threshold` / `nms_threshold`）
- 同一图像输入（`/image`），同一图像尺寸（默认 640×480）

因此下游 `obj_serial` / websocket / 录像 **完全不需要改动**；未来要删其中一条，**只动对应的那侧文件**，对另一个文件零影响。

### 7.1 切换用法

```bash
# 默认（workaround）
ros2 launch /root/dev_ws/appli/launch/run_all.launch.py

# 切到 native
ros2 launch /root/dev_ws/appli/launch/run_all.launch.py dnn_engine:=native

# 或 export 环境变量（run_all 读取）
export DNN_ENGINE=native && ros2 launch /root/dev_ws/appli/launch/run_all.launch.py

# 自定义 task JSON / 模型
ros2 launch ... dnn_engine:=native dnn_task_json:=/path/to/your.json
ros2 launch ... dnn_engine:=workaround model_file:=/path/to/your.bin
```

### 7.2 删除其中一条的步骤（互不影响）

**删 workaround（NCHW + obj_dnn）**：
1. 删除 `obj_detect/obj_detect/obj_dnn.py`；
2. 删除 `obj_detect/setup.py` 中 `obj_dnn = obj_detect.obj_dnn:main` 的 `entry_points` 条目；
3. 在 `obj_detect/launch/obj_detect_v11.launch.py` 中删除 `obj_dnn_node`（包括它的 `workaround_*` 启动参数，模拟条件 `dnn_engine=='workaround'`）；
4. 把 `launch/run_all.launch.py` 的 `os.environ.get("DNN_ENGINE", "workaround")` 默认改为 `"native"`；
5. native 链对外接口未变，下游文件零影响。

**删 native（NHWC + dnn_node_example）**：
1. 删除 `obj_detect/launch/obj_detect_v11.launch.py` 中的 `objdet_nv12_codec_node` / `shared_mem_node` / `dnn_node_example_node`（含 `native_*` 启动参数与 `dnn_engine=='native'` 条件）；
2. 删除 `dnn/task_obj_v11_nhwc.json` 与 `dnn/yolo11_x5_nhwc.bin`；
3. workaround 链对外接口未变，下游文件零影响。

### 7.3 验收要点（现场回验 native 链）

1. 启动后 `ros2 topic list | grep hobot_dnn_detection` 看到话题；
3. `ros2 topic echo /hobot_dnn_detection --once` 能收到 PerceptionTargets（含 `rois`、`type`）；
3. 物块特写图（LRCP AR0234 拍）下置信度与 workaround 链相当（参考 >0.5）；
4. `obj_serial` 串口下行正常（`ros2 topic echo /serial_send`）。

如 native 链实景下召回/精度比 workaround 差，回退即可：
```bash
ros2 launch /root/dev_ws/appli/launch/run_all.launch.py   # 默认 workaround
```

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
