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
├── dnn/                         # 模型资产：task_obj.json + YOLOv5s .bin（250720_v5s_672）
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

CLASS 映射：`rcf=0x31 红圆环 · gcf=0x32 绿圆环 · bcf=0x33 蓝圆环 · rof=0x34 红目标 · gof=0x35 绿目标 · bof=0x36 蓝目标`

行为规则：默认 mode=2（二维码+检测都发）→ 扫到有效码切 mode=1；只发离画面中心最近的
目标，且满足区域过滤（x∈[140,500]、y<420，圆环不受限）；无二维码时每 50 帧发心跳
`0000000`。调试可 `ros2 topic echo /serial_send`。

## 5. 相机分配

- 单 USB 相机分时复用（同一总线带宽不足，见 launch 注释）：先扫码、扫码完成释放后再起检测相机
- 相机按 v4l2 帧率分配：帧率高的给二维码（黑白相机），检测用 960×544@120
- 注意：`find_camera()` 探测 `video0/video2`，当前板子只有 `video0/video1`（单相机），
  `video2` 不存在会导致 launch 启动失败（TypeError: float vs None）——复测前先核对相机实况

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

### 6.2 framework/ 新增节点状态（2026-09-05）

| 文件 | 状态 | 说明 |
|---|---|---|
| `map_model.py` | ✅ 完成 | 5×5 通行矩阵 + 13 候选点 / 8 固定节点 |
| `road_judge.py` | ⚠️ 部分 | `nearest_candidate` 已实现；`judge()` 误差圈判定待实现 |
| `serial_protocol.py` | ✅ 完成 | 文本协议编码/解码（shot / ack / 25 位地图） |
| `mission_dispatcher.py` | ✅ 通信完成 | `run_prescan` 时序 + 串口读写；`scan_angle` 视觉钩子待接 |
| `obstacle_detector.py` | ❌ 待实现 | YOLOv11 / CV 分割 + 单应性或区间判定（决定 `scan_angle` 能否返回 True） |
| `camera_pan.py` | ⏸ 不实现 | 云台控制归下位机 |
| `path_planner.py` | ⏸ 不实现 | 路径规划归下位机 |
| `DESIGN.md` | ⚠️ 部分过时 | 协议/状态以本 README 与 `serial_protocol.py` 为准 |

半成品：**13 候选点照片区间标定**——0°/45° 照片人工框选斜四边形 ROI（标注器 skill
`map-quad-annotator`，板端 :8888 页面）。当前 0° 已框 6 个（待补 label 与剩余），45° 未框。

## 7. Git 与回滚

- 项目为 git 仓库（main 分支，origin 已配置），历史提交见 `git log`
- 新增内容（framework/、README 等）独立提交，回滚方式：
  ```bash
  git log --oneline -10        # 查看提交
  git revert <commit>           # 反向回滚（推荐，保留历史）
  git reset --hard <commit>     # 硬回退（慎用，丢失之后改动）
  ```
- 当前未推送本地提交时：`git status -sb` 会显示 `[ahead N]`，推送用 `git push origin main`