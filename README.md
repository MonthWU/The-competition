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
├── framework/                   # 【新增】避障预扫描框架（见 DESIGN.md）
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

今年仅新增避障：开局斜视摄像头（车顶）0°/45°/90° 三帧预扫描 → 识别黑色障碍物
（YOLOv11 + CV 兜底）→ 误差圈判定障碍道路（17→13 候选点）→ BFS 路径规划 → 串口下发
路径指令 → 再执行原任务。详见 `framework/DESIGN.md`（只增不改，原代码零改动）。

## 7. Git 与回滚

- 项目为 git 仓库（main 分支，origin 已配置），历史提交见 `git log`
- 新增内容（framework/、README 等）独立提交，回滚方式：
  ```bash
  git log --oneline -10        # 查看提交
  git revert <commit>           # 反向回滚（推荐，保留历史）
  git reset --hard <commit>     # 硬回退（慎用，丢失之后改动）
  ```
- 当前未推送本地提交时：`git status -sb` 会显示 `[ahead N]`，推送用 `git push origin main`