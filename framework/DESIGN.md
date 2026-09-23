# framework/ —— 避障预扫描方案设计（2027 智能搬运增量）

> **状态说明**：本文档已与实际代码对齐（2026-09-23）；README §5/§6 为更高优先级速查表。
> 后续变更请先改 README，再同步本文档。

## 0. 架构约束（硬性）

- **RDK（视觉感知层）职责**：扫码、原任务检测、全局扫描 3 帧拍照 + 障碍识别 + 串口通信
- **下位机（控制层）职责**：云台转角（0°/45°/90°）、行走控制、路径规划、地图解读
- 因此 `framework/camera_pan.py` / `framework/path_planner.py` **不实现**——保留为骨架占位，避免误用
- **串口分时**：预扫描阶段 `mission_dispatcher` 独占 `ttyS1`；原任务阶段 `obj_serial` 独占

## 1. 背景与规则（摘自比赛命题）

- 场地：2400×2400mm，灰色行车道（十字车道宽 400mm），4 个 450×450 淡黄区，2 个启停区（300×300）
- 障碍物：**黑色几何体 φ50×100mm，数量随机抽签**（比赛可能更换形状，按"黑色几何体"通用识别设计）
- 二维码：A4 板 1 个，每赛场位置随机，4 组三位数（与原任务一致，不变）

## 2. 三层结构与文件清单（2026-09-23 同步）

### 通信编排层 ✅

| 文件 | 状态 | 说明 |
|---|---|---|
| `serial_protocol.py` | ✅ | [] ASCII 帧编解码（启停帧/shot/ack/地图帧）+ 校验 |
| `mission_dispatcher.py` | ✅ | 状态机 run_prescan + START_DIR_MAP（启停位→0° 基线方向） |

### 数据模型层 🟡

| 文件 | 状态 | 说明 |
|---|---|---|
| `map_model.py` | ✅ | 5×5 通行矩阵（8 节点 + 13 候选点 + 4 空白）+ MapModel 类 |
| `road_judge.py` | 🟡 | 接收标注结果；自动判定骨架（nearest_candidate）保留供参考 |

### 感知执行层 🟡

| 文件 | 状态 | 说明 |
|---|---|---|
| `framework/dnn/` | ✅ | yolo11_x5_obstacle.bin（1 类 ball, md5 6fd337ab…）+ task_obj_obstacle.json |
| `obstacle_detector.py` | 🟡 | 模型路径已就位；detect/pixel_to_map 占位（推理交 dnn_node_example） |
| `map_scanner.py` | ❌ | LRCP AR0234 by-id 已固化默认设备；capture/release 待实现 |
| `camera_pan.py` | ⏸ | 不实现 |
| `path_planner.py` | ⏸ | 不实现 |
| `DESIGN.md` | ⚠️ | 本文档，与 README 保持同步 |

## 3. 串口协议（v3，2026-09-08 定稿）

### 3.1 帧格式

- 链路：`/dev/ttyS1`，115200，8N1，无硬件流控
- 字符集：ASCII，帧头 `[` + 载荷 + 帧尾 `]`，无换行；帧内字段分隔 ` `（见 `serial_protocol.FIELD_SEP`）

### 3.2 时序

```
单片机: [num]  (启停位 0~24, 右上=4, 右下=24 启停区, 仅这两角有效)
RDK   : 记录 start_id; [ack]
单片机: [shot] (云台转到 0° 基线后触发)
RDK   : 拍 0° 帧 + 识别 → [ack]
单片机: [shot] (云台 45°)
RDK   : 拍 45° 帧 + 识别 → [ack]
单片机: [shot] (云台 90°)
RDK   : 拍 90° 帧 + 识别 → [ack]
RDK   : 三次拍完 → 地图帧 [count ID… CHK]  (仅发一次)
```

### 3.3 地图帧编码

- 格式：`[count 障碍ID… 校验]`
- count：0~3
- 障碍ID：`id = row*5 + col`（5×5 row-major, 0~24）
- 校验：`count XOR 所有 ID`，两位大写 HEX
- 示例：障碍位于 (0,1) 与 (0,3) → ids=[1,3], count=2, 校验=2^1^3=0x00 → `[2 1 3 00]`；无障碍 → `[0 00]`

## 4. 启停位 → 0° 基线方向（2026-09-08 固化）

| start_id | 含义 | 0° 基线方向（5 格 ID 序列） |
|---|---|---|
| 4 | 右上角 (0,4) | 4 → 9 → 14 → 19 → 24（右列向下） |
| 24 | 右下角 (4,4) | 24 → 23 → 22 → 21 → 20（底行向左） |

- 云台 0°→90° 为**顺时针旋转**
- 写入 `MissionDispatcher.START_DIR_MAP`（已固化）

## 5. 相机映射（2026-09-23 固化，by-id）

| 任务 | 相机（USB Port）| by-id 出图路径 |
|---|---|---|
| 扫码 qrc_skandier | KS1A293（Port 2）| `usb-KINGSEN_KS1A293-video-index0` |
| 检测 obj_detect | DCXIN（Port 4）| `usb-DCXIN_DCXIN_Camera_01.00.000-video-index0` |
| 全局扫描 framework/map_scanner | LRCP AR0234（Port 3）| `usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0` |

- 三路同挂 USB Bus01（480M）同一 Hub，**不并发常开、每阶段用完即 kill**
- `framework/map_scanner.py` 默认视频参数写入 `_DEFAULT_VIDEO_DEVICE` 常量
- `/dev/video*` 编号会随插拔漂移，**永远用 by-id 路径**

## 6. 端到端时序与代码位置

| 步 | 触发者 | RDK 动作 | 下位机动作 | 代码位置 |
|---|---|---|---|---|
| 0 | 上电 | 等待 | 云台转到 0° 基线，发 `[num]` | `mission_dispatcher.wait_start` |
| 1 | RDK | 查 START_DIR_MAP，发 `[ack]` | 等 ack | `send_ack` |
| 2 | 下位机 | 等 `[shot]`，拍 0°，回 `[ack]` | 云台转 0°，发 `[shot]` | `wait_trigger` + `scan_angle(0)` |
| 3 | 同上 | 拍 45° | 云台转 45°，发 `[shot]` | `scan_angle(45)` |
| 4 | 同上 | 拍 90° | 云台转 90°，发 `[shot]` | `scan_angle(90)` |
| 5 | RDK | road_judge 判定 → MapModel 标障碍 → 发地图帧 `[count ID… CHK]` | 接收地图重建通行 | `send_map` |
| 6 | RDK | 关扫描相机 → 拉起原任务 run_all.launch.py | 按地图走 | （未实现） |

## 7. scan_angle 实现要点（2026-09-23 待开发）

1. **子 launch** `framework/launch/prescan.launch.py`：起 LRCP AR0234（by-id, 640×640 MJPG）+
   hobot_shm + hobot_codec_decode + dnn_node_example（task=task_obj_obstacle.json）+ prescan_dnn_node
2. **prescan_dnn_node.py**：订阅 `/hobot_dnn_detection`（PerceptionTargets），过滤 type=='ball' 的 bbox
3. **scan_angle(angle)**：subprocess 拉起 prescan.launch.py → spin dnn_node → 拿到一帧 ball 检测结果 → 单应性像素→网格坐标 → 调用 road_judge.judge() → MapModel.set_obstacles() → 返回 True
4. **单应性矩阵** `obstacle_detector.calibrate()` 需现场拍照标定；占位版可用 13 候选点先验坐标做线性估计

## 8. road_judge 对齐模型（2026-09-23 用户定义）

- 障碍判定**不再由 RDK 几何就近算**——而由用户在标注器 skill（`map-quad-annotator`，板端 :8888 页面）实际标注
- 标注结果作为输入喂给 `RoadJudge.judge()`；road_judge 负责：
  - 阈值过滤（`hit_threshold`，默认 0.5）
  - 候选点有效性校验（仅 13 x 位置可命中）
  - 置信度合并（同一点多帧检测去重）
  - 返回最终障碍候选点集合 → MapModel.set_obstacles()
- `nearest_candidate()` 保留供参考/降级模式

## 9. 待确认 / 待实现

### 待实现（依赖硬件或下位机）

- [ ] map_scanner.py：打开 LRCP ARPC 摄像头（by-id）+ capture/release
- [ ] obstacle_detector.detect() / pixel_to_map() / calibrate()：单应性标定 + 像素→网格映射
- [ ] road_judge.judge()：接收模型标注结果，阈值 + 合并策略
- [ ] mission_dispatcher.run_prescan 后衔接 run_all.launch.py（拉起原任务）

### 待确认

- [ ] 下位机路径指令协议（DESIGN §5 中提及的样例协议）
- [ ] 单应性标定流程（现场拍照 + 角点选取）
- [ ] 13 候选点照片区间标定收尾（0° 已框 6 个缺 label, 45° 未框）

### 已确认

- [x] START_DIR_MAP（4→右列向下 / 24→底行向左）
- [x] 全局扫描相机 = LRCP AR0234 by-id
- [x] 检测相机 = DCXIN by-id
- [x] 扫码相机 = KS1A293 by-id
- [x] 串口协议 v3（[] 帧）