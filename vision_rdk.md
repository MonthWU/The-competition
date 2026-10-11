# vision_rdk.md

# RDK_CJLU_26 项目视觉技术报告

## 1. 项目概述
本项目运行于 **RDK X5** 平台，采用 **ROS2 + YOLO + OpenCV + UART** 的组合方式，完成目标粗定位、红圆精检测、距离估计和横向偏移测量，并将结果通过串口协议发送给下位机，同时通过网页提供可视化预览。

当前最终交付版本目录为：

```text
/root/Monthwu/RDK_vision/RDK_CJLU_26_VDone
```

该版本已经具备：
- systemd 开机自启
- USB 摄像头自启动打开
- ROS2 节点稳定运行
- UART 14B / 50Hz 新协议输出
- 网页实时画面展示
- 距离与横向偏移输出

---

## 2. 项目目标
本项目的视觉目标是：

1. 使用摄像头实时采集图像；
2. 使用 YOLO 对目标 `guide_marker` 进行粗定位，输出 ROI；
3. 在 ROI 内使用 OpenCV 检测红圆；
4. 基于红圆几何特征估计：
   - `distance`
   - `vision_x`
5. 通过 ROS2 话题与 UART 协议对外发布结果；
6. 通过网页实时显示检测画面。

项目强调以下工程约束：
- OpenCV 的核心任务是**红圆检测**；
- YOLO 仅负责**粗定位 ROI**，不直接输出最终距离；
- 当前重点优先保证 **200~300 cm** 区间的距离精度；
- 近距离允许误差相对更大；
- 串口协议必须稳定，不随调试随意变化。

---

## 3. 硬件与系统环境

### 3.1 平台
- 板卡：**RDK X5**
- 系统：Ubuntu 22.04 + TROS Humble
- ROS 发行版：**Humble**

### 3.2 摄像头
- 类型：USB UVC 摄像头
- 当前枚举设备：
  - `/dev/video0`
  - `/dev/video1`
- 当前项目使用：
  - 优先尝试 `/dev/video0`
  - 候选还包含 `/dev/video1`、`/dev/video2` 以及数字索引兜底

### 3.3 串口
- UART 设备：`/dev/ttyS1`
- 对应 40Pin：
  - TX：Pin 8
  - RX：Pin 10
- 波特率：`115200`
- 格式：`8N1`

---

## 4. 软件架构
项目采用 ROS2 节点式结构，主要包含两个核心节点：

### 4.1 `/vision_node`
职责：
- 打开摄像头；
- 获取图像帧；
- 调用 YOLO 粗定位；
- 在 ROI 内进行 OpenCV 红圆检测；
- 估计 `distance` 和 `vision_x`；
- 发布 `/vision/target`；
- 推送网页视频流。

### 4.2 `/uart_sender`
职责：
- 订阅 `/vision/target`；
- 将 `distance` 和 `vision_x` 打包为固定 UART 二进制协议；
- 按固定 50Hz 周期发送；
- 发布 `/uart/tx` 调试信息。

---

## 5. 视觉处理链路

## 5.1 图像输入
当前图像参数：

```text
分辨率：640 x 480
目标帧率：30 FPS
像素格式：MJPG
```

摄像头打开策略：
- 只尝试显式候选，不自动盲扫 `/dev/video*`；
- 在 systemd 自启动场景下优先按真实设备路径（如 `/dev/video0`）打开，避免强制转换成 index 0；
- 启动脚本会等待 `/dev/video*` 节点出现，并额外等待数秒，提升开机后相机 ready 概率。

---

## 5.2 YOLO 粗定位
YOLO 在本项目中的职责不是直接输出最终测距，而是：

> **先找到目标区域 guide_marker，给出矩形 ROI，供 OpenCV 在 ROI 内进一步做红圆精检测。**

### 当前类别
```text
guide_marker
```

### 当前行为
- 首帧必跑；
- ROI 丢失时重新跑；
- 按 `detect_period_sec` 周期刷新；
- 输出 `bbox = [x1, y1, x2, y2]`。

### 当前作用
- 缩小 OpenCV 搜索范围；
- 减少背景误检；
- 提高整体效率。

---

## 5.3 OpenCV 红圆检测
在 YOLO 给出的 ROI 内执行红圆检测。

### 主要步骤
1. 将 ROI 图像转换到 HSV；
2. 用双红色区间做颜色提取；
3. 做开闭运算去噪；
4. 查找轮廓；
5. 对候选轮廓做：
   - `minEnclosingCircle`
   - `fitEllipse`（优先）
6. 选出最可信的候选红圆。

### 输出几何量
- 红圆中心：`(cx, cy)`
- 包围圆半径：`radius`
- 椭圆长轴：`ellipse_major`
- 椭圆短轴：`ellipse_minor`
- 有效直径像素：`effective_diameter_px`

### 当前使用的有效直径
当前配置：

```python
use_minor_axis_as_diameter = True
```

即：

```text
effective_diameter_px = ellipse_minor
```

这样做的原因是：
- 当前场景存在透视；
- 短轴通常比直接用外接圆半径更稳；
- 对目标工作区（200~300cm）表现更好。

---

## 5.4 横向偏移计算
横向偏移定义为：

```text
vision_x = target_center_x - image_center_x
```

当前 640×480 下：
- 图像中心线：`x = 320`
- `vision_x < 0`：目标在左侧
- `vision_x > 0`：目标在右侧
- 合理范围：

```text
-320 ~ 320 px
```

---

## 6. 距离估计模型
当前项目的 `distance` 采用**参考比例模型**，不是完全依赖标准针孔标定。

### 6.1 当前模型
```text
distance_raw = reference_distance_cm * reference_diameter_px / effective_diameter_px
```

随后叠加固定偏置修正：

```text
distance = distance_raw + distance_bias_cm
```

### 6.2 当前关键参数
当前版本配置如下：

```text
real_diameter_cm      = 20.0
reference_distance_cm = 175.5
reference_diameter_px = 52.1
focal_length_mm       = 3.24
pixel_size_um         = 3.0
distance_bias_cm      = 20.0
```

说明：
- `real_diameter_cm`：红圆实际物理直径；
- `reference_distance_cm`：参考标定距离；
- `reference_diameter_px`：该参考距离下观测到的有效像素直径；
- `distance_bias_cm`：对整体系数误差做固定偏置修正；
- `focal_length_mm` 与 `pixel_size_um` 已保留，可用于后续切换到针孔模型。

### 6.3 标定策略
项目当前明确采用：

> **优先保证 200~300cm 区间准确性，近距离允许误差更大。**

因此当前参数不是全距离段折中最优，而是对主工作区做偏置调优。

---

## 7. ROS2 通信设计

### 7.1 关键节点
```text
/vision_node
/uart_sender
```

### 7.2 关键话题
#### `/vision/target`
文本格式：
```text
d,a_x
```

示例：
```text
223.6696,-79.4091
```

其中：
- `d`：distance，单位 cm
- `a_x`：vision_x，单位 pixel

#### `/uart/tx`
用于调试 UART 发送：
```text
distance=224.0088,vision_x=-79.6369
```

#### `/vision/error`
保留兼容旧链路或扩展用途。

---

## 8. UART 协议（最终版）
当前 UART 协议已经统一为新版固定格式。

### 8.1 发送频率
```text
50Hz
```

### 8.2 数据帧格式
```text
[0]      1B   FRAME_HEAD   = 0x91
[1..4]   4B   distance     int32_le
[5..8]   4B   vision_x     int32_le
[9..12]  4B   timestamp_ms uint32_le
[13]     1B   FRAME_TAIL   = 0x93
```

总长度：
```text
14B
```

### 8.3 数值缩放
发送前：
```text
distance_i32 = round(distance * 100)
vision_x_i32 = round(vision_x * 100)
```

下位机还原：
```text
distance = distance_i32 / 100.0
vision_x = vision_x_i32 / 100.0
```

### 8.4 当前串口参数
```text
设备：/dev/ttyS1
波特率：115200
格式：8N1
```

---

## 9. 网页可视化
当前项目内置网页预览：

```text
http://192.168.1.125:8000/
```

当前网页显示内容包括：
- 摄像头实时画面
- 中心线
- YOLO ROI 框
- 红圆轮廓
- FPS
- distance
- vision_x
- diam_px

网页主要用于：
- 现场调试；
- 验证 YOLO ROI 是否正确；
- 验证红圆检测是否稳定；
- 快速观察 distance/vision_x 输出是否合理。

---

## 10. 自启动与运行稳定性
项目当前已经做过较多工程化处理，以保证冷启动后能自动运行。

### 10.1 systemd 自启动
服务名称：
```text
rdk_cjlu_26_vdone.service
```

当前状态：
- 开机自启已启用；
- systemd 服务可自动拉起项目。

### 10.2 摄像头自启动修复
已做的修复包括：
- 启动脚本等待 `/dev/video*` 出现；
- 额外等待设备 ready；
- Camera 打开方式优先按真实设备路径打开，而不是强制用 index。

### 10.3 ROS2 崩溃保护
为了避免 service 停止 / 重启时出现上下文已关闭却继续 publish 的崩溃，已补充：
- `rclpy.ok()` 检查；
- publish 异常保护；
- 重复 shutdown 保护；
- `uart_node` 和 `vision_node` 的收尾保护。

---

## 11. 当前版本的重要工程约束
阅读或修改本项目时，需要牢记以下约束：

1. **YOLO 只负责 ROI，不直接给最终 distance**
2. **OpenCV 核心任务是红圆**
3. **UART 协议必须保持 14B / 50Hz**
4. **distance 当前优先照顾 200~300cm 精度**
5. **近距离允许误差更大**
6. **重启程序时必须先停旧进程，再启动新进程**
7. **当前运行版目录为 `RDK_CJLU_26_VDone`**

---

## 12. 当前运行检查方式

### 查看服务状态
```bash
systemctl status rdk_cjlu_26_vdone.service
```

### 查看节点
```bash
source /opt/tros/humble/setup.bash
ros2 node list
```

### 查看话题
```bash
ros2 topic list
```

### 查看视觉结果
```bash
ros2 topic echo /vision/target --once
```

### 查看串口调试输出
```bash
ros2 topic echo /uart/tx --once
```

### 查看服务日志
```bash
journalctl -u rdk_cjlu_26_vdone.service -n 100 --no-pager
```

---

## 13. 当前已知现象与风险
### 13.1 已确认可用
- 自启动可拉起；
- 摄像头可打开；
- UART 新协议正常发送；
- ROS2 节点正常存在；
- 网页可预览；
- 红圆检测链路可运行。

### 13.2 当前仍需关注
- 距离模型在非主工作区（尤其近距离）可能漂移；
- 若现场目标姿态变化很大，`ellipse_minor` 可能不再是最优尺度；
- 若后续更换镜头、分辨率或红圆尺寸，必须重新标定。

---

## 14. 一句话项目总结
本项目是一条运行在 **RDK X5 + ROS2** 上的专用视觉链路：

> **YOLO 负责粗定位 `guide_marker`，OpenCV 在 ROI 内检测红圆，输出 `distance` 与 `vision_x`，并通过固定 14B / 50Hz UART 协议发送给下位机，同时通过网页提供实时可视化。**
