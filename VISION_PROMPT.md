# VISION_PROMPT.md

## 角色定位
你正在处理一个运行于 **RDK X5** 的 ROS2 视觉项目，版本目录为：

```text
/root/Monthwu/RDK_vision/RDK_CJLU_26_VDone
```

你的任务不是泛化地讨论计算机视觉，而是理解并维护一条**专用视觉链路**：

- 用 **YOLO** 做目标粗定位（ROI）
- 用 **OpenCV** 在 ROI 内检测红圆
- 输出：
  - `distance`
  - `vision_x`
- 通过 ROS2 与 UART 对外发布
- 支持网页实时预览

---

## 项目目标
该项目的核心目标是：

> 在实时摄像头画面中，先用 YOLO 粗定位目标区域，再在该区域内检测红圆，并稳定输出红圆相关的距离和横向偏移。

注意：
- **OpenCV 的主要目标是红圆，不是白框，不是通用几何识别**
- 当前版本已经尽量裁掉与红圆无关的调试逻辑
- 优先保证 **200~300cm 区间** 的测距精度
- 近距离允许误差更大

---

## 视觉链路概览

### 1. 输入
- 摄像头：USB UVC 摄像头
- 当前分辨率：
  ```text
  640 x 480
  ```
- 当前帧率目标：
  ```text
  30 FPS
  ```

### 2. 粗定位（YOLO）
YOLO 只负责找到 `guide_marker`，输出一个矩形框 `bbox` 作为 ROI。

当前特点：
- 不是每帧都跑
- 首帧必跑
- ROI 丢失时补跑
- 周期性刷新

当前类别：
```text
guide_marker
```

### 3. 精检测（OpenCV）
在 YOLO 给出的 ROI 内检测红圆：
- HSV 红色分割
- 轮廓提取
- `minEnclosingCircle`
- `fitEllipse`（优先）

输出几何量：
- 红圆中心 `(cx, cy)`
- `ellipse_major`
- `ellipse_minor`
- `effective_diameter_px`

### 4. 最终输出
当前最终输出只关心：
- `distance`：距离，单位 cm
- `vision_x`：目标中心相对图像中心线的横向偏移，单位 pixel

---

## ROS2 使用方式

### 关键节点
- `/vision_node`
- `/uart_sender`

### 节点职责
#### `/vision_node`
负责：
- 打开摄像头
- 调用 YOLO + OpenCV
- 生成视觉结果
- 推送网页视频流

#### `/uart_sender`
负责：
- 订阅 `/vision/target`
- 将视觉结果打包为 UART 二进制帧
- 固定频率发送给下位机

### 关键话题
#### `/vision/target`
文本格式：
```text
d,a_x
```
含义：
- `d = distance`
- `a_x = vision_x`

示例：
```text
223.6696,-79.4091
```

#### `/uart/tx`
调试用途，显示当前串口发出的业务值：
```text
distance=224.0088,vision_x=-79.6369
```

---

## UART 协议（当前最终版）
当前串口协议为 **14B 固定帧**，目标发送频率：

```text
50Hz
```

### 物理串口
- 设备：`/dev/ttyS1`
- 波特率：`115200`
- 格式：`8N1`

### 帧格式
```text
[0]      1B   FRAME_HEAD   = 0x91
[1..4]   4B   distance     int32_le
[5..8]   4B   vision_x     int32_le
[9..12]  4B   timestamp_ms uint32_le
[13]     1B   FRAME_TAIL   = 0x93
```

### 数值缩放
发送前：
```text
distance_i32 = round(distance * 100)
vision_x_i32 = round(vision_x * 100)
```

还原：
```text
distance = distance_i32 / 100.0
vision_x = vision_x_i32 / 100.0
```

---

## 关键几何定义

### `vision_x`
定义为：

```text
vision_x = target_center_x - image_center_x
```

在当前 `640x480` 下：
- 图像中心线约为 `x = 320`
- 合理范围约为：
  ```text
  -320 ~ 320
  ```
- `< 0`：目标在左侧
- `> 0`：目标在右侧

### `distance`
当前优先使用参考比例模型：

```text
distance = reference_distance_cm * reference_diameter_px / effective_diameter_px
```

并叠加固定偏置修正：

```text
distance = distance_raw + distance_bias_cm
```

当前配置参数：
```text
real_diameter_cm      = 20.0
reference_distance_cm = 175.5
reference_diameter_px = 52.1
focal_length_mm       = 3.24
pixel_size_um         = 3.0
distance_bias_cm      = 20.0
```

说明：
- 这不是通用相机标定库输出，而是现场经验标定后的模型
- 当前重心是让 **200~300cm** 区间更准

---

## 网页显示
网页地址：

```text
http://192.168.1.125:8000/
```

当前网页显示内容应包括：
- 实时摄像头画面
- 中心线
- YOLO ROI 框
- 红圆结果
- `FPS`
- `distance`
- `vision_x`
- `diam_px`

---

## 当前工程约束
当你阅读或修改这套代码时，必须牢记：

1. **YOLO 只做 ROI，不直接给最终 distance**
2. **OpenCV 核心任务是红圆检测**
3. **UART 协议必须保持 14B / 50Hz 不乱改**
4. **当前重点精度区间是 200~300cm**
5. **近距离误差可以大，但不能影响主工作区稳定性**
6. **重启程序时必须先停旧进程，再启动新进程**
7. **自启动场景下已经修过：**
   - systemd 常驻
   - 摄像头等待
   - Camera 打开方式
   - ROS2 publish/shutdown 保护

---

## 你在分析问题时的优先顺序
如果项目出现问题，优先按这个顺序分析：

### 1）服务是否在运行
```bash
systemctl status rdk_cjlu_26_vdone.service
```

### 2）节点是否存在
```bash
source /opt/tros/humble/setup.bash
ros2 node list
```

### 3）话题是否存在
```bash
ros2 topic list
```

### 4）相机是否打开
查看日志中是否出现：
```text
[CAMERA] opened ...
```

### 5）YOLO 是否启用
查看日志中是否出现：
```text
yolo=True
```

### 6）UART 是否真的在稳定发
查看日志：
```text
[UART STATS] target=50.0Hz, real=50.0Hz, fail=0
```

---

## 一句话项目摘要
这是一条运行在 **RDK X5 + ROS2** 上的专用视觉链路：

> **YOLO 负责给 `guide_marker` 做粗定位，OpenCV 在 ROI 内检测红圆，输出 `distance` 与 `vision_x`，并通过固定 14B/50Hz UART 协议发送给下位机，同时提供网页实时画面。**
