# 发挥题二视觉与串口流程

## 1. 输入与安全前提

- 任务命令为 `[task,3]`；浅绿色纵向 A4 是初始摆放区、目标区和 210 mm × 297 mm 尺度基准。
- 碎片初始在 A4 上半区域内共面、随机摆放且互不重叠，目标矩形必须完整位于下半区域。
- 每次扫描都从当前 A4 内有效纸面重新估计 Lab 背景中位数和离散度；不依赖纸外黑背景，
  采样不稳定则返回 `BACKGROUND_SAMPLE_UNSTABLE`。
- 必须加载与最终分辨率和裁剪模式匹配的实测相机标定。`camera_calibration_valid=false` 时
  固定返回 `CALIBRATION_REQUIRED`，不生成计划。

## 2. 唯一处理链

```text
[task,3]
  -> 直接请求绿色 A4 扫描
  -> 实测内参/畸变参数 undistort
  -> green_a4_* HSV 检测 A4 四角
  -> A4 单应性校正到 210 mm x 297 mm 纸面
  -> 排除页面边缘和中线带，动态采样浅绿色 A4 纸面 Lab 背景
  -> Lab 颜色距离 + opening/closing 得到非纸面前景
  -> findContours(RETR_EXTERNAL, CHAIN_APPROX_NONE)
  -> 多 epsilon approxPolyDP 初值
  -> 按边分配完整轮廓点、fitLine、相邻直线交点
  -> 通过 cv::contourArea/轮廓拟合面积计算碎片面积，不按内部颜色纯色面积估计
  -> 面积、3--5 边、最小边长、拟合 RMS/最大残差和 1--4 片门控
  -> 同一任务代的首次有效扫描检查全部轮廓完整位于上半区
  -> 90--120 mm × 50--90 mm、总轮廓面积/矩形面积、每片外边和外框覆盖四边约束
  -> 完整目标轮廓位于 A4 下半区安全边界
  -> 先尝试由 1--2 块组成的小矩形组合
  -> 小矩形连接线和最终大矩形全部内部连接处做梯度/Lab/灰度/SSIM 连续性判断
  -> 若没有小矩形组合，判断是否存在轮廓大小和形状一致的可交换碎片
  -> 若存在可交换碎片，按发挥题一几何逻辑求等价解并选连接处连续性综合最高的一种
  -> 若既无小矩形也无可交换块，直接退回发挥题一几何求解
  -> 缓存整套 PuzzlePlacement 队列
  -> 发送队首 [x0,y0,x1,y1,angle] 并等待 [move,ok]
  -> 收到确认后直接发送下一条
  -> 全部到位后 COMPLETE -> IDLE/NO_TASK
```

`RETR_EXTERNAL` 使扑克牌内部黑色和红色图案不成为碎片边界；碎片面积以外轮廓面积
为准。当前任务三在几何候选成立后替换为三段式选择：第一段寻找 1--2 块小矩形组合，
并要求小矩形连接线与最终大矩形全部内部连接处连续；第二段在无小矩形时检测同形同尺寸
可交换碎片，并在发挥题一几何等价解中选择连接处连续性综合最高的一种；第三段在两类
特化条件均不命中时直接退回发挥题一几何求解。

连接处连续性使用 Lab 颜色差、灰度 ZNCC、梯度幅值 ZNCC 和灰度 SSIM 条带评分。
弱纹理或纯白区域不能单独生成猜测输出；只有已经通过 A4、轮廓、几何、重叠、尺寸
和下半区边界硬门控的候选，才允许进入发挥题二的纹理连续性选择。

## 3. 坐标与闭环

- `x0,y0` 和 `x1,y1` 均由 A4 平面点投影回去畸变后的相机图像，再以图像中心为原点；
  X 向右、Y 向下，单位像素。
- `angle` 是完整刚体旋转差，顶视图逆时针为正、顺时针为负，范围 `[-180,180)`。
- 发挥题二先与发挥题一保持一致：任何时刻仅有一个五字段帧等待确认；无待执行请求、
  重复或乱序 `[move,ok]` 不推进任务。
- 首次求解时必须缓存当前计划中的全部未放置碎片；收到当前片 `[move,ok]` 后不得
  重新扫描当前 A4 场景，而是直接下发缓存队列中的下一片，避免机械结构遮挡污染图像。
- 花纹内容只在上述闭环成立后扩展：可通过 `challenge_two_pattern_enabled`、纹理权重和
  接缝条带采样参与小矩形确认、同形块交换消歧和最终接缝复核，但不得跳过 A4、
  轮廓、几何、重叠和下半区边界硬门控。
- `challenge_two_pattern_edge_priority_enabled` 控制花纹边优先比较；`pattern_edge_min_strength`
  只定义“这条边是否有足够花纹信息量”，不是拼接通过阈值。
- `challenge_two_specialized_logic_enabled=true` 是发挥题二默认路径；旧白边外框启发式
  默认关闭，不作为当前发挥题二的主判断。
- `challenge_two_small_rectangle_min_rectangularity` 控制 1--2 块小矩形组合的矩形度门槛。
- `challenge_two_same_shape_relative_tolerance` 和
  `challenge_two_same_shape_angle_tolerance_deg` 控制同形可交换块的轮廓相似判定。

## 4. 主要拒绝状态

- `CALIBRATION_REQUIRED`：实测标定未启用或参数结构无效。
- `GREEN_A4_NOT_FOUND`：浅绿 A4 四边形未通过门控。
- `BACKGROUND_SAMPLE_TOO_SMALL` / `BACKGROUND_SAMPLE_UNSTABLE`：浅绿色 A4 纸面无法可靠采样。
- `TASK3_NO_VALID_PIECES` / `PIECE_COUNT_INVALID`：轮廓质量或数量不满足要求。
- `INITIAL_PIECE_OUTSIDE_UPPER_REGION`：至少一片初始碎片越过上半区有效边界。
- `NO_FEASIBLE_PLACEMENT`：没有候选通过外框、几何、下半区放置或多解消歧硬门控。
- `PATTERN_CONTINUITY_REJECTED`：已命中小矩形或同形块特化条件，但连接处梯度/纹理
  连续性未通过。
- `LOW_QUALITY_LAYOUT`：仅在显式配置正数绝对分数上限时，最优候选超过该上限。
- `AMBIGUOUS_LAYOUT`：仅在显式配置正数最小分差时，最优与次优候选分差不足。

上述状态均不会进入串口发送队列。

## 5. 待实机验收

当前参数是工程初值，不代表已达到现场精度。必须用真实 A4、扑克牌碎片、最终照明和固定
相机采集标注集，覆盖上半区随机摆放、靠近页面边界与靠近中线的拒绝样本，报告纸面
背景误分率、计数准确率、轮廓 IoU、顶点误差 P50/P95、边线 RMS/
最大残差、完整拼图正确率、非唯一场景下花纹排序正确率、求解 P50/P95 延迟和串口逐片握手结果。
