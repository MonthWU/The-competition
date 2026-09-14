# 基础题视觉与串口协议

## 1. 三题共用入口与任务 1 流程

任务 1、2、3 均由任务命令直接触发绿色矩形边框扫描，再进入各自的题目流程。绿色框
是三题共用的场景入口，不是任务 1 的专用条件。绿纸使用 `green_a4_*` HSV 掩膜；
基础题和任务 2 在 A4 校正图内动态采样绿色纸面 Lab 背景并反选碎片，
`white_piece_*` HSV 仅作为关闭背景反选时的可配置回退。绿框检测仍以
闭合凸四边形、面积和绿色边框覆盖率为主，不把 `210:297` 的 A4 长宽比作为硬门槛。
只保留“不过度狭长”等几何质量门控，现场阈值仍需用真实绿纸、白块和照明验证。

```text
[task,1]
  -> 直接请求绿色 A4 扫描
  -> 通过三题共用的绿色矩形边框检测
  -> 沿 A4 长边按 region_split_ratio=0.5 分为上、下区域
  -> 提取上半区及已放入下半区的 4 块基础题拼图
  -> 匹配已登记四块模板并求解 100 mm x 60 mm 图示布局
  -> 逐片发布 PuzzlePlacement
  -> serial_bridge_node 发送 [x0,y0,x1,y1,angle]
  -> 每片执行完成后重新扫描复核
  -> 全部到位后短暂发布 COMPLETE，再回到 IDLE/NO_TASK
```

任务 2 已按发挥题一的自由目标矩形闭环流程实现，详见
[`challenge_task_one.md`](challenge_task_one.md)。任务 3 先复用任务 2 的逐片闭环，并在
A4 内浅绿色纸面动态 Lab 背景分割、精确轮廓和几何求解之后扩展纹理接缝评分，详见
[`challenge_task_two.md`](challenge_task_two.md)；任一标定、分割或几何质量门控
失败都不会发送猜测结果。

## 2. 通用坐标和角度层

坐标计算放在独立的 `puzzle_geometry` 包中，不属于任务 1：

- 单应性点变换；
- 原始相机像素转“相机中心为原点”的像素坐标；
- 半周角归一化；
- 目标角与源角的有符号旋转差；
- 多边形最长边方向角。

`PuzzlePlacement` 同时携带以下坐标，任务流程只选择输出坐标系，不重复计算：

- `source/target_center_camera_px`：原始相机中心为 `(0,0)`，单位像素；
- `source/target_center_a4_mm`：A4 左上角为原点，单位毫米；
- `source/target_center_workspace_mm`：固定工作区绝对毫米坐标，仅在实测映射有效时可用。

基础题默认配置为 `camera_center_px`。X 向图像右侧为正，Y 默认向图像下方为正；
如控制侧规定 Y 向上，可将 `camera_y_axis_up` 改为 `true`。A4 局部目标点先通过
运行时检测得到的 `a4_to_image_homography` 映射回原始相机像素，再减去图像中心，
不会把透视矫正图中心误当作相机中心。

## 3. 五字段帧

默认输出：

```text
[x0,y0,x1,y1,angle]
```

- 无换行、无校验和，完整 ASCII 帧一次入队，禁止与陀螺仪帧交叉写入。
- `x0,y0`：当前碎片中心。
- `x1,y1`：该碎片目标中心。
- `angle`：拼图块从当前姿态转到目标姿态所需的完整刚体旋转角；顶视图逆时针为
  正、顺时针为负，归一化到 `[-180,180)` 度。这对应
  `placement_angle_mode: delta`，也是基础题固定协议语义。
- `source` 与 `target` 模式仅保留为诊断兼容项，基础题不得用它们替代旋转差下发。
- 小数位由 `placement_decimals` 配置，默认 2 位，使用固定 `.` 小数点。
- 每个 `PuzzlePlacement` 事件只发送一次，不按 60 Hz 重复旧位姿。
- 26E 默认 MCU<->Jetson 串口为 115200bps。串口屏直连 Jetson 时默认使用
  9600bps 8N1；UART0(`/dev/ttyTHS3`) 与 UART1(`/dev/ttyTHS1`) 均可按
  `mcu_port`/`hmi_port` 分配角色。串口屏也可连接单片机并由单片机转发 HMI 与
  Jetson 两端数据。

示例：

```text
[12.35,-6.50,30.00,40.25,-15.00]
```

该示例中的 `-15.00` 表示拼图块还需从顶部观察顺时针旋转 15°。四块模板的精确顶点、
面积和边长见 [基础题四块拼图几何定义](basic_task_geometry.md)。

串口写成功只能证明 Jetson 已完整发送帧，不能证明 MCU 已解析并执行；端到端验收还需
MCU 解析计数、字段值或执行侧反馈。

## 4. Debug 显示

`mode:=debug` 时，网页图像显示：

- 三题共用的绿色矩形外轮廓；
- 上、下区域及中间分界线；
- 每个碎片的轮廓、编号、区域；
- 每个碎片中心十字及相机中心像素坐标；
- 当前场景状态、碎片数、求解耗时和最近串口帧。

`mode:=work` 不启动网页，也不发布持续 debug 图像。

## 5. 串口屏参数持久化

串口屏发送的所有 `[cmd,val]` 参数修改必须写入 `vision_system.yaml` 中
`hmi_control_node.ros__parameters.hmi_value_*` 对应键，写入成功后才回
`vis ACK,1`；写入失败或越界时回 `vis ACK,0`。

已存在运行期订阅者的参数通过 `vision/hmi/command` 同时发布，可实时生效的立即生效。
无法在当前 launch 内重配的参数只保证重启后生效：`debug` 开关写入
`hmi_value_debug`，默认 `mode:=auto` 启动时读取该值，`1` 进入 debug，`0` 进入 work。
显式启动参数 `mode:=debug` 或 `mode:=work` 仍优先于 YAML 中的 HMI debug 值。
