# 26E_vision ROS2 workspace

本项目面向2026年E题“拼图装置”的视觉部分。Windows目录是唯一权威源码，
Jetson目录只用于部署和实机验证。标准项目 `Copyfiles/ros2_ws` 不受本项目修改影响。

## 默认视觉链路

```text
固定顶部相机
  -> 三题共用绿色框检测
  -> 工作平面四角与平面校正
  -> 上半区初始碎片约束门控
  -> OpenCV背景差分和多边形提取
  -> 基础题四模板刚体配准；未知碎片使用约束搜索
  -> 矩形闭合、无重叠、无空洞及纹理接缝门控
  -> 完整位于下半区的目标位姿
  -> 控制侧执行
  -> 三题统一闭环复核；发挥题二在几何闭环后扩展花纹评分
```

默认不启动YOLO、TensorRT或Kalman节点。复制保留的旧包只作为历史基础，默认launch
不会启动它们。

## 核心节点

- `puzzle_perception_node`
  - 只在收到 `puzzle/scan_request` 后采集。
  - 从连续帧中选择最清晰帧，以独立的 `green_a4_*` HSV 范围检测绿色矩形框，
    不强制其符合 A4 长宽比，再由四角生成矫正图。
  - 基础题和发挥题一在 A4 校正图内动态采样绿色纸面 Lab 背景并反选碎片；
    `white_piece_*` HSV 仅作为关闭背景反选时的可配置回退。
  - 排除A4分界线和配置的电磁铁颜色。
  - 基础题随后仍须满足固定 4 块的面积、`3/4/4/4` 顶点和直线拟合门控；发挥题一
    使用同一前景入口，但继续执行未知碎片的 20 mm 边长和 1--4 片几何求解门控。
  - 任务 3 先用已确认标定参数去畸变，再把纵向 A4 校正到 `210 mm × 297 mm` 平面；
    在 A4 内动态采样浅绿色纸面 Lab 背景并提取非纸面碎片，使用完整外轮廓、稳健直线
    拟合和交点细化；任务 2、3 的初始碎片轮廓必须完整位于上半区。
  - 输出A4局部多边形、拾取候选点和固定工作区坐标映射。
- `puzzle_solver_node`
  - 基础题优先匹配已登记的 4 块图示模板，目标固定为 `100 mm × 60 mm`；
    模板不一致时拒绝按通用矩形猜测。
  - 若轮廓只有经过镜像反射才能匹配模板，输出
    `BASIC_TEMPLATE_FLIP_REQUIRED:P...`，不生成仅靠平面旋转无法执行的假目标。
  - 相邻公共接缝默认保留可配置的 `0.2 mm` 名义间隙，只平移刚体而不缩放拼图块；
    该间隙是视觉规划量，实物落位仍需最终视觉复核。
  - 非基础题已知模板路径关闭后，通用几何求解使用子集动态规划组织二叉合并。
  - 支持整边匹配和T形接缝所需的端点对齐部分边匹配。
  - 使用边长、端点、重叠、矩形度、尺寸范围和可选接缝纹理评分。
  - 最优解与次优解差距不足时输出 `AMBIGUOUS_LAYOUT`。
  - 发挥题一使用 90--120 mm × 50--90 mm 尺寸范围，目标方向和下半区内中心位置自由；
    完整目标轮廓必须留在下半区安全边界内，全等片产生的合法等价解按移动成本确定性
    选择，不沿用基础题的严格分差拒绝。
  - 发挥题二与发挥题一保持同一几何求解链；扑克牌白边只作为外框候选证据和排序
    信息，不禁止内部拼接；白边过多、低白度或整边贴黑都不单独否决矩形候选。
  - 发挥题二只有在矩形旋转、全等片互换等多解已经解出后，才用连接边梯度连续性、
    中心对称性、对角特殊性和非字符角纯白色度消歧。
- `puzzle_coordinator_node`
  - 管理启动、工具离开、扫描、下发单片目标、执行完成和重新复核。
  - 龙门架或电磁铁未离开关键区域时不允许扫描。
  - 每次控制侧完成动作后重新求解，已经到位的碎片不会再次下发。
  - 发挥题一参考基础题动作后复核流程：收到当前片 `[move,ok]` 后重新扫描当前 A4
    场景，复核已放置碎片，并用未知碎片通用几何求解继续下发下一片。
  - 发挥题二与发挥题一保持同一逐片闭环：收到当前片 `[move,ok]` 后重新扫描和重求解；
    但每轮求解会把白边作为外框候选证据，再只对多解执行消歧。
- `puzzle_geometry`
  - 提供与任务编号无关的单应性、相机中心像素、角度归一化和旋转差计算；控制旋转
    统一为顶视图逆时针正、顺时针负。
- `serial_bridge_node`
  - 作为Jetson侧 UART 唯一拥有者，默认 MCU 使用 UART1(`/dev/ttyTHS1`)、
    串口屏使用 UART0(`/dev/ttyTHS3`)。MCU口为115200bps 8N1，串口屏口为
    9600bps 8N1；UART0/UART1可按接线互换，但波特率按角色配置。
  - 接收任务命令并把每个 `PuzzlePlacement` 格式化为
    `[x0,y0,x1,y1,angle]`；串口屏可直连Jetson，也可由单片机转发。
  - 把单一待执行请求对应的 `[move,ok]` 映射为带 `piece_id` 的
    `puzzle/placement_done`；重复或无请求确认不会推进任务。
- `hmi_control_node`
  - 解析串口屏`[cmd,val]`、`[get,cmd]`和`[task,num]`帧。
  - 所有串口屏参数修改先永久写入`vision_system.yaml`对应`hmi_value_*`键，
    写入成功后才ACK；已接入运行期命令话题的参数实时生效，debug开关在
    `mode:=auto`的下一次启动生效。
- `vision_interfaces`
  - 新增 `PuzzlePiece`、`PuzzleScene`、`PuzzlePlacement` 和 `PuzzlePlan`。
- `vision_bringup`
  - `work` 启动三个拼图核心节点、串口桥和HMI控制节点。
  - `debug` 在work基础上额外启动网页预览，并启用限频调试图像。
  - 默认`mode:=auto`、`serial_enabled:=true`、`hmi_enabled:=true`；
    除非用户明确要求关闭串口，运行和调试都保持串口节点启用。

基础题四块拼图的尺寸、顶点、面积和角度协议见
[`docs/basic_task_geometry.md`](docs/basic_task_geometry.md)。

## 坐标系

- 工作区绝对坐标系固定在地面/工作平面，不随A4、龙门架或电磁铁移动。
- A4局部坐标系原点为纸张左上角，X向右、Y向下，单位为毫米。
- `workspace_homography_px_to_mm` 是固定相机像素到工作区绝对平面的映射。
- `workspace_mapping_valid` 默认是 `false`。未完成实测映射前，求解器拒绝向控制侧输出绝对坐标。
- 基础题串口默认使用原始相机中心为原点的像素坐标，因此不会把未标定的 A4
  局部毫米坐标冒充成工作区绝对坐标；详见 `docs/basic_task_protocol.md`。

## 构建

```bash
cd ~/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  puzzle_geometry \
  vision_interfaces \
  puzzle_perception_node \
  puzzle_solver_node \
  puzzle_coordinator_node \
  serial_bridge_node \
  vision_bringup
```

运行单元测试：

```bash
colcon test --packages-select puzzle_solver_node
colcon test-result --verbose
```

## 运行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_bringup vision_system.launch.py mode:=work
```

若调试环境暂未安装声明的 `python3-flask` 运行依赖，可仅启动调试日志和图像话题：

```bash
ros2 launch vision_bringup vision_system.launch.py mode:=debug web_enabled:=false
```

只迭代视觉、不打开硬件串口时使用：

```bash
ros2 launch vision_bringup vision_system.launch.py \
  mode:=debug web_enabled:=true serial_enabled:=false
```

上述是例外用法。默认启动不传`serial_enabled:=false`，串口桥和HMI控制节点都会启用。

控制侧确认工具离开视觉区域后启动；当前生命周期不再依赖 `puzzle/tool_clear` 话题：

```bash
ros2 topic pub --once puzzle/start std_msgs/msg/Bool "{data: true}"
```

视觉下发单片目标：

```text
puzzle/placement_request  vision_interfaces/PuzzlePlacement
```

串口桥随后按当前配置发送一次：

```text
[x0,y0,x1,y1,angle]
```

控制侧完成指定碎片后，使用相同 `piece_id` 确认：

```bash
ros2 topic pub --once puzzle/placement_done vision_interfaces/msg/PlacementDone \
  "{task_id: 1, generation: 1, piece_id: 1}"
```

协调节点随后直接重新扫描并进行闭环复核。

## 配置

- 固定设备、话题、坐标映射：`src/vision_bringup/config/vision_system.yaml`
- 串口拓扑、HMI默认值和串口屏持久化参数：`src/vision_bringup/config/vision_system.yaml`
- 比赛现场阈值和求解权重：`src/vision_bringup/config/competition_tuning.yaml`
- 项目长期规则：`AGENTS.md`
- 视觉目标：`docs/vision_goal.md`

绿色 HSV 阈值采用用户本次给定值；绿色纸面 Lab 背景反选、电磁铁和几何阈值仍是初始工程值，
必须在真实绿纸、碎片和现场照明下验证。
基础题固定模板允许 `10 mm` 短边，使用独立的 `basic_min_piece_edge_mm` 参数；现场碎片
仍使用题面规定的 `min_piece_edge_mm=20.0`，两者不得混用。
绿色框检测是任务 1、2、3 的共用前置入口；任务 2（发挥题一）流程见
[`docs/challenge_task_one.md`](docs/challenge_task_one.md)，任务 3（发挥题二）流程见
[`docs/challenge_task_two.md`](docs/challenge_task_two.md)。
