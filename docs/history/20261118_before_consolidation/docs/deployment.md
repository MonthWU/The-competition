# 构建与交付

## 2026-11-18 TODO1本地分项目增补

本地根start.sh现支持`[without-radar|with-radar] [work|debug] [--check|--check-files]`，省略分项目时默认without-radar；start_coloritems.sh仍为独立物块入口。两个分项目各有`projects/<名称>/start.sh`、`config/board.json`与`config/route.json`。下文“两个公开入口”的旧业务说明需结合此增补阅读；本次尚未更新板端。

规划源码为planning.py、navigation.py，Application可选创建规划模块，HMI增加理论路径。用户随后明确无雷达版直接从二维码开始，已将navigation.enabled=false且移除无雷达route.json；只有有雷达版保留规划接口。两版配置均从本版config目录解析共享模型和原生库路径；部署必须共同包含projects、src、config、assets、native、scripts和docs，禁止只传子目录。with_radar的雷达路径、标定与定位快照接口等待填写，不能用假值通过检查。有雷达版场地confirmed=false时HMI显示待配置且没有路线。

本地Python3.10语法、setuptools包构建、人工算法与HMI绘图状态检查通过；WSL仅做Bash语法和--check，不启动识别应用。实际命令、跳过项及浏览器工具限制见VALIDATION.md和artifacts/todo1_commands_20261118.md。本轮后续用户明确授权测试二维码：独立QR相机已启动并关闭，成功识别134+213+314+321；通过SSH stdin在内存加载本地当前CameraModule，没有写入板端源码或/tmp，没有UART发送、雷达或物块相机启动。完整分项目仍未部署。

本次串口更新：源码新增target_pose_frame及三点拟合角度，运行标靶帧改为0x55（来源、α、dx、dy），详见SERIAL_PROTOCOL.md；旧0x54实机测试记录不能作为本版接收验证。部署前完整备份，完成Python/ROS构建、配置及人工/模拟检查后保持待启动；本轮不修改MCU或新开实机链路。

2026-11-18后续实机测试增补：用户授权后已运行并停止独立物块链路，验证YOLO→1080p OpenCV、主机偏差发送及超时恢复。测试修复UVC回读与自适应圆半径后完成原生库、Python检查及ROS2重建，13项相关检查通过。测试结束相机/UART2已释放；没有启用常驻服务。详见TODO2_ALIGNMENT.md最新记录及本轮artifacts/live_focus_20261118。下文待启动/未实测描述保留为此前交付历史。

## 两个公开启动入口与work/debug（2026-11-18复核）

项目根目录公开入口固定为两个Bash文件，共用scripts/start.sh与同一Python应用：

| 命令 | 业务流程 | 模式 |
| --- | --- | --- |
| `bash start.sh` | 二维码→物块检测与标靶精定位 | 默认work |
| `bash start.sh debug` | 相同主流程 | debug |
| `bash start_coloritems.sh` | 跳过二维码，直接物块检测与标靶精定位 | 默认work |
| `bash start_coloritems.sh debug` | 相同独立物块流程 | debug |

work与debug使用完全相同的设备、模型、阈值、任务状态机及MCU协议。work正常运行的Python输出、日志及原生stdout/stderr写到项目log/runtime/work-日期时间-PID.log，终端保持安静；警告、异常及非零退出码保留。debug直接在终端显示诊断，包括相机处理耗时、雷达圈信息及串口收发；不以跳过模块实现debug。两模式性能仍需分别实测，不能宣称相同帧率。

两个入口均可追加`--check`或`--check-files`，例如`bash start.sh debug --check-files`。显式只检查命令在终端显示JSON结果，不创建应用、不打开相机/UART、不启动HMI。非法模式返回2。两个公开脚本和内部脚本均可直接执行或通过bash执行，部署时设置可执行权限。

“正常启动”对应当前已实现的board.json二维码→物块流程；雷达仍关闭，TODO1的有/无雷达分项目与动态规划未完成。启动运行依旧由用户明确指示，本轮只做静态检查与假应用测试。

## 2026-11-18 新项目部署与TODO2

本轮目标`/root/Monthwu/GongChuang_vision_26`，源码来自本地car26_new。源项目重大修改前已备份到`../artifacts/RDK_CJLU_V05_todo2_20261118`。板端旧项目、失败迁移及历史备份移至独立`/root/Monthwu_archive/20261118_todo2`归档；具体结果以本轮部署结果JSON为准。保留旧服务配置和状态，停止并禁用旧coloritems服务，移出活动项目列表。新应用不自动启动、不自动启用开机自启。

新增入口登记：

| 入口 | 用途 | 运行条件 |
| --- | --- | --- |
| `start.sh [work/debug]` | board.json的二维码→物块流程 | 用户手动运行 |
| `start_coloritems.sh [work/debug]` | board-object.json独立物块与圆心精定位 | 用户手动运行 |
| `scripts/build_focus.sh` | 编译C++ OpenCV共享库 | 仅构建，不开设备 |

部署包括src、config、assets、native、scripts、docs、tests和两个入口；不传输Windows/WSL仿真环境、旧build/install或本地缓存。BPU模型和object.names从已确认实机项目读取，模型SHA256为bedffa66d94d8434cade9a42dd3937f135d416857633d632a63a2af71838457f。板端执行原生圆检测编译、car26_system的colcon构建、编译检查、人工测试和--check-files；这些均不启动识别应用或硬件采集。

当前配置的雷达关闭，未填入假外参或假地图；有/无雷达动态规划拆分属于TODO1，不用本轮部署宣称完成。首次实机运行须核对分辨率切换视场、曝光实际效果、近圆误检、恢复流程和MCU 0x54解析；协议详见TODO2_ALIGNMENT.md。

## 当前交付与依赖

新项目为一个 `ament_python` 包 `car26_system`，内含四个功能模块；可直接以 `python3 -m car26` 启动，也可使用 ROS2 包入口。2026-11-18板端目录为 `/root/Monthwu/GongChuang_vision_26`，TROS环境为 `/opt/tros/humble/setup.bash`。旧car26_ws已归档，新项目原生库和ROS2包构建通过，保持待启动。下文2026-11-18以前的未部署描述为历史交付记录。

板端需 Python 3、numpy、OpenCV（含 V4L2 与 QRCodeDetector）、pyserial 和 RDK X5 BPU Python `hobot_dnn.pyeasy_dnn`。pyzbar 与 zbar 是二维码可选回退；模型和标定文件位于项目的 assets，不由 ROS 包安装自动搬运。BPU API 使用板端平台包，不从 PyPI 安装同名替代。

直接入口不要求 source TROS；可选 ROS2 build/launch 必须 source TROS 及新项目 install 环境。不再依赖 `hobot_usb_cam`、`ai_msgs` 或旧 `obj_detect`/`qrc_skandier` 包。

## 脚本登记

| 脚本/入口 | 用途 | 状态与参数 |
| --- | --- | --- |
| `scripts/start.sh` | 直接启动一个新应用 | 本地已编写；参数为配置路径、work/debug；默认 system.json/work；exec 保留信号传递 |
| `scripts/build.sh` | TROS 下构建唯一包 | 本地已编写；`colcon build --packages-select car26_system`；未在板端执行 |
| `scripts/check.ps1` | 本地编译及人工数据 unittest | 用户已授权纯算法测试，已执行；不打开设备 |
| `car26` / `python3 -m car26` | 同一 CLI | 必填 `--config`；可选 `--mode work|debug`、`--check`、`--check-files` |
| `launch/car26.launch.py` | 可选 ROS2 启动包装 | 必填 `config:=绝对路径`，`mode:=work` 默认；仅一个 ExecuteProcess |
| `simulation/setup_environment.sh` | 独立WSL Ubuntu 24.04安装环境 | root执行，仅仿真发行版 |
| `simulation/build.sh` | Linux内同步、构建和检查上游场地包 | sim执行，参数为本地仓库的Linux路径 |
| `simulation/start.ps1` | Windows启动Gazebo场地 | 默认软件渲染；可选Layout、SoftwareRender、HardwareRender（排障） |
| `simulation/start_lidar.ps1` / `simulation/start_lidar.sh` | 默认蓝区1折叠整车、C1扫描；可选SLAM | -StartZone 1/2，-Slam，-StartClass 0/1/2；-BodyOcclusion Auto/Model/Wheels/Disabled（Auto在简化SLAM下恢复四轮，其他为Model）；-DetailedModel需-FixturePose；-FixedLidar无车 |
| `simulation/启动SLAM仿真.cmd` | 双击启动场地、简化车体与SLAM | 默认蓝区1、类别0；真实串口未连接；关闭GUI清理自有辅助进程 |
| `simulation/slam/build.sh` | 构建独立car26_slam C++包并运行核心检查 | sim执行；Linux slam_build/slam_install，不构建或部署实机应用 |
| `simulation/slam/run_slam.py` | 管理Cartographer、栅格地图、前端、外参与仿真时钟/陀螺仪桥接 | robot仅接受外参与车体遮挡；start-class、log-dir；view-config仅转交地图窗口；由入口管理生命周期 |
| `simulation/slam/gyro_adapter.py` | Gazebo IMU缩减为实际单轴数据合同 | /gyro/sim_raw → /gyro/z；100 Hz，启动相对角，不转发加速度 |
| `simulation/slam/map_viewer.py` | 独立实时SLAM地图窗口 | 只读/map、局部/场地位姿与状态；5Hz刷新；固定5×5场地格；view-config仅用于蓝区显示对齐；--exit-after/--evidence用于有界检查 |
| `simulation/slam/map_view_geometry.py` | 显示专用SE(2)坐标与栅格图像变换 | 不发布话题，不为定位核心提供起点；由地图窗口调用 |
| `simulation/slam/check_map_viewer.py` | 人工ROS消息与Tk窗口有界验证 | 域由调用者指定；CAR26_VIEW_CHECK_OUTPUT指定证据目录；验证两蓝区、25格与确认/失效切换，不测SLAM精度 |
| `simulation/slam/start_map_viewer.sh` / `simulation/打开SLAM地图.cmd` | 重新打开日常域83的地图窗口 | 不重启仿真或算法；关闭此窗口不会停止建图 |
| `simulation/slam/verify_replay.sh` | 独立运动房间传感器回放 | 域85；估计轨迹、地图与未定位不输出三值检查 |
| `simulation/slam/verify_gazebo.sh` | 蓝区1静态场地SLAM检查 | 域86；可选--gui；截图、100 Hz单轴流及地图检查 |
| `simulation/slam/verify_visibility.sh` | 两圆柱有回波、标靶区域无回波的真实Gazebo/Cartographer检查 | 域89；独立分区；关闭车体遮挡；可选--gui；不停止日常会话 |
| `simulation/slam/verify_wheels.sh` | 四轮真实回波、自体过滤与轮子阴影的Gazebo/SLAM检查 | 域90；独立分区；不要求被轮子挡住的圆柱产生回波；可选--gui |
| `simulation/inspect_field_scan.py` | 只读当前仿真扫描与地图，独立统计对象回波/占据格 | truth、output；可选require-visibility；真值仅用于评估，不供给SLAM；需要调用者指定ROS域 |
| `simulation/slam/verify_frontend.sh` | 前端三值门控边界检查 | 域87；明确注入位姿桩，不代表SLAM精度 |
| `simulation/check_lidar.py` | 有界仿真扫描链路检查 | world、output；可选gui；只清理本次进程 |
| `simulation/create_lidar_world.py` | 根据实机配置生成C1仿真覆盖场景 | world、output；可选测试位姿、system-config、noise-stddev；保留上游资源 |
| `simulation/field_geometry.py` | 二维码板接地、生成器调用的水平扫描高度可见性修正 | CLI为world、output、可选models的接地覆盖；C1生成器另调用高度函数，将低矮地面/标靶外观排除雷达层，GUI与碰撞不变；不改上游 |
| `simulation/start_field.sh` | 普通场地使用同一二维码板接地修正 | layout参数；由start.ps1调用，软件渲染规则保持一致 |
| `simulation/c1_scan_bridge.py` | 补充C1标称扫描时序 | world；/scan_raw → /scan；由仿真入口管理 |
| `simulation/test_c1_adapter.py` | C1角度、无效点和分包协议回归检查 | Python unittest；不打开设备 |
| `simulation/verify.sh` | 图形、SDF与扫描链路检查入口 | sim执行；可选--gui |
| `simulation/启动雷达仿真.cmd` | 默认整车、雷达、SLAM和独立地图窗口 | 转交start_lidar.ps1 -CarModel -Slam；无车诊断直接用PS入口-FixedLidar |
| `simulation/启动整车仿真.cmd` | 同步开启轻量整车、SLAM和独立地图窗口 | 转交start_lidar.ps1 -CarModel -Slam；静态姿态，光心/零角未标定 |
| `simulation/create_car_world.py` | 生成整车场景、自体遮挡真值、robot校准文件及独立view显示预设 | --start-zone 1/2为折叠蓝区预设；--body-occlusion model/wheels/disabled（直接调用默认model，wheels仅简化车体）；诊断--x/--y指雷达世界坐标；view不得作为估计输入 |
| `simulation/simple_car.py` | 四轮、车板、机械手等16个几何物体及扫描平面截面 | 可选folded几何姿态；无STL或驱动 |
| `simulation/cad/inspect_solidworks.py` | Windows只读审计CAD副本并导出原生STL | source、output；需要本机SolidWorks和pywin32；恢复导出偏好，不保存原装配 |
| `simulation/cad/prepare_car.py` | 生成接地网格、雷达位置和扫描平面线段 | 读取已审计原生STL与cad/alignment.json；光心偏移仍为假设 |
| `simulation/cad/preview_model.py` | 导入模型的三视图与尺寸检查 | 基于原生STL生成cad_views.png、geometry_summary.json |

没有新增 systemd、自启安装或独立停止脚本。应用以前台 SIGINT/SIGTERM 关闭其持有设备。不会自动停止旧项目；新旧应用不能同时持有相同设备。

## 本地检查

```powershell
& 'D:\AIworkplace\RDKx5\26GongChuan_vision\car26_new\scripts\check.ps1'
Set-Location 'D:\AIworkplace\RDKx5\26GongChuan_vision\car26_new\src\car26_system'
python setup.py build --build-base '..\..\artifacts\package_build'
```

Windows 缺少 TROS/colcon，不能声称完成 ROS2 ARM64 构建。setuptools 构建和人工测试的结果见 VALIDATION.md。

## 经确认后才执行的板端交付步骤

1. SSH 只读确认 hostname 为 `RDKx5-Monthwu-01`、目录和当前代码状态，重新核对 authority docs。确认新项目实际目标目录以及设备参数。
2. 向用户展示具体同步清单、目标目录、备份位置和哈希；按 database.md 获得板端写入确认。优先独立目录，不在未知状态下覆盖现有 `/root/Monthwu/car26_ws`。
3. 备份需要替换的文件，传输源码、config、assets、docs；核对 SHA256。不传输 Windows 构建产物或 __pycache__。
4. 在已确认的新工作区 `source /opt/tros/humble/setup.bash` 后执行 `bash scripts/build.sh`，检查包入口、HMI 页面、launch 安装完整。
5. 用户填写新设备、图像参数、外参和地图；运行 `--check` / `--check-files`。路径存在不等于端口可用，不能用空参数先启动猜测设备。
6. 启停旧链路、新链路需分别按 database.md 阶段 4 确认。硬件验证由用户执行；此阶段没有自动安装服务或开机自启。

板端启动示例（仅在上述步骤完成后）：

```bash
bash scripts/start.sh /实际新项目目录/config/system.json work
# 或，构建并 source 新 install 后：
ros2 launch car26_system car26.launch.py config:=/实际新项目目录/config/system.json mode:=debug
```

## 硬件验收材料

用户记录：模型输入/六输出布局与实际结果；两路相机尺寸、实际帧率、颜色与过滤效果；QR 原文四帧和实际 MCU 接收；三次 shot/ack/map 时序与起点标定；物块/标靶像素坐标；C1 型号、健康、扫描频率、点数、安装与地图匹配；HMI 连接和设备断开；SIGINT/SIGTERM 资源释放；相同输入 work/debug 结果及性能开销。

相机 read、模型推理、处理、UART、MCU 和机构动作时间分开记录，不能用 processing_ms 代替实际采集延迟或运动效果。C1 安装、电源、预热按 RPLIDAR_C1_REFERENCE.md 核对。

依赖实现参考：[pyserial 官方 API](https://pyserial.readthedocs.io/en/stable/pyserial_api.html)、[D-Robotics pyeasy_dnn 文档](https://d-robotics.github.io/rdk_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_pyeasy_dnn/)、[RDK X5 模型应用说明](https://d-robotics.github.io/rdk_doc/en/Algorithm_Application/model_zoo/rdk_x5_guide/)。平台兼容性须以实际板端版本及构建结果为准。
