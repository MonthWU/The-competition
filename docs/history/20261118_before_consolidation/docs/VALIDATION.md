# 本地验证记录

## 2026-11-18 默认整帧扫码ROI

按用户要求将整帧扫描设为明确默认：system.json与board.json中cameras.qr.roi=null；缺省同样整帧，不受物块block_roi或预扫描地图ROI影响。只有手动填写[x,y,width,height]才裁剪，参数与边界均检查；运行状态显示有效qr_roi及vision_mode=qr。当前新架构原本无中心裁剪，故此次属于默认契约明确化及可配置接口补充。

新增四项回归检查：默认/缺省ROI时两种解码器收到原图对象；OpenCV实际解码人工二维码位于640×400画面的四角与中心共五种位置；手动ROI两种解码器裁剪一致；非法ROI和运行时越界被拒绝。均使用人工图与模拟接口，不打开相机或UART。不能把人工图角落解码当作实机场景边缘光学效果验收。当前修改尚未持久部署到板端。

相关26项测试通过；最终全套90项中88通过、2项因板端环境跳过；setuptools本地包构建通过。实际命令及结果见artifacts/qr_full_frame_commands_20261118.md。

## 2026-11-18 无雷达流程澄清与二维码实测

用户明确无雷达版直接从二维码开始，不需要上位机规划。该版mission.entry=qr、radar.enabled=false、navigation.enabled=false，移除无雷达route.json。模拟主循环验证不会创建NavigationModule、不会启动或读取雷达；全套86项仍为84项通过、2项跳过。

用户随后授权二维码实测。核对板端hostname=RDKx5-Monthwu-01、aarch64、当前AGENTS和三阶段文档，确认相机无占用。通过SSH标准输入仅在内存中载入当前本地CameraModule（SHA256 3045ab4125cc32ede3f21b788e7797a107c21ce30ece03bf14eb535c3679fc08），没有覆盖板端持久源码或写/tmp。

640×400二维码相机第3帧识别到`134+213+314+321`，包括清理的总测试耗时1.517秒，采集线程共处理4帧，close成功。未打开UART、雷达或物块相机，未发送任何二维码报文；这不是MCU接收或完整相机切换验收。结果JSON、相机预览和测试程序分别为artifacts/qr_live_result_20261118.json、qr_live_preview_20261118.jpg与qr_live_probe_20261118.py。正常解码一次不能证明所有角度、距离和光照条件均可靠；当前画面较亮，进一步覆盖率调参应另做有界测试。

测试结束后再次查询fuser -v /dev/video2 /dev/video0 /dev/ttyS2，返回1且stdout/stderr均为空，确认二维码相机、物块相机和UART2均无占用。首次采用多层引号的释放查询因Python语法解析失败未执行，改用标准输入执行只读查询后核对通过。实际命令与内存测试驱动见artifacts/qr_live_commands_20261118.md。

本地分项目源码尚未部署，不能把内存实测等同于板端更新完成。下文TODO1初始实施记录中的固定路线说法已由本次用户澄清覆盖，无雷达版不再使用该规划分支。

## 2026-11-18 TODO1分项目与规划

用户确认全部实机场地参数未定，保留配置接口；本轮仅规划结果与网页显示。修改前完整项目zip备份及SHA256见TODO1_NAVIGATION.md。

新增tests/test_navigation.py共15项，覆盖固定一次规划、任务顺序/返回、阻断绕行、无路、道路净空/切触、行驶中接入、未知地图门控、动态新障碍及记忆、定位失效/错误坐标系/未来时间/非有限位置、雷达过期、观测同步、安装外参及C1极性、容量超限和无雷达主循环无设备访问。全套86项：84项通过，2项跳过（板端Bash综合入口测试、ARM原生圆检测库）。另在WSL执行实际新入口的语法与--check验证，不把该项补充当作ARM板端验收。

Python3.10 AST语法检查通过；setuptools构建至artifacts/todo1_package_build，含新增规划模块和更新HMI页面。网页JS经node --check通过；artifacts/todo1_hmi_check.cjs使用页面实际脚本和模拟DOM/Canvas，检查固定路线、动态绕行、无路、定位失效、缺失配置及断连后的路线清除。人工地图及输出见artifacts/todo1_plan_examples.json，没有写入任何实机地图模板。

浏览器测试工具初始化报SyntaxError，未能进行真实浏览器截图和视觉检查；绘图逻辑检查不等于浏览器视觉验收。没有实机部署、BPU/相机/雷达采集、物理路径或MCU运动验证。动态模式仍需实机定位生产者、雷达外参与自体回波处理；本轮没有自动清除历史阻断或新增运动报文。

without_radar配置检查通过；with_radar预期报告RADAR_DEVICE_REQUIRED，真实设备路径保持空白。场地confirmed=false表示尚无可用路线，即使基础配置检查通过也不代表规划参数已填写。

## 2026-11-18 二维码流程修复

仅本地人工帧、模拟相机及解码器，不打开相机、串口或ROS节点。新增`tests/test_qr_capture.py`九项测试；最初七项在修复前全部失败，修复后连同新增的可选备用解码和角色切换测试全部通过。

覆盖：单帧扫码成功不被后续空帧或其他码覆盖、优先消费后仍可读取最新视频帧、原帧序号和时间戳保留、非法UTF-8不终止采集、OpenCV解码异常使用备用解码、空码与其他码制不遮蔽后续有效码、无备用解码器时继续工作，以及关闭/切换角色清空旧事件。

全套`python -B -m unittest discover -s tests -v`共71项：69项通过，2项跳过（板端Bash启动入口及ARM原生圆检测库）。现有过期帧拒绝、四次发送及flush次序、发送失败阻止相机切换等回归测试继续通过。未部署到板端，未进行扫码、UART接收或实机切换验收。MCU ACK、长度限制、扫码超时和发送整体超时不属于本次三个已复现缺陷的修复。

本次实际命令见`artifacts/qr_review_commands_20261118.md`的修复部分。

日期：2026-10-13。项目：`D:/AIworkplace/RDKx5/26GongChuan_vision/car26_new`。用户明确授权本地纯算法测试；全部使用人工数据、mock 设备接口，没有启动真实应用链路、相机、串口或 ROS 节点。

## 已完成

| 检查 | 结果与证据 |
| --- | --- |
| Python 编译 | PASS；`scripts/check.ps1` 对 src 执行 compileall |
| 人工数据单元测试 | **43 项 PASS**；原始输出 `artifacts/unittest.txt` |
| 与板端 Python 3.10 语法兼容 | PASS；实际应用源码使用 AST feature_version 3.10 检查 |
| 不导入旧节点架构 | PASS；应用源码无 rclpy、ai_msgs、obj_detect、qrc_skandier、framework 导入 |
| Bash 启动/构建脚本 | PASS；Git Bash `bash -n`，只检查语法 |
| HMI JavaScript | PASS；`node --check artifacts/hmi.js`，没有启动服务器 |
| package.xml | PASS；XML 解析 |
| setuptools 包构建/wheel | PASS；`artifacts/package_build.txt`；wheel 内全部应用源码与当前字节一致，含 HMI HTML、ament marker、package.xml 和 launch |
| 旧相机对照配置结构 | PASS；`--check` 返回 0，不检查或打开硬件 |
| 新设备默认配置 | EXPECTED CONFIG_INVALID，返回 2；缺相机路径、雷达路径、外参确认和地图标定，不能启动 |
| 资产来源 | PASS；六份资产对应板端 SHA256，见 `assets/manifest.json` |
| 现有用户修改 | PASS；三份旧物块文件 SHA256 与清理任务保护清单一致 |

机器可读汇总在 `artifacts/validation.json`。wheel 在 `artifacts/wheels/`，只用于核对包装完整；部署仍需项目 config/assets，不能只装 wheel 就启动。setuptools 对直接 setup.py 调用有弃用提示，构建成功；板端 ament_python 构建是另一项待验证结果。

测试覆盖：MCU 噪声/分段/嵌套命令、地图校验、坐标小端编码、二维码原文、部分写入；三次扫描票数和次序、过期/重复帧、扫描及命令超时、迟到数据不延长窗口、歧义拒绝；写入失败阻止切换和统一清理；work/debug 模拟输入业务动作一致；相机失败释放、尺寸不符拒绝、线程未退出禁止开下一相机；C1 描述符、RPM 请求校验、点流同步及分段、坐标方向、圆周聚类；标靶过滤、RGB 增益与 NV12 排列、CHW/NHWC DFL 与错误 cls 标签；配置继承和标定门重叠；HMI 最近事件有界、断开的未激活相机流释放客户端名额。

## 板端只读证据

SSH 到 `root@192.168.137.14` 成功，hostname 为 `RDKx5-Monthwu-01`，架构 aarch64，Python 3.10.12。只读模块可发现性检查：numpy、cv2、serial、hobot_dnn 存在，pyzbar 未找到；colcon 为 `/usr/local/bin/colcon`。这些结果不是导入、模型加载或硬件运行测试。QR 默认使用 OpenCV，pyzbar 仅可选回退。

读取板端 MCU 协议确认小写 `[ack]`，只读查询并核对模型和标定 SHA256。本次没有板端文件写入，没有设备进程启停，没有 ROS2 ARM64 构建。

## 待实物确认

新相机路径/尺寸/实际 FPS、模型在新成像条件的准确性、BPU 实际六输出缓冲布局、QR 检测覆盖率、C1 查询/转速/频率/点数/断流、外参与三阶段朝向及地图标定、MCU 实际收发与物理动作、HMI 显示及并发、信号关闭和 V4L2 read 阻塞行为。

Windows 本地测试及包构建不能替代这些验收。运行前按 CONFIGURATION.md 补齐参数，部署与启动遵守 database.md 和 deployment.md。
