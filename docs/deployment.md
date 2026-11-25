# 构建、部署与交付

2026-11-25????????model.bin?Git?????????????SHA256 bedffa66d94d8434cade9a42dd3937f135d416857633d632a63a2af71838457f?10490514?????????redblock?yellowblock?greenblock?blackblock?lightblueblock?darkblueblock?targetOne?targetTwo?targetThree??????????????????????????????PT????????????artifacts/model_switch_20261125/before?????SSH???ARP????????????????????????????????????????????????GitHub?????????docs/MODEL_SWITCH_20261125.md?

2026-11-24旧模型类别顺序纠正：此前恢复旧权重时错误沿用11月14日新模型顺序。已找到artifacts/RDK_CJLU_V01/dnn内SHA256同为0af8e289007e0ba883fdd1a6bae6f6a7ec2f0821f850443d6180cc43167dd112的原模型及配套classes.names，顺序确认为targetOne、targetTwo、targetThree、red1、black1、green1、yellow1、blue1、blue2；V05/V06归档object.names亦一致。本次按配套文件重建索引映射，更新清单并新增按模型哈希检查顺序的回归测试。旧模型权重、曝光50、增益48及debug light_unable自启保持；此前“类别顺序沿用”的结论作废。证据和部署结果见artifacts/old_model_order_20261124。

2026-11-24用户重新明确要求“换成老模型”。本轮恢复11月14日换模前备份的物块模型，SHA256为0af8e289007e0ba883fdd1a6bae6f6a7ec2f0821f850443d6180cc43167dd112，大小3412759字节，与资产清单previous_manifest_sha256一致。同步更新模型类别哈希绑定及资产清单，输出类别顺序按当次换模明确保留的原classes.names顺序，蓝色语义和串口码不变。保持曝光50、增益48及debug light_unable自启；原子替换前完整备份，模型加载/离线推理、板端回归与启动检查后恢复既有服务。效果和MCU闭环另行验收，记录见artifacts/old_model_20261124。下文此前保持新模型的说明为历史记录。

2026-11-24用户取消模型切换，要求降低曝光改善圆盘可见性。本轮保持原模型及debug light_unable自启，将light_unable的exposure_time_absolute从150降为50（15ms→5ms），gain仍48、brightness仍128。实时画面观察圆盘轮廓及表面细节改善；配置持久化后重启既有服务并核对参数，不代表圆心精度或颜色准确率验收。备份、对比画面和实际命令见artifacts/exposure_lower_20261124。

2026-11-24用户要求开机自启动改为不补光、debug模式。本轮将car26-object.service持久启动参数设为debug light_unable，删除此前仅本次开机有效的90-debug.conf覆盖，保持enabled并重启既有物块服务。无补光参数为手动曝光150、gain48、brightness128；此选项选择相机参数，不控制补光灯供电。未重启板卡，冷启动验收仍待进行。备份、实际命令和运行核验见artifacts/autostart_debug_unlit_20261124。此条覆盖下文此前开机work及默认补光的历史记录。

2026-11-24部署完成：用户明确要求“部署板端”，本轮将指令分辨率切换、补光参数选择和仅debug启用HMI等41个差异文件同步至/root/Monthwu/GongChuang_vision_26。200项板端测试无跳过全部通过，ROS2重新构建、32个安装源码文件一致性以及两种补光参数共五组入口检查通过；本地13项网页协议测试通过。核对时原car26-object.service实际active，本轮停服部署后恢复work运行，enabled设置保留；只读核验相机/UART/BPU由该进程持有，NRestarts=0，8080无监听。未主动发送定位指令、运行debug实机或修改MCU；640×360与720p切换耗时、精度、FPS及MCU闭环仍待实机验收。以下“尚未部署/本地”段落保留为过程记录，本条为当前部署状态。完整备份：/root/Monthwu_archive/RDK_CJLU_V17_deploy_20261124_37aa46f8/project_before.tar.gz；SHA256：3436a48af158b599528d903f5d37e8654f6829e56a6862fe96181c74b170dab2。证据及实际命令见artifacts/deploy_board_20261124/REPORT.md与COMMANDS.md。

2026-11-24HMI串口交互仅本地：debug才启动HmiModule，work不监听HTTP、不编码网页JPEG；独立圆盘工具传入同一运行模式。网页与UART的二维码重置及任务选择统一协议帧、解析器、队列和主线程处理，MCU在work同样可交互。无新增节点、原生库或启动脚本。后续部署须同步application.py、communication/protocol.py、camera/module.py、hmi/module.py、hmi/index.html、cli.py及scripts/disc_preview.py，构建ROS2安装副本并按授权验证debug有网页/work无监听及串口控制。应连同此前未部署的补光/指令分辨率修改核对完整差异。回归、清单和实际命令见artifacts/hmi_serial_20261124；协议详见MCU_SERIAL_PROTOCOL.md与HMI.md。

2026-11-24补光接口仅本地完成：既有start.sh、start_coloritems.sh及两分项目start.sh均经scripts/start.sh接受light_able/light_unable（默认light_able），后者传给CLI的--lighting。更新配置board.json、Python config.py/cli.py和共享脚本，无新入口、依赖或原生库。部署时应连同当前指令切分辨率版本一并核对差异并备份，同步对应文件、构建ROS2安装副本，再执行两补光模式--check/--check-files；本轮未部署或开启相机。参数来源、语法与验证见LIGHTING_PROFILES.md；修改前备份及交付记录在artifacts/lighting_profiles_20261124/。

2026-11-24本地新增指令切分辨率流程，尚未部署：平时640×360，0x61圆盘及0x62标靶进入时1280×720，结束恢复640。无新原生库或启动入口。189项本地检查179通过、10板端环境跳过；当前新模式切换耗时、FPS和视场未实测，不能沿用此前持续720p性能作为本版验收。未来部署文件清单见artifacts/command_modes_20261124/manifest.json，届时先核对板端当前文件、完整备份，再限定同步、板端构建和检查；不要复制本地仿真/虚拟磁盘/快照目录。本轮用户要求只完成本地。

2026-11-24模型调用优化部署：detector/module/config及system.json、推理重叠测试同步，models.object.overlap_capture=true由三种入口继承；无新原生库或依赖。179项板端测试、Python/ROS2构建及两入口--check-files通过，安装源码一致；正常负载29.95FPS、压力29.74、恢复29.87。完整备份/root/Monthwu_archive/RDK_CJLU_V15_infer_20261124_e8f46567/project_before.tar.gz，SHA256为f253295fedaca8cc8ad2e2384e946f61e0914d099a1889a5d6d64e196ffba8f3。报告和命令见artifacts/infer_opt_20261124/REPORT.md及COMMANDS.md。最终服务停止、设备释放。

2026-11-24双缓冲采集交付：config/board.json物块buffer_count=2；module/modes/config及生命周期测试同步板端，172项板端测试全部通过，Python/ROS2构建及两入口--check-files通过，安装源码一致，无新原生依赖或启动入口。V14完整备份/root/Monthwu_archive/RDK_CJLU_V14_capture_20261124_628bff32/project_before.tar.gz，SHA256为1ce16b3efbc6937b1177fc44013b41ccfde62e8fec5c94657c48bf7995c238e4。正常网页负载约22FPS、压力约17FPS、恢复22；服务停止。完整命令及延迟代价见artifacts/capture_opt_20261124/REPORT.md和COMMANDS.md。

2026-11-24新增分类候选原生库：先bash scripts/build_candidates.sh生成native/lib/libcar26_candidates.so，再bash scripts/build.sh。models.object.candidate_library在独立物块/有雷达/无雷达均解析到该库；--check-files核对文件存在。169项板端测试、30组同张量对照和两个入口检查通过，安装Python与源码一致。完整备份/root/Monthwu_archive/RDK_CJLU_V13_candidates_20261124_c8191d70/project_before.tar.gz，SHA256为45898741d5f06ce60ef2c4e70e1f04a0888d72172549e85b12f4c07ed578ceca。报告及实际命令见artifacts/candidate_opt_20261124/REPORT.md和COMMANDS.md；服务保持停止。

2026-11-24后续有界实机验证完成：720p工作格式回读及两窗口约14.97FPS通过，未改业务代码或配置，无需重建。辅助采样脚本首轮获取PID失败，经修正后40秒补测成功；测试后原服务仍inactive、相机/UART无占用，自启enabled保留。报告和实际命令见artifacts/720p_live_20261124/REPORT.md与COMMANDS.md。

2026-11-24本轮720p交付：采集及block/target配置统一1280×720@30，YOLO仍为640×360全图处理并补边至模型640×640，精定位复用原有C++局部ROI算法。未改模型或原生库源码，无需重编未变原生库；检查模拟测试、ROS2构建、安装配置及入口--check-files。服务按用户要求停止，不启动设备。完整板端备份/root/Monthwu_archive/RDK_CJLU_V12_720p_20261124_44306711/project_before.tar.gz，SHA256为5f1f5cc9360dea5a01e906690f9eae8f9600f8735bb5180a40c8ad0e61d7b587。实际同步、检查和命令见artifacts/720p_20261124/RESULT.md与COMMANDS.md。

2026-11-22本次已部署：Web三分区及0x61确认后持续检测圆盘直到有效检出、回复一次9字节偏差的流程已同步至板端/root/Monthwu/GongChuang_vision_26。板端164项测试全部通过、无跳过，ROS2构建及两入口--check-files通过。部署前物块服务已停止，本次保持inactive，不启动相机、UART或雷达，实机及MCU接收待联调。下文此前“本地未部署”等说明为过程记录；本次备份及命令见deployment.md和artifacts/deploy_hmi_disc_20261122。

2026-11-22最新本地更正：收到0x61确认后持续检测至有效检出，再发送一次9字节回复并停算；不再是只检测一帧。修改camera/module.py及回归测试/文档，复用原C++库与串口编码，无新入口或依赖。未部署或启停板端；详情及本次日志见DISC_ONCE.md。

2026-11-22圆盘单次请求本地交付：修改communication/protocol.py、communication/module.py、camera/module.py、types.py、application.py及测试/文档。无需重建未变的C++圆盘库，没有新增启动入口或设备运行。本地163项检查零失败、9项环境跳过；源码快照、命令和日志见DISC_ONCE.md。尚未部署，后续需同步全部上述Python文件、构建ROS2，并同步MCU的9字节0x61偏差回复解析，旧5字节状态解析不能直接作为新回复解析器。

2026-11-22 Web三分区仅本地完成，尚未部署：新增HMI.md及test_hmi_controls.py；修改application、camera/module、communication/module、telemetry及hmi模块/页面。157项检查零失败（9项因本机环境跳过），浏览器模拟检查通过。未改原生C++库或启动入口；部署时需同步上述源码与index.html并按本文执行Python/ROS2检查，再依对应授权切换既有服务。历史板端验证不代表本版已生效。

2026-11-22类别重映射：新增assets/models/yolo11_x5.classes.json及camera/classmap.py，将实际输出顺序与classes.name业务目录分离，启动和--check-files核对模型SHA256。当前bin未改变，复用ARM原生库；需同步配置中的class_map路径并构建ROS2，恢复car26-object.service后核对实机标签。备份/root/Monthwu_archive/RDK_CJLU_V10_classmap_20261122_88872cf1/project_before.tar.gz，SHA256为757657a46d96e1a251dc154e61a43cdb30585cbcadaeac0cfee0dbb3e608642c。实际命令和结果见artifacts/classmap_v2_20261122。

2026-11-22后续交付：0x61正式流程同帧运行YOLO和圆盘对准，原始采集图abs(dx)、abs(dy)均<=5后停止圆盘计算，保留YOLO；进入新0x61重算。使用既有start_coloritems.sh/car26-object.service，复用未变的libcar26_disc.so，本轮不重编未变原生库。首次备份/root/Monthwu_archive/RDK_CJLU_V09_disc_yolo_20261122_be7f520c/project_before.tar.gz，SHA256为336d0a160825d60a44418fadcd8b5c1c6f9553a32cab6ee0b3a03a29b5eec27a。验证、运行及实际命令见artifacts/disc_yolo_20261122，细节见DISC_CENTER.md。

2026-11-22部署完成：新类别、分辨率框架及新串口协议已同步至板端/root/Monthwu/GongChuang_vision_26，并恢复部署前正在运行的car26-object.service。板端132项测试全通过、ROS2构建及--check-files通过，新版UART已观察到0x71物块包。MCU接收和物理效果待联调；下文旧“仅本地/尚未部署”描述为此前过程记录。

本轮备份：`/root/Monthwu_archive/20261122_protocol_f8190173/project_before.tar.gz`；SHA256：`eb8419fbcd3a1740f56a93de8e28390e9845e644efdbd5bbded1139dbf08487d`。实际脚本、命令与检查输出在本地artifacts/deploy_protocol_20261122；板端部署元数据和检查输出在该备份同目录。新增artifacts/COLCON_IGNORE，避免隔离测试副本被colcon发现为重复包。原生源码与模型哈希未变，复用现有ARM原生库并通过其测试；未传Windows库。

2026-11-22最终同步：二维码状态0x60/业务0x70，物块状态0x61/业务0x71，标靶状态0x62/业务0x72。类别以assets/models/classes.name的用户给定顺序为准，lightblueblock为浅蓝、darkblueblock为深蓝；合法二维码不含]，接收端在0x70类型后读至首个0x5D结束，正文中的[不是新帧头。本轮仅本地，尚未部署。

2026-11-22用户指定当前类型码：标靶0x72、物块合并包0x71、二维码0x70；状态通知0x60/0x61/0x62。仅本地修改，尚未部署；后续历史实测段落中的旧类型码仍代表当时版本。

## 2026-11-22 物块全类别合并包（本地待部署）

物块发送改为0x71类型单包：单字节num表示实际发送0～6种，每种为类别码及检测框中心相对画面中心的dx/dy（int16小端），没有框四角或宽高。包长4+5×num，num=0仍发送当前空帧。物块不再仅取最近一块，同类重复异常取最高置信度，显式ROI保留。标靶姿态仍独立发送。

源码改动为communication/protocol.py及communication/module.py，新增/更新测试和协议文档，无新入口、依赖或原生库改动。本地126项测试：124通过、2跳过。备份project_before.tar、实际命令和测试输出在artifacts/all_blocks_20261122_141149。尚未部署、构建ARM/ROS2或启停设备；后续需同时同步MCU的0x71合并包解析、num种类语义及int16偏差，旧7字节单物块绝对坐标接收逻辑不再适用。

## 2026-11-22 状态交流及统一帧头尾（本地待部署）

新增0x61/0x62状态通知，线上格式为原始字节`5B Flagnum 2C BOOL 5D`，BOOL只允许00/01。状态集合支持嵌套与独立退出，收到通知只记录进度，不能强制控制任务。相机线程记录有界模式事件，主线程按帧快照顺序发送UART；正常停止先关闭相机再发送退出。所有上位机报文（含二维码）改为`[`/`]`，原二进制载荷保持。协议细节与二维码不含]、正文[不重置的分帧规则见STATE_EXCHANGE.md。

本轮新增camera状态事件接口、通信状态解析/发送及本地回归测试，无新启动入口、依赖或原生库改动。源码快照及实际命令位于artifacts/protocol_brackets_20261122_135850。本地119项测试：117通过、2板端专用跳过；尚未部署、构建ARM/ROS2或启停服务。后续部署须连同application.py、camera/module.py、communication/protocol.py和communication/module.py一致同步；MCU也须同步帧头尾、固定长度解包与状态通知语义，旧FF/FE接收程序不能直接验收本版本。

## 2026-11-22 本地项目改名

当前本地路径D:/AIworkplace/RDKx5/GongChuan_vision2026/car26_new，工作区名称GongChuan_vision2026。内部car26_new、car26_system及板端/root/Monthwu/GongChuang_vision_26保持现状。全部文件已迁移，旧26GongChuan_vision仅余被Windows进程占用的空目录。管理索引和当前本地引用已同步；历史artifacts、审计状态及只读appli内部记录保留原文。本轮备份、迁移文件清单及执行脚本在artifacts/rename_GongChuan_vision2026_20261122。

## 2026-11-22 双功能分辨率框架（本地待部署）

编辑config/board.json的cameras.object.recognition_modes：block与target各有width、height、fps。默认均1920×1080@30，即使以后统一分辨率也保留两份模式入口；可分别调整，例如block为1280×720@30、target为1920×1080@30。两种功能共用设备、曝光、模型及相机线程，设备参数相同就不重开。旧capture_*仅作为未配置模式字段时的回退；显式模式字段优先。物块算法处理图仍使用外层width/height，避免切换采集尺寸改变物块过滤及报文尺度。

不同采集模式切换会重开并等待target_focus.settle_frames（设置UVC时至少3帧），仅通过尺寸检查且稳定后的当前帧才发布；硬件必须实际支持所选尺寸、帧率与MJPG/YUYV。标靶精定位图及source=1偏差采用target采集尺寸，调整target尺寸后需同步核对MCU像素尺度、圆半径参数及mode_transform视场标定。source=0仍使用外层width/height。两份默认配置保持原报文尺度。

本地scripts/check.ps1执行Python编译和109项测试，107项通过、2项跳过（板端Bash入口和ARM原生库）。已用模拟设备验证相同尺寸不重开、720p→1080p→720p恢复、非1080p种子映射、配置拒绝及重开失败清空旧结果。备份和实际命令见artifacts/recognition_modes_20261122_133914。未执行SSH、板端部署、ROS2/ARM构建或设备运行，实机切换耗时和曝光稳定性待板端验证。

## 2026-11-21 持续1080p物块采集（当前版本）

后续实测模式切换：进入精定位225.69～280.87毫秒（4次，均值248.98毫秒），返回YOLO83.22～89.04毫秒（3次，均值86.18毫秒）。按切换开始至首个有效结果的处理循环结束计时；采集始终1920×1080，实测记录见VALIDATION.md最新节和本地artifacts/fixed1080_20261121/switch-measurement.txt。

物块相机只打开一次，capture_width=1920、capture_height=1080、capture_fps=30；board.json的width=640、height=360用于YOLO处理图和其坐标。精定位使用同一流的原图，进入/退出均不重开相机、不重设UVC参数、不额外等待3帧。启动仍保留UVC设定回读及稳定帧。HMI的capture_*与算法width/height分开显示，预览随算法分别为640×360或1920×1080。

仅修改camera/module.py、config.py、config/board.json、tests/test_target_focus.py及项目文档。完整板端备份为`/root/Monthwu_archive/20261121_180309_fixed1080/project_before.tar.gz`；SHA256：`bf1bb92a0057429d528187fc48968590d52d46b0a189c24cccbc29433907e639`。部署停启现有car26-object.service，板端106项测试无跳过、compileall、ROS2构建及物块入口--check-files通过，无需重建未修改的原生库。

实机单流45秒观察通过，YOLO约10～11处理帧/秒；该数值不是30FPS采集参数或预览参数。精定位新增focus_ready/yolo_ready事件及transition_ms，用于测首个有效结果延迟；不能将旧2.5秒换分辨率测量当作本版本耗时。模型、协议、曝光值不变；source=0与source=1像素尺度保持原约定。记录在本地artifacts/fixed1080_20261121。

## 2026-11-21 标靶检测修正

已修正YOLO的640×360直接拉伸问题：改为等比例补边，输出框去掉补边并还原原图像素；按用户指令整体删除中心高度过滤及max_center_y_ratio配置。模型、25%置信度阈值、面积和重叠规则未变。

备份：`/root/Monthwu_archive/20261121_174526_target_geometry_fix/project_before.tar.gz`；SHA256：`4548c7487c285c16212c5417febe2de3f88400be4c3f99077ff2175ff1f1a97e`。停止现有服务后限定同步源码/配置/测试，板端compileall、105项测试（无跳过）、ROS2构建、物块入口--check-files通过，再恢复服务。无需重建未修改的C++原生库。

当前实机已识别三个标靶、自动进入1920×1080圆心精定位并发送0x55。精定位时YOLO暂停，检测数为0是该模式的既定状态，不表示重新漏检；画面显示圆及α、dx/dy。设备精度和MCU接收另行验收。源码回滚需先停止此服务，恢复该备份中的本次改动文件和配置，构建后启动；不恢复其它项目或修改系统串口。

本地命令、部署与画面证据：`artifacts/target_fix_20261121/`。

## 2026-11-21 物块识别开机自启动（当前配置）

用户明确要求“为物块检测配置开机自启动”。板端已安装并启用`car26-object.service`，服务源文件为`service/car26-object.service`，安装位置为`/etc/systemd/system/car26-object.service`，由`multi-user.target`拉起。

启动入口为`/bin/bash /root/Monthwu/GongChuang_vision_26/start_coloritems.sh work`：直接物块识别及标靶精定位，跳过二维码；串口`/dev/ttyS2`、115200、8N1，HMI为`http://192.168.137.14:8080/`（IP以实际网络为准）。异常退出后5秒重试，设备未就绪时由重试等待；手动停止服务不会触发自动重启。正常退出码0也不会自动重试。停止时SIGTERM清理资源，最多等待15秒。

安装前完整备份：`/root/Monthwu_archive/20261121_173351_object_autostart/project_before.tar.gz`；SHA256：`adfd17d72f1d5ec4f7bacb68bd7a2e86bff9535b47191742b94da884db360d90`。本次停止原手动进程5899，再启动systemd服务，避免相机及串口两个所有者。服务启用状态、应用启动、HTTP与JPEG流通过检查；未重启板卡验证冷启动，不将其表述为已完成断电开机验收。

实际安装脚本与输出位于本地`artifacts/object_autostart_20261121/`。没有改业务源码、模型或协议，未重编译ROS2或原生库。下文2026-11-18的停止/无自启记录为历史状态。

后续管理命令（本段为操作说明，并非本次全部已执行）：

```bash
systemctl status car26-object.service --no-pager
systemctl stop car26-object.service
systemctl start car26-object.service
journalctl -u car26-object.service -n 50 --no-pager
# 取消自启并停止识别：
systemctl disable --now car26-object.service
```

服务持有设备时，不再并行执行手动启动入口。完整备份恢复属于另一次部署操作，按对应授权执行；仅取消自启可使用上述disable命令，无需恢复整个项目。

2026-11-18后续二维码预览重置键已部署。独立扫码预览使用`PYTHONPATH=src/car26_system python3 -B artifacts/qr_preview_20261118.py`，默认最多一小时；必须取得对应相机运行授权，且不能与正式应用或其它8080预览同时启动。该脚本仅相机、解码及HMI，不打开UART、雷达或物块相机。页面POST /api/qr/reset只重置该预览结果，正式Application默认没有此端点。当前新版本ROS2构建和板端97项人工/模拟测试通过；本轮因用户正在浏览扫码而保留预览运行，不把下文此前“应用停止”状态当作当前设备状态。备份与验证见VALIDATION.md最新节。

2026-11-18本轮全部业务修改已部署至/root/Monthwu/GongChuang_vision_26，主机RDKx5-Monthwu-01 / aarch64；本地源项目为car26_new。应用保持停止，无新增自启或常驻服务。启动示例不是本轮实际运行记录。

## 入口

白色大圆盘圆心测试：`bash scripts/build_disc.sh`构建专用ARM库，`bash start_disc_test.sh --check`检查配置及设备路径，`bash start_disc_test.sh --mode debug`开启物块相机圆心预览，默认一小时、Ctrl+C退出。`--image 路径 --output 路径`离线检测不开设备。参数在config/disc.json，预览`/camera/object`，不创建UART或YOLO；使用前停止占用相机或8080的其它程序。算法、备份和验证见DISC_CENTER.md及artifacts/disc_center_20261122。

独立二维码手动测试入口：`bash start_qr_test.sh`。使用已部署的`artifacts/qr_preview_20261118.py`，仅开启扫码与带重置按钮的HMI，前台最多运行一小时，Ctrl+C关闭；不创建UART或物块模块。`bash start_qr_test.sh --check`只检查依赖、配置与QR设备路径。已有正常独立扫码预览时提示复用；其它程序占用端口时退出并提示手动停止物块服务，不自动更改服务状态。测试结束需恢复物块识别时执行`systemctl start car26-object.service`。脚本与既有预览配套放在板端`/root/Monthwu/GongChuang_vision_26`；不能只把此脚本复制到没有项目源码的目录运行。

| 命令（板端项目目录内） | 用途 |
| --- | --- |
| `bash start.sh [work\|debug]` | 默认无雷达二维码→物块 |
| `bash start.sh without-radar [work\|debug]` | 显式无雷达，不创建规划器 |
| `bash start.sh with-radar [work\|debug]` | 动态理论规划；当前设备等配置缺失 |
| `bash projects/without_radar/start.sh [work\|debug]` | 无雷达分项目入口 |
| `bash projects/with_radar/start.sh [work\|debug]` | 有雷达分项目入口 |
| `bash start_coloritems.sh [work\|debug]` | 独立物块及标靶精定位 |
| `scripts/start.sh 配置路径 [work\|debug]` | 公共CLI包装 |
| `scripts/build_focus.sh` | 只构建ARM OpenCV共享库 |
| `scripts/build_candidates.sh` | 只构建ARM YOLO候选筛选库，不启动设备 |
| `scripts/build.sh` | source TROS并构建car26_system |
| `scripts/check.ps1` | Windows编译及人工/mock测试 |

默认work；debug只改变诊断去向。追加--check或--check-files不开设备、不创建Application；后者检查模型、原生库和设备路径。work日志位于最终配置目录的上级项目log/runtime：默认无雷达为projects/without_radar/log/runtime，独立物块为根log/runtime。

无雷达只有config/board.json，不需要route.json；有雷达保留route.json、定位快照和标定接口。

## 同步与依赖

共同交付src、config、assets、native源码、scripts、projects、docs、tests、AGENTS、README和根入口。共享源码、模型及库各一份，相对路径以最终配置目录解析，不单独复制子项目。

不传Windows/WSL仿真、历史大备份、build/install、缓存或本地二进制库。本轮初始差异34文件；资产和原生源码哈希与板端一致，保留模型并在板端重编译原生库，随后补充整理文档及证据。

依赖Python3、numpy、OpenCV/V4L2、pyserial、hobot_dnn.pyeasy_dnn、g++和OpenCV开发库；pyzbar可选。ROS2构建使用/opt/tros/humble/setup.bash及colcon，直接Python启动不要求source TROS。

## 本轮已执行

1. 核对主机、目录、authority文档及设备无占用。
2. 完整备份至/root/Monthwu_archive/20261118_all_changes_be13ca07/workspace_before.tar.gz，19466929字节，SHA256为ac507f657fdc918b5f0defab3bc1f7e87d678e3ee7a621557530594474591557。
3. 限定差异文件原子同步及内容哈希核对，保留未变模型。
4. build_focus.sh、build.sh、Python compileall通过，ARM原生库及car26_system构建成功。
5. 设置CAR26_FOCUS_LIBRARY后运行板端90项人工/模拟测试，全部通过，无跳过。
6. 默认无雷达、显式无雷达debug和独立物块--check-files通过。有雷达--check按预期返回2及RADAR_DEVICE_REQUIRED。
7. 整理项目文档、逐字归档旧文档，同步证据并核对最终内容。应用及设备保持空闲。

本轮不打开相机、UART、雷达或真实HMI服务。此前独立扫码识别134+213+314+321；当前0x55的MCU接收与完整任务仍未验收。

## 检查命令

```bash
cd /root/Monthwu/GongChuang_vision_26
bash scripts/build_focus.sh
bash scripts/build.sh
PYTHONPATH=src/car26_system python3 -m compileall -q src
CAR26_FOCUS_LIBRARY="$PWD/native/lib/libcar26_focus.so" PYTHONPATH=src/car26_system:tests python3 -B -m unittest discover -s tests -v
bash start.sh --check-files
bash start_coloritems.sh debug --check-files
```

上述命令只构建及模拟检查。实际执行记录、manifest及结果在artifacts/deploy_20261118。

## 回滚和待验收

回滚前取得对应停止授权，核对备份SHA256，只恢复本项目目录并重建原生库和ROS2包，不覆盖其它项目，不恢复Windows库或旧工作区配置。

后续分别验收不同距离/光照/边缘位置扫码、相机切换及阻塞关闭、YOLO和圆定位误差、0x55 MCU解析、C1外参和定位同步、动态路线与真实运动。主机写入及静态构建不能替代物理结果。整理前文档保存在history/20261118_before_consolidation。

## 2026-11-22 Web及确认搜索部署完成

最终备份：/root/Monthwu_archive/RDK_CJLU_V11_hmi_disc_20261122_3ae03ee3/project_before.tar.gz。SHA256：74dc3c1df05ce9b727791f2c13b133053bc6502c83be72091a3426c47451ada0。首次安装页面哈希检查失败已自动回滚；原因是板端系统时钟为2000年，旧构建文件时间为2026年，增量构建错误沿用缓存。未修改系统时间；重新完整备份后，将本包build/install缓存移至备份目录再构建，校验安装后的Python及HTML与源码一致。服务保持原停止状态，自启enabled设置未改。
