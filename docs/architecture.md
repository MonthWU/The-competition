# 项目架构

2026-11-25 用户要求更换桌面 model.bin 并 Git 上传。本地物块模型现为 bedffa66d94d8434cade9a42dd3937f135d416857633d632a63a2af71838457f（10490514 字节），类别索引为 redblock、yellowblock、greenblock、blackblock、lightblueblock、darkblueblock、targetOne、targetTwo、targetThree。已共同更新模型、类别绑定与资产清单，核对同哈希导出记录及同源 PT 类别元数据。此前旧模型说明为历史记录。板端 SSH 超时且 ARP 未发现设备，本轮尚未部署或实机验收。详情见 docs/MODEL_SWITCH_20261125.md。

2026-11-24旧模型类别顺序纠正：此前恢复旧权重时错误沿用11月14日新模型顺序。已找到artifacts/RDK_CJLU_V01/dnn内SHA256同为0af8e289007e0ba883fdd1a6bae6f6a7ec2f0821f850443d6180cc43167dd112的原模型及配套classes.names，顺序确认为targetOne、targetTwo、targetThree、red1、black1、green1、yellow1、blue1、blue2；V05/V06归档object.names亦一致。本次按配套文件重建索引映射，更新清单并新增按模型哈希检查顺序的回归测试。旧模型权重、曝光50、增益48及debug light_unable自启保持；此前“类别顺序沿用”的结论作废。证据和部署结果见artifacts/old_model_order_20261124。

2026-11-24用户重新明确要求“换成老模型”。本轮恢复11月14日换模前备份的物块模型，SHA256为0af8e289007e0ba883fdd1a6bae6f6a7ec2f0821f850443d6180cc43167dd112，大小3412759字节，与资产清单previous_manifest_sha256一致。同步更新模型类别哈希绑定及资产清单，输出类别顺序按当次换模明确保留的原classes.names顺序，蓝色语义和串口码不变。保持曝光50、增益48及debug light_unable自启；原子替换前完整备份，模型加载/离线推理、板端回归与启动检查后恢复既有服务。效果和MCU闭环另行验收，记录见artifacts/old_model_20261124。下文此前保持新模型的说明为历史记录。

2026-11-24用户取消模型切换，要求降低曝光改善圆盘可见性。本轮保持原模型及debug light_unable自启，将light_unable的exposure_time_absolute从150降为50（15ms→5ms），gain仍48、brightness仍128。实时画面观察圆盘轮廓及表面细节改善；配置持久化后重启既有服务并核对参数，不代表圆心精度或颜色准确率验收。备份、对比画面和实际命令见artifacts/exposure_lower_20261124。

2026-11-24用户要求开机自启动改为不补光、debug模式。本轮将car26-object.service持久启动参数设为debug light_unable，删除此前仅本次开机有效的90-debug.conf覆盖，保持enabled并重启既有物块服务。无补光参数为手动曝光150、gain48、brightness128；此选项选择相机参数，不控制补光灯供电。未重启板卡，冷启动验收仍待进行。备份、实际命令和运行核验见artifacts/autostart_debug_unlit_20261124。此条覆盖下文此前开机work及默认补光的历史记录。

2026-11-24部署完成：用户明确要求“部署板端”，本轮将指令分辨率切换、补光参数选择和仅debug启用HMI等41个差异文件同步至/root/Monthwu/GongChuang_vision_26。200项板端测试无跳过全部通过，ROS2重新构建、32个安装源码文件一致性以及两种补光参数共五组入口检查通过；本地13项网页协议测试通过。核对时原car26-object.service实际active，本轮停服部署后恢复work运行，enabled设置保留；只读核验相机/UART/BPU由该进程持有，NRestarts=0，8080无监听。未主动发送定位指令、运行debug实机或修改MCU；640×360与720p切换耗时、精度、FPS及MCU闭环仍待实机验收。以下“尚未部署/本地”段落保留为过程记录，本条为当前部署状态。完整备份：/root/Monthwu_archive/RDK_CJLU_V17_deploy_20261124_37aa46f8/project_before.tar.gz；SHA256：3436a48af158b599528d903f5d37e8654f6829e56a6862fe96181c74b170dab2。证据及实际命令见artifacts/deploy_board_20261124/REPORT.md与COMMANDS.md。

2026-11-24本地HMI改为仅debug启用；work不启动HTTP或编码网页JPEG。网页重置/任务切换生成与MCU相同的[qr_reset]/[task_qr]/[task_object]命令帧，经有界队列、CommandParser及主线程统一处理，UART两种模式均可控制；网页不等待MCU回环、不伪造UART读写。具体见HMI.md及MCU_SERIAL_PROTOCOL.md。未部署、未启动设备，本条覆盖下文Web常开与独立网页控制路径的历史描述。

2026-11-24本地新增补光参数接口：现有启动脚本接受light_able/light_unable，配置继承后选择物块相机UVC参数，重开block/disc/target均沿用。默认50/16/128，旧项目无补光150/48/128；不改变二维码及指令分辨率流程。尚未部署，接口和来源见LIGHTING_PROFILES.md。

2026-11-24最新本地规则（尚未部署）：用户确认平时实际采集640×360，YOLO仍补边至640×640；收到原始5B 61 2C 01 5D后切1280×720搜索圆盘，检出后只产生一次既定9字节回复并恢复640×360。收到原始5B 62 2C 01 5D后，冻结当时最近一次有效YOLO结果，切1280×720按其框映射的局部ROI精定位，期间暂停YOLO；不再按三靶角度条件自动进入。结束/取消及原有退出条件后恢复640×360。本轮板卡未联网，用户要求先完成本地修改，未部署、构建或运行板端。此条覆盖下文此前持续720p和0x62完全被动的历史说明。 当前三采集档位为block=640×360、disc/target=1280×720，单设备所有者与双缓冲保持；详见COMMAND_CAPTURE_MODES.md。

2026-11-24模型调用优化：models.object.overlap_capture=true，唯一BPU工作线程执行原模型，唯一相机线程在等待期间预取一张下一帧；任务及预取均有界，原read时间戳随帧保留。确认/精定位/重开切换丢弃预取并保留原门控。正常网页负载29.95FPS、压力29.74、恢复29.87；BPU仍约15.7ms，预取后额外等待约0.04ms。179项板端测试及6组36张量同输入对照通过。详见INFERENCE_OVERLAP.md；服务停止且设备已释放。

2026-11-24采集优化：物块相机增加buffer_count=2，启动核对驱动读回，缺省配置仍1；默认block/target共享双缓冲并保持单一设备所有者。720p/MJPG/30FPS及像素解码、分类模型保持。正常网页负载约22FPS、5秒窗口约21.8～22.2，高频完整状态压力下约17、解除后恢复22。3缓冲无明显增速且增加排队，未采用。约20ms短测图像交付年龄代价见CAPTURE_BUFFERS.md。测试结束，服务停止且设备释放。

2026-11-24分类解码优化：新增camera/candidates.py及native/yolo_candidates.cpp，C++保守筛选原始logit后仅对候选计算概率，保留原类别、DFL、NMS和坐标规则。169项板端测试及30组同张量对照通过。配对后处理6.75→2.37ms，完整FPS仍约14.97；实时后处理采样约3.08ms。三配置共用候选ARM库，详见CANDIDATE_POSTPROCESS.md。测试结束，服务停止且设备已释放。

2026-11-24后续实机测试：用户选择当前画面与FPS，720p MJPG设备回读30FPS，应用75秒窗口14.97FPS、均值66.66ms/帧，40秒补测同为14.97FPS。三标靶精定位未触发，本轮未验收圆心精度。测试后释放相机/UART，car26-object.service继续inactive，enabled设置保留。详见artifacts/720p_live_20261124/REPORT.md。

2026-11-24当前配置：按用户要求改为持续1280×720 MJPEG采集、请求30fps，block/target两模式均为720p。YOLO对完整画面缩至640×360并等比例补边为640×640识别；三标靶满足原门控后，在同一720p原图的三个局部ROI内精定位，期间暂停YOLO。模式切换不重开相机。YOLO偏差原点仍为(320,180)，精定位及圆盘原图偏差原点为(640,360)，使用720p原生像素。此条覆盖下文此前默认1080p的描述；历史实测数值保留。当前服务已按用户要求停止，本次不自动启动。

2026-11-22本次已部署：Web三分区及0x61确认后持续检测圆盘直到有效检出、回复一次9字节偏差的流程已同步至板端/root/Monthwu/GongChuang_vision_26。板端164项测试全部通过、无跳过，ROS2构建及两入口--check-files通过。部署前物块服务已停止，本次保持inactive，不启动相机、UART或雷达，实机及MCU接收待联调。下文此前“本地未部署”等说明为过程记录；本次备份及命令见deployment.md和artifacts/deploy_hmi_disc_20261122。

2026-11-22用户最新更正（本地未部署）：收到0x61确认后持续逐帧检测圆盘，直到有效检出才回复一次9字节偏差并停止；未检出或无效偏差继续检测，不设置搜索超时。未收到确认时不检测，YOLO继续。退出物料阶段或关闭应用取消搜索；此条替代下文“只检测一张原图”的描述，当前详细规则见DISC_ONCE.md。

2026-11-22最新圆盘规则（本地未部署）：正式0x61物料阶段仅收到MCU原始5B 61 2C 01 5D确认才对之后新采集的一张原图检测一次，成功回复9字节5B 61 2C dxLE16 2C dyLE16 5D，空闲不检测、不再持续搜索至5像素达标。YOLO继续。此次为状态通知增加明确的0x61确认例外，不强制切换Mission或标靶模式；其他状态仍只记录。接口与边界见DISC_ONCE.md，此条覆盖下文历史圆盘持续计算及状态通知完全被动的说明。

2026-11-22 Web三分区本地完成：二维码/物块窗口固定同时呈现，关闭或过期画面黑屏；TX/RX各保留独立原始记录；正式应用接入二维码结果重置和二维码/物块任务切换，HTTP只排队，由主线程执行。结果在扫码交接后保留，重置不改变当前任务。雷达/规划放入折叠区。尚未部署本次修改；接口与验证见HMI.md。此条覆盖下文此前“正式Application不注册重置回调”的历史说明。

2026-11-22类别映射修正：业务类别目录classes.name与模型输出索引分离；当前bin使用yolo11_x5.classes.json明确绑定索引和SHA256，按red/yellow/green/black/lightblue/darkblue/targetOne/targetTwo/targetThree解码。用户确认blue1浅蓝、blue2深蓝。串口语义码保持，详见MODEL_CLASS_MAPPING.md；下文早期将classes.name行号当作模型索引的说明由本条覆盖。

2026-11-22后续：正式0x61物块阶段同帧执行YOLO和白色大圆盘圆心计算；原始采集图上两轴绝对偏差均<=5像素时锁定，停止该阶段后续圆盘计算，YOLO继续。重新进入0x61重置；0x62三标靶精定位独立。配置disc_centering，状态及验证口径见DISC_CENTER.md；圆盘偏差发布HMI，不新增UART类型。

2026-11-22部署完成：新类别、分辨率框架及新串口协议已同步至板端/root/Monthwu/GongChuang_vision_26，并恢复部署前正在运行的car26-object.service。板端132项测试全通过、ROS2构建及--check-files通过，新版UART已观察到0x71物块包。MCU接收和物理效果待联调；下文旧“仅本地/尚未部署”描述为此前过程记录。

2026-11-22最终同步：二维码状态0x60/业务0x70，物块状态0x61/业务0x71，标靶状态0x62/业务0x72。类别以assets/models/classes.name的用户给定顺序为准，lightblueblock为浅蓝、darkblueblock为深蓝；合法二维码不含]，接收端在0x70类型后读至首个0x5D结束，正文中的[不是新帧头。本轮仅本地，尚未部署。

2026-11-22用户指定当前类型码：标靶0x72、物块合并包0x71、二维码0x70；状态通知0x60/0x61/0x62。仅本地修改，尚未部署；后续历史实测段落中的旧类型码仍代表当时版本。

2026-11-22后续物块发送改为单包全类别：0x71类型、单字节num（发送的物块种类数0～6），每种一条类别码及框中心相对画面中心的int16小端dx/dy，不发送整个框，不再仅取最近一块。同类异常重复候选取ROI内最高置信度，num=0明确发送当前空帧。标靶报文仍独立，精定位暂停YOLO时不伪发物块空包。完整格式见SERIAL_PROTOCOL.md，本轮仅本地验证。

2026-11-22依command.md及用户确认新增状态交流：所有上位机发往MCU报文使用`[`/`]`，包含二维码0x70；Flagnum=0x61物料转盘、0x62标靶，BOOL使用原始00/01。双端各状态独立记录，收到通知不控制相机或Mission。相机将真实模式进入/退出写入有界事件队列，与帧原子取出后由主线程发UART；关闭成功后发送退出。细节及二维码不含]的分帧规则见STATE_EXCHANGE.md。本轮仅本地，板端服务仍是此前版本。

2026-11-22本地工作区改名为GongChuan_vision2026，源码仍位于内部car26_new。项目名称变更不改变四模块、ROS包名及板端现有路径。

2026-11-22本地新增物块相机双功能分辨率框架：`cameras.object.recognition_modes.block`（物块识别）与`target`（标靶圆心定位）分别配置采集width/height/fps。默认两者均1920×1080@30，继续保持单流；物块YOLO处理图仍由cameras.object.width/height指定为640×360，并保留YOLO标靶粗定位作为自动进入精定位的门控。模式采集参数相同时仅切算法；不同时在同一相机线程内清空旧结果、释放设备、按目标配置重开、初始化曝光并等待稳定帧，结束/超时/丢失后恢复block配置。状态recognition_mode显示block或target，vision_mode保留yolo/circle兼容。标靶种子及框尺度按目标模式实际尺寸映射；同流强制等比例映射，不同流使用target_focus.mode_transform校正视场。缺省recognition_modes的旧配置仍使用公共capture_*。本次仅本地完成，未部署或切换板端现有服务；以下2026-11-21设备验证属于此前版本。

2026-11-21后续按用户要求，物块相机改为持续1920×1080采集，YOLO处理其640×360缩小图，精定位处理同一流的1080p原图。进入/退出精定位均不释放相机、不改曝光、不丢稳定帧；只有应用启动时初始化设备。报文source=0仍为640×360坐标，source=1仍为1080p偏差；模式视场映射改为同帧等比例映射，不使用旧跨分辨率mode_transform。状态capture_width/capture_height显示实际采集设置，width/height显示当前算法图像。板端106项测试和构建通过，实机YOLO约10～11处理帧/秒。后续实测见VALIDATION.md。

2026-11-21标靶检测修正已部署：YOLO输入等比例缩放并居中填充114灰边，解码时撤销补边和缩放还原原图坐标；物块、标靶的中心高度过滤整体删除，不再使用max_center_y_ratio。面积及重叠过滤保留。板端105项测试和ROS2构建通过；实机已由YOLO触发1080p圆心精定位并发送0x55。记录见VALIDATION.md最新节。

2026-11-21当前运行由`car26-object.service`管理，开机直接启动`start_coloritems.sh work`（物块及标靶识别，跳过二维码），HMI端口8080，串口`/dev/ttyS2`。用户已明确授权本项开机自启；服务源文件`service/car26-object.service`，异常退出5秒重试。启用状态及当前识别、JPEG预览已核验，尚未重启板卡做冷启动验收。下文2026-11-18预览/停止状态属于历史记录；服务管理与备份见deployment.md最新节。

2026-11-18后续增加camera/preview.py的QrPreviewState及HMI可选重置回调。独立扫码预览使用它维护可清空的内容、计数、时间及重置代次，拒绝重置前开始采集的旧帧；正式Application不注册重置回调，不改变任务状态机或MCU行为。界面增加成功结果区与重置按钮，前端防止重置前GET响应回填旧值。新版本板端97项人工/模拟测试及ROS2构建通过；当前依用户指令保留独立扫码预览，完整识别应用仍未启动。

2026-11-18本轮修改已部署至/root/Monthwu/GongChuang_vision_26。当前ARM原生库和ROS2构建、板端90项人工/模拟测试通过，应用保持停止；设备效果与MCU闭环另行验收。本地源码范围car26_new，appli及其它项目只读参考。

## 四模块

| 模块 | 位置 | 职责 |
| --- | --- | --- |
| 摄像头 | car26/camera | 唯一相机线程；扫码、YOLO、圆定位、过滤、预览及可选录像 |
| 雷达 | car26/radar | 唯一C1线程；查询、扫描解析和聚类 |
| 通信 | car26/communication | 主线程独占MCU UART；命令解析、编码和部分写入补齐 |
| HMI | car26/hmi | 读取有界快照，显示图像、通信、雷达和理论路径 |

application.py协调模块和资源，mission.py为无设备状态机，types.py定义不可变结果，telemetry.py保存有界状态、100条事件及各角色最新预览。planning.py和navigation.py是可选规划辅助，不新增运动报文。

## 分项目

无雷达配置projects/without_radar/config/board.json：entry=qr、radar.enabled=false、navigation.enabled=false。启动直接二维码→物块，不做预扫描、不等待起点或shot、不创建规划器，不需要route.json。

有雷达配置projects/with_radar/config/board.json：dynamic规划，地图在同目录route.json。设备device为空，外参未标定，地图confirmed=false，定位快照未指定；不填仿真真值或假参数。当前只实现理论规划及网页显示，未进行实车定位和运动验收。细节见TODO1_NAVIGATION.md。

根start.sh默认无雷达，支持without-radar或with-radar；start_coloritems.sh直接物块。两版共用源码、资产和原生库，路径以最终配置目录解析。

## 二维码

2026-11-22用户明确取消“扫码摄像头＋扫码器”双硬件需求（原TODO3/NR-09）。二维码阶段仅保留现有摄像头扫码，不接入额外扫码器。

打开QR相机 → OpenCV解码 → 必要时pyzbar回退 → 首个非空UTF-8结果 → 四次5B 70原文5D发送 → flush → 关闭QR相机 → 打开物块相机。

roi=null或缺省将完整图像交给两个解码器；显式[x,y,width,height]才裁剪，越界拒绝。物块ROI和地图ROI不限制扫码。状态显示有效qr_roi与vision_mode=qr。原新架构本已整帧，本次将默认配置和四角测试明确化。

成功扫码保存独立单槽事件，不被后续空帧覆盖；保留原帧序号和采集时间，主循环仍拒绝过期帧。非法UTF-8只拒绝当前内容，备用解码器继续检查空码或非法码之后的候选；不裁剪原文空格，也不恢复早期数字任务码规则。

四次write和flush成功才交接；写入失败不打开下一相机。主机发送不等于MCU接收，ACK、整体发送超时和长度上限仍待完善。

## 物块与标靶

2026-11-22新增白色大圆盘外缘圆心独立测试入口`start_disc_test.sh`：物块相机采集1080p，C++进行白色分割及可见弧椭圆拟合，HMI显示圆心和相对画面中心偏差。通过尺寸、弧覆盖及支持率排除夹具小孔，失败清空结果。详细算法、参数和图像中心定义见`DISC_CENTER.md`。

QR阶段不加载YOLO；物块角色打开时加载YOLO11 BPU模型。输入按比例缩放补边并在输出时还原坐标。颜色物块按面积、重叠和显式ROI过滤，各有效类别全部合包发送，不按参考点选取唯一物块；物块与标靶均不限制中心高度。

三类标靶各一个且几何有效时，拟合统一α，范围[-90°,90°)。YOLO输出标靶二640×360处理图中心差。|α|<=3°且标靶二唯一最近时，同一线程暂停全部YOLO，改用持续1080p采集流的原图定位三个圆心，更新α及标靶二偏差；不重开相机或重设曝光。缺圆不发送旧数据；角度超限、丢失或总超时、MCU结束/取消均恢复缩小图YOLO。

当前标靶为0x72十字节帧，颜色物块为0x41～0x46。0x54、标靶绝对坐标和两两连线角度属于旧版本。协议见SERIAL_PROTOCOL.md，流程见TODO2_ALIGNMENT.md。

## 雷达与规划

C1使用460800、8N1和Standard模式。原始顺时针角转换为车体+x向前、+y向左，经实测安装外参及同步定位转换到场地坐标。只沿确认的道路图搜索，并按障碍、车体外接圆和余量避碰。

定位缺失、过期、坐标系不符或外参未确认时撤下路线。保留任务内已观测阻断，不将缺少回波等同于道路清空；容量超限锁定无路线。实机定位生产者、自体回波过滤、车辆运动约束及真实场地仍待完善。

旧full入口WAIT_START→WAIT_SHOT→COLLECT及三次扫描地图上报保留兼容，不是新分项目动态规划。Windows simulation是独立Gazebo/Cartographer参考，不向实机注入出生点或真值。场地要求见SLAM_FIELD_CONSTRAINTS.md。

## 资源和验证

角色切换先关闭并join旧线程，再开新设备。read阻塞导致5秒关闭超时时报告CAMERA_STOP_TIMEOUT，不开下一相机；Python不能强制终止阻塞线程，保留为硬件验收项。

work与debug业务和报文相同，前者诊断留档、后者显示终端，不保证相同帧率。SIGINT/SIGTERM释放所持资源。采集时间是主机read前的单调时刻，不是曝光或精确硬件同步证据。

当前检查见VALIDATION.md。旧架构和协议文档逐字保存在history/20261118_before_consolidation，历史内容不覆盖本页。
