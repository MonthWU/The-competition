# Web 人机交互

## 2026-11-24 当前接口（本地，未部署）

仅debug且hmi.enabled=true时启动Web服务；work不监听HTTP、不创建Web线程，也不编码相机网页预览JPEG。配置中的hmi.enabled=true不能越过work限制。独立圆盘诊断入口同样遵守debug门控。

用户确认网页与下位机共用串口协议和执行入口，网页点击立即执行，不需要实际UART往返或MCU回传。网页两个业务按钮统一编码为通信协议帧，主线程用CommandParser解析后执行；串口接收同名命令进入同一队列和处理函数，work/debug均可用。HTTP线程只排队，不操作相机或MCU设备。

| 业务动作 | ASCII串口命令帧 | 作用 |
| --- | --- | --- |
| 重置二维码结果 | `[qr_reset]` | 清空结果、计数及时间，设置旧帧截止时间，不切换任务 |
| 切换到二维码 | `[task_qr]` | 先关闭旧相机再进入二维码；已经在二维码时不重开 |
| 切换到物块 | `[task_object]` | 先关闭旧相机再进入物块；已经在物块时不重开 |
| 按当前状态切换 | `[task_switch]` | 与旧网页切换按钮相同；入队时固定为上述明确目标之一 |

网页的“切换当前任务”按钮自动按当前任务编码task_qr或task_object，避免排队期间状态变化导致目标漂移。下位机建议发送明确目标命令，重复帧不会反复开关相机。ASCII命令没有换行、逗号或新增类型码；例如`[qr_reset]`为5B 71 72 5F 72 65 73 65 74 5D，首个q是ASCII字符，不是0x71物块业务包。网页显示器优先识别完整控制命令，避免该字节重叠造成误解释。

仅运行态QR/OBJECT可执行；object_enabled=false拒绝任务切换。已有单槽控制队列与CONTROL_BUSY拒绝保持。MCU半帧和粘包由原串口解析器重组；未知命令不触发HMI动作。执行失败保留failed状态并沿用应用资源清理。HTTP返回accepted/request_id/frame_hex，最终状态仍在/api/status.control发布，包含source=web/uart与frame_hex。网页提交不写入实际TX/RX记录，实际UART读取才记RX；没有新增ACK、串口回显或自动重传。下位机可继续利用原有真实相机状态通知观察任务变化；主机执行状态不等于MCU接收确认。

当前实现源码：application.py、communication/protocol.py、hmi/module.py、camera/module.py；验证及实际命令见artifacts/hmi_serial_20261124。以下早期描述中“work也开启Web”及网页专用控制路径由本节覆盖。只读状态查询、图片流和雷达折叠面板不属于设备控制命令。

control.results保留最近32次请求的request_id/status/error。网页按ID从当前状态或历史结果确认操作，避免MCU后续请求覆盖此前Web结果；10秒未取得结果时解除按钮等待并显示确认超时，不擅自重发或宣称操作成功。

2026-11-22串口显示修正：页面按MCU_SERIAL_PROTOCOL.md解析物块0x71（num、类别码、int16小端dx/dy）、标靶0x72、圆盘0x61九字节回复、五字节状态及二维码0x70；仅二维码正文按UTF-8解码。时间/长度、中文协议含义、原始HEX分块呈现，不再展示二进制载荷的乱码文本。例5B 71 01 42 B8 00 28 00 5D为1种黑色物块，dx=+184、dy=+40像素。RX仍是read片段；严格完整且字段合法才解释，不完整、粘包或未知记录保留原始HEX并提示未解码，不伪报串口协议错误。帧重组不属于本显示修正。11项页面解码用例和桌面/390像素窄屏检查通过，证据见artifacts/tx_decode_20261122。

2026-11-22本次已部署：Web三分区及0x61确认后持续检测圆盘直到有效检出、回复一次9字节偏差的流程已同步至板端/root/Monthwu/GongChuang_vision_26。板端164项测试全部通过、无跳过，ROS2构建及两入口--check-files通过。部署前物块服务已停止，本次保持inactive，不启动相机、UART或雷达，实机及MCU接收待联调。下文此前“本地未部署”等说明为过程记录；本次备份及命令见deployment.md和artifacts/deploy_hmi_disc_20261122。

2026-11-22本地完成，尚未部署本次修改、重启板端服务或进行实机验收。

页面分三个固定区域：双相机窗口；独立TX/RX串口记录；二维码结果重置及二维码/物块任务切换。雷达、理论路径和诊断保留在折叠区。桌面相机并排，窄屏上下排列，始终保留两个窗口。未开启、等待首帧、图像超过3秒未更新及状态连接中断均显示黑底。主流程仍按任务独占相机，不因此同时打开两个设备。

二维码结果在自动交接物块后保留。重置只清空本机识别结果，不复位MCU、不自动进入二维码任务；扫码阶段拒绝重置前开始采集的旧帧。需要重新扫码时使用任务切换进入二维码，既有首个有效结果四次发送、flush后自动进入物块的流程保持。

POST /api/qr/reset及POST /api/task/switch使用同源、空正文请求。正式应用返回accepted及request_id，只排队；唯一主线程执行，在/api/status的control字段发布pending/done/failed。按钮等待确认期间禁用，失败显示错误。独立QR预览保留原同步reset响应，未注册任务回调时无切换端点。正式应用仅运行态QR/OBJECT阶段可操作，object_enabled=false拒绝任务切换。切换调用CameraModule.switch，先关闭旧相机，关闭超时则停止应用并清理，不打开下一设备。done表示任务提交完成，设备首帧就绪由相机窗口另行呈现。

TX/RX各保留最近100条，互不挤占，显示时间、HEX、转义文本及累计字节数。RX是实际read片段，可能含半帧、粘包和未识别字节；TX在每次实际write成功后记录，包含部分写入。主机write成功不证明MCU收到。

验证：157项Python检查，148通过、9项跳过，零失败。浏览器模拟验证重置、任务往返切换、409反馈、相机换流清除旧画面、过期黑屏、503断线/恢复及390像素窄屏，无页面脚本错误。截图artifacts/hmi_desktop_20261122.png及hmi_mobile_20261122.png使用模拟状态和模拟图像。测试日志artifacts/hmi_check_20261122.txt。

修改前完整备份artifacts/hmi_20261122_before.zip；另存修改前src/tests/docs/config/projects为artifacts/hmi_20261122_source_before.zip。未执行SSH、设备启停、板端同步、ROS2或ARM构建。
