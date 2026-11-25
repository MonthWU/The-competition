# 从现有代码迁移

## 新实现与数据来源

新代码位于 `src/car26_system/car26`，重新实现应用资源管理、模块接口和任务状态机。没有导入 `appli` 的节点类、ROS 消息、dispatcher、旧 launch 或路径规划壳。

保留的是实际有效算法和 MCU 字节协议：NV12 排列、DFL 回归、按类别 NMS、面积与位置过滤、标靶冲突规则、旧 ROI 检测点、三次扫描投票，以及通信帧。默认新设备参数独立配置。以下六份资产通过只读 SSH 与板端核对，哈希见 `assets/manifest.json`：两份模型、两份类别文件、ROI、旧相机颜色标定。模型逐字节与板端相同；类别文件的本地换行格式不同，因此新项目保存板端字节。

| 现有能力 | 新位置与结果 |
| --- | --- |
| `obj_dnn.py` / `obs_dnn.py` 推理 | `camera/detector.py`，按 box 实际形状决定 cls/box 配对内存顺序，绕开错误 cls 布局标签 |
| `target_area_filter.py` 用户当前修改 | `camera/processing.py`，保留 2000 面积、下半幅中心过滤、重合 2/3 判 3 的规则 |
| `obj_serial.py` | `communication/`，同一个串口所有权覆盖任务、二维码、物块，避免阶段间竞争 |
| `mission_dispatcher.py` / `ObstacleLocator` | `mission.py` / `mapping.py`，任务状态与副作用分开；旧 ROI 可作为对照配置 |
| QR 相机关闭与物块相机启动 | 四帧写入和 flush 完成后应用直接切换，取消 ROS 话题交接等待 |
| 图像/串口预览 | 新 HMI 页面，最多 100 条最近事件和每个角色最新 JPEG |
| 视频记录 | 由摄像头模块在真实帧尺寸确认后创建 writer，默认关闭、限时停止录像 |
| C1 资料 | 新增实际 UART Standard 扫描模块，资料保存在 `docs/RPLIDAR_C1_REFERENCE.md` |

`work/debug` 共用上述全部功能，debug 只增加诊断输出。新项目不包含 GPIO 关机服务、失效工具、旧 parser、native/workaround 双链、旧兼容别名、旧 A0F0 协议。当前两个模型只有上述九类物块/标靶和障碍 block，因此不携带旧 rof/gof/bof 等类别编码。现有 `call_opened()` 已注释停用，新通信模块也不发送那份旧启动帧。

## 行为边界

握手回包准确为小写 `[ack]`。地图帧 `[count id... CHECK]` 中 CHECK 是 count XOR 各 ID 的两位大写十六进制。二维码是 `FF 37 UTF-8原文 FE`，原文非空即发送四次。物块是 `FF 41..46 x低 x高 y低 y高 FE`，标靶是 `FF 51..53 x低 x高 y低 y高 FE`；坐标为像素，不转换成毫米。

初始等待改为无限等待；三个 shot 等待默认 30 秒。旧 dispatcher 代码初始等待是有限超时，旧文档中的无限等待描述不能当作旧代码实证。新项目支持初始 `[skip]` 和独立 QR 入口；旧 ROI 只有起点 4 有效，起点 24 不会共享起点 4 标定。

雷达原始数据不新增未经 MCU 确认的串口帧。它通过通信/状态快照供 HMI 查看；只有标定匹配出的障碍 ID 才进入既有地图帧。C1 Standard 与 DenseBoost 都是 5 kHz 标称采样，本实现选择更直接的 Standard 模式；不实现 DenseBoost 相邻包插值或 PWM 控制。

旧 `appli` 是参考和回退材料，当前用户修改的三个物块文件保持原样。本次部署目标目录尚未选定；不把新目录直接覆盖板端运行项目，也不安装旧或新服务。
