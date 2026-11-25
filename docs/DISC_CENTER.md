# 白色大圆盘圆心检测

2026-11-24最新本地规则（尚未部署）：用户确认平时实际采集640×360，YOLO仍补边至640×640；收到原始5B 61 2C 01 5D后切1280×720搜索圆盘，检出后只产生一次既定9字节回复并恢复640×360。收到原始5B 62 2C 01 5D后，冻结当时最近一次有效YOLO结果，切1280×720按其框映射的局部ROI精定位，期间暂停YOLO；不再按三靶角度条件自动进入。结束/取消及原有退出条件后恢复640×360。本轮板卡未联网，用户要求先完成本地修改，未部署、构建或运行板端。此条覆盖下文此前持续720p和0x62完全被动的历史说明。 圆盘算法仍处理720p原图；独立诊断历史记录不等于新指令流程实机验收。

2026-11-24当前配置：按用户要求改为持续1280×720 MJPEG采集、请求30fps，block/target两模式均为720p。YOLO对完整画面缩至640×360并等比例补边为640×640识别；三标靶满足原门控后，在同一720p原图的三个局部ROI内精定位，期间暂停YOLO。模式切换不重开相机。YOLO偏差原点仍为(320,180)，精定位及圆盘原图偏差原点为(640,360)，使用720p原生像素。此条覆盖下文此前默认1080p的描述；历史实测数值保留。当前服务已按用户要求停止，本次不自动启动。 圆盘确认触发与回复规则仍以DISC_ONCE.md为准，不恢复历史持续对准算法。

2026-11-22本次已部署：Web三分区及0x61确认后持续检测圆盘直到有效检出、回复一次9字节偏差的流程已同步至板端/root/Monthwu/GongChuang_vision_26。板端164项测试全部通过、无跳过，ROS2构建及两入口--check-files通过。部署前物块服务已停止，本次保持inactive，不启动相机、UART或雷达，实机及MCU接收待联调。下文此前“本地未部署”等说明为过程记录；本次备份及命令见deployment.md和artifacts/deploy_hmi_disc_20261122。

2026-11-22用户最新更正：正式0x61确认后持续搜索直到有效检出，再回复一次9字节偏差并停止；未收到确认时不计算。以下“仅测一帧”的过程记录已被替代，当前规则见DISC_ONCE.md，本地未部署。

2026-11-22最新正式流程见DISC_ONCE.md：仅0x61阶段收到MCU进入确认才检测一次原图，成功回复9字节0x61偏差包；其它时间不检测。本规则替代下文历史“持续检测、5像素锁存、不新增串口类型”的说明。独立诊断入口保留，未部署本次更新。

## 当前正式流程：0x61中YOLO与圆盘对准（2026-11-22后续）

用户要求0x61物块阶段运行YOLO，同时计算白色大圆盘圆心；当原始采集图上`abs(dx)<=5 AND abs(dy)<=5`时锁定本次对准，后续本阶段只执行YOLO，不再调用圆盘检测器。比较浮点偏差，不先四舍五入；丢失或无效圆心不算达标。达标后偏离不自动恢复圆心计算；重新启动相机或退出后再次进入0x61时清零重算。现有0x62三标靶精定位独立执行，不计算大圆盘；返回0x61时重新对准。

入口为正式`start_coloritems.sh`或扫码交接后的物块阶段，沿用同一个相机线程和同一采集帧：YOLO用640×360缩小图，圆盘用默认1920×1080原图。因此5像素容差属于1080p原图坐标，不是YOLO缩小坐标。原有0x71物块包及0x72标靶包保持，圆盘结果通过HMI发布，不新增圆盘UART报文。

配置`disc_centering`：`enabled`、`parameter_file`、`native_library`及`tolerance_px=[5,5]`；实机board.json启用，基础system.json默认关闭，两版分项目配置均修正了文件相对路径。HMI的`camera.disc_state`为searching/aligned/inactive/disabled，`disc_aligned`记录锁定，`disc_processed_frames`只统计该阶段真正调用检测的帧数。达标后序号和YOLO继续更新，圆盘调用计数停止，`disc=null`，不以旧偏差冒充当前测量。预览仍显示YOLO框，圆盘外缘和偏差按比例绘制，数值保留原图坐标；锁定后显示DISC ALIGNED - YOLO。

以下独立圆盘诊断入口仍保留供单独验证，不替代上述正式0x61流程。

2026-11-22用户指定目标为白色大圆盘外缘，二维码已停止。通过物块相机1920×1080采集，独立检测并在预览和`/api/status`中发布图像圆心及偏差。入口不创建UART、雷达或YOLO；与物块服务、二维码预览互斥。

## 使用

板端项目根目录执行：

```bash
bash scripts/build_disc.sh
bash start_disc_test.sh --check
bash start_disc_test.sh --mode debug
```

默认work模式，运行日志写入`log/disc/preview.log`；debug同时显示终端。前台Ctrl+C退出，默认一小时，可用`--maximum-seconds 60`设定1～3600秒。与原物块服务共用8080，若服务运行先`systemctl stop car26-object.service`。预览为`http://<板卡IP>:8080/camera/object`；综合状态页需选择“物块与标靶”。本入口结束后可用`systemctl start car26-object.service`恢复原物块检测。

离线图像测试不开设备：

```bash
bash start_disc_test.sh --image artifacts/disc_center_20261122/object_scene.jpg --output artifacts/disc_center_20261122/scene_detected.jpg
```

## 算法和输出

核心为`native/disc_center.cpp`，ARM本机构建`native/lib/libcar26_disc.so`，Python接口`camera/disc.py`。缩小图像后高斯降噪，分别生成低饱和度白色掩膜和亮度自适应掩膜：后者用V通道Otsu阈值与bright_minimum_value的较大值，并允许配置范围内偏色。每个掩膜闭运算后开运算，去掉窄缝及噪声连接。分别处理最大的三个有效连通轮廓，排除图像边框上的点；以确定性RANSAC抽取五点拟合椭圆并反复用内点细化。核对直径、轴比、弧覆盖、可见周界支持和椭圆内部白色比例，排除小孔、矩形及不足弧段。跨分支选择支持点最多的有效结果。失败返回None，每帧独立检测，不沿用上一帧结果。

`camera.disc`包含：

- `center=[x,y]`：原始图像外缘拟合中心，浮点像素，允许因圆盘出画而位于画面外。
- `axes=[width,height]`、`angle_deg`：OpenCV旋转椭圆的轴径与角度。
- `offset=[dx,dy]`：中心减去画面中心，向右/下为正；1080p中心为(960,540)。
- `residual_px`：支持点到拟合椭圆的近似径向残差均方根，原图像素。
- `coverage_deg`、`visible_support`、`white_fraction`、`inliers`：弧支持及白色区域的质量指标，不是经标定的置信概率。
- `segmentation`、`value_threshold`：实际使用的low_saturation或brightness分支及该帧HSV亮度阈值。

绿线表示外缘椭圆，红十字表示拟合中心，青十字为画面中心，黄线连接两者。无目标显示`WHITE DISC: NOT FOUND`，状态中`disc_detected=false,disc=null`。

这是图像中椭圆轮廓的几何中心；透视投影下不保证等于圆盘物理中心的精确投影。毫米坐标、畸变补偿和精确物理中心需要相机及平面标定。当前上缘出画的实拍结果不是物理真值验收。

## 参数

全部17项参数来自`config/disc.json`，`--config`可指定替代文件，启动读取，修改后重启生效。当前参数用本次低照度及较亮偏色两种实拍图验证；其它背景、照明或视距变化仍需核对。

| 参数 | 当前值及含义 |
| --- | --- |
| processing_width | 960，处理图最大宽度，不放大原图 |
| maximum_saturation / minimum_value | 32 / 25，OpenCV HSV的S上限及V下限，范围0～255 |
| bright_maximum_saturation / bright_minimum_value | 80 / 80，亮度分支允许的S上限和自适应V阈值下限 |
| morphology_size | 5，处理图上的奇数形态学核，闭运算后开运算 |
| minimum_component_area_ratio | 0.06，候选连通区域占处理图总面积的下限 |
| minimum_diameter_ratio | 0.50，椭圆短轴直径相对处理图短边的下限，排除夹具小圆孔 |
| maximum_diameter_ratio | 2.0，椭圆长轴直径相对处理图长边的上限 |
| minimum_axis_ratio | 0.60，短轴/长轴比下限，限制狭长伪椭圆 |
| inlier_distance_px | 3.0，处理图近似径向内点距离上限 |
| minimum_inliers | 70，每条轮廓最多均匀取1200点后的最少支持点数 |
| minimum_coverage_deg | 130，5度角度桶累计支持覆盖下限，排除短弧外推 |
| minimum_visible_support | 0.55，可见周界均匀采样点中有轮廓支持的比例下限 |
| minimum_white_fraction | 0.65，椭圆内可见采样点中的白色区域比例下限 |
| iterations | 480，每个候选区域RANSAC抽样次数 |
| minimum_enclosure_ratio | 0.88，拟合椭圆须包围至少88%的当前连通外轮廓填充区域采样点，容许少量白色夹具连接并排除局部小椭圆 |

## 验证资料

新增`tests/test_disc.py`覆盖完整倾斜椭圆、上缘出画、夹具遮挡、小孔/矩形/空图/短弧拒绝、参数及图像检查、实拍低照度图和镜像、较亮偏色图、成功后目标丢失。实拍回归图`tests/data/disc_clipped_scene.jpg`及`disc_bright_scene.jpg`均为本次物块相机图像，参考中心范围为人工轮廓估计，不是标定测量。

修改前板端完整备份：`/root/Monthwu_archive/RDK_CJLU_V08_disc_20261122_dd83ee22/project_before.tar.gz`，SHA256为`1be34a4f8022ab0faaa9a2dc10100926667d1cf848ae9f7d2fe2e40a629a030a`。命令、部署脚本、测试和实时记录在`artifacts/disc_center_20261122`。

最终板端8项专项测试、140项全项目测试均通过，无跳过；新原生库及ROS2构建通过。扫码已停止，独立圆盘预览PID14246保持运行，最长一小时，物块服务inactive但enabled保留；fuser确认仅video0被预览持有，QR及ttyS2空闲。

包围约束后约18秒连续检查取得44个不同帧，44帧均检出。圆心中位数约(804.13,83.49)，x范围794.06～823.17、y范围56.34～157.40，标准差约5.85/21.68像素；单帧算法耗时中位数347.63毫秒，处理约2.11FPS。采样期间物块及遮挡发生变化，未提供标定真值；这些是该窗口的检出与波动记录，不是物理精度或其它场景成功率。部分出画和遮挡的圆心仍有波动，后续控制使用前需针对实际精度目标标定与验收。
