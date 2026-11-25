# 配置说明

`system.json` 为新设备模板；`legacy_camera.json` 通过 `extends` 合并同目录模板，只覆盖现有三相机参数。文件路径相对最终配置文件所在目录解析；在其他目录新建配置时，需要相应调整 `models`、ROI 和输出路径。命令行 `--mode` 覆盖配置中的模式。

## 用户后续填写项

| 配置项 | 含义 |
| --- | --- |
| `cameras.qr.device` / `cameras.object.device` | 稳定 V4L2 by-id 设备路径；两路设备不能相同 |
| `cameras.qr.roi` | 默认`null`（或省略）表示整帧扫码，直接将完整画面传给OpenCV及pyzbar，不做中心裁剪；显式`[x,y,width,height]`才裁剪，像素整数，宽高>0且矩形必须完全位于当前帧内 |
| 每路 `width/height/fps/fourcc` | 实物支持参数，默认值仅作初始模板；第一帧尺寸不匹配会失败 |
| `radar.device` | C1串口稳定路径；不可与communication.device相同。当前board为MCU `/dev/ttyS2`，system模板仍为`/dev/ttyS1` |
| `radar.extrinsic` | 雷达在车体参考系中的 `[x_m,y_m,yaw_deg]`，yaw 逆时针为正 |
| `radar.extrinsic_calibrated` | 完成实物外参标定后才设 true |
| `radar_mapping.targets_by_start` | 每起点的候选格 ID 与中心坐标；坐标单位米 |
| `radar_mapping.heading_deg_by_start` | 起点参考系中 0/45/90 三个触发阶段的实际扫描朝向；逆时针为正 |

以下雷达地图标定属于legacy full预扫描入口，不是新有雷达分项目的道路图；新动态规划见TODO1_NAVIGATION.md及projects/with_radar/config/route.json。这里的符号须替换成实测数值，不是可直接运行的JSON：

```text
targets_by_start = {
  "4": [{"id": 13, "x_m": <实测中心x>, "y_m": <实测中心y>}, ...],
  "24": [{"id": 13, "x_m": <实测中心x>, "y_m": <实测中心y>}, ...]
}
heading_deg_by_start = {
  "4": [<第一次朝向>, <第二次朝向>, <第三次朝向>],
  "24": [<第一次朝向>, <第二次朝向>, <第三次朝向>]
}
```

仅填写实际标定过的起点；未标定的起点收到命令后报 `MAP_START_NOT_CALIBRATED`，不回握手成功。候选 ID 为现有地图的 1/3/5/7/9/11/12/13/15/17/19/21/23。提供全部可能障碍位置，不能只填写期望答案。匹配半径及宽度门限需要用实物雷达数据确认，初始值不是验收结果。

## 其他设置

- `mission.entry`: `full`、`qr`或`object`；默认无雷达版使用qr，独立物块使用object，full保留旧预扫描兼容。`object_enabled=false`时扫码后退出。full的prescan_source=radar使用C1，camera使用第三路相机和旧ROI；新分项目不自动进入该流程。
- `camera_processing.reference_px`: 历史最近物块选择参数，保留配置兼容，但当前全类别合包不再使用它。dx/dy始终由检测框中心减当前处理图中心计算。`block_roi=null`关闭附加物块ROI；显式ROI仍生效，num为ROI内实际发送的物块种类数。
- 二维码ROI独立于物块`block_roi`和预扫描地图ROI。system.json与board.json均显式设置`cameras.qr.roi=null`，有/无雷达分项目继承此默认值。HMI相机状态的`qr_roi`显示实际扫描矩形，例如640×400全帧为`[0,0,640,400]`，`vision_mode=qr`表示扫码阶段。
- `models.*.rgb_gains`: RGB 顺序，默认中性；新相机需重做颜色校准。模型必须是当前六输出、640×640 NV12 YOLO11 BPU 模型；整数输出直接拒绝，禁止无依据当浮点解码。
- `radar.frequency_hz`: 8–12 Hz，默认 10；Standard 每圈约 500 点为典型估算，实际频率、点数由 HMI 记录。质量与量程门限可调。
- `hmi`: 默认 8080，允许最多四路同时图像流，图像编码最多 10 FPS。实际摄像头工作 FPS 与预览 FPS 无关。
- `recording`: 默认关闭；启用时保存当前角色的视频，单次角色最多录制 300 秒，然后停止录像并保持业务。再次切换角色会创建新文件；文件不会自动清理，需用户管理磁盘。视频标称 FPS 采用配置值，若实际处理较慢，回放时间不是物理时序证据。
- `communication.transport`: 当前只允许 `serial`。MCU 尚未定义 CAN 帧协议，不能自行制定兼容协议。

`--check` 仅检查配置结构、参数及必填项；不打开设备。`--check-files` 另外检查模型、ROI 和设备路径是否存在；仍不加载模型或打开设备。两者通过不代表设备独占、相机 FPS、BPU 推理或物理效果通过。
