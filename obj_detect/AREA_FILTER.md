# 物块模型检测框面积过滤

统一入口 `obj_detect_v11.launch.py` 默认设置 `min_target_area_px=2000`。
面积为原图坐标中的检测框 `width * height`，单位为像素²；小于阈值的目标过滤，等于阈值的目标保留。
规则适用于物块模型的全部类别，包括物块和放置区标识。障碍预扫描和二维码链路不经过此过滤器。

workaround 和 native 两种引擎都先发布到 `/hobot_dnn_detection_raw`，
`obj_target_area_filter` 过滤后发布到原来的 `/hobot_dnn_detection`，网页与串口均使用过滤后的结果。
即使一帧目标全部过滤，仍发布空目标帧，并保留时间戳、帧率与性能字段。

样本实测小黑色框面积为 552–625 像素²，红色/浅蓝色物块框面积为 5236/6232 像素²；
2000 像素²可过滤这些小框并保留样本中的两个实际物块。较远或较小的真实目标也可能被过滤，现场可调整阈值。

```bash
# 启动时覆盖阈值
ros2 launch /root/dev_ws/appli/launch/run_all.launch.py min_target_area_px:=2000

# 在线调整，下一条推理消息生效
ros2 param set /obj_target_area_filter min_target_area_px 2000

# 查看当前阈值
ros2 param get /obj_target_area_filter min_target_area_px

# 0 关闭面积门限，仍剔除缺失或零尺寸检测框
ros2 param set /obj_target_area_filter min_target_area_px 0
```

## 板端验证（2026-10-03）

`obj_detect` 构建成功，四项面积边界检查通过。默认 workaround 链路完成真实 LRCP 相机采图、
原始/过滤话题对比、样本推理及 PTY 串口测试：2000 阈值下没有小面积目标进入过滤输出；
样本中的四个小黑框被剔除，红色和浅蓝色物块保留；1998 像素²目标不发送，2000 像素²目标保留并发送。
在线调整至 1000 后，同一小框立即保留并能发送，验证后恢复 2000。

native 支路已接入同一过滤器，但本次 native 推理进程在收到实拍图像后 `exit -11`，未产出原始检测。
使用备份的修改前 launch 对照，同样 `exit -11`；此为既有 native 推理故障，本次未改动其模型或解码实现。
当前通过完整板端验证的是默认 workaround 链路。

面积过滤任务的测试结束后已关闭测试控制组并释放相机、串口。
后续完整流程任务已将正式 `object_scan_enabled` 设置为 true，并贯通各阶段串口。
