# 2026-07-30 绿色 A4 实机定位参数迭代与精度验证

## 结论

绿色 A4 在目标 Jetson、IMX219、1920x1080 YUY2、正常室内照明下完成多位置和多方向
实机验证。最终保留窄 HSV 与既有形态学参数，在四边形初值之后增加完整轮廓稳健直线
拟合，以相邻边交点输出亚像素四角。

最终参数：

~~~yaml
green_a4_h_min: 60
green_a4_h_max: 90
green_a4_s_min: 78
green_a4_s_max: 201
green_a4_v_min: 91
green_a4_v_max: 245
green_a4_morph_kernel: 7
green_a4_morph_iterations: 2
a4_polygon_epsilon_ratio: 0.02
green_a4_line_fit_band_px: 6.0
green_a4_line_fit_min_points: 12
green_a4_max_corner_refine_px: 25.0
~~~

## 参数淘汰证据

| 方案 | 实机结果 | 结论 |
|---|---:|---|
| 原窄 HSV，中心摆位 | 100/100；角点 P95 2.43 px | 作为阈值基线 |
| H35..90、S78..255、V70..255 | 99/99 收到帧检出；P95 5.45 px | 范围过宽，淘汰 |
| 仅将 Hmin 降到 42 | 100/100；P95 3.37 px，最大 7.14 px | 不如窄 HSV，淘汰 |
| 形态学核/次数网格 | 最佳相对基线仅改善 0.016 px，面积稳定性略差 | 保留 7x7、2 次 |
| epsilon 0.012..0.025 | 同一原始 60 帧四角结果相同 | 保留 0.02 |
| stable_frames 3/5/7/9 | 7 帧仅小幅改善，但会进一步增加扫描延迟 | 保留 3 |

旧手机实拍图只需降低 Hmin 即可检出，但这样会降低当前目标 Jetson 的角点稳定性。项目
最终运行参数以目标相机为准，不用一个过宽 HSV 同时包住不同相机的色彩响应。

## 轮廓细化原始帧对照

当前最不利斜放姿态的 200 张无 Web 标注原始帧：

| 方法 | 成功帧 | 角点 P95 | 最大偏差 |
|---|---:|---:|---:|
| approxPolyDP 整数顶点 | 200/200 | 2.76 px | 5.17 px |
| fitLine，2 px 带宽 | 180/200 | 1.96 px | 2.59 px |
| fitLine，4 px 带宽 | 200/200 | 0.69 px | 1.17 px |
| fitLine，6 px 带宽 | 200/200 | 0.40 px | 0.45 px |
| fitLine，8 px 带宽 | 200/200 | 0.40 px | 0.47 px |

6 px 后收益进入平台，因此选择更保守的 6 px，不继续扩大边线采样带。

## 部署后二次实机验收

每个静止姿态均发送 100 次扫描请求；移动过程中的帧不计入静态结果。

| 姿态 | 中心（相机中心坐标 px） | 检出 | 角点 P95 | 最大偏差 | 延迟 P95 |
|---|---:|---:|---:|---:|---:|
| 偏左斜放 | (27.58, 108.10) | 100/100 | 0.49 px | 0.63 px | 278 ms |
| 偏右、近横向 | (197.73, 85.22) | 100/100 | 0.58 px | 0.67 px | 280 ms |
| 中部纵向 | (79.02, 14.37) | 100/100 | 1.43 px | 1.69 px | 284 ms |

按当前配置约 3 px/mm 换算，三组 P95 重复性约为 0.16、0.19、0.48 mm，均小于
1 mm。这里验证的是静止纸张的角点重复性和画面可视对齐，不是带人工真值标注的绝对
坐标误差；工作区绝对映射仍必须通过真实标定点验收。

调试图：

- [偏右近横向](green_a4_live_20260730/pose_landscape.jpg)
- [中部纵向](green_a4_live_20260730/pose_portrait.jpg)

## 软件与运行验证

- Jetson ARM64 构建：puzzle_perception_node、vision_bringup 通过。
- 测试：16 tests，0 errors，0 failures，0 skipped。
- 运行参数已确认加载 6 px、12 点、25 px 三个细化门限。
- debug 保持四节点唯一运行，Web 端口 5000 与 1920x1080 图像流正常。
- 安全 debug 未启动 serial_bridge_node。
- 当前 PIECE_COUNT_INVALID 来自画面没有有效拼图块，不是 A4 定位失败。

扫描延迟 P95 约 278--284 ms，未因直线拟合相对修改前 286 ms 发生退化，但仍高于
docs/vision_goal.md 中 200 ms 的初始性能目标；这是后续性能优化项，不能写成已通过。

## 备份与回滚

- Windows：
  Copyfiles/backups/26E_vision/20260730_133256_green_a4_line_refine/
- Jetson：
  ~/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260730_133256_green_a4_line_refine/

恢复上述备份中的五个源码/配置文件后，重新构建
puzzle_perception_node vision_bringup。回滚本次新增验证材料时，再删除本报告和
docs/validation/green_a4_live_20260730/。
