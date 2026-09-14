# 2026-07-30 基础题反光拼图背景取反验证

## 结论

用户提出的“A4 绿色背景二值化后取反获得拼图轮廓”方向成立，但正式实现采用当前 A4
纸面动态 Lab 色度中位数，而不是固定 RGB 或只认白色。当前实景中的一块强反光亮金属
和三块偏暗金属均被提取为完整轮廓。

最终实时结果为：A4 有效，识别 4 块，顶点拓扑 `4/4/3/4`，场景状态
`SCENE_VALID_LOCAL_ONLY`。求解器进一步确认 P2、P4 只有镜像后才能匹配官方模板，返回
`BASIC_TEMPLATE_FLIP_REQUIRED:P2,P4` 且不生成放置坐标。当前物理场景不是可由 X/Y 加
平面旋转完成的状态，因此没有伪报 `SOLVED`。

## 候选比较

| 方法 | 当前实景结果 | 结论 |
|---|---|---|
| 白色 HSV | 0 块，`PIECE_COUNT_INVALID` | 淘汰，暗反光金属必然漏检 |
| 固定绿色 HSV 取反 | 4 个轮廓，下半区误前景为 0 | 可作基线，但阈值依赖光照和纸张批次 |
| 动态三通道 Lab 距离 | 阈值 24 时有纸面伪轮廓，28 时得到 4 块 | 亮度梯度敏感 |
| 动态 Lab `a/b` 色度距离 | 阈值 12 时得到 4 块，下半区误前景为 0 | 采用 |

## 实现参数与门控

- `basic_use_a4_background_inversion: true`
- `basic_background_distance_threshold: 12.0`
- `basic_background_max_mad: 8.0`
- `basic_background_sample_stride: 4`
- `basic_background_chroma_only: true`
- `basic_contour_epsilon_max_mm: 3.0`

3.0 mm 只用于从反光锯齿轮廓生成初始 `3/4/4/4` 角点候选。最终顶点仍经过完整轮廓
稳健直线拟合，并保留线 RMS、最大残差、面积比例、最短边和固定模板刚体配准门控。

## 验证证据

- 修改前：[before_scene.json](before_scene.json) 为 A4 已检测但 0 块。
- 阈值比较：[mask_sweep.json](mask_sweep.json) 和
  [mask_sweep_montage.jpg](mask_sweep_montage.jpg)。
- 修改后轮廓叠加：[after_debug.jpg](after_debug.jpg)。
- 最终结构化结果：[final_result.json](final_result.json)。
- Jetson ARM64 感知测试 5/5 通过。
- Jetson ARM64 求解器 GTest 9/9 通过，包含正常模板、0.2 mm 间隙、模板拒绝和翻面诊断。
- 当前常驻 debug 进程组为 `37243`，启动参数包含 `serial_enabled:=false`；网页 HTTP 200，
  本次状态快照图像速率约 `28.7 FPS`，当前场景求解约 `7.16 ms`。
- 常驻进程只有感知、求解、协调器和网页预览；无 `serial_bridge_node`，`/dev/ttyTHS1`
  无本项目持有者，网页记录的串口发送帧为空。

## 当前边界

- P2、P4 当前处于镜像翻面状态；五字段 X/Y/角度协议不能表达翻面动作。
- 在 P2、P4 实际翻面并重新扫描前，不能把当前实景宣称为基础题 `SOLVED`。
- 本次没有启动串口桥或机械移动，没有验证真实落位和实物 0.2 mm 间隙。
