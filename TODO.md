# TODO —— 待完成事项（2026-09-25 整理）

> 本文件汇总项目中**已知但未完成**的工作。完成后请勾掉并标注日期/提交号。

---

## 1. 物块识别（obj_detect · 9 类模型）

**当前状态（2026-09-27 验收）**：**完好** —— 使用新模型（`d2ec3e70…`）+ `obj_dnn` 绕行推理层。
- 3 次连续采样：最高置信度 **0.95 / 0.93 / 0.92**，多类别稳定检出（blue2 / red1 / black1）
- 相机 LRCP（`/dev/video2`）✅ ~12.7 fps ✅ 串口帧协议正确 ✅ Web 预览 `:8000` ✅
- ⚠️ 该模型输出为 **NCHW** → 依赖 `obj_dnn` 适配层；回到标准 TROS 仍需重导为 NHWC

### 1.1 ⛔ 导出格式问题（阻塞"回到标准 TROS"）

**两次导出各对一半** —— 目标是拿到「NHWC + 高置信度」的版本：

| 版本 | md5 | box 输出 | 实测效果 | 结论 |
|---|---|---|---|---|
| 9/25 版 | `0ffc3f3e…` | **NHWC** ✅ | 置信度仅 **0.07** ❌ | 格式对、效果差 |
| **9/27 版（当前在用）** | `d2ec3e70…` | **NCHW** ❌ | 置信度 **0.95** ✅ | 效果好、格式错 |

训练 mAP50 = 0.9+ 且 9/27 版实测正常 → **训练没问题**；9/25 版的问题出在**那一次导出/量化**。

**格式判据（已受控验证）**：只看 box 张量 layout —— `NHWC` → TROS 可直接用；
`NCHW` → `dnn_node_example` 必段错误（exit -11）。dtype（F32/int32）、cls 布局、
输入尺寸与模型尺寸不一致，**均不影响**。

**9/25 版置信度塌陷的可疑点（供重导参考）**：
1. 归一化 mean/scale 与训练不一致（YOLO 训练一般 `img/255`）—— 最常见
2. 校准数据集（PTQ 50~100 张）与实际场景差异大
3. 工具链版本不一致（板端日志有 `hbrt 3.15.54.0` vs `model build 3.15.55.0`）
4. 另：训练用 letterbox、而 TROS 用拉伸 resize，同图实测差 **45%**（0.069 → 0.0997）

- [ ] 排查（按优先级）：
      1. **导出浮点模型（不量化）跑同一张图** —— 决定性验证：若回到 0.9 即确认是量化问题
      2. **归一化 mean/scale 是否与训练一致**（YOLO 训练一般 `img/255`）—— 最常见元凶
      3. 校准数据集（PTQ 50~100 张）换成**实际场景图**
      4. 工具链版本一致性（板端日志有 `hbrt 3.15.54.0` vs `model build 3.15.55.0` 警告）
- [ ] 本机 Docker（**已装 29.4.1，daemon 未运行**）可承担重导：
      需 **ONNX + 50~100 张实际场景校准图**

**附：训练/部署预处理不一致的实测影响** —— 训练用 letterbox，而 TROS
`dnn_node_example` 用直接拉伸 resize。同一张 3024×4032 竖图，两者差 **45%**
（0.069 → 0.0997）。建议训练时加入拉伸增强，否则部署侧先天吃亏。

### 1.2 新模型修好后：去除绕行适配层

- [ ] 删除 `obj_detect/obj_detect/obj_dnn.py`
- [ ] 删除 `obj_detect/launch/obj_detect_v11_native.launch.py`
- [ ] `obj_detect/setup.py` 移除 `obj_dnn` 入口
- [ ] `launch/run_all.launch.py` 切回 `obj_detect_v11.launch.py`（标准 TROS）
- [ ] 端到端验收：标准 `dnn_node_example` 出框 + `obj_serial` 串口帧正确

**模型格式判据（2026-09-25 受控实验得出）**：只看 box 张量 layout ——
`NHWC` → TROS 可直接用；`NCHW` → 必崩。dtype（F32/int32）、cls 布局、
输入尺寸与模型尺寸不一致，**均不影响**。

---

## 2. 障碍物识别（framework 避障预扫描）

**当前状态（2026-09-27 实测）**：**代码层已打通**（4 处断点已修，可 import / 实例化 / 调用），
但**整条链路尚不可用** —— 卡在两个问题上：模型响应弱 + TROS 在"有候选"时崩溃。

- [x] **修代码断点**（2026-09-25 完成）—— 实际修了 **4 处**（比原盘点多 1 处）：
      - `mission_dispatcher.__init__` 补齐 `obstacle_detector` / `road_judge` /
        `_launch_proc` / `_rclpy_init`（此前一进 `scan_angle` 就 AttributeError）
      - `_grid_id(...)` → `grid_id(...)`（该名字本就 import 自 serial_protocol）
      - `RoadJudge.__init__` 去掉 `raise NotImplementedError`；并把语义混用的
        `hit_threshold` 拆为 `hit_threshold`（网格距离 1.5）/ `conf_threshold`（置信度 0.5）
      - **新发现**：`obstacle_detector.py` 的 `from map_model import grid_id` 是错的
        （该名字实际定义在 `serial_protocol.py`）→ 模块 **import 即 ImportError**，
        framework 整条链路连启动都进不去
      - 验证：`MissionDispatcher()` 可构造、视觉组件可实例化、
        `judge_from_hits({1:0.9, 3:0.7, 13:0.2, 6:0.8})` → `[(0,1),(0,3)]`（正确过滤）
- [x] **相机亮度修正**（2026-09-25）：必须通过 `hobot_usb_cam` **节点参数**
      设 `brightness=128 / gain=48`（v4l2 预设会被节点覆盖）

### 2.1 ⛔ 模型相关问题（**用户指示暂缓处理，先不修**）

- [ ] **模型响应过弱**：实测对现场障碍物最高响应仅 **0.16~0.17**（配置阈值 0.4），
      识别成功率 **0.0%**（650 帧 / 0 命中）。**已排除亮度因素** —— 提亮后
      （画面均值 68→140）响应反而略降至 0.160。疑似训练场景 / 目标尺寸与实拍差异
- [ ] **TROS parser 在"有候选框"时段错误**：
      阈值 0.4（0 候选）时长期稳定运行；阈值 0.05（有候选）时**第一帧后 SIGSEGV**，
      且 `out box size` 一次都未输出。怀疑与 cls 输出 `(1,80,80,1)` 的 NCHW 标注有关。
      → 出路：绕行推理层（参考物块的 `obj_dnn`）或重导模型

### 2.2 🔴 相机/配置问题（代码侧可独立处理，**建议优先**）

- [x] **`prescan.launch.py` 的 640×360 会导致 dnn 段错误** → **已修**（2026-09-27）
      根因：`640×360 → 640×640` 是纵向放大 **1.78×**，触发 `hobot_cv` VPS 问题；
      已改为 **1280×720**（缩小到 640×640，实测稳定不崩）
- [ ] `prescan.launch.py` 给 dnn 节点带了 `--log-level warn`，调试时看不到
      `out box size` 等检测细节 → 调试期改 info 或加开关

### 2.3 🟡 功能缺口

- [x] **单应性标定工具**（2026-09-27 完成）：新增 `framework/calibrate_homography.py`
      （`capture` 抓帧叠网格 / `solve` 求 H / `verify` 验证 / `status` 查状态），
      并修复 `obstacle_detector.py` 两处使标定链路此前**根本走不通**的问题：
      ① `pixel_to_grid()` 函数内只 import numpy 却调用 cv2 → 标定后一用就 `NameError`
      ② `homography_path` 默认空串 → 标定文件永不被加载，`pixel_to_grid` 恒返回占位 (2,2)
      ⏳ **待现场数据**：现场采 4~8 组「网格坐标 ↔ 像素坐标」后执行
      `python3 calibrate_homography.py solve "row,col:x,y" ...` 即完成标定
- [x] **`map_scanner.py` 是死代码** → **已删除**（2026-09-27；无引用、方法全未实现）
- [x] **端到端联调**（2026-09-27 完成）：`os.openpty` 虚拟串口对 + mock `scan_angle`，
      协议闭环验证通过 —— `[4] → ack → 3×[shot] → 3×ack → 地图帧 [0 00]`
      （无障碍时 count=0、校验=00，符合 v3 协议）

---

## 3. 文档同步

- [x] `README` §4：图像系 960×544 → **640×480**；区域过滤补实现说明 + 参数化（2026-09-25）
- [x] `README` §5：DCXIN 亮度问题**重写** —— 无 Auto(0)、必须走 `hobot_usb_cam`
      节点参数、实测数据（2026-09-25）
- [x] `README` §6.2：`obstacle_detector` / `road_judge` / `mission_dispatcher`
      状态与实际对齐（2026-09-25）

---

## 4. 其他

- [ ] `origin` 指向 `http://127.0.0.1:3000/neolux/GongzongAppli.git`（本机端口转发），
      3000 端口未启动时 `git push` 不可用 —— 需确认正确的远程地址
- [ ] `hobot_usb_cam` 打开不存在的设备路径时会**静默 fallback 到 `/dev/video0`**
      （已两次导致"开错相机"却不报错）。建议各 launch 起相机前统一加
      `os.path.exists()` 校验（`run_all` / `prescan` 已有，`obj_detect_v11` 待补）
