# TODO —— 待完成事项（2026-09-25 整理）

> 本文件汇总项目中**已知但未完成**的工作。完成后请勾掉并标注日期/提交号。

---

## 1. 物块识别（obj_detect · 9 类模型）

**当前状态（2026-09-25）**：**可用** —— 使用**旧模型** + `obj_dnn` 绕行推理层。
实测：相机 LRCP（`/dev/video2`）→ `obj_dnn`（~18fps）→ 5 个目标、最高置信度 **0.86**，
串口帧协议正确，Web 预览 `:8000` 正常。

### 1.1 ⛔ 新模型导出问题（阻塞"回到标准 TROS"的根因）

新模型 `best_bayese_640x640_nv12.bin` **格式已正确**（cls/box 均为 NHWC、9 类，
实测不再段错误），但**置信度异常塌陷**：

| 测试 | 最高响应 |
|---|---|
| 旧模型 + 相机画面 | **0.86** ✅ 正常 |
| 新模型 + 相机画面 | 0.043 ❌ |
| 新模型 + **训练集图片**（模型见过的图） | **0.069**（拉伸）/ **0.0997**（letterbox）❌ |

训练 mAP50 = 0.9+，却对训练集图片只给 0.1 → **问题在导出/量化环节，不在训练**。

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

**已就绪**：相机 DCXIN（640×360 + 亮度修正）、模型 `zaw.bin`
（1 类 `block`、box 为 NHWC、实测兼容 TROS）。

- [ ] **修 3 个代码断点**（与模型无关，阻断整条链路）：
      - `mission_dispatcher.scan_angle()` 引用未定义的 `self.obstacle_detector` /
        `self.road_judge` → `AttributeError`
      - 同函数用 `_grid_id(...)`，但文件顶部 import 的是 `grid_id` → `NameError`
      - `road_judge.RoadJudge.__init__` 直接 `raise NotImplementedError` → 类无法实例化
- [ ] **识别率问题**：实测对现场障碍物最高响应仅 **0.125**（配置阈值 0.4），
      疑似目标过小 / 与训练场景差异；提亮画面**无效**（实测响应反而降到 0.029）。
      需确认模型训练数据与现场摆放距离
- [ ] **单应性标定**：`obstacle_detector.pixel_to_grid()` 未标定时返回占位 (2,2)，
      需现场标定 `calibrate()`
- [ ] `map_scanner.py` 三个方法仍为 `NotImplementedError`（骨架）
- [ ] `prescan.launch.py` 给 dnn 节点带了 `--log-level warn`，调试时可改 info

---

## 3. 文档同步

- [ ] `README` §4：区域过滤规格写的是「图像系 960×544、x∈[140,500]、y<420」，
      与实际不符（实际 640×480）。`obj_serial` 已参数化并默认改为
      `x∈[380,640]、y≤480`（基于实测物块位置外扩）
- [ ] `README` §5：DCXIN 的 `v4l2-ctl` 修复命令需修正 —— 该机 `auto_exposure`
      **只支持 1(Manual) / 3(Aperture Priority)，没有 Auto(0)**；且必须通过
      `hobot_usb_cam` **节点参数**设置（节点启动时会用自己的默认值覆盖 launch
      之前的 v4l2 预设，已实测）
- [ ] `README` §6.2：`obstacle_detector` / `road_judge` 状态与实际对齐

---

## 4. 其他

- [ ] `origin` 指向 `http://127.0.0.1:3000/neolux/GongzongAppli.git`（本机端口转发），
      3000 端口未启动时 `git push` 不可用 —— 需确认正确的远程地址
- [ ] `hobot_usb_cam` 打开不存在的设备路径时会**静默 fallback 到 `/dev/video0`**
      （已两次导致"开错相机"却不报错）。建议各 launch 起相机前统一加
      `os.path.exists()` 校验（`run_all` / `prescan` 已有，`obj_detect_v11` 待补）
