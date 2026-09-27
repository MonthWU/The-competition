# TODO —— 待完成事项（2026-09-27 更新）

> 汇总项目中**已知但未完成**的工作。已完成项保留存档并标注日期/提交号，便于追溯。

---

## 状态速览

| 模块 | 状态 | 说明 |
|---|---|---|
| **二维码扫描** | ✅ **完成** | by-id 锁定 + 相机身份校验 + 退出残留清零；全流程实测通过 |
| **物块识别** | ✅ **可用** | 新模型（`d2ec3e70`）置信度 **0.95**；⚠️ 输出 NCHW，依赖 `obj_dnn` 绕行层 |
| **障碍物识别** | 🟡 **工程侧完成** | 断点/相机/标定工具/协议联调全绿；⛔ 卡在模型 |
| **文档同步** | ✅ **完成** | README §4/§5/§6.2 已对齐实际 |

---

## 待办（按优先级）

### 🔴 P0 · 需你侧输入才能推进

- [ ] **物块模型重导为 NHWC**
      当前 `d2ec3e70` **效果好（0.95）但格式是 NCHW** → 依赖绕行层。
      目标：保持高置信度的同时把 **box 输出改成 NHWC** → 即可删除 `obj_dnn` 适配层
      （判据与可疑点见 §附录 A）
- [ ] **障碍物模型**（你已指示暂缓）—— 两个问题：
      ① 响应过弱（**0.16~0.17** vs 阈值 0.4）→ 成功率 **0%**，且提亮无效
      ② TROS parser 在**有候选框**时 SIGSEGV（0 候选时正常）
- [ ] **现场标定数据**：采 4~8 组「网格坐标 ↔ 像素坐标」后执行
      `python3 /root/dev_ws/appli/framework/calibrate_homography.py solve "row,col:x,y" ...`
- [ ] **本机 Docker**（已装 29.4.1，daemon 未运行）可承担重导：
      需 **ONNX + 50~100 张实际场景校准图**

### 🟡 P1 · 可独立处理（无需你配合）

- [ ] `prescan.launch.py` 的 dnn 节点带 `--log-level warn` → 调试期改 info 或加开关
      （现在看不到 `out box size` 等检测细节）
- [ ] `obj_detect_v11.launch.py` 补 by-id 存在性校验（`os.path.exists`）
      —— `hobot_usb_cam` 打开不存在的路径会**静默 fallback 到 `/dev/video0`**，
      已两次造成"开错相机却不报错"
- [ ] `obj_detect_v11.launch.py` 的 `dnn_example_image_width/height`（960/544）是死参数，可清理

### 🟢 P2 · 环境/杂项

- [ ] `origin` 指向 `http://127.0.0.1:3000/neolux/GongzongAppli.git`（本机端口转发），
      3000 未启动时 `git push` 不可用 → 需确认正确远程地址（当前 `ahead 29`）

---

## 已完成（存档）

### 二维码扫描（2026-09-25）
- [x] 扫码链路改 **by-id 固定**（原默认 `/dev/video2` 实测是检测相机，会静默开错）
- [x] `qrc_cam` 加**相机身份校验**（`expect_id`，不匹配即报错退出）
- [x] 退出残留清零（`destroy_node` 后访问句柄 / `join` 无超时 / 回调内 shutdown，共 3 处）
- [x] **全流程实测**（2026-09-27）：扫码 → kill 闭环 → 检测相机接管，全部正常

### 物块识别（obj_detect）
- [x] `run_all` 一键入口切到 v11 链路（P6）
- [x] `obj_serial` 区域过滤**补实现**（此前 `xin/yin` 定义了却从未被引用）+ 参数化（P4）
- [x] `ref_pt` 参考点参数化（P5，默认画面中心）
- [x] 修 `obj_detect_v11.launch.py` 的**双 `/dev/` 拼接** bug（导致开错相机）
- [x] 模型替换 + 3 次采样验收（**0.95 / 0.93 / 0.92**，多类别稳定）
- [x] 删除过期 `model_name`（换模型后会报 `Find model fail`）

### 障碍物识别（framework）
- [x] **修 4 处代码断点**（含 1 处 import 阶段的 `ImportError`）
- [x] **相机亮度修正**（必须走 `hobot_usb_cam` 节点参数）
- [x] **相机尺寸 640×360 → 1280×720**（640×360 会让 dnn 段错误）
- [x] **删除死代码** `map_scanner.py`
- [x] **协议闭环联调**：`[4] → ack → 3×[shot] → 3×ack → [0 00]`
- [x] **单应性标定工具**（含修 `pixel_to_grid` 的 cv2 未导入、H 路径未生效两处 bug）

### 文档
- [x] README §4（图像系 + 区域过滤）/ §5（DCXIN 亮度）/ §6.2（节点状态）
- [x] README §4 CLASS 映射补 9 类 + 保留旧 6 类兼容说明

---

## 附录 A · 模型格式判据（已受控验证）

**只看 box 张量 layout**：`NHWC` → TROS 可直接用；`NCHW` → `dnn_node_example` 必段错误（exit -11）。
dtype（F32/int32）、cls 布局、输入尺寸与模型尺寸不一致，**均不影响**。

两级对照实验：

| 版本 | md5 | box 输出 | 实测效果 |
|---|---|---|---|
| 9/25 版 | `0ffc3f3e…` | NHWC ✅ | 置信度仅 **0.07** ❌ |
| **9/27 版（当前在用）** | `d2ec3e70…` | NCHW ❌ | 置信度 **0.95** ✅ |

→ **两次导出各对一半**；训练 mAP50 = 0.9+ 且 9/27 版实测正常 → **训练没问题**，
9/25 版的问题出在**那一次导出/量化**。

**9/25 版置信度塌陷的可疑点**（供重导参考）：
1. 归一化 mean/scale 与训练不一致（YOLO 训练一般 `img/255`）—— 最常见
2. 校准数据集（PTQ 50~100 张）与实际场景差异大
3. 工具链版本不一致（`hbrt 3.15.54.0` vs `model build 3.15.55.0`）
4. 训练用 letterbox 而 TROS 用拉伸 resize，同图实测差 **45%**（0.069 → 0.0997）

**自检命令**（导出后跑一次即可判断能否上 TROS）：
```bash
python3 -c "
from hobot_dnn import pyeasy_dnn as dnn
for i,o in enumerate(dnn.load('你的.bin')[0].outputs):
    p=o.properties; print('OUT[%d]'%i, p.layout, p.shape, p.dtype)
"
```

---

## 附录 B · 相机映射（三路固化，by-id）

| 任务 | 相机 | by-id | 分辨率 |
|---|---|---|---|
| 扫码 | KS1A293 | `usb-KINGSEN_KS1A293-video-index0` | 640×400 @240 |
| 物块检测 | LRCP AR0234 | `usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0` | 640×480 |
| 全局扫描 | DCXIN | `usb-DCXIN_DCXIN_Camera_01.00.000-video-index0` | **1280×720**（原 640×360 会崩） |

> **DCXIN 亮度**：必须通过 `hobot_usb_cam` **节点参数**设 `brightness=128 / gain=48`
> （该机无真正的 Auto 曝光；launch 前的 v4l2 预设会被节点覆盖）。
