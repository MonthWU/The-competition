# TODO —— 当前待办（2026-10-04）

当前运行依据见 [SCHOOL_RUNBOOK.md](SCHOOL_RUNBOOK.md)，具体证据与影响见 [当前问题清单](CURRENT_ISSUES.md)。正式入口 `start_all.sh`，直接扫码入口 `start_simple.sh`；`object_scan_enabled=true`，默认推理 `workaround`。

## 已完成

- [x] 两个启动入口与板端结构整理，真实相机 + PTY 三阶段软件联调。
- [x] `appli.service` 开机启用，初始无限等待，shot 仍为 30 秒超时。
- [x] 初始 `[skip]` 跳过障碍阶段，关闭 UART 后直接转二维码。
- [x] `[4]`、`[24]` 共享三角度 ROI；原始标定数据不变；73 项测试通过。
- [x] 二维码原文四帧发送及相机切换；物块面积过滤、单物块/全部标靶输出和录像关闭验证。

## 当前未完成

- [ ] 实际赛场障碍 13 候选点、三角度投票和最终地图验收。
- [ ] 实际二维码与 MCU 真串口收帧、云台动作及完整比赛流程验收。
- [ ] 物块现场误检、图像参考点和 ROI 标定；按结果决定是否启用区域过滤。
- [ ] 备用 native 实拍 `exit -11` 定位与同场景对照。
- [ ] 录像保留容量/时长和磁盘水位策略，同步打包耗时及丢帧验证。
- [ ] 空闲时安排整机重启，验证冷启动自启动与设备枚举。
- [ ] 预扫描串口有界缓冲、错误检测和失帧重同步。
- [ ] 二维码/物块图像及检测断流监测与恢复策略。
- [ ] 比赛持续运行的稳定性与端到端时延验收。
- [ ] Windows 源码副本与板端提交比较后同步，历史模型诊断与过期待办整理。

本轮仅整理和提交，不执行上述会接触硬件、改变参数或启停节点的验收与修复。

> 以下是早期开发记录，包含已过时的模型、相机和标定状态；当前校赛待办与验收边界以上方清单和 [SCHOOL_RUNBOOK.md](SCHOOL_RUNBOOK.md) 为准。

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

- [x] **远程仓库已就绪**（2026-09-27）：新增 remote **`gh`** →
      `https://github.com/MonthWU/The-competition.git`，项目已推送到分支 **`26GongChuan_vision`**
      （快进推送 `b7afb2b..1c43833`，未用 force；远程 108 个文件）
      · 日常推送：`git push gh 26GongChuan_vision`（需 GitHub PAT 认证，**不是账号密码**）
      · 旧 `origin`（`127.0.0.1:3000` 端口转发）保留未动，已不再使用
      · 合并时并入了远程原有的 `LICENSE` + `webapp/`（opencv_web_tuner 光照/阈值调参工具）

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

### 物块模型 · NHWC 替代版（2026-09-29 落地）
- [x] **新增 NHWC 物块模型** `dnn/yolo11_x5_nhwc.bin`（md5 `d31741bb…`，10.34 MB）
      —— 导出时 cls/box 已转置 NHWC，可被 TROS `dnn_node_example` 直接吃（无需 `obj_dnn` 绕行层）
- [x] **新建** `dnn/task_obj_v11_nhwc.json`（dnn_Parser=yolov8，reg_max=16，class_num=9）
- [x] **`obj_detect_v11.launch.py` 增加 `dnn_engine` 启动参数**
      （`workaround` 默认 / `native`），两条链路 topic / 参数完全对齐
- [x] **`launch/run_all.launch.py` 通过 `DNN_ENGINE` 环境变量透传**
- [x] **README §2/§3/§6.3/§7 同步**：双链路表 + 切换用法 + 单删指南（删 workaround 5 步 / 删 native 3 步，互不影响）

#### P0 · 现场回验 native 链
- [ ] **LRCP AR0234 物块特写图** 实景对比 workaround（md5 `d2ec3e70`）vs native（md5 `d31741bb`）：
      置信度、召回、定位精度三指标 ≥ 现役（参考 baseline：workaround 0.93/0.92/0.95）
- [ ] 若 native 召回/精度 ≥ workaround → 把 `run_all.launch.py` 的 `DNN_ENGINE` 默认改为 `"native"`，
      随后择期下线 workaround（删 `obj_dnn.py` + 删 launch 的 workaround 分支 + 删 setup.py entry_points）

#### P1 · native 链可选优化
- [ ] `dnn_node_example_node` 当前 `arguments=["--ros-args", "--log-level", "warn"]`，
      现场调通期可改 `info` 或暴露开关便于查 `out box size`
- [ ] `task_obj_v11_nhwc.json` 默认 `score_threshold=0.4 / nms_threshold=0.5`，
      现场按工作距离/重叠情况微调

#### 不受影响项
- 障碍物识别（framework/）链路不动
- 二维码（qrc_skandier）链路不动
- 串口协议（`obj_serial`）下游订阅接口不动
- 相机 by-id 路径 / 拉流分辨率参数不动
