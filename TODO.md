# TODO —— 待完成事项（2026-09-25 整理）

> 本文件汇总项目中**已知但未完成**的工作。完成后请勾掉并标注日期/提交号。

---

## 1. 物块识别（obj_detect · 9 类模型）

**现状**：板上 `dnn/yolo11_x5.bin` 仍是旧版（md5 `f90fdfa1…`，2026-09-08），
**输出 NCHW**，与 TROS `parser_yolov8` 不兼容 → `dnn_node_example` 段错误（exit -11）。
因此此前增加了一层绕行适配（`obj_dnn` + `obj_detect_v11_native.launch.py`）。

- [ ] **上传新导出的物块模型**（`dnn/yolo11_x5.bin`）
      导出后必须自检：**box 那一路（通道数 64 = 4×reg_max）layout 必须是 `NHWC`**
- [ ] **去除绕行适配层**（模型格式确认正确后执行）：
      - 删除 `obj_detect/obj_detect/obj_dnn.py`
      - 删除 `obj_detect/launch/obj_detect_v11_native.launch.py`
      - `obj_detect/setup.py` 移除 `obj_dnn` 入口
      - `launch/run_all.launch.py` 切回 `obj_detect_v11.launch.py`（标准 TROS 链路）
- [ ] 端到端验收：标准 `dnn_node_example` 出框 + `obj_serial` 串口帧正确
- [ ] （可选）`obj_detect_v11.launch.py` 中的 `dnn_example_image_width/height` 是
      死参数（节点并不使用），可清理

**模型格式自检命令**：
```bash
python3 -c "
from hobot_dnn import pyeasy_dnn as dnn
for i,o in enumerate(dnn.load('/root/dev_ws/appli/dnn/yolo11_x5.bin')[0].outputs):
    p=o.properties; print('OUT[%d]'%i, p.layout, p.shape, p.dtype)
"
```

**兼容性判据（2026-09-25 两次受控实验得出）**：只看 box 张量 layout ——
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
