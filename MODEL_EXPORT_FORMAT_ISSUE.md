# 模型导出格式问题反馈（RDK X5 / TROS）

> 用途：直接转发给模型导出方 / 训练团队。中英双语。

---

## 中文版

**【模型导出格式问题 —— RDK X5 / TROS】**

### 现象
贵方导出的 `best_bayese_640x640_nv12_nchw.bin`（另：物块模型 `yolo11_x5.bin` 同类）在
**RDK X5 + TROS humble** 上运行官方 `dnn_node_example` 时，节点在**收到第一帧图像后立即
`Segmentation fault`**（崩在后处理阶段，`out box size` 一行都来不及输出）。

### 定位
与 D-Robotics **官方模型**对比后，差异在**输出张量的 layout**：

| 模型 | 输入 | cls 输出 | box 输出 | 结果 |
|---|---|---|---|---|
| 官方 `yolov8_640x640_nv12` | NCHW (1,3,640,640) NV12 | **NHWC** (1,80,80,80) | **NHWC** (1,80,80,64) | ✅ 正常 |
| 官方 `yolov5s_672x672_nv12` | NCHW | **NHWC** | **NHWC** | ✅ 正常 |
| **我们的模型** | NCHW | **NCHW** (1,80,80,1) ⚠️ | NHWC (1,80,80,64) | ❌ 段错误 |

TROS 的 `parser_yolov8` 会**按 `layout` 字段分派解析路径**；cls 被标为 NCHW 时走到
与官方不同的分支，导致越界访问 → SIGSEGV。

### 请修改（两点）

1. **输入端保持不动**
   `NCHW (1, 3, 640, 640) uint8 NV12` —— 这是 D-Robotics 工具链的固定要求，
   **不要改成 NHWC**。

2. **输出端全部导出为 `NHWC`**，特别是 **cls 分支**：
   - 当前：`layout=NCHW, shape=(1,80,80,1)`
   - 期望：`layout=NHWC, shape=(1,80,80,1)`
   - box 分支 `(1,80,80,64)` 已是 NHWC，**保持不变**
   - 三个尺度（80 / 40 / 20）**都要改**

### 导出后自检（`layout` 一列必须全是 NHWC）

```bash
python3 -c "
from hobot_dnn import pyeasy_dnn as dnn
for i, o in enumerate(dnn.load('模型.bin')[0].outputs):
    p = o.properties
    print('OUT[%d]' % i, p.layout, p.shape, p.dtype)
"
```

### 补充说明
`(1,80,80,1)` 的 shape 本身看起来已是 HWC（因 C=1，NCHW/NHWC 内存等价），
**疑似只是导出时 layout 标注写成了 NCHW**。若确认如此，**仅修正标注即可**，
无需改动网络结构。

### 参考
D-Robotics 官方 YOLO 导出流程的默认输出就是「**输入 NCHW + 全部输出 NHWC**」，
建议直接沿用其配置（`rdk_model_zoo` 中的导出脚本）。

---

## English Version

**[Model Export Format Issue — RDK X5 / TROS]**

### Symptom
The exported `best_bayese_640x640_nv12_nchw.bin` (same for the object model `yolo11_x5.bin`)
crashes with a **`Segmentation fault`** in TROS `dnn_node_example` on RDK X5, immediately
**after the first frame is received** (during post-processing — it dies before printing
`out box size`).

### Root cause
Compared with D-Robotics' official models, the difference is the **output tensor layout**:

| Model | Input | cls | box | Result |
|---|---|---|---|---|
| Official `yolov8_640x640_nv12` | NCHW (1,3,640,640) NV12 | **NHWC** (1,80,80,80) | **NHWC** (1,80,80,64) | ✅ OK |
| Official `yolov5s_672x672_nv12` | NCHW | **NHWC** | **NHWC** | ✅ OK |
| **Ours** | NCHW | **NCHW** (1,80,80,1) ⚠️ | NHWC (1,80,80,64) | ❌ SIGSEGV |

TROS `parser_yolov8` **dispatches on the `layout` field**; when cls is tagged NCHW it takes
a different code path than the official models, resulting in an out-of-bounds access.

### Please change (two points)

1. **Keep the input as-is**
   `NCHW (1, 3, 640, 640) uint8 NV12` — required by the D-Robotics toolchain.
   **Do NOT change it to NHWC.**

2. **Export all outputs as `NHWC`**, especially the **cls branch**:
   - now: `layout=NCHW, shape=(1,80,80,1)`
   - want: `layout=NHWC, shape=(1,80,80,1)`
   - box `(1,80,80,64)` is already NHWC — **keep unchanged**
   - apply to **all three scales** (80 / 40 / 20)

### Self-check after export (the `layout` column must be NHWC everywhere)

```bash
python3 -c "
from hobot_dnn import pyeasy_dnn as dnn
for i, o in enumerate(dnn.load('model.bin')[0].outputs):
    p = o.properties
    print('OUT[%d]' % i, p.layout, p.shape, p.dtype)
"
```

### Note
Since C=1, the shape `(1,80,80,1)` is memory-identical for NCHW/NHWC — it may be **only a
mis-tagged layout**. If so, fixing the tag alone should be sufficient; no network structure
change is needed.

### Reference
D-Robotics' official YOLO export pipeline outputs "**input NCHW + all outputs NHWC**" by
default. Reusing its configuration (scripts in `rdk_model_zoo`) is recommended.
