# 启停区 1 障碍 ROI 标注与拟合

2026-09-29 的 `source_annotations.json` 保存用户重新标注的两组 0°、45°、90° 图像点列，分别对应 `angle_pic1/2/3.jpg` 和 `angle_pic1b/2b/3b.jpg`。这六张照片在板端 `/root/dev_ws/appli/_tmp_scan_imgs/`，可在需要时生成叠加总览；仓库只保存标注数据和代码。旧的第一组标注有误，已从当前文件集移除。

## 文件用途

| 文件 | 用途 |
| --- | --- |
| `source_annotations.json` | 六组原始点列和三角形确认记录；原始 `w` 保留用户提交值 |
| `fit_obstacle_rois.py` | 校验输入、配对同编号区域、融合并清理小幅重叠 |
| `merged_runtime.json` | `school_profile.json` 指向的运行 ROI，只有 `n`、`label`、`pts`、`w` |
| `fit_summary.json` | 每个角度的覆盖、角点误差、重叠清理及置信度 |
| `_fit_output/fit_overview_3angles_2groups.jpg` | 提供 `--image-dir` 时生成的六张原图叠加检查图，不提交到仓库 |

候选点 13 是序号，对应 `map_model.py` 的 `(4,3)` 和串口地图 ID 23。新版运行 ROI 在 45°、90° 包含它；三个角度的序号并集为 1–13。0° 的 label 4、45° 第一组的 label 12 是用户确认的三角形：原点列各有一个内部点，拟合时取外侧三点并重新计算运行面积。45° 第二组的 label 12 有一条短顶边，与第一组三角形配对后标为中等置信度。

## 重算与检查

在项目根目录运行：

```bash
python3 framework/dnn/obstacle_roi/fit_obstacle_rois.py
```

脚本默认将结果写入忽略版本管理的 `framework/dnn/obstacle_roi/_fit_output/`。如六张原图可用，可增加 `--image-dir /root/dev_ws/appli/_tmp_scan_imgs` 生成叠加图。也可用 `--source`、`--out` 指定输入与输出目录。校验会拒绝面积不符、越界、未确认的凹多边形、重复标签，以及无法合理配对或重叠超过较小框 10% 的区域。

核对 `_fit_output/summary.json` 中三个角度的 `overlap_count_after` 均为零、并集为 1–13，逐张查看叠加图；确认后才将 `_fit_output/all_angles_runtime.json` 复制为 `merged_runtime.json`。定位器接受凸三角形或凸四边形。现场仍需用真实障碍逐点确认像素区域，尤其是 0° label 4、45° label 12 和 45°/90° label 13；这里的照片标注检查不等于赛场验收。
