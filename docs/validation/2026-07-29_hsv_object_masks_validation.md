# 2026-07-29 绿纸与白色拼图独立 HSV 验证

## 修改范围

- Windows 权威副本：`D:/CodexFolder/JetsonNano/Copyfiles/26E_vision`
- Jetson 部署镜像：`/home/jetson/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision`
- 目标身份：`aarch64 / monthwu-jetson / jetson`
- 绿纸/绿框范围：`H 60--90, S 78--201, V 91--245`
- 白色拼图初始范围：`H 0--179, S 0--77, V 160--255`

白色范围的 H 不限，因为低饱和白色的色相不稳定；其 S 上限低于绿色 S 下限，保证两套
三维 HSV 区间不重叠。白色阈值尚未用现场实拍样本标定，只作为独立且可运行时调整的初值。

## 实现门控

- `green_a4_*` 只用于绿纸/绿框检测。
- `white_piece_*` 只用于白色拼图块前景提取。
- 两套参数均从 `competition_tuning.yaml` 运行时加载。
- 参数超出 OpenCV HSV 范围，或绿色与白色三维范围发生重叠时，节点启动失败并报告
  `BUG_POINT:HSV_RANGE` 或 `BUG_POINT:HSV_OBJECT_OVERLAP`。
- 白色拼图提取不再复用绿色范围，也不再走原 Lab 背景距离主路径。

## Windows 合成验证

YAML 解析及 7 个边界/反例像素结果：

```text
YAML_PARSE_OK
GREEN_HSV=(60, 90, 78, 201, 91, 245)
WHITE_HSV=(0, 179, 0, 77, 160, 255)
SYNTHETIC_GREEN_MASK=[255, 255, 0, 0, 0, 0, 0]
SYNTHETIC_WHITE_MASK=[0, 0, 255, 255, 0, 0, 0]
MASK_OVERLAP_PIXELS=0
```

其中绿色数组前两项对应绿色上下边界；白色数组第 3、4 项对应典型低饱和高亮白色。

## Jetson 构建与测试

选择性构建结果：

```text
Finished <<< puzzle_perception_node
Finished <<< vision_bringup
Summary: 2 packages finished
```

新增/更新的两个感知测试：

```text
green_a4_detector  Passed
hsv_object_mask    Passed
100% tests passed, 0 tests failed out of 2
```

工作区累计测试结果：

```text
Summary: 11 tests, 0 errors, 0 failures, 0 skipped
```

最终部署核对：14 个本次相关文件与 Windows 权威副本逐一 SHA256 一致，差异数为 0；
测试完成后 `ros2 node list` 为空。

## 回滚

- Windows 修改前备份：
  `D:/CodexFolder/JetsonNano/Copyfiles/backups/26E_vision/20260729_194746_hsv_masks`
- Jetson 修改前备份：
  `/home/jetson/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729_195333_hsv_masks`
- Jetson 备份包 SHA256：
  `4343cc6ebc05336cb8458379a7b4c29bb65f7dd76048aae23114f093648bc1d1`

Windows 备份中 11 个既有文件的 SHA256 复核差异数为 0；Jetson 归档可正常列出和解压读取。

Windows 回滚时按 `backup_manifest.md` 恢复 11 个既有文件，并删除清单中的 3 个新增文件。
Jetson 回滚时把 `pre_hsv_masks_source.tar.gz` 解包回 `26E_vision`，再删除以下在该部署镜像中
原本不存在的文件，最后重新构建感知与 bringup 包：

```text
src/puzzle_perception_node/include/puzzle_perception_node/hsv_object_mask.hpp
src/puzzle_perception_node/test/test_hsv_object_mask.cpp
docs/puzzle_piece_scheme_selection.md
docs/validation/2026-07-29_hsv_object_masks_validation.md
```

## 未覆盖

本轮没有启动真实摄像头或串口，也没有用户现场绿纸/白色拼图图像，因此尚不能证明真实光照
下的召回率、误检率和轮廓毫米误差。现场验证应分别统计绿色 mask、白色 mask，并确认交集、
阴影区域和高光区域；只修改对应对象的 YAML 参数，不得让两套范围重叠。
