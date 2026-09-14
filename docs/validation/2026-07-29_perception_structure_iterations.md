# 2026-07-29 感知代码结构迭代记录

## 范围与停止条件

- 范围：`Copyfiles/26E_vision`，优先优化绿纸/白块 HSV 感知链路及其测试边界。
- 不修改：标准 `Copyfiles/ros2_ws`、求解器业务、串口协议和真实硬件配置。
- 迭代上限为 10 轮；当后续候选只剩机械搬移、没有新增测试边界或依赖收益时提前停止。

## 迭代结果

1. 建立源码规模、职责和重复逻辑基线；确定感知节点为本轮唯一高收益热点。
2. 将绿纸、白块和磁铁的 18 个整数成员收敛为 3 个 `HsvRange`，ROS/YAML 参数名不变。
3. 新增纯函数 `piece_foreground_mask.hpp`，从轮廓几何中拆出 HSV、颜色排除、页边、分界带和形态学职责。
4. 新增组合测试，覆盖白块保留、绿纸排除、磁铁排除、页边清零、分界带清零和非法参数闭合失败。
5. 集中 HSV 启动校验；非法范围、绿白重叠、白块与磁铁排除范围重叠均明确拒绝。
6. 用 CMake 函数统一三个测试目标，移除无用 include，并修正 `CHAIN_APPROX_NONE` 文档漂移。
7. 复审并修复独立掩膜默认分界带会隐式清掉一行的问题；负值现在明确表示禁用。
8. 第 8--10 轮未强行执行：剩余候选主要是继续拆分 128 行轮廓流程，不能增加可测试边界或降低跨包依赖，收益不足以承担回归风险。

## 结构指标

```text
puzzle_perception_node.cpp: 722 -> 705 lines
extract_pieces():           154 -> 128 lines
HSV primitive members:       18 -> 0
typed HSV groups:              0 -> 3
node inline colour-mask calls: 2 -> 0
```

新增前景模块为 90 行左右的头文件纯函数，不依赖 ROS 节点状态，可直接单元测试。

## Windows 静态检查

```text
YAML_PARAMETER_COMPATIBILITY_OK
HSV_PARAMETER_KEYS=18
HSV_TYPED_GROUPS=3
HSV_VALIDATION_GATES=3
NODE_INLINE_COLOR_MASK_CALLS=0
STALE_REFERENCES=0
BACKUP_FILES=8
BACKUP_MISMATCH=0
```

## Jetson 构建与测试

- 身份：`aarch64 / monthwu-jetson / jetson`
- 最终构建：`puzzle_perception_node` 成功，无编译警告。
- 首次构建发现 ROS 整数参数返回值的窄化警告；在 Windows 主副本显式转换后重新部署，最终清除。
- 感知包相关测试：3/3 通过。
- 工作区累计：`12 tests, 0 errors, 0 failures, 0 skipped`。

```text
green_a4_detector       Passed
hsv_object_mask         Passed
piece_foreground_mask   Passed
```

## 节点烟测与清理

使用 GStreamer `videotestsrc` 合成画面启动 `puzzle_perception_node`，未占用真实摄像头或串口：

```text
NODE_LIST_DURING
/puzzle_perception_node
NODE_LIST_AFTER
PROCESS_GROUP_AFTER
```

首次烟测因 SSH 命令行未保留带空格的管线参数而在 ROS 参数解析前退出；改用编码脚本保留参数边界后通过。该问题不涉及项目源码。

最终清理与部署核对：

```text
DEPLOY_HASH_FILES=10
DEPLOY_HASH_MISMATCH=0
RESIDUAL_PROCESS_COUNT=0
```

## 备份与回滚

- Windows 修改前备份：
  `D:/CodexFolder/JetsonNano/Copyfiles/backups/26E_vision/20260729_200611_structure_iterations`
- Jetson 修改前备份：
  `/home/jetson/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729_201502_structure_iterations`
- Jetson 备份 SHA256：
  `0d5d970079b2cfa5594ebb5a8124ddb339237b3846ae196afa25879795823cd7`

Windows 回滚按 `backup_manifest.md` 恢复 8 个既有文件，并删除清单列出的 3 个新增文件。
Jetson 回滚时将 `pre_structure_iterations.tar.gz` 解包回 `26E_vision`，删除以下新增文件，
然后重新构建 `puzzle_perception_node`：

```text
src/puzzle_perception_node/include/puzzle_perception_node/piece_foreground_mask.hpp
src/puzzle_perception_node/test/test_piece_foreground_mask.cpp
docs/validation/2026-07-29_perception_structure_iterations.md
```

## 验证边界

本轮证明的是代码结构、参数兼容、合成掩膜行为和节点启动清理；没有使用真实绿纸、白色拼图或现场光照，因此不新增任何真实识别精度结论。
