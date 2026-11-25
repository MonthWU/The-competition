# 当前模型类别映射（2026-11-25）

2026-11-25 用户要求更换桌面 model.bin 并 Git 上传。本地物块模型现为 bedffa66d94d8434cade9a42dd3937f135d416857633d632a63a2af71838457f（10490514 字节），类别索引为 redblock、yellowblock、greenblock、blackblock、lightblueblock、darkblueblock、targetOne、targetTwo、targetThree。已共同更新模型、类别绑定与资产清单，核对同哈希导出记录及同源 PT 类别元数据。此前旧模型说明为历史记录。板端 SSH 超时且 ARP 未发现设备，本轮尚未部署或实机验收。详情见 docs/MODEL_SWITCH_20261125.md。

| 输出索引 | 来源名称 | 业务名称 |
| --- | --- | --- |
| 0 | red1 | redblock |
| 1 | yellow1 | yellowblock |
| 2 | green1 | greenblock |
| 3 | black1 | blackblock |
| 4 | blue1 | lightblueblock |
| 5 | blue2 | darkblueblock |
| 6 | targetOne | targetOne |
| 7 | targetTwo | targetTwo |
| 8 | targetThree | targetThree |

blue1 为浅蓝 0x45，blue2 为深蓝 0x46；串口码由业务名称决定。业务目录 classes.name 行号不是模型输出索引。

以下为旧模型历史记录：

# 当前模型输出索引与业务类别

2026-11-24已切回旧模型并纠正索引。模型SHA256：`0af8e289007e0ba883fdd1a6bae6f6a7ec2f0821f850443d6180cc43167dd112`。

原始证据：`artifacts/RDK_CJLU_V01/dnn/yolo11_x5.bin`与当前模型SHA256相同，使用同目录配套`classes.names`；V05/V06归档`object.names`顺序一致。此前沿用新模型顺序的结论作废。

| 输出索引 | 原始名称 | 业务名称 | 串口语义码 |
| --- | --- | --- | --- |
| 0 | targetOne | targetOne | 标靶0x72姿态包 |
| 1 | targetTwo | targetTwo | 标靶0x72姿态包 |
| 2 | targetThree | targetThree | 标靶0x72姿态包 |
| 3 | red1 | redblock | 0x41 |
| 4 | black1 | blackblock | 0x42 |
| 5 | green1 | greenblock | 0x43 |
| 6 | yellow1 | yellowblock | 0x44 |
| 7 | blue1 | lightblueblock | 0x45 |
| 8 | blue2 | darkblueblock | 0x46 |

blue1为浅蓝、blue2为深蓝；串口业务码按类别字符串决定，不按模型输出索引计算。`classes.name`仅作为业务类别目录，其行号不是模型输出索引。

新模型bedffa66...的历史顺序是red1、yellow1、green1、black1、blue1、blue2、targetOne、targetTwo、targetThree，不能用于当前0af8e289...旧模型。历史文件保存在`artifacts/old_model_20261124/before`及`artifacts/old_model_order_20261124/before`。

`classmap.load_model_labels`核对模型SHA256、索引连续性及标签唯一性后返回输出顺序。新增按已核实模型哈希对应原始顺序的回归检查，避免只修改哈希却沿用另一模型的映射。板端201项回归通过，保持曝光50及`debug light_unable`自启；颜色与定位的实物精度仍需验收。

备份、部署及实际命令见`artifacts/old_model_order_20261124/REPORT.md`和`COMMANDS.md`。
