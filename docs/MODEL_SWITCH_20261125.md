# 2026-11-25 模型替换及 Git 上传

本地已替换为用户提供的 C:/Users/MonthWU/Desktop/model.bin，保留加载文件名 assets/models/yolo11_x5.bin。
新模型：10490514 字节，SHA256 bedffa66d94d8434cade9a42dd3937f135d416857633d632a63a2af71838457f。
旧模型：SHA256 0af8e289007e0ba883fdd1a6bae6f6a7ec2f0821f850443d6180cc43167dd112。

来源：与 2026-11-09 导出清单及此前离线仿真的二进制副本哈希一致；类别顺序与同源 PT 类别元数据及原部署绑定表一致。
顺序：redblock、yellowblock、greenblock、blackblock、lightblueblock、darkblueblock、targetOne、targetTwo、targetThree。
二进制、类别绑定和资产清单已共同更新。独立物块、有雷达、无雷达配置共用该模型。

验证：Python compileall 通过；201 项本地测试中 191 通过、10 项因缺少板端环境跳过。首次回归发现一项契约测试写死旧模型顺序，已按本次模型修正，全部重跑通过。本轮未重新推理；此前同哈希仿真只作为来源证据。

板端：原地址 192.168.137.14 SSH 超时，ARP 未发现设备。尚未板端替换、ARM/ROS2 构建或实机验收。

Git：外层本地仓库无提交、无远程，故以当前 car26_new 为根建立独立源码快照，上传至 https://github.com/MonthWU/The-competition/tree/codex/gongchuan-model-20261125 。包含源码、配置、模型、原生源码、启动脚本、测试与文档；排除仿真、历史备份、缓存、设备生成库及运行日志。
初次快照提交 e9dbd21 已推送且远端核验一致。随后修复了 PowerShell 管道导致的新增中文说明编码损失；最终提交号以 Git 推送回执为准。

本地备份：car26_new/artifacts/model_switch_20261125/before。
本地检查日志：car26_new/artifacts/model_switch_20261125/checks_final.log。
实际命令：car26_new/artifacts/model_switch_20261125/COMMANDS.md。
