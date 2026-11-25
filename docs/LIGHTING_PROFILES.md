# 相机补光参数选择

2026-11-24用户取消模型切换，要求降低曝光改善圆盘可见性。本轮保持原模型及debug light_unable自启，将light_unable的exposure_time_absolute从150降为50（15ms→5ms），gain仍48、brightness仍128。实时画面观察圆盘轮廓及表面细节改善；配置持久化后重启既有服务并核对参数，不代表圆心精度或颜色准确率验收。备份、对比画面和实际命令见artifacts/exposure_lower_20261124。

2026-11-24本地接入，尚未部署或实机验收。仅选择DCXIN物块相机的曝光参数，不控制补光灯供电。

| 启动参数 | 场景 | auto_exposure | exposure_time_absolute | gain | brightness |
| --- | --- | ---: | ---: | ---: | ---: |
| `light_able`（默认） | 当前补光灯开启 | 1（手动） | 50（5ms） | 16 | 128 |
| `light_unable` | 无补光参数档（本次降低曝光） | 1（手动） | 50（5ms） | 48 | 128 |

来源：当前`config/board.json`的50/16/128，以及只读旧项目`../appli/docs/deployment.md`的“2026-11-14 补光后相机参数”段：明确记载当前50/16/128、补光前原值150/48/128。旧项目不同相机或更早参数不混入本接口。

在**car26_new目录**（部署时对应项目根目录）执行：

```bash
# 默认无雷达，二维码→物块
bash start.sh work light_able
bash start.sh work light_unable
# 独立物块及标靶
bash start_coloritems.sh work light_able
bash start_coloritems.sh work light_unable
# 诊断或仅检查
bash start_coloritems.sh debug light_unable
bash start.sh light_unable --check
```

运行模式和补光参数可交换先后，缺省分别为work和配置的lighting_mode（正式board配置为light_able）。`start.sh without-radar/with-radar`中的项目选择仍放第一位，两分项目入口同样支持补光参数。有雷达配置仍需填写真实设备与标定，选择补光参数不能绕过校验。

共享脚本也支持`--lighting light_unable`。Python/ROS安装后的CLI使用`--lighting light_able|light_unable`。`--check`输出lighting_mode与object_uvc_controls，不运行应用、不打开相机、不调用v4l2；`--check-files`仍会核对真实设备/模型路径但不启动设备。非法模式、重复模式及缺失参数以退出码2拒绝。

两个profile保存在`config/board.json`的`cameras.object.lighting_profiles`中，参数先随extends继承，再由CLI覆盖选择，最终写入内存中的uvc_controls，不改配置文件。未配置lighting_mode的旧模板维持原参数；明确选择不存在或非法profile时拒绝启动。

所选参数覆盖block/disc/target三个采集档位，重开后沿用现有首帧出图→曝光模式3→1→写入曝光/亮度/增益→读回核对→稳定帧的流程。分辨率仍为block 640×360、disc/target 1280×720；指令触发逻辑、二维码相机、模型、串口及对焦/白平衡不变。参数读回成功不等于现场光照与识别精度验收。

本地检查及命令见`artifacts/lighting_profiles_20261124/RESULT.md`、`COMMANDS.md`；修改前副本在其`before/`。本轮不新增启动脚本或原生库；后续部署须同步共享脚本、配置和Python源码，构建ROS2安装副本后再按授权验证设备。
