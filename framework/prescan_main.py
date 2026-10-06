#!/usr/bin/env python3
"""障碍物预扫描阶段入口（供 start_all.sh 调用）。

流程（协议 v3，见 README §6.1）：
  等串口 [num]（启停位置）→ 回 [ack]
  → 等 3 次 [shot]（云台 0/45/90°，每次拍一帧做障碍识别）→ 每次回 [ack]
  → 三次完成后发地图帧 [count 障碍ID… CHK]

用法:
  python3 prescan_main.py [超时秒数] [串口设备]
    超时秒数  默认 30（每一步等 [num]/[shot] 的最大等待）
    串口设备  默认用 serial_protocol.SERIAL_DEV（/dev/ttyS1）

退出码: 0=成功  1=失败（超时/串口打不开等）
"""
import sys

from mission_dispatcher import MissionDispatcher

timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
dev = sys.argv[2] if len(sys.argv) > 2 else None

print("[prescan] 障碍物预扫描阶段启动（超时 %.0fs，串口 %s）"
      % (timeout, dev or "默认(/dev/ttyS1)"))
print("[prescan] 等待下位机发 [num] 启停位置 ...")

disp = MissionDispatcher()
ok = disp.run_prescan(dev=dev, trigger_timeout=timeout)

print("[prescan] 结果: %s" % ("成功" if ok else "失败"))
print("[prescan] 串口已释放（后续主任务由 obj_serial 使用）")
sys.exit(0 if ok else 1)
