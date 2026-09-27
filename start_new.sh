#!/bin/bash
# =============================================================================
# start_new.sh —— 完整流程启动（新流程）
#
#   阶段 1  障碍物预扫描   ：DCXIN 全局扫描 → 障碍识别 → 下发地图帧给下位机
#   阶段 2  二维码扫描     ：扫码 → 扫到有效码后杀掉扫码链路
#   阶段 3  物块识别       ：检测相机接管 → YOLO 物块识别 → 串口下发 + Web 预览
#
# 与 start_old.sh 的唯一区别：**多了阶段 1**。
#
# 用法:  bash start_new.sh [预扫描超时秒数，默认 30]
# =============================================================================

set -u

HOME=/root
WS=$HOME/dev_ws/appli
PRESCAN_TIMEOUT="${1:-30}"

export CAM_TYPE=usb
export ROS_DOMAIN_ID=42

source /opt/tros/humble/setup.bash
source "$WS/install/setup.bash"

echo "=============================================================="
echo " start_new.sh —— 完整流程（障碍物 → 二维码 → 物块）"
echo " 工作区: $WS    预扫描超时: ${PRESCAN_TIMEOUT}s"
echo "=============================================================="

# ---------- 阶段 1：障碍物预扫描 ----------
echo
echo ">>> [1/2] 障碍物预扫描阶段（DCXIN + 障碍模型）"
echo ">>> 等待下位机发 [num] 启停位置 → 3 次 [shot] → 地图帧"
cd "$WS/framework" || exit 1
python3 prescan_main.py "$PRESCAN_TIMEOUT"
PRESCAN_RC=$?
echo ">>> 预扫描退出码: $PRESCAN_RC  （0=成功 / 1=失败）"
if [ "$PRESCAN_RC" -ne 0 ]; then
    echo ">>> ⚠️ 预扫描未成功（超时或串口异常）。"
    echo ">>>    按协议「无障碍也算完成」，此处**仍继续**主任务；"
    echo ">>>    如需严格模式，请把下面一行改为 exit $PRESCAN_RC"
fi

# ---------- 阶段 2 + 3：二维码 + 物块识别 ----------
echo
echo ">>> [2/2] 主任务阶段（二维码扫描 → 物块识别）"
cd "$WS" || exit 1
ros2 launch "$WS/launch/run_all.launch.py"
MAIN_RC=$?

echo
echo "=============================================================="
echo " 结束：预扫描=$PRESCAN_RC  主任务=$MAIN_RC"
echo "=============================================================="
exit "$MAIN_RC"
