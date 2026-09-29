#!/bin/bash
# =============================================================================
# start_old.sh —— 部分流程启动（老流程 + 二维码先跑）
#
#   阶段 1  二维码扫描     ：KS1A293 (240fps@640x400) → qrc_cam → qrc_scanner → qrc_cam_killer
#                            扫到有效码后杀掉扫码链路（按 README §2 行为）
#   阶段 2  物块识别       ：检测相机接管 → obj_dnn → 串口下发 + Web 预览 (8000)
#
# **跳过障碍物预扫描阶段** —— 与 start_new.sh 的唯一区别。
#
# 用法:  bash start_old.sh
# =============================================================================

# 2026-09-28 修：去掉顶层 set -u
#   - /opt/tros/humble/setup.bash 在 set -u 下会因 AMENT_TRACE_SETUP_FILES 未定义而崩
#   - 改为每个 source 临时关 set -u，source 完恢复
# 关键链路详见 framework/launch/prescan_native.launch.py / obj_detect_v11_native.launch.py / qrc_skandier.launch.py

HOME=/root
WS=$HOME/dev_ws/appli

export CAM_TYPE=usb
export ROS_DOMAIN_ID=42

set +u
source /opt/tros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

echo "=============================================================="
echo " start_old.sh —— 老流程 + 二维码先跑"
echo " 工作区: $WS"
echo "=============================================================="

# ---------- 清理残留（防止端口/相机冲突）----------
echo
echo ">>> [0/3] 清理上一轮残留（launch / nginx / http server / qrc 链路）"
pkill -TERM -f 'ros2 launch.*/root/dev_ws/appli' 2>/dev/null
sleep 2
pkill -TERM -f 'qrc_cam\b|qrc_scanner\b|qrc_cam_killer\b' 2>/dev/null
pkill -TERM -f 'lib/websocket/websocket' 2>/dev/null
pkill -TERM -f 'python3 -m http.server' 2>/dev/null
pkill -KILL -f 'nginx: master process ./sbin/nginx' 2>/dev/null
sleep 2
# 兜底：按端口杀
for port in 8000 8080 8888; do
    PIDS=$(ss -tlnp 2>/dev/null | awk -v p=":$port" '$0 ~ p {print $0}' | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u)
    for pid in $PIDS; do
        [ -n "$pid" ] && kill -KILL "$pid" 2>/dev/null
    done
done
sleep 2
echo ">>> 残留进程: $(ps -ef | grep -E 'ros2 launch|qrc_skandier|prescan|obs_dnn|websocket|nginx|http.server' | grep -v grep | wc -l)"
echo ">>> 端口 8000/8080/8888: $(ss -tlnp 2>/dev/null | grep -E ':(8000|8080|8888)' | wc -l)"

# ---------- 阶段 1 + 2：二维码 → 物块识别 ----------
echo
echo ">>> [1/3] 二维码阶段（KS1A293 → qrc_cam/scanner/killer，扫到有效码后自动退出）"
echo ">>> [2/3] 物块识别阶段（LRCP AR0234 → obj_dnn → 串口 + Web 8000）"
echo ">>> 说明：二维码与物块是串行（qrc_cam_killer 发 /kill_qrc 杀掉扫码链路后启动物块）"
echo ">>> 启动 run_all.launch.py（包含 qrc_skandier + obj_detect_v11_native）"
echo

cd "$WS" || exit 1
ros2 launch "$WS/launch/run_all.launch.py"
MAIN_RC=$?

echo
echo "=============================================================="
echo " 结束：主任务退出码=$MAIN_RC"
echo "=============================================================="
exit "$MAIN_RC"