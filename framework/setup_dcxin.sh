#!/bin/bash
# setup_dcxin.sh —— DCXIN 相机亮度/增益修正（2026-09-25）
#
# 为什么需要这个脚本
# ------------------
# 该相机固件的 auto_exposure 只接受 1(Manual) / 3(Aperture Priority)，
# **没有真正的 Auto(0)**（实测设 0 报 Invalid argument）。
# 而出厂默认的 3 在 UVC 摄像头上是空转的——没有可变光圈可调，
# 曝光时间控制被标记为 inactive（只读），因此画面亮度实际只由
# brightness / gain 决定。出厂值 brightness=50 / gain=0 明显偏暗：
#
#     出厂状态          ：画面均值 68.6（中央区仅 28.8）
#     修正后(128/48)    ：画面均值 138.9
#
# ⚠️ 每次 USB 复位 / 重新上电后这两个参数会还原为出厂值，需重新执行本脚本。
#    因此 prescan.launch.py 在起相机之前会自动调用它。
#
# 用法：
#   bash framework/setup_dcxin.sh [by-id设备路径]

DEV="${1:-/dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0}"

if [ ! -e "$DEV" ]; then
    echo "[setup_dcxin] 警告：设备不存在 $DEV —— 跳过（不阻塞启动）"
    exit 0
fi

echo "[setup_dcxin] 应用亮度/增益修正: $DEV"

# 保持出厂默认的 Aperture Priority 模式（唯一可用的"自动"档），
# 把 brightness / gain 补到该相机固件的默认值。
v4l2-ctl -d "$DEV" --set-ctrl=auto_exposure=3 2>/dev/null
v4l2-ctl -d "$DEV" --set-ctrl=brightness=128 2>/dev/null
v4l2-ctl -d "$DEV" --set-ctrl=gain=48 2>/dev/null

echo "[setup_dcxin] 当前参数："
v4l2-ctl -d "$DEV" --get-ctrl=auto_exposure,brightness,gain 2>&1

exit 0
