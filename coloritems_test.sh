#!/bin/bash
# coloritems_test —— 物块识别调试入口：跳过障碍预扫描与二维码阶段，
# 直接启动 obj_detect_v11 物块链路。camera_daemon:=false 使 LRCP 相机
# 立即出图（不等 /kill_qrc），便于现场调试物块颜色/置信度/误检。
# 网页叠加画面: http://<板端IP>:8000
WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || exit 2

usage() {
    printf 'Usage: bash coloritems_test.sh [--check] [ros2 launch 参数...]\n'
    printf '跳过障碍/二维码，直接物块识别调试；参数原样透传 obj_detect_v11.launch.py。\n'
    printf '例: bash coloritems_test.sh dnn_engine:=native score_threshold:=0.5\n'
}

CHECK_ONLY=0
EXTRA=()
for arg in "$@"; do
    case "$arg" in
        --check) CHECK_ONLY=1 ;;
        --help|-h) usage; exit 0 ;;
        *) EXTRA+=("$arg") ;;
    esac
done

SERIAL_DEVICE="${APPLI_SERIAL_DEVICE:-/dev/ttyS1}"

# 预检：物块链路所需设备与模型。不需要扫码相机 KS1A293、
# 障碍相机 DCXIN 与障碍模型。预检先于停止旧任务：资源缺失时
# 不打断正在运行的正式流程。
missing=0
for path in \
    /dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0 \
    "$WS/dnn/yolo11_x5.bin" \
    "$WS/dnn/classes.names" \
    "$SERIAL_DEVICE" \
    /opt/tros/humble/setup.bash \
    "$WS/install/setup.bash"; do
    [ -e "$path" ] || { printf '[coloritems] MISSING_REQUIRED_RESOURCE: %s\n' "$path" >&2; missing=1; }
done
[ "$missing" -eq 0 ] || exit 1

if [ "$CHECK_ONLY" -eq 1 ]; then
    printf '[coloritems] CHECK_OK\n'
    exit 0
fi

python3 "$WS/scripts/stop_project.py" --root "$WS" --caller "$$" || exit 2
# A stopped prescan must not fall through into a new QR launch.
trap 'exit 130' INT
trap 'exit 143' TERM

export CAM_TYPE=usb
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_HOME="${ROS_HOME:-/root/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROS_HOME/log}"
mkdir -p "$ROS_LOG_DIR" || exit 2

# TROS setup reads unset variables; enable nounset only afterward.
source /opt/tros/humble/setup.bash || exit 2
source "$WS/install/setup.bash" || exit 2
set -u

printf '[coloritems] OBJECT_DETECT_ONLY; launch args: %s\n' "${EXTRA[*]:-（默认）}"
cd "$WS" || exit 2
exec ros2 launch "$WS/obj_detect/launch/obj_detect_v11.launch.py" \
    "camera_daemon:=false" "serial_device:=$SERIAL_DEVICE" "${EXTRA[@]}"
