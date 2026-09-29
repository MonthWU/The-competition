#!/bin/bash
# School vision flow. The profile can stop after QR while the object camera is unavailable.

WS=/root/dev_ws/appli
PRESCAN_TIMEOUT="${1:-30}"
PROFILE="${APPLI_SCHOOL_PROFILE:-$WS/framework/school_profile.json}"
SERIAL_DEVICE="${APPLI_SERIAL_DEVICE:-/dev/ttyS1}"

MODE=$(python3 - "$PROFILE" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    enabled = json.load(stream).get("object_scan_enabled", True)
if not isinstance(enabled, bool):
    raise ValueError("object_scan_enabled must be a JSON boolean")
print("full" if enabled else "qr_only")
PY
) || exit 2

preflight() {
    local missing=0
    local path
    for path in \
        /dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0 \
        /dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0 \
        "$SERIAL_DEVICE" \
        "$WS/framework/dnn/yolo11_x5_obstacle.bin"; do
        if [ ! -e "$path" ]; then
            printf '[appli] MISSING_REQUIRED_RESOURCE: %s\n' "$path" >&2
            missing=1
        fi
    done
    if [ "$MODE" = full ]; then
        for path in \
            /dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0 \
            "$WS/dnn/yolo11_x5.bin"; do
            if [ ! -e "$path" ]; then
                printf '[appli] MISSING_REQUIRED_RESOURCE: %s\n' "$path" >&2
                missing=1
            fi
        done
    fi
    return "$missing"
}

if [ "$PRESCAN_TIMEOUT" = --check ]; then
    preflight
    exit $?
fi
if ! [[ "$PRESCAN_TIMEOUT" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    printf '[appli] INVALID_PRESCAN_TIMEOUT: %s\n' "$PRESCAN_TIMEOUT" >&2
    exit 2
fi
if ! preflight; then
    exit 2
fi

export CAM_TYPE=usb
export ROS_DOMAIN_ID=42

# The TROS setup script reads unset variables; enable nounset only afterward.
source /opt/tros/humble/setup.bash || exit 2
source "$WS/install/setup.bash" || exit 2
set -u

printf '[appli] STAGE_1_OBSTACLE_SCAN\n'
cd "$WS/framework" || exit 2
python3 prescan_main.py "$PRESCAN_TIMEOUT" "$SERIAL_DEVICE"
prescan_rc=$?
if [ "$prescan_rc" -ne 0 ]; then
    printf '[appli] PRESCAN_FAILED: rc=%s; main task not started\n' "$prescan_rc" >&2
    exit "$prescan_rc"
fi

cd "$WS" || exit 2
if [ "$MODE" = qr_only ]; then
    printf '[appli] STAGE_2_QR_SCAN_ONLY\n'
    exec ros2 launch "$WS/launch/run_qr_only.launch.py" "serial_device:=$SERIAL_DEVICE"
fi
printf '[appli] STAGE_2_QR_SCAN_THEN_OBJECT_SCAN\n'
exec ros2 launch "$WS/launch/run_all.launch.py"
