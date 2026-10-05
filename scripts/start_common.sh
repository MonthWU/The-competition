#!/bin/bash
# Shared implementation for the two project entry points.

appli_preflight() {
    local missing=0
    local path
    local required=(
        /dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0
        "$SERIAL_DEVICE"
        "$WS/install/setup.bash"
        /opt/tros/humble/setup.bash
    )
    if [ "$ENTRY_MODE" = all ] && [ "${1:-full}" != wait_for_start ]; then
        required+=(
            /dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0
            "$WS/framework/dnn/yolo11_x5_obstacle.bin"
        )
    fi
    if [ "$OBJECT_MODE" = full ]; then
        required+=(
            /dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0
            "$WS/dnn/yolo11_x5.bin"
        )
    fi
    for path in "${required[@]}"; do
        if [ ! -e "$path" ]; then
            printf '[appli] MISSING_REQUIRED_RESOURCE: %s\n' "$path" >&2
            missing=1
        fi
    done
    return "$missing"
}

appli_start() {
    ENTRY_MODE="$1"
    shift
    local option="${1:-}"
    if [ "$option" = --help ] || [ "$option" = -h ]; then
        if [ "$ENTRY_MODE" = all ]; then
            printf 'Usage: bash start_all.sh [timeout_seconds|--check|--help]\n'
        else
            printf 'Usage: bash start_simple.sh [--check|--help]\n'
        fi
        return 0
    fi
    if [ "$#" -gt 1 ]; then
        printf '[appli] INVALID_ARGUMENTS: use --help\n' >&2
        return 2
    fi
    PRESCAN_TIMEOUT="${option:-30}"
    if [ "$option" != --check ]; then
        if [ "$ENTRY_MODE" = all ]; then
            if ! [[ "$PRESCAN_TIMEOUT" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
                printf '[appli] INVALID_PRESCAN_TIMEOUT: %s\n' "$PRESCAN_TIMEOUT" >&2
                return 2
            fi
        elif [ -n "$option" ]; then
            printf '[appli] INVALID_ARGUMENTS: use --help\n' >&2
            return 2
        fi
    fi
    if [ "$option" != --check ]; then
        python3 "$WS/scripts/stop_project.py" --root "$WS" --caller "$$" || return 2
        # A stopped prescan must not fall through into a new QR launch.
        trap 'exit 130' INT
        trap 'exit 143' TERM
    fi
    PROFILE="${APPLI_SCHOOL_PROFILE:-$WS/framework/school_profile.json}"
    SERIAL_DEVICE="${APPLI_SERIAL_DEVICE:-/dev/ttyS1}"
    OBJECT_MODE=$(python3 - "$PROFILE" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    enabled = json.load(stream).get("object_scan_enabled", True)
if not isinstance(enabled, bool):
    raise ValueError("object_scan_enabled must be a JSON boolean")
print("full" if enabled else "qr_only")
PY
    ) || return 2
    if [ "$option" = --check ]; then
        appli_preflight
        return "$?"
    fi
    # Defer obstacle-only resources until the MCU selects a numeric start.
    appli_preflight wait_for_start || return 2

    export CAM_TYPE=usb
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
    export ROS_HOME="${ROS_HOME:-/root/.ros}"
    export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROS_HOME/log}"
    mkdir -p "$ROS_LOG_DIR" || return 2

    # TROS setup reads unset variables; enable nounset only afterward.
    source /opt/tros/humble/setup.bash || return 2
    source "$WS/install/setup.bash" || return 2
    set -u

    if [ "$ENTRY_MODE" = all ]; then
        printf '[appli] STAGE_1_OBSTACLE_SCAN\n'
        cd "$WS/framework" || return 2
        python3 prescan_main.py "$PRESCAN_TIMEOUT" "$SERIAL_DEVICE"
        local prescan_rc=$?
        if [ "$prescan_rc" -ne 0 ]; then
            printf '[appli] PRESCAN_FAILED: rc=%s; main task not started\n' "$prescan_rc" >&2
            return "$prescan_rc"
        fi
    fi

    cd "$WS" || return 2
    if [ "$OBJECT_MODE" = qr_only ]; then
        printf '[appli] STAGE_2_QR_SCAN_ONLY\n'
        exec ros2 launch "$WS/launch/run_qr_only.launch.py" "serial_device:=$SERIAL_DEVICE"
    fi
    printf '[appli] STAGE_2_QR_SCAN_THEN_OBJECT_SCAN\n'
    exec ros2 launch "$WS/launch/run_all.launch.py" "serial_device:=$SERIAL_DEVICE"
}
