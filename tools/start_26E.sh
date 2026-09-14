#!/usr/bin/env bash

set -o pipefail
umask 022

readonly PROJECT_DIR="${HOME}/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision"
readonly WEB_URL="http://127.0.0.1:5000/"
readonly LOCK_FILE="/tmp/start_26E.lock"

project_is_running() {
  pgrep -f "${PROJECT_DIR}/install/(puzzle_perception_node|puzzle_solver_node|puzzle_coordinator_node|serial_bridge_node|hmi_control_node|web_tuner_node)/" >/dev/null 2>&1
}

web_is_ready() {
  curl --fail --silent --show-error --max-time 2 "${WEB_URL}" >/dev/null 2>&1
}

wait_for_web() {
  local attempts="${1:-60}"
  local attempt
  for ((attempt = 1; attempt <= attempts; ++attempt)); do
    if web_is_ready; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

open_preview() {
  if command -v chromium-browser >/dev/null 2>&1; then
    nohup chromium-browser \
      --disable-gpu \
      --disable-dev-shm-usage \
      --new-window \
      "${WEB_URL}" >/dev/null 2>&1 &
    return 0
  fi
  if command -v chromium >/dev/null 2>&1; then
    nohup chromium \
      --disable-gpu \
      --disable-dev-shm-usage \
      --new-window \
      "${WEB_URL}" >/dev/null 2>&1 &
    return 0
  fi
  if command -v xdg-open >/dev/null 2>&1; then
    nohup xdg-open "${WEB_URL}" >/dev/null 2>&1 &
    return 0
  fi
  printf '未找到可用浏览器，请手动打开 %s\n' "${WEB_URL}" >&2
  return 1
}

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  printf 'start_26E 正在启动项目，等待 Web 预览就绪。\n'
  if wait_for_web 90; then
    open_preview
    exit 0
  fi
  printf '等待 Web 预览超时，请检查另一个 start_26E 终端。\n' >&2
  exit 1
fi

if project_is_running; then
  if wait_for_web 10; then
    printf '26E debug 项目已在运行，直接打开现有 Web 预览。\n'
    open_preview
    exit 0
  fi
  printf '检测到 26E 项目进程，但 Web 预览未就绪。为避免摄像头、串口或端口冲突，本次不重复启动。\n' >&2
  exit 1
fi

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  printf '缺少 /opt/ros/humble/setup.bash，无法启动 ROS 2。\n' >&2
  exit 1
fi
if [[ ! -r "${PROJECT_DIR}/install/setup.bash" ]]; then
  printf '缺少 %s/install/setup.bash，请先构建 26E 项目。\n' "${PROJECT_DIR}" >&2
  exit 1
fi
if ! command -v curl >/dev/null 2>&1; then
  printf '缺少 curl，无法检查 Web 预览状态。\n' >&2
  exit 1
fi

mkdir -p "${PROJECT_DIR}/logs"
readonly LOG_FILE="${PROJECT_DIR}/logs/start_26E_$(date +%Y%m%d-%H%M%S).log"
exec > >(tee -a "${LOG_FILE}") 2>&1

printf '启动 26E 项目：debug 模式，Web 预览已启用。\n'
printf '日志：%s\n' "${LOG_FILE}"

source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
cd "${PROJECT_DIR}"

preview_wait_pid=""
(
  if wait_for_web 90; then
    open_preview
  else
    printf 'Web 预览在 45 秒内未就绪，请检查本终端日志。\n' >&2
  fi
) &
preview_wait_pid=$!

cleanup_preview_waiter() {
  if [[ -n "${preview_wait_pid}" ]]; then
    kill "${preview_wait_pid}" >/dev/null 2>&1 || true
    wait "${preview_wait_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup_preview_waiter EXIT

ros2 launch vision_bringup vision_system.launch.py mode:=debug web_enabled:=true
exit_code=$?
printf '26E debug 项目已退出，退出码：%d\n' "${exit_code}"
exit "${exit_code}"
