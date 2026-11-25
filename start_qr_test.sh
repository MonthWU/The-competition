#!/usr/bin/env bash
# Purpose: start the deployed QR-only preview with a reset button for manual testing.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  --help|-h)
    cat <<'HELP'
用法：bash start_qr_test.sh [--check]
开启二维码识别和网页预览，支持重置；不发送串口、不切换物块相机。
默认前台运行，最多一小时；按 Ctrl+C 停止本次测试。
--check 只检查依赖、配置和相机路径，不打开设备。
如果独立扫码预览已运行，直接显示现有页面地址。
如果物块服务占用网页端口，请先执行：systemctl stop car26-object.service
测试结束后需要恢复物块识别时执行：systemctl start car26-object.service
HELP
    exit 0 ;;
  ''|--check) ;;
  *) echo '参数错误，请使用 --help 查看用法。' >&2; exit 2 ;;
esac
if (( $# > 1 )); then
  echo '参数过多，请使用 --help 查看用法。' >&2
  exit 2
fi
export PYTHONPATH="${project_root}/src/car26_system${PYTHONPATH:+:${PYTHONPATH}}"
export PYTHONDONTWRITEBYTECODE=1
exec python3 -u - "$project_root" "${1:-}" <<'PY'
import json
import runpy
import socket
import sys
import urllib.request
from pathlib import Path

from car26.config import load_config
from car26.camera.module import CameraModule
from car26.camera.preview import QrPreviewState
from car26.hmi.module import HmiModule

root = Path(sys.argv[1])
preview = root / 'artifacts/qr_preview_20261118.py'
if not preview.is_file():
    raise SystemExit('缺少扫码预览程序：' + str(preview))
# 既有预览使用固定板端配置；提前验证，避免运行时才发现路径不符。
board_config = Path('/root/Monthwu/GongChuang_vision_26/config/board.json')
config = load_config(board_config)
if not config.data['hmi']['enabled']:
    raise SystemExit('请先在配置中启用 hmi.enabled。')
device = Path(config.data['cameras']['qr']['device'])
if not device.exists():
    raise SystemExit('二维码相机不存在：' + str(device))
if sys.argv[2] == '--check':
    print('QR_TEST_CHECK_OK：依赖、配置和二维码相机路径有效，未打开设备。')
    raise SystemExit(0)

port = config.data['hmi']['port']
host = config.data['hmi']['host']
local_host = '127.0.0.1' if host == '0.0.0.0' else host
local_url = f'http://{local_host}:{port}'
with socket.socket() as probe:
    occupied = probe.connect_ex((local_host, port)) == 0
if occupied:
    try:
        with urllib.request.urlopen(local_url + '/api/status', timeout=3) as response:
            state = json.load(response)
        reusable = (state.get('application', {}).get('state') == 'qr_preview'
                    and state.get('mission', {}).get('qr_reset_enabled') is True
                    and state.get('mission', {}).get('uart_enabled') is False
                    and state.get('camera', {}).get('role') == 'qr')
    except Exception:
        reusable = False
    if reusable:
        print(f'独立扫码预览已在运行，请打开 http://<板卡IP>:{port}/ ，可使用页面重置按钮。')
        print('本次没有新建进程；Ctrl+C 不会停止此前运行的预览。')
        raise SystemExit(0)
    raise SystemExit(f'网页端口 {port} 已被占用。若为物块服务，请先执行：systemctl stop car26-object.service')

print(f'开启扫码测试，请打开 http://<板卡IP>:{port}/ ，最长一小时，Ctrl+C 停止。')
runpy.run_path(str(preview), run_name='__main__')
PY
