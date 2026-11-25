# Purpose: verify runtime output and exit codes with a fake application that never opens hardware.
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from car26.config import load_config
from test_application import ROOT


PROBE = '''
import logging, os, sys
from unittest.mock import Mock, patch
from car26.cli import main
class FakeApplication:
    # ***
    # 函数名字：__init__
    # 函数作用：为无设备测试创建假的应用状态容器。
    # 参数意义与范围：
    #   self：当前FakeApplication实例。
    #   configuration：已验证Configuration对象，仅记录配置，不打开设备。
    # ***
    def __init__(self, configuration):
        self.telemetry = Mock()
    # ***
    # 函数名字：run
    # 函数作用：输出人工诊断并返回非零状态以测试日志路由与退出码。
    # 参数意义与范围：
    #   self：当前FakeApplication实例。
    # ***
    def run(self):
        print('PYTHON_STDOUT')
        print('PYTHON_STDERR', file=sys.stderr)
        os.write(1, b'NATIVE_STDOUT\\n')
        os.write(2, b'NATIVE_STDERR\\n')
        if sys.platform.startswith('linux'):
            import ctypes
            ctypes.CDLL(None).printf(b'NATIVE_BUFFERED\\n')
        logging.warning('WARNING_RECORD')
        logging.debug('DEBUG_RECORD')
        return 7
with patch('car26.application.Application', FakeApplication):
    raise SystemExit(main(['--config', sys.argv[1], '--mode', sys.argv[2]]))
'''


class StartupModeTests(unittest.TestCase):
    # ***
    # 函数名字：run_probe
    # 函数作用：在独立进程中运行假的应用并收集终端输出及日志。
    # 参数意义与范围：
    #   self：当前StartupModeTests实例。
    #   mode：待验证模式；仅允许work或debug。
    # ***
    def run_probe(self, mode):
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory)
            config = project / 'config' / 'board.json'
            config.parent.mkdir()
            data = load_config(ROOT / 'config/board-object.json').data
            config.write_text(json.dumps(data), encoding='utf8')
            result = subprocess.run([sys.executable, '-c', PROBE, str(config), mode],
                                    capture_output=True, text=True)
            logs = [p.read_text(encoding='utf8') for p in (project/'log/runtime').glob('*.log')]
            return result, logs

    # ***
    # 函数名字：test_work_is_quiet_and_keeps_failures
    # 函数作用：验证work终端静默、Python与原生输出留档且退出码保持不变。
    # 参数意义与范围：
    #   self：当前StartupModeTests实例。
    # ***
    def test_work_is_quiet_and_keeps_failures(self):
        result, logs = self.run_probe('work')
        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertEqual(result.stdout, '')
        self.assertEqual(result.stderr, '')
        self.assertEqual(len(logs), 1)
        for text in ('PYTHON_STDOUT','PYTHON_STDERR','NATIVE_STDOUT','NATIVE_STDERR','WARNING_RECORD'):
            self.assertIn(text, logs[0])
        self.assertNotIn('DEBUG_RECORD', logs[0])
        if sys.platform.startswith('linux'):
            self.assertIn('NATIVE_BUFFERED', logs[0])

    # ***
    # 函数名字：test_debug_displays_diagnostics
    # 函数作用：验证debug显示两路输出与调试日志且不创建work日志。
    # 参数意义与范围：
    #   self：当前StartupModeTests实例。
    # ***
    def test_debug_displays_diagnostics(self):
        result, logs = self.run_probe('debug')
        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertIn('PYTHON_STDOUT', result.stdout)
        self.assertIn('NATIVE_STDOUT', result.stdout)
        for text in ('PYTHON_STDERR','NATIVE_STDERR','WARNING_RECORD','DEBUG_RECORD'):
            self.assertIn(text, result.stderr)
        self.assertEqual(logs, [])

    # ***
    # 函数名字：test_entries_forward_mode_and_check_without_devices
    # 函数作用：验证两个Bash入口正确转发模式和只检查参数并拒绝非法模式。
    # 参数意义与范围：
    #   self：当前StartupModeTests实例，实际Bash验证仅在Linux执行。
    # ***
    @unittest.skipUnless(sys.platform.startswith('linux'), 'Bash entry verification runs on the board')
    def test_entries_forward_mode_and_check_without_devices(self):
        for entry in ('start.sh','start_coloritems.sh'):
            for args, mode in (([], 'work'), (['work'], 'work'), (['debug'], 'debug')):
                result = subprocess.run(['bash',str(ROOT/entry),*args,'--check'],capture_output=True,text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(json.loads(result.stdout)['mode'], mode)
            invalid = subprocess.run(['bash',str(ROOT/entry),'invalid'],capture_output=True,text=True)
            self.assertEqual(invalid.returncode,2)
            self.assertIn('INVALID_MODE',invalid.stderr)


if __name__=='__main__': unittest.main()
