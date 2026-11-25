# Purpose: verify lighting selection, capture-mode persistence and real Bash forwarding without devices.
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from car26.config import load_config
from car26.camera.modes import object_capture_settings
from car26.camera.module import CameraModule
from car26.telemetry import Telemetry

ROOT = Path(__file__).resolve().parents[1]


class LightingProfileTests(unittest.TestCase):
    # ***
    # 函数名字：test_selected_controls_reach_uvc_and_mismatch_fails
    # 函数作用：模拟首帧后驱动设置与回读，确认两套参数均下发且读回错值会中止。
    # 参数意义与范围：
    #   self：当前LightingProfileTests实例，不打开相机或执行真实v4l2命令。
    # ***
    def test_selected_controls_reach_uvc_and_mismatch_fails(self):
        for lighting in ('light_able', 'light_unable'):
            config = load_config(ROOT / 'config/board-object.json', lighting=lighting)
            module = CameraModule(config, Telemetry())
            for profile in ('block', 'disc', 'target'):
                settings = object_capture_settings(config.data['cameras']['object'], profile)
                controls = settings['uvc_controls']
                readback = Mock(stdout='\n'.join(f'{k}: {v}' for k, v in controls.items()))
                with patch('car26.camera.module.subprocess.run', return_value=readback) as run:
                    module._stream_controls(settings)
                    self.assertEqual(run.call_count, 4)
                    written = run.call_args_list[2].args[0][-1]
                    self.assertIn(f'exposure_time_absolute={controls["exposure_time_absolute"]}', written)
                    self.assertIn(f'gain={controls["gain"]}', written)
                readback.stdout = readback.stdout.replace(f'gain: {controls["gain"]}', 'gain: 0')
                with patch('car26.camera.module.subprocess.run', return_value=readback):
                    with self.assertRaisesRegex(RuntimeError, 'CAMERA_UVC_READBACK_MISMATCH'):
                        module._stream_controls(settings)

    # ***
    # 函数名字：test_profiles_preserve_capture_and_other_roles
    # 函数作用：检查三种正式配置在两套补光参数下均保持原任务、分辨率、二维码和模型设置。
    # 参数意义与范围：
    #   self：当前LightingProfileTests实例。
    # ***
    def test_profiles_preserve_capture_and_other_roles(self):
        for filename in ('config/board-object.json', 'projects/without_radar/config/board.json',
                         'projects/with_radar/config/board.json'):
            original = load_config(ROOT / filename)
            for lighting, exposure, gain in (('light_able', 50, 16), ('light_unable', 50, 48)):
                with self.subTest(filename=filename, lighting=lighting):
                    config = load_config(ROOT / filename, 'debug', lighting)
                    self.assertEqual(config.data['lighting_mode'], lighting)
                    self.assertEqual(config.mode, 'debug')
                    for key in ('models', 'mission', 'target_focus'):
                        self.assertEqual(config.data[key], original.data[key])
                    self.assertEqual(config.data['cameras']['qr'], original.data['cameras']['qr'])
                    for profile in ('block', 'disc', 'target'):
                        settings = object_capture_settings(config.data['cameras']['object'], profile)
                        self.assertEqual(settings['uvc_controls'], dict(auto_exposure=1,
                            exposure_time_absolute=exposure, brightness=128, gain=gain))
                        self.assertEqual((settings['width'], settings['height']),
                                         (640, 360) if profile == 'block' else (1280, 720))
                    if 'with_radar' not in filename:
                        self.assertEqual(config.validate(), [])
            self.assertEqual(load_config(ROOT / filename).data, original.data)

    # ***
    # 函数名字：test_invalid_profiles_fail_before_devices
    # 函数作用：拒绝未知模式、缺失参数、布尔数值和越界曝光，避免静默使用另一套参数。
    # 参数意义与范围：
    #   self：当前LightingProfileTests实例。
    # ***
    def test_invalid_profiles_fail_before_devices(self):
        with self.assertRaisesRegex(ValueError, 'INVALID_LIGHTING_MODE'):
            load_config(ROOT / 'config/board-object.json', lighting='unknown')
        original = load_config(ROOT / 'config/board-object.json').data
        for invalid in (None, {}, dict(auto_exposure=1, exposure_time_absolute=True, brightness=128, gain=48),
                        dict(auto_exposure=1, exposure_time_absolute=-1, brightness=128, gain=48)):
            with tempfile.TemporaryDirectory() as directory:
                data = copy.deepcopy(original)
                data['cameras']['object']['lighting_profiles']['light_unable'] = invalid
                path = Path(directory) / 'config.json'
                path.write_text(json.dumps(data), encoding='utf-8')
                with self.assertRaisesRegex(ValueError, 'INVALID_LIGHTING_PROFILE'):
                    load_config(path, lighting='light_unable')

    # ***
    # 函数名字：test_cli_check_reports_resolved_parameters
    # 函数作用：在独立进程中确认CLI检查展示所选曝光并拒绝错误参数，检查不打开设备。
    # 参数意义与范围：
    #   self：当前LightingProfileTests实例。
    # ***
    def test_cli_check_reports_resolved_parameters(self):
        for lighting, exposure in ((None, 50), ('light_unable', 50)):
            args = [] if lighting is None else ['--lighting', lighting]
            result = subprocess.run([sys.executable, '-m', 'car26', '--config',
                str(ROOT / 'config/board-object.json'), *args, '--check'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            payload = json.loads(result.stdout)
            self.assertEqual(payload['object_uvc_controls']['exposure_time_absolute'], exposure)
            self.assertEqual(payload['lighting_mode'], lighting or 'light_able')

    # ***
    # 函数名字：test_bash_entries_forward_and_reject_conflicts
    # 函数作用：真实Bash执行现有入口，以假的python3截获参数，检查顺序兼容和冲突拒绝。
    # 参数意义与范围：
    #   self：当前LightingProfileTests实例；使用Linux Bash或Windows已安装的Git Bash。
    # ***
    def test_bash_entries_forward_and_reject_conflicts(self):
        git = shutil.which('git')
        bash = Path(git).parents[1] / 'bin/bash.exe' if os.name == 'nt' and git else shutil.which('bash')
        if not bash or not Path(bash).is_file():
            self.skipTest('No local Bash available')
        with tempfile.TemporaryDirectory() as directory:
            bin_dir = Path(directory) / 'bin'
            bin_dir.mkdir()
            probe = bin_dir / 'python3'
            probe.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n', encoding='utf-8')
            probe.chmod(0o755)
            for entry in ('start.sh', 'start_coloritems.sh', 'projects/without_radar/start.sh',
                          'projects/with_radar/start.sh'):
                for args, expected in ((['light_unable', 'debug', '--check'], ('debug', 'light_unable')),
                                       (['work', 'light_able', '--check'], ('work', 'light_able')),
                                       (['--check', '--lighting', 'light_unable'], ('work', 'light_unable'))):
                    command = ['export PATH="$PWD/bin:$PATH"; exec bash "$@"', 'probe',
                               (ROOT / entry).as_posix(), *args]
                    result = subprocess.run([str(bash), '-c', *command], cwd=directory,
                                            capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    forwarded = result.stdout.splitlines()
                    self.assertEqual(forwarded[forwarded.index('--mode') + 1], expected[0])
                    self.assertEqual(forwarded[forwarded.index('--lighting') + 1], expected[1])
                    self.assertIn('--check', forwarded)
            for args in (['light_able', 'light_unable'], ['--lighting'], ['light_invalid'], ['work', 'debug']):
                result = subprocess.run([str(bash), (ROOT / 'start.sh').as_posix(), *args],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
                self.assertEqual(result.stdout, '')


if __name__ == '__main__':
    unittest.main()
