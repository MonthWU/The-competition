# Purpose: verify device-free application coordination and failure cleanup.
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from car26.application import Application
from car26.config import load_config
from car26.mapping import RadarMapper
from car26.types import CameraFrame, Obstacle, RadarFrame


ROOT = Path(__file__).resolve().parents[1]


class ApplicationTests(unittest.TestCase):
    def configuration(self):
        config = load_config(ROOT / "config/legacy_camera.json")
        config.data["mission"]["entry"] = "qr"
        config.data["mission"]["object_enabled"] = False
        config.data["hmi"]["enabled"] = False
        return config

    # ***
    # 函数名字：fake_application
    # 函数作用：创建仅扫码流程的应用假设备，用于验证发送顺序与失败清理。
    # 参数意义与范围：
    #   self：当前ApplicationTests实例。
    #   mode：运行模式字符串work或debug。
    #   fail_write：是否模拟第三次写入失败；bool。
    # ***
    def fake_application(self, mode, fail_write=False):
        configuration = self.configuration()
        configuration.data["mode"] = mode
        app = Application(configuration)
        history = []
        comm, camera, radar, hmi = Mock(), Mock(), Mock(), Mock()
        comm.poll.return_value = []
        camera.drain_mode_events.return_value = ()
        # ***
        # 函数名字：send
        # 函数作用：记录模拟发送，并按测试选项注入写入异常。
        # 参数意义与范围：
        #   frame：完整bytes报文。
        # ***
        def send(frame):
            history.append(("send", frame))
            if fail_write and len(history) == 3:
                raise OSError("MCU_WRITE_TIMEOUT")
        comm.send.side_effect = send
        comm.flush.side_effect = lambda: history.append(("flush", None))
        camera.role = None
        # ***
        # 函数名字：switch
        # 函数作用：记录模拟相机角色切换。
        # 参数意义与范围：
        #   role：相机角色字符串qr或object。
        # ***
        def switch(role):
            camera.role = role
            history.append(("camera", role))
        camera.switch.side_effect = switch
        camera.result.return_value = (CameraFrame("qr", 1, 0, 640, 400, qr_text=" QR 原文 "), ())
        camera.close.side_effect = lambda: history.append(("camera_stop", None))
        app.communication, app.camera, app.radar, app.hmi = comm, camera, radar, hmi
        return app, history

    def test_work_and_debug_same_business_effects(self):
        histories = []
        for mode in ("work", "debug"):
            app, history = self.fake_application(mode)
            with patch("car26.application.signal.signal"), patch("car26.application.time.monotonic", return_value=0):
                self.assertEqual(app.run(), 0)
            histories.append(history)
        self.assertEqual(histories[0], histories[1])
        self.assertEqual([action for action, _ in histories[0]][:7],
                         ["camera", "send", "send", "send", "send", "flush", "camera_stop"])

    def test_send_failure_prevents_camera_handoff_and_closes_modules(self):
        app, history = self.fake_application("work", True)
        with patch("car26.application.signal.signal"), patch("car26.application.time.monotonic", return_value=0):
            with self.assertLogs("car26.app", level="ERROR"):
                self.assertEqual(app.run(), 1)
        self.assertFalse(any(action == "flush" for action, _ in history))
        self.assertFalse(any(action == "camera" and role == "object" for action, role in history))
        app.radar.close.assert_called_once()
        app.communication.close.assert_called_once()

    def test_stale_qr_frame_never_reaches_mcu(self):
        app, history = self.fake_application("work")
        with patch("car26.application.signal.signal"), patch("car26.application.time.monotonic", return_value=10):
            with self.assertLogs("car26.app", level="ERROR"):
                self.assertEqual(app.run(), 1)
        self.assertFalse(any(action == "send" for action, _ in history))

    def test_device_placeholders_and_calibration_are_explicit_errors(self):
        errors = load_config(ROOT / "config/system.json").validate()
        self.assertIn("RADAR_DEVICE_REQUIRED", errors)
        self.assertIn("RADAR_EXTRINSIC_CALIBRATION_REQUIRED", errors)
        self.assertIn("CAMERA_DEVICE_REQUIRED:object", errors)
        self.assertIn("RADAR_MAP_CALIBRATION_REQUIRED", errors)

    def test_legacy_configuration_has_no_missing_settings(self):
        self.assertEqual(load_config(ROOT / "config/legacy_camera.json").validate(), [])

    def test_can_requires_defined_mcu_protocol(self):
        configuration = self.configuration()
        configuration.data["communication"]["transport"] = "can"
        self.assertIn("MCU_CAN_PROTOCOL_NOT_DEFINED", configuration.validate())

    def test_inheritance_cycle_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "bad.json"
            p.write_text('{"extends":"bad.json"}')
            with self.assertRaisesRegex(ValueError, "CYCLE"):
                load_config(p)

    def test_radar_map_uses_start_specific_heading(self):
        configuration = load_config(ROOT / "config/system.json")
        settings = configuration.data["radar_mapping"]
        settings["targets_by_start"] = {"4": [{"id": 13, "x_m": 0, "y_m": 1}]}
        settings["heading_deg_by_start"] = {"4": [90, 135, 180]}
        configuration.data["radar"]["extrinsic_calibrated"] = True
        mapper = RadarMapper(configuration)
        frame = RadarFrame(1, 0, (), (Obstacle(1, 0, .1, 10),))
        self.assertGreater(mapper.observations(frame, 4, 0)[13], .99)
        self.assertFalse(mapper.supports(24))

    def test_radar_overlap_gates_do_not_guess(self):
        configuration = load_config(ROOT / "config/system.json")
        configuration.data["radar_mapping"]["targets_by_start"] = {"4": [
            {"id": 1, "x_m": 1, "y_m": 0}, {"id": 3, "x_m": 1.01, "y_m": 0}]}
        configuration.data["radar_mapping"]["heading_deg_by_start"] = {"4": [0, 45, 90]}
        frame = RadarFrame(1, 0, (), (Obstacle(1, 0, .1, 10),))
        self.assertEqual(RadarMapper(configuration).observations(frame, 4, 0), {})
