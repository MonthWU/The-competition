# Purpose: verify bracket command parsing, frame bytes and bounded communication diagnostics.
import unittest

from car26.communication.module import CommunicationModule
from car26.communication.protocol import CommandParser, map_frame, qr_frame, select_targets, target_frame
from car26.telemetry import Telemetry
from car26.types import Detection


class FakeTransport:
    def __init__(self, maximum=2):
        self.maximum = maximum
        self.output = bytearray()

    def write(self, value):
        count = min(self.maximum, len(value))
        self.output.extend(value[:count])
        return count


class ProtocolTests(unittest.TestCase):
    def test_fragmented_nested_and_noise(self):
        parser = CommandParser()
        self.assertEqual(parser.feed(b"noise[4][sho"), ["4"])
        self.assertEqual(parser.feed(b"t][bad[24][skip]"), ["shot", "24", "skip"])

    def test_bounded_and_non_ascii(self):
        parser = CommandParser(8)
        self.assertEqual(parser.feed(b"[" + b"a" * 40 + b"][\xff][4]"), ["4"])
        self.assertEqual(parser.dropped, 2)

    def test_map_checksum(self):
        self.assertEqual(map_frame([3, 1]), b"[2 1 3 00]")
        self.assertEqual(map_frame([13]), b"[1 13 0C]")
        self.assertEqual(map_frame([]), b"[0 00]")
        with self.assertRaises(ValueError):
            map_frame([25])

    # ***
    # 函数名字：test_raw_qr_unicode_and_spaces
    # 函数作用：验证扫码转发使用方括号且保留UTF-8原文。
    # 参数意义与范围：
    #   self：当前ProtocolTests实例。
    # ***
    def test_raw_qr_unicode_and_spaces(self):
        text = " 原文\n123 "
        self.assertEqual(qr_frame(text), b"[\x70" + text.encode() + b"]")

    # ***
    # 函数名字：test_coordinate_endianness
    # 函数作用：验证方括号物块报文保留坐标小端编码。
    # 参数意义与范围：
    #   self：当前ProtocolTests实例。
    # ***
    def test_coordinate_endianness(self):
        self.assertEqual(target_frame("darkblueblock", 320, 240), bytes.fromhex("5b464001f0005d"))
        with self.assertRaises(ValueError):
            target_frame("targetOne", -1, 2)

    # ***
    # 函数名字：test_all_blocks_and_marks
    # 函数作用：验证距离不同的物块均保留，并保留有效标靶。
    # 参数意义与范围：
    #   self：当前ProtocolTests实例。
    # ***
    def test_all_blocks_and_marks(self):
        far = Detection("redblock", .9, 0, 0, 10, 10)
        near = Detection("lightblueblock", .8, 310, 220, 330, 240)
        mark = Detection("targetOne", .7, 0, 0, 100, 100)
        self.assertEqual(select_targets([far, near, mark], (320, 240)), [far, near, mark])

    def test_partial_writes_complete_exact_frame(self):
        transport = FakeTransport()
        comm = CommunicationModule({"write_timeout_s": 1}, Telemetry(), transport)
        comm.send(b"[ack]")
        self.assertEqual(transport.output, b"[ack]")
        self.assertEqual(comm.sent, 5)

    def test_zero_write_aborts(self):
        comm = CommunicationModule({"write_timeout_s": 1}, Telemetry(), FakeTransport(0))
        with self.assertRaisesRegex(OSError, "MCU_SHORT_WRITE"):
            comm.send(b"[ack]")

    def test_hmi_events_bounded_and_snapshot_isolated(self):
        telemetry = Telemetry()
        for i in range(110):
            telemetry.event("communication", "tx", index=i)
        result = telemetry.snapshot()
        self.assertEqual(len(result["events"]), 100)
        self.assertEqual(result["events"][0]["index"], 10)
        result["application"]["state"] = "changed"
        self.assertEqual(telemetry.snapshot()["application"]["state"], "created")
