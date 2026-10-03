from functools import partial
from types import SimpleNamespace

import pytest

from obj_detect.obj_serial import ObjSerial
from qrc_skandier import qrc_cam_killer, qrc_scanner


def make_logger():
    return SimpleNamespace(info=lambda message: None, warn=lambda message: None)


@pytest.mark.parametrize("payload", ["123+231", "156+123+516+231", "hello", "0000000", " 123+231 ", "中文"])
def test_scanner_forwards_decoded_text(monkeypatch, payload):
    published = []
    scanner = SimpleNamespace(
        get_logger=make_logger,
        res_pub=SimpleNamespace(publish=lambda message: published.append(message.data)),
    )
    monkeypatch.setattr(qrc_scanner, "ros2cv", lambda message: object())
    monkeypatch.setattr(qrc_scanner.pyzbar, "decode", lambda image: [SimpleNamespace(data=payload.encode("utf-8"))])
    qrc_scanner.QrcScanner.scan_code(scanner, object())
    assert published == [payload]


@pytest.mark.parametrize("decoded", [[], [SimpleNamespace(data=b"")], [SimpleNamespace(data=b"\xff")]])
def test_scanner_does_not_publish_placeholder(monkeypatch, decoded):
    published = []
    scanner = SimpleNamespace(
        get_logger=make_logger,
        res_pub=SimpleNamespace(publish=lambda message: published.append(message.data)),
    )
    monkeypatch.setattr(qrc_scanner, "ros2cv", lambda message: object())
    monkeypatch.setattr(qrc_scanner.pyzbar, "decode", lambda image: decoded)
    qrc_scanner.QrcScanner.scan_code(scanner, object())
    assert published == []


@pytest.mark.parametrize("payload", ["123+231", "hello", "0000000", " 123+231 ", "中文"])
def test_decoded_text_stops_scanner(monkeypatch, payload):
    kills = []
    killer = SimpleNamespace(
        get_logger=make_logger,
        qrc_kill_pub=SimpleNamespace(publish=lambda message: kills.append(message.data)),
        _done=False,
    )
    monkeypatch.setattr(qrc_cam_killer.time, "sleep", lambda duration: None)
    qrc_cam_killer.QrcCamKiller.kill_qrc(killer, SimpleNamespace(data=payload))
    assert kills == ["kill"]
    assert killer._done


@pytest.mark.parametrize("payload", ["123+231", "156+123+516+231", "hello", "0000000", " 123+231 ", "中文"])
@pytest.mark.parametrize("qr_only", [False, True])
def test_serial_forwards_original_text_four_times(payload, qr_only):
    frames = []
    serial_node = SimpleNamespace(
        mode=0 if qr_only else 2,
        qr_only=qr_only,
        _done=False,
        ser=SimpleNamespace(write=lambda frame: frames.append(bytes(frame)), flush=lambda: None),
        qrc_forwarded_pub=SimpleNamespace(
            get_subscription_count=lambda: 1, publish=lambda message: None),
        get_logger=make_logger,
        pub_sent=lambda message: None,
    )
    serial_node.send_qrc = partial(ObjSerial.send_qrc, serial_node)
    ObjSerial.qrc_callback(serial_node, SimpleNamespace(data=payload))
    expected = b"\xff\x37" + payload.encode("utf-8") + b"\xfe"
    assert frames == [expected] * 4
    assert serial_node.mode == 1
    assert serial_node._done == qr_only
    ObjSerial.qrc_callback(serial_node, SimpleNamespace(data=payload))
    assert frames == [expected] * 4


def test_empty_text_does_not_send_or_stop(monkeypatch):
    published = []
    node = SimpleNamespace(
        mode=0,
        qr_only=True,
        _done=False,
        get_logger=make_logger,
        send_qrc=lambda payload: published.append(payload),
        qrc_kill_pub=SimpleNamespace(publish=lambda message: published.append(message.data)),
    )
    monkeypatch.setattr(qrc_cam_killer.time, "sleep", lambda duration: None)
    ObjSerial.qrc_callback(node, SimpleNamespace(data=""))
    qrc_cam_killer.QrcCamKiller.kill_qrc(node, SimpleNamespace(data=""))
    assert published == []
    assert node.mode == 0
    assert not node._done


def test_camera_handoff_follows_four_uart_writes_and_flush():
    events = []
    node = SimpleNamespace(
        mode=2, qr_only=False, _done=False, get_logger=make_logger,
        send_qrc=lambda payload: events.append("write"),
        ser=SimpleNamespace(flush=lambda: events.append("flush")),
        qrc_forwarded_pub=SimpleNamespace(
            get_subscription_count=lambda: 1,
            publish=lambda message: events.append("handoff")))
    ObjSerial.qrc_callback(node, SimpleNamespace(data="123+231"))
    assert events == ["write"] * 4 + ["flush", "handoff"]


def test_failed_uart_write_does_not_request_camera_handoff():
    events = []
    def fail(payload):
        raise OSError("UART_WRITE_FAILED")
    node = SimpleNamespace(
        mode=2, qr_only=False, _done=False, get_logger=make_logger,
        send_qrc=fail,
        qrc_forwarded_pub=SimpleNamespace(publish=lambda message: events.append("handoff")))
    with pytest.raises(OSError, match="UART_WRITE_FAILED"):
        ObjSerial.qrc_callback(node, SimpleNamespace(data="123+231"))
    assert events == []
