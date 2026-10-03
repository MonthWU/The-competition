"""Check camera handoff ordering and AVI closure using real OpenCV files."""

from types import SimpleNamespace
import subprocess
import time

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter

from obj_detect import obj_camd, obj_vid_dumper


def test_duplicate_handoff_does_not_start_another_camera():
    node = SimpleNamespace(
        _requested=False, _camera_launch=None, _release_deadline=0,
        get_logger=lambda: SimpleNamespace(info=lambda message: None))
    obj_camd.ObjCamd.on_qrcam_killed(node, SimpleNamespace(data="kill"))
    deadline = node._release_deadline
    obj_camd.ObjCamd.on_qrcam_killed(node, SimpleNamespace(data="kill"))
    assert node._requested and node._release_deadline == deadline


def test_object_camera_waits_until_qr_device_is_released(monkeypatch):
    starts = []
    node = SimpleNamespace(
        _requested=True, _camera_launch=None,
        _release_deadline=time.monotonic() + 10,
        get_parameter=lambda name: SimpleNamespace(value="/dev/test_qr"),
        obj_cam_launch=lambda: starts.append("started"))
    monkeypatch.setattr(obj_camd.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=0))
    obj_camd.ObjCamd.check_camera(node)
    assert starts == []
    monkeypatch.setattr(obj_camd.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=1))
    obj_camd.ObjCamd.check_camera(node)
    assert starts == ["started"]


def test_camera_child_exit_is_a_task_failure():
    node = SimpleNamespace(_camera_launch=SimpleNamespace(poll=lambda: 0))
    with pytest.raises(RuntimeError, match="OBJECT_CAMERA_EXITED"):
        obj_camd.ObjCamd.check_camera(node)


@pytest.fixture
def recorder(tmp_path):
    rclpy.init()
    node = obj_vid_dumper.ObjVidDumper("test")
    node.set_parameters([Parameter("video_dir", value=str(tmp_path))])
    try:
        yield node, tmp_path
    finally:
        node.close_video()
        node.destroy_node()
        rclpy.shutdown()


def assert_video_readable(path):
    capture = cv2.VideoCapture(str(path))
    try:
        ok, frame = capture.read()
        assert ok and frame.shape == (480, 640, 3)
        assert capture.get(cv2.CAP_PROP_FRAME_COUNT) > 0
    finally:
        capture.release()


def test_no_empty_video_while_waiting_and_last_segment_is_closed(recorder):
    node, directory = recorder
    node.timer_update_fd()
    node.timer_update_fd()
    assert list(directory.glob("*.avi")) == []
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    node.dump_frame(frame)
    time.sleep(0.06)
    node.dump_frame(frame)
    node.close_video()
    videos = list(directory.glob("*.avi"))
    assert len(videos) == 1
    assert_video_readable(videos[0])


def test_rotation_closes_old_segment_before_opening_next(recorder):
    node, directory = recorder
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    node.dump_frame(frame)
    node.timer_update_fd()
    assert len(list(directory.glob("*.avi"))) == 1
    node.dump_frame(frame)
    node.close_video()
    videos = list(directory.glob("*.avi"))
    assert len(videos) == 2
    for path in videos:
        assert_video_readable(path)


def test_failed_archive_retains_original_recording(recorder, monkeypatch):
    node, directory = recorder
    node.dump_frame(np.zeros((480, 640, 3), dtype=np.uint8))
    def fail(*args, **kwargs):
        raise subprocess.CalledProcessError(1, args[0])
    monkeypatch.setattr(obj_vid_dumper.subprocess, "run", fail)
    node.timer_packup_callback()
    videos = list(directory.glob("*.avi"))
    assert len(videos) == 1
    assert_video_readable(videos[0])
