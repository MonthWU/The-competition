"""Exercise the deployed entry with real cameras and an isolated test profile."""

import collections
import glob
import json
import os
import pty
import select
import signal
import subprocess
import termios
import time
import traceback
from pathlib import Path
import urllib.request
import asyncio
import websockets

import cv2
import numpy as np
import rclpy
from ai_msgs.msg import PerceptionTargets
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


ROOT = str(Path(__file__).resolve().parents[1])
FIXTURES = ROOT + "/obj_detect/test/data"
OUT = os.environ.get("CONTINUITY_OUT", "/tmp/appli_continuity")
os.makedirs(OUT, exist_ok=True)
started = time.monotonic()
phase = "startup"
state = {
    "profile_override": {"object_scan_enabled": True, "fixed_obstacle_id": 19},
    "prescan_transport": "PTY simulated MCU",
    "main_transport": "same PTY simulated MCU across all stages",
    "images": {}, "detections": {}, "qr_results": [], "kills": [],
    "serial_events": [], "milestones": [], "image_shapes": {},
    "qr_fixture_used": False, "object_fixture_used": False,
    "small_object_targets": 0,
}
wire = bytearray()
process = None
node = None
master = slave = None
log = None
videos_before = set(glob.glob(ROOT + "/_tmp_videos/*"))


def milestone(label):
    item = {"name": label, "seconds": round(time.monotonic() - started, 3)}
    state["milestones"].append(item)
    print(json.dumps(item), flush=True)


def image_cb(msg, qr=False):
    key = ("qr_camera" if not state["qr_fixture_used"] else "qr_fixture_input") if qr else phase
    state["images"][key] = state["images"].get(key, 0) + 1
    if key not in state["image_shapes"]:
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            state["image_shapes"][key] = list(frame.shape)
            cv2.imwrite(OUT + "/" + key + ".jpg", frame)


def detection_cb(msg):
    counter = state["detections"].setdefault(phase, {"messages": 0, "classes": {}})
    counter["messages"] += 1
    for target in msg.targets:
        counter["classes"][target.type] = counter["classes"].get(target.type, 0) + 1
        if phase.startswith("object_"):
            if not target.rois or target.rois[0].rect.width * target.rois[0].rect.height < 2000:
                state["small_object_targets"] += 1


def serial_cb(msg):
    if len(state["serial_events"]) < 200:
        state["serial_events"].append({"phase": phase, "data": msg.data})


def pump(timeout=0.05):
    rclpy.spin_once(node, timeout_sec=timeout)
    ready, _, _ = select.select([master], [], [], 0)
    if ready:
        wire.extend(os.read(master, 8192))


def wait_until(predicate, seconds, label):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        pump()
        if predicate():
            milestone(label)
            return
        if process.poll() is not None:
            raise RuntimeError(f"ENTRY_EXITED rc={process.returncode} waiting={label}")
    raise TimeoutError(label)


def websocket_image_bytes():
    """Check binary image delivery with the board's existing WebSocket library."""
    async def receive_image():
        async with websockets.connect(
            "ws://127.0.0.1:8080", open_timeout=3, close_timeout=1
        ) as client:
            await client.send(json.dumps({"filter_prefix": ""}))
            deadline = time.monotonic() + 6
            total = 0
            while time.monotonic() < deadline:
                pump()
                payload = await asyncio.wait_for(client.recv(), timeout=3)
                if isinstance(payload, bytes):
                    total += len(payload)
                    if total > 1024:
                        return total
            return total
    return asyncio.run(receive_image())


try:
    with open(ROOT + "/framework/school_profile.json", encoding="utf-8") as stream:
        profile = json.load(stream)
    assert profile["object_scan_enabled"] is True, "FORMAL_OBJECT_SCAN_DISABLED"
    profile["fixed_obstacle_id"] = 19
    profile_path = OUT + "/test_profile.json"
    with open(profile_path, "w", encoding="utf-8") as stream:
        json.dump(profile, stream, indent=2)
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    state["prescan_device"] = device
    rclpy.init()
    node = Node("full_continuity_monitor")
    node.create_subscription(CompressedImage, "/image", image_cb, 10)
    node.create_subscription(CompressedImage, "/qrc_image", lambda m: image_cb(m, True), 10)
    node.create_subscription(PerceptionTargets, "/hobot_dnn_detection", detection_cb, 10)
    node.create_subscription(String, "/qrc_result", lambda m: state["qr_results"].append(m.data), 10)
    node.create_subscription(String, "/kill_qrc", lambda m: state["kills"].append(m.data), 10)
    node.create_subscription(String, "/serial_send", serial_cb, 10)
    qr_pub = node.create_publisher(CompressedImage, "/qrc_image", 10)
    sample_pub = node.create_publisher(CompressedImage, "/image", 10)
    environment = os.environ.copy()
    environment.update(APPLI_SCHOOL_PROFILE=profile_path, APPLI_SERIAL_DEVICE=device)
    log = open(OUT + "/entry.log", "wb")
    process = subprocess.Popen(["bash", ROOT + "/start_all.sh", "30"],
                               env=environment, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    state["entry_pid"] = process.pid
    wait_until(lambda: not (termios.tcgetattr(slave)[3] & termios.ECHO), 30, "PRESCAN_SERIAL_OPEN")
    os.write(master, b"[4]")
    wait_until(lambda: wire.count(b"[ack]") >= 1, 10, "START_ACK")
    for index, angle in enumerate((0, 45, 90)):
        phase = "prescan_" + str(angle)
        os.write(master, b"[shot]")
        wait_until(lambda: wire.count(b"[ack]") >= index + 2, 30, "SCAN_ACK_" + str(angle))
    phase = "qr"
    wait_until(lambda: b"[1 19 12]" in wire, 10, "MAP_FRAME")
    deadline = time.monotonic() + 5
    while not state["kills"] and time.monotonic() < deadline:
        pump()
        if process.poll() is not None:
            raise RuntimeError("ENTRY_EXITED_IN_QR")
    if not state["kills"]:
        wait_until(lambda: state["images"].get("qr_camera", 0) >= 5,
                   20, "REAL_QR_CAMERA_IMAGES")
        state["qr_fixture_used"] = True
        msg = CompressedImage(format="jpeg", data=open(FIXTURES + "/valid_school_qr.jpg", "rb").read())
        deadline = time.monotonic() + 10
        while not state["kills"] and time.monotonic() < deadline:
            qr_pub.publish(msg)
            pump(0.1)
    if not state["kills"]:
        raise TimeoutError("QR_DECODE_AND_KILL")
    milestone("QR_DECODE_AND_KILL")
    phase = "object_live"
    wait_until(lambda: state["images"].get(phase, 0) >= 5, 30, "OBJECT_CAMERA_IMAGES")
    live_started = time.monotonic()
    deadline = live_started + 15
    while time.monotonic() < deadline:
        pump()
        if process.poll() is not None:
            raise RuntimeError("ENTRY_EXITED_IN_OBJECT")
    state["object_live_observation_seconds"] = round(time.monotonic() - live_started, 3)
    milestone("OBJECT_LIVE_OBSERVATION_COMPLETE")
    # Verify known coordinates after independently observing the real camera.
    state["object_fixture_used"] = True
    phase = "object_fixture"
    msg = CompressedImage(format="jpeg", data=open(FIXTURES + "/school_obj_sample.jpg", "rb").read())
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        sample_pub.publish(msg)
        pump(0.2)
    milestone("OBJECT_FIXTURE_CHECK_COMPLETE")
    try:
        with urllib.request.urlopen("http://127.0.0.1:8000/", timeout=3) as response:
            state["web_http_status"] = response.status
    except Exception as exc:
        state["web_error"] = str(exc)
    state["websocket_payload_bytes"] = 0
    try:
        state["websocket_payload_bytes"] = websocket_image_bytes()
    except Exception as exc:
        state["websocket_error"] = str(exc)
    state["new_video_files"] = sorted(set(glob.glob(ROOT + "/_tmp_videos/*")) - videos_before)
    state["camera_users"] = subprocess.run(
        ["fuser", "/dev/video0", "/dev/video2", "/dev/video4", "/dev/ttyS1"],
        capture_output=True, text=True).stderr
    qr_sent = [e for e in state["serial_events"] if "qrc:" in e["data"]]
    object_sent = [e for e in state["serial_events"]
                   if e["phase"].startswith("object_") and "qrc:" not in e["data"]]
    state["checks"] = {
        "four_prescan_acks": wire.count(b"[ack]") == 4,
        "one_map_frame": wire.count(b"[1 19 12]") == 1,
        "three_scan_images": all(state["images"].get("prescan_" + str(a), 0) > 0 for a in (0,45,90)),
        "three_scan_inference": all(state["detections"].get("prescan_" + str(a), {}).get("messages", 0) > 0 for a in (0,45,90)),
        "qr_camera_image": state["images"].get("qr_camera", 0) > 0,
        "qr_result": bool(state["qr_results"]),
        "one_kill": state["kills"] == ["kill"],
        "four_qr_serial_notifications": len(qr_sent) == 4,
        "four_qr_uart_frames": wire.count(b"\xff\x37" + state["qr_results"][0].encode("utf-8") + b"\xfe") == 4,
        "fixture_object_uart": b"\xff\x41\x5c\x01\xd1\x00\xfe" in wire,
        "object_camera_image": state["images"].get("object_live", 0) > 5,
        "object_inference": state["detections"].get("object_live", {}).get("messages", 0) > 5,
        "small_object_targets_filtered": state["small_object_targets"] == 0,
        "object_serial_notifications": len(object_sent) > 0,
        "full_entry_stays_running": process.poll() is None,
        "recording_created": bool(state["new_video_files"]),
        "web_http": state.get("web_http_status") == 200,
        "websocket_image_payload": state["websocket_payload_bytes"] > 1024,
    }
    state["passed"] = all(state["checks"].values())
except Exception:
    state["passed"] = False
    state["error"] = traceback.format_exc()
    print(state["error"], flush=True)
finally:
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if log is not None:
        log.close()
    for fd in (master, slave):
        if fd is not None:
            os.close(fd)
    state["prescan_wire_hex"] = bytes(wire).hex(" ")
    state["elapsed_seconds"] = round(time.monotonic() - started, 3)
    state["entry_stop_returncode"] = process.returncode if process else None
    state["video_checks"] = []
    for path in state.get("new_video_files", []):
        capture = cv2.VideoCapture(path)
        ok, frame = capture.read()
        state["video_checks"].append({"file": path, "first_frame": ok, "frames": int(capture.get(cv2.CAP_PROP_FRAME_COUNT))})
        capture.release()
    video_ok = bool(state["video_checks"]) and all(v["first_frame"] and v["frames"] > 0 for v in state["video_checks"])
    state.setdefault("checks", {})["all_videos_closed_and_readable"] = video_ok
    state["passed"] = bool(state.get("passed")) and video_ok
    with open(OUT + "/summary.json", "w", encoding="utf-8") as stream:
        json.dump(state, stream, indent=2, ensure_ascii=False)
    print("FULL_CONTINUITY_PASS=" + str(state["passed"]), flush=True)
    print(json.dumps({k:v for k,v in state.items() if k != "serial_events"}, ensure_ascii=False), flush=True)

raise SystemExit(0 if state["passed"] else 1)
