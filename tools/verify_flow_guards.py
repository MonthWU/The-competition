"""Verify startup failure gates and whole-launch cleanup on node failure."""

import json
import os
import pty
import signal
import subprocess
import termios
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = Path(os.environ.get("FLOW_GUARDS_OUT", "/tmp/appli_flow_guards"))
OUT.mkdir(parents=True, exist_ok=True)
checks = {}


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


def prescan_case(name, payload, marker):
    master, slave = pty.openpty()
    env = os.environ.copy()
    env["APPLI_SERIAL_DEVICE"] = os.ttyname(slave)
    env.pop("APPLI_SCHOOL_PROFILE", None)
    log_path = OUT / (name + ".log")
    with log_path.open("wb") as log:
        process = subprocess.Popen(["bash", str(ROOT / "start_new.sh"), "2"],
                                   env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            while termios.tcgetattr(slave)[3] & termios.ECHO:
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("prescan did not open PTY")
                time.sleep(0.05)
            if payload is not None:
                os.write(master, payload)
            process.wait(timeout=10)
        finally:
            stop(process)
            os.close(master)
            os.close(slave)
    text = log_path.read_text(errors="replace")
    checks[name] = process.returncode != 0 and marker in text and "STAGE_2" not in text
    print(name, checks[name], flush=True)


prescan_case("missing_start_blocks_main", None, "PRESCAN_FAILED")
prescan_case("uncalibrated_start_24_blocks_main", b"[24]", "CALIBRATION_REQUIRED")

master, slave = pty.openpty()
try:
    log_path = OUT / "node_failure.log"
    with log_path.open("wb") as log:
        process = subprocess.Popen([
            "ros2", "launch", str(ROOT / "launch/run_all.launch.py"),
            "serial_device:=" + os.ttyname(slave),
            "model_file:=/tmp/does-not-exist-appli-guard.bin",
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            process.wait(timeout=35)
        finally:
            stop(process)
    text = log_path.read_text(errors="replace")
    checks["node_failure_stops_full_launch"] = (
        process.returncode != 0 and "PROCESS_FAILED" in text)
finally:
    os.close(master)
    os.close(slave)

result = {"checks": checks, "passed": all(checks.values())}
(OUT / "summary.json").write_text(json.dumps(result, indent=2))
print(json.dumps(result), flush=True)
raise SystemExit(0 if result["passed"] else 1)
