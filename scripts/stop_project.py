"""Stop this workspace's previous task before either manual entry starts."""

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import time


PROC = Path("/proc")


@dataclass
class Process:
    pid: int
    parent: int
    started: str
    state: str
    args: list
    cwd: Path


def processes():
    result = {}
    for directory in PROC.iterdir():
        if not directory.name.isdigit():
            continue
        try:
            fields = (directory / "stat").read_text().rsplit(")", 1)[1].split()
            args = (directory / "cmdline").read_bytes().decode(errors="replace").split("\0")
            cwd = (directory / "cwd").resolve(strict=True)
            process = Process(int(directory.name), int(fields[1]), fields[19],
                              fields[0], [arg for arg in args if arg], cwd)
            if process.state != "Z":
                result[process.pid] = process
        except (OSError, ValueError, IndexError):
            continue  # Processes may exit while /proc is being read.
    return result


def descendants(rows, parents):
    found = set(parents)
    while True:
        added = {pid for pid, row in rows.items() if row.parent in found} - found
        if not added:
            return found
        found.update(added)


def protected_processes(rows, caller):
    # Preserve the new entry, its parents (including SSH) and its own children.
    protected = descendants(rows, {caller})
    parent = rows.get(caller)
    while parent is not None and parent.parent not in protected:
        protected.add(parent.parent)
        parent = rows.get(parent.parent)
    return protected


def in_workspace(path, root):
    return path == root or root in path.parents


def project_process(row, root):
    if not row.args:
        return False
    # Identify launchers and installed nodes by program/script, rather than
    # matching arbitrary command text (which could belong to SSH or an editor).
    for arg in row.args[:2]:
        path = Path(arg)
        if not path.is_absolute():
            path = row.cwd / path
        if not in_workspace(path, root):
            continue
        relative = path.relative_to(root)
        if (relative.parts and relative.parts[0] in {"install", "build"}) or str(relative) in {
            "start_all.sh", "start_simple.sh", "start_new.sh", "start_old.sh",
            "framework/prescan_main.py", "prescan_main.py",
        }:
            return True
    if any(Path(arg).name == "ros2" for arg in row.args[:2]) and "launch" in row.args:
        index = row.args.index("launch") + 1
        if index < len(row.args):
            launch = Path(row.args[index])
            if not launch.is_absolute():
                launch = row.cwd / launch
            return in_workspace(launch, root)
    return False


def stop_service(caller):
    result = subprocess.run(
        ["systemctl", "show", "appli.service", "-p", "LoadState", "-p", "ControlGroup"],
        text=True, capture_output=True, timeout=10,
    )
    if result.returncode:
        raise RuntimeError("cannot inspect appli.service: " + result.stderr.strip())
    properties = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    if properties.get("LoadState") == "not-found":
        return
    group = properties.get("ControlGroup", "")
    caller_groups = [line.split(":", 2)[2]
                     for line in (PROC / str(caller) / "cgroup").read_text().splitlines()]
    if group and any(value == group or value.startswith(group + "/") for value in caller_groups):
        print("[appli] KEEP_CURRENT_SERVICE: entry belongs to appli.service", flush=True)
        return  # Boot/restart must not stop the service that is launching us.
    # Also cancel pending automatic restarts, even when ControlGroup is empty.
    print("[appli] STOP_PREVIOUS_SERVICE", flush=True)
    subprocess.run(["systemctl", "stop", "appli.service"], check=True, timeout=25)


def stop_processes(root, caller):
    rows = processes()
    protected = protected_processes(rows, caller)
    roots = {pid for pid, row in rows.items()
             if pid not in protected and project_process(row, root)}
    targets = {pid: rows[pid].started for pid in descendants(rows, roots) - protected}

    def remaining():
        current = processes()
        return {pid: current[pid] for pid, started in targets.items()
                if pid in current and current[pid].started == started}

    def send(rows_to_signal, sig):
        for pid in rows_to_signal:
            # Recheck start time immediately before signaling to avoid PID reuse.
            if pid in remaining():
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass

    active = remaining()
    for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        if not active:
            break
        print("[appli] STOP_PREVIOUS_TASK: signal=%s pids=%s" %
              (sig.name, ",".join(map(str, sorted(active)))), flush=True)
        # SIGINT reaches the captured task tree, including children of a bash
        # launcher, so ROS nodes can close cameras, UART and recordings.
        send(active, sig)
        deadline = time.monotonic() + timeout
        while active and time.monotonic() < deadline:
            time.sleep(0.1)
            active = remaining()
    if active:
        raise RuntimeError("previous task did not exit: " + str(sorted(active)))
    rows = processes()
    protected = protected_processes(rows, caller)
    leftover = [pid for pid, row in rows.items()
                if pid not in protected and project_process(row, root)]
    if leftover:
        raise RuntimeError("previous task spawned remaining processes: " + str(leftover))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--caller", type=int, required=True)
    args = parser.parse_args()
    try:
        stop_service(args.caller)
        stop_processes(args.root.resolve(strict=True), args.caller)
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print("[appli] STOP_PROJECT_FAILED: " + str(error), flush=True)
        return 2
    print("[appli] PREVIOUS_PROJECT_STOPPED", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
