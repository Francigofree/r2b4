"""Host/developer helpers behind the R2B4 launcher.

This module has no robot authority.  Hardware diagnostics are guarded against a
running resident runtime; all other helpers are ordinary host/development tools.
"""

from __future__ import annotations

import contextlib
import datetime as dt
import fcntl
import os
from pathlib import Path
import subprocess
import sys
import time
from collections.abc import Iterator, Sequence

from v3 import test_runner


class HostCliError(RuntimeError):
    pass


ALIASES = {
    "tests": "pytest",
    "script": "tool",
    "disk": "disc",
    "memory": "mem",
    "temperature": "temp",
    "processes": "ps",
    "network": "net",
    "gittre": "gitre",
}

COMMANDS = frozenset({
    "install", "where", "root", "version", "gitre", "git", "pytest", "test",
    "tools", "tool", "cpu", "cpu2", "disc", "mem", "temp", "ps", "net", "usb",
    "i2c", "host", *ALIASES.keys(),
})

HARDWARE_TOOLS = frozenset({
    "r2b4_voice_mic_test", "v3_camera_test", "v3_encoder_ab_probe",
    "v3_person_detection_test", "v3_sensor_measurement",
})


def _run(command: Sequence[str], *, root: Path) -> int:
    try:
        completed = subprocess.run(list(command), cwd=root, check=False)
    except FileNotFoundError as exc:
        raise HostCliError(f"command not found: {command[0]}") from exc
    return int(completed.returncode)


def install(root: Path) -> int:
    source = (root / "r").resolve()
    preferred = [Path.home() / ".local" / "bin", Path.home() / "bin"]
    path_dirs = [Path(value).expanduser() for value in os.environ.get("PATH", "").split(os.pathsep) if value]
    destination_dir = next((path for path in preferred if path in path_dirs), preferred[0])
    destination_dir.mkdir(parents=True, exist_ok=True)
    destination = destination_dir / "r"
    if destination.exists() or destination.is_symlink():
        destination.unlink()
    destination.symlink_to(source)
    print(f"installed: {destination} -> {source}")
    if destination_dir not in path_dirs:
        print(f"PATH note: add {destination_dir} to PATH")
    return 0


def version(root: Path) -> int:
    def capture(*args: str) -> str:
        cp = subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True, check=False)
        return cp.stdout.strip() if cp.returncode == 0 else "-"

    dirty = subprocess.run(
        ["git", "-C", str(root), "status", "--porcelain"], capture_output=True, text=True, check=False
    )
    print(f"root:   {root}")
    print(f"branch: {capture('branch', '--show-current')}")
    print(f"commit: {capture('rev-parse', '--short=12', 'HEAD')}")
    print(f"dirty:  {'YES' if dirty.stdout.strip() else 'NO'}")
    return 0


def profile_test(root: Path, argv: list[str]) -> int:
    return test_runner.main(argv)


def list_tools(root: Path) -> int:
    rows: list[tuple[str, Path]] = []
    for parent in (root / "tools", root):
        if parent.is_dir():
            for path in sorted(parent.glob("*.py")):
                rows.append((path.stem, path.relative_to(root)))
    print("Repo Python tools:")
    for name, path in rows:
        print(f"  {name:<34} {path}")
    if not rows:
        print("  (none)")
    return 0


def _find_tool(root: Path, name: str) -> Path:
    filename = name if name.endswith(".py") else f"{name}.py"
    for path in (root / "tools" / filename, root / filename):
        if path.is_file():
            return path
    raise HostCliError(f"tool not found: {name}; use 'r tools'")


@contextlib.contextmanager
def hardware_guard(root: Path) -> Iterator[None]:
    from v3.operator_controller import OperatorController

    runtime = root / "runtime"
    runtime.mkdir(parents=True, exist_ok=True)
    with (runtime / ".r2b4_operator.lock").open("a+") as handle:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        if OperatorController(root).status().get("runtime_running"):
            raise HostCliError("resident V3 runtime owns physical hardware; stop it before this diagnostic")
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def run_tool(root: Path, argv: list[str]) -> int:
    if not argv:
        raise HostCliError("tool name required; use 'r tools'")
    path = _find_tool(root, argv[0])
    command = [sys.executable, str(path), *argv[1:]]
    if path.stem in HARDWARE_TOOLS:
        with hardware_guard(root):
            return _run(command, root=root)
    return _run(command, root=root)


def _read_cpu() -> dict[str, tuple[int, int]]:
    result: dict[str, tuple[int, int]] = {}
    with Path("/proc/stat").open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.startswith("cpu"):
                break
            parts = line.split()
            name = parts[0]
            if name == "cpu" or not name[3:].isdigit():
                continue
            values = [int(value) for value in parts[1:]]
            total = sum(values)
            idle = (values[3] if len(values) > 3 else 0) + (values[4] if len(values) > 4 else 0)
            result[name] = (total, idle)
    return result


def _temperature_c() -> float | None:
    path = Path("/sys/class/thermal/thermal_zone0/temp")
    try:
        value = float(path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return None
    return value / 1000.0 if value > 1000.0 else value


def cpu(root: Path, argv: list[str]) -> int:
    try:
        duration = float(argv[0]) if argv else 30.0
        interval = float(argv[1]) if len(argv) > 1 else 1.0
    except ValueError as exc:
        raise HostCliError("SECONDS and INTERVAL must be numbers") from exc
    if len(argv) > 2 or duration <= 0.0 or interval <= 0.0:
        raise HostCliError("usage: r cpu [SECONDS>0] [INTERVAL>0]")

    output_dir = root / "runtime" / "cpu"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / f"cpu_{dt.datetime.now():%Y%m%d_%H%M%S}.log"
    print(f"CPU monitor: {duration:g}s, interval {interval:g}s")
    print(f"log: {log_path}")
    previous = _read_cpu()
    started = time.monotonic()
    with log_path.open("w", encoding="utf-8", buffering=1) as log:
        while time.monotonic() - started < duration:
            remaining = duration - (time.monotonic() - started)
            time.sleep(min(interval, max(0.0, remaining)))
            current = _read_cpu()
            pieces: list[str] = []
            for name in sorted(current, key=lambda value: int(value[3:])):
                if name not in previous:
                    continue
                total_delta = current[name][0] - previous[name][0]
                idle_delta = current[name][1] - previous[name][1]
                usage = 0.0 if total_delta <= 0 else 100.0 * (total_delta - idle_delta) / total_delta
                pieces.append(f"{name}={usage:5.1f}%")
            previous = current
            load = os.getloadavg()
            temp = _temperature_c()
            temp_text = "temp=n/a" if temp is None else f"temp={temp:.1f}C"
            line = (
                f"{dt.datetime.now():%H:%M:%S} t={time.monotonic()-started:5.1f}s "
                f"load={load[0]:.2f}/{load[1]:.2f}/{load[2]:.2f} {temp_text} | "
                + " ".join(pieces)
            )
            print(line)
            log.write(line + "\n")
    print(f"CPU log saved: {log_path}")
    return 0


def disc(root: Path) -> int:
    _run(["df", "-h", str(root)], root=root)
    print("\nR2B4 data sizes:")
    for path in (root / "runtime", root / "runtime" / "captures", root / "runtime" / "cpu"):
        if path.exists():
            _run(["du", "-sh", str(path)], root=root)
    return 0


def temp() -> int:
    value = _temperature_c()
    print("CPU temperature: unavailable" if value is None else f"CPU temperature: {value:.1f} C")
    return 0


def host(root: Path) -> int:
    print(f"Host: {os.uname().nodename}")
    print(f"Kernel: {os.uname().sysname} {os.uname().release} {os.uname().machine}")
    _run(["uptime"], root=root)
    temp()
    print("\nMemory:")
    _run(["free", "-h"], root=root)
    print("\nDisk:")
    _run(["df", "-h", str(root)], root=root)
    return 0


def execute(command: str, argv: list[str], root: Path) -> int:
    command = ALIASES.get(command, command)
    no_args = {"install", "where", "root", "version", "tools", "disc", "mem", "temp", "ps", "net", "usb", "i2c", "host"}
    if command in no_args and argv:
        raise HostCliError(f"usage: r {command}")
    if command == "install": return install(root)
    if command == "where":
        print(f"launcher: {root / 'r'}\nrepo:     {root}")
        return 0
    if command == "root": print(root); return 0
    if command == "version": return version(root)
    if command == "gitre": return _run([sys.executable, str(root / "gitre.py"), *argv], root=root)
    if command == "git": return _run(["git", "-C", str(root), *argv], root=root)
    if command == "pytest": return _run([sys.executable, "-m", "pytest", *argv], root=root)
    if command == "test": return profile_test(root, argv)
    if command == "tools": return list_tools(root)
    if command == "tool": return run_tool(root, argv)
    if command == "cpu": return cpu(root, argv)
    if command == "cpu2": return _run([sys.executable, str(root / "tools" / "r2b4_cpu2.py"), "--root", str(root), *argv], root=root)
    if command == "disc": return disc(root)
    if command == "mem": return _run(["free", "-h"], root=root)
    if command == "temp": return temp()
    if command == "ps": return _run(["ps", "-eo", "pid,psr,pcpu,pmem,etime,comm,args", "--sort=-pcpu"], root=root)
    if command == "net":
        rc = _run(["ip", "-brief", "address"], root=root)
        print("\nRoutes:")
        _run(["ip", "route"], root=root)
        return rc
    if command == "usb": return _run(["lsusb"], root=root)
    if command == "i2c":
        with hardware_guard(root): return _run(["i2cdetect", "-y", "1"], root=root)
    if command == "host": return host(root)
    raise HostCliError(f"unsupported host command: {command}")


__all__ = ["ALIASES", "COMMANDS", "HostCliError", "execute"]
