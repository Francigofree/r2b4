#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
R2B4 OS – egyetlen belépési pont: robotvezérlő + webfelület távoli elérésre.
- Elindítja a FastGUI-t (fastgui), majd a robotvezérlőt (cont.py).
  A cont.py nem nyit saját GUI ablakot (SKIP_GUI=1).
Használat: python3 os.py
Távoli elérés: http://<RPi_IP>:7860
"""

import os
import sys
import threading
import re
import subprocess
import signal
import time

# Libcamera C++ log csöndesítés (kamera init ne szemeteljen a terminálba)
os.environ.setdefault("LIBCAMERA_LOG_LEVELS", "*:ERROR")

# A cont.py ne indítson saját GUI ablakot – az os.py indítja a webszervert
os.environ["SKIP_GUI"] = "1"

# Projekt gyökér a Python path-on
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from log.log_paths import publish_latest_alias, process_session_dir, set_process_session_dir  # noqa: E402

SESSION_DIR = set_process_session_dir(process_session_dir(create=True), export_env=True, create=True)
LOG_FILE = str(SESSION_DIR / "runtime" / "os.log")

class Tee:
    def __init__(self, filename, stream, flush_interval_s=0.25):
        self.file = open(filename, "a", encoding="utf-8")
        self.stream = stream
        self.flush_interval_s = max(0.02, float(flush_interval_s))
        self._last_flush_ts = time.monotonic()
        self._lock = threading.RLock()
    def write(self, data):
        with self._lock:
            self.file.write(data)
            self.stream.write(data)
            now = time.monotonic()
            if len(data) >= 4096 or ("\n" in data and (now - self._last_flush_ts) >= self.flush_interval_s):
                self.file.flush()
                self.stream.flush()
                self._last_flush_ts = now
    def flush(self):
        with self._lock:
            self.file.flush()
            self.stream.flush()
            self._last_flush_ts = time.monotonic()
    def fileno(self):
        return self.stream.fileno()
    def isatty(self):
        return self.stream.isatty()
    @property
    def encoding(self):
        return self.stream.encoding

# Redirect stdout and stderr. Agent runtime manager már ugyanide irányítja a
# gyerekfolyamat stdoutját, ilyenkor a duplázó Tee kikapcsolható.
os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
if os.environ.get("R2B4_SKIP_OS_TEE") != "1":
    sys.stdout = Tee(LOG_FILE, sys.stdout)
    sys.stderr = Tee(LOG_FILE, sys.stderr)
publish_latest_alias(LOG_FILE)

# GUI port (környezeti változó felülírja)
GUI_PORT = int(os.environ.get("FLASK_PORT", "7860"))


def is_port_in_use(port: int) -> bool:
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(("127.0.0.1", port)) == 0


def wait_for_port(port: int, timeout_sec: float = 4.0, step_sec: float = 0.2) -> bool:
    """Rövid readiness ellenőrzés: a GUI port ténylegesen hallgat-e."""
    import time
    deadline = time.monotonic() + max(0.1, timeout_sec)
    while time.monotonic() < deadline:
        if is_port_in_use(port):
            return True
        time.sleep(max(0.05, step_sec))
    return is_port_in_use(port)


def find_other_os_instances():
    """
    Visszaadja a jelenlegi folyamattól eltérő, futó os.py példányokat.
    Cél: ne indulhasson párhuzamosan két teljes runtime (GUI+controller),
    mert az tipikusan port/GPIO ütközést okoz.
    """
    pattern = re.compile(r"(^|\s)(python|python3)?(\s+|.*/)?os\.py(\s|$)")
    my_pid = os.getpid()
    out = []
    try:
        proc = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            capture_output=True,
            text=True,
            check=False,
        )
    except Exception:
        return out

    for raw in (proc.stdout or "").splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) < 2:
            continue
        try:
            pid = int(parts[0])
        except Exception:
            continue
        if pid == my_pid:
            continue
        args = str(parts[1] or "").strip()
        low = args.lower()
        if "agent_runtime_manager.py" in low:
            continue
        if pattern.search(low):
            out.append((pid, args))
    return out


def _pid_exists(pid: int) -> bool:
    try:
        os.kill(int(pid), 0)
        return True
    except Exception:
        return False


def _send_signal(pid: int, sig: int) -> None:
    # Ha a folyamat saját session leader (agent manager által indítva), ez lezárja a teljes csoportot.
    try:
        os.killpg(int(pid), int(sig))
        return
    except Exception:
        pass
    try:
        os.kill(int(pid), int(sig))
    except Exception:
        pass


def _wait_pids_stopped(pids, timeout_sec: float) -> bool:
    deadline = time.monotonic() + max(0.2, float(timeout_sec))
    while time.monotonic() <= deadline:
        alive = [pid for pid in pids if _pid_exists(pid)]
        if not alive:
            return True
        time.sleep(0.2)
    return not any(_pid_exists(pid) for pid in pids)


def stop_other_os_instances(graceful_timeout_sec: float = 6.0, hard_timeout_sec: float = 3.0) -> bool:
    others = find_other_os_instances()
    if not others:
        return True

    pids = sorted({int(pid) for pid, _ in others if int(pid) > 0})
    if not pids:
        return True

    pid_text = ", ".join(str(pid) for pid in pids)
    print(f"[OS] Futó régi os.py példány(ok) leállítása indulás előtt (PID: {pid_text}).", file=sys.stderr)

    for pid in pids:
        _send_signal(pid, signal.SIGINT)
    if _wait_pids_stopped(pids, timeout_sec=graceful_timeout_sec):
        return True

    print("[OS] FIGYELEM: Régi os.py példány még fut, SIGTERM küldés.", file=sys.stderr)
    for pid in pids:
        _send_signal(pid, signal.SIGTERM)
    if _wait_pids_stopped(pids, timeout_sec=hard_timeout_sec):
        return True

    print("[OS] FIGYELEM: Régi os.py példány még fut, SIGKILL küldés.", file=sys.stderr)
    for pid in pids:
        _send_signal(pid, signal.SIGKILL)
    return _wait_pids_stopped(pids, timeout_sec=1.5)


def _run_uvicorn(app_path: str, port: int, access_log: bool = False):
    """Uvicorn szerver indítása adott app-ra és portra."""
    import uvicorn
    # log_config=None esetén nem próbálja meg a Click-et konfigurálni, 
    # így elkerülhető az "Unable to configure formatter 'default'" hiba Tee használata mellett.
    uvicorn.run(
        app_path,
        host="0.0.0.0",
        port=port,
        log_level="info",
        access_log=access_log,
        log_config=None,
    )


def run_gui():
    """FastGUI indítása (fastgui.main)."""
    if is_port_in_use(GUI_PORT):
        print(
            f"[OS] FIGYELEM: GUI port {GUI_PORT} már foglalt. GUI indítás kihagyva.",
            file=sys.stderr,
        )
        return
    try:
        _run_uvicorn("fastgui.main:app", GUI_PORT, access_log=False)
    except Exception as e:
        print(f"[OS] GUI hiba: {e}", file=sys.stderr)


def main():
    if not stop_other_os_instances(graceful_timeout_sec=6.0, hard_timeout_sec=3.0):
        print("[OS] HIBA: Régi os.py példány leállítása nem sikerült, új indítás megszakítva.", file=sys.stderr)
        return 1

    # Apply the service mask before the GUI or any controller worker thread is
    # created. The main thread is narrowed to the control CPU immediately
    # before AlbaController enters its 50 Hz owner loop.
    from config_manager import config as global_config
    from controller.runtime_affinity import apply_runtime_affinity, config_from_root

    apply_runtime_affinity(config_from_root(global_config.data), role="service")

    print("[OS] R2B4 OS indítás: GUI + robotvezérlő")
    print(f"[OS] GUI: http://0.0.0.0:{GUI_PORT}  (távolról: http://<RPi_IP>:{GUI_PORT})")

    if is_port_in_use(GUI_PORT):
        print(
            f"[OS] FIGYELEM: GUI port {GUI_PORT} már foglalt. Meglévő GUI példányt használok.",
            file=sys.stderr,
        )
    else:
        t_gui = threading.Thread(target=run_gui, daemon=True, name="r2b4-gui")
        t_gui.start()

    # GUI readiness probe: ne csak fix sleep legyen, hanem ellenőrzés is.
    if wait_for_port(GUI_PORT, timeout_sec=4.0, step_sec=0.2):
        print(f"[OS] GUI elérhető a {GUI_PORT} porton.")
    else:
        print(f"[OS] FIGYELEM: GUI nem válaszol időben (port: {GUI_PORT}). A vezérlő ettől még indul.", file=sys.stderr)

    # Robotvezérlő futtatása (blokkoló – ez a fő szál)
    from cont import AlbaController
    AlbaController().run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
