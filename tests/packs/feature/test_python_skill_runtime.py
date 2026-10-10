"""File skills execute in isolated CPython; host cancellation needs no yielding."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest

from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.skill_runtime import SkillRuntime


def eventually(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("skill observation deadline expired")
        time.sleep(0.01)


def complete(runtime, run_id):
    eventually(lambda: not runtime.active)
    return runtime.status(run_id)


def process_running(pid):
    try:
        # An orphan zombie has already stopped executing; init owns its reaping.
        return Path(f"/proc/{pid}/stat").read_text().split(") ", 1)[1][0] != "Z"
    except FileNotFoundError:
        return False


def test_library_discovers_without_import_and_uses_ordinary_saved_files(tmp_path):
    marker = tmp_path / "must_not_import"
    library = SkillLibrary(tmp_path / "robot_skills")
    source = f'''"""Observe a named region."""
from pathlib import Path
Path({str(marker)!r}).touch()
async def run(robot, region: str, threshold=3, **options) -> dict:
    return {{"region": region}}
'''
    saved = library.create("monitor_region", source, test_source="def test_example():\n    assert 2 + 2 == 4\n")
    assert saved["description"] == "Observe a named region."
    assert saved["parameters"]["region"] == {"required": True, "type": "str"}
    assert saved["parameters"]["threshold"]["required"] is False
    assert not marker.exists()
    assert SkillLibrary(library.root).list("region")[0]["source_hash"] == saved["source_hash"]
    assert (library.root / "tests" / "test_monitor_region.py").exists()
    with pytest.raises(FileExistsError):
        library.create("monitor_region", "async def run(robot): return 2")
    with pytest.raises(SyntaxError):
        library.update("monitor_region", "async def run(")
    assert library.load("monitor_region").source == source
    for invalid in ("../outside", "run", "status"):
        with pytest.raises(ValueError):
            library.create(invalid, source)
    broken = "async def run("
    (library.root / "monitor_region.py").write_text(broken)
    assert library.load("monitor_region").source == broken
    assert library.list()[0]["availability"]["ready"] is False
    library.update("monitor_region", source)
    assert library.list()[0]["availability"]["ready"] is True


def test_standard_python_runs_without_factory_registration_or_llm(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    saved = library.create("calculate", '''
import asyncio
from dataclasses import dataclass
@dataclass
class Reading:
    value: int
def factorial(n):
    return 1 if n < 2 else n * factorial(n - 1)
async def run(robot, values, threshold=2):
    selected = []
    for value in values:
        await asyncio.sleep(0)
        reading = Reading(value)
        if reading.value >= threshold:
            selected.append(factorial(reading.value))
    return {"selected": selected}
''')
    runtime = SkillRuntime(library, tmp_path / "unused.sock")
    try:
        first = runtime.start("calculate", {"values": [1, 2, 4]})
        result = complete(runtime, first["run_id"])
        assert result["state"] == "SUCCEEDED", result
        assert result["result"] == {"selected": [2, 24]}
        assert result["source_hash"] == saved["source_hash"]
        assert result["pid"] != os.getpid()
        # The saved .py is sufficient after a new library/runtime instance.
        runtime.close()
        runtime = SkillRuntime(SkillLibrary(library.root), tmp_path / "unused.sock")
        second = runtime.start("calculate", {"values": [3]})
        assert complete(runtime, second["run_id"])["result"] == {"selected": [6]}
    finally:
        runtime.close()


def test_source_update_does_not_reload_running_module(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    started, finish = tmp_path / "started", tmp_path / "finish"
    old = library.create("wait_local", f'''
import asyncio
from pathlib import Path
async def run(robot):
    Path({str(started)!r}).touch()
    while not Path({str(finish)!r}).exists():
        await asyncio.sleep(0.01)
    return "loaded-first"
''')
    runtime = SkillRuntime(library, tmp_path / "unused.sock")
    try:
        first = runtime.start("wait_local")
        eventually(started.exists)
        new = library.update("wait_local", 'async def run(robot): return "loaded-second"\n')
        finish.touch()
        first_result = complete(runtime, first["run_id"])
        assert first_result["result"] == "loaded-first"
        assert first_result["source_hash"] == old["source_hash"]
        second = runtime.start("wait_local")
        second_result = complete(runtime, second["run_id"])
        assert second_result["result"] == "loaded-second"
        assert second_result["source_hash"] == new["source_hash"]
    finally:
        runtime.close()


def test_non_yielding_worker_and_children_are_stopped_with_bounded_output(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    child_pid = tmp_path / "child_pid"
    library.create("busy", f'''
import subprocess
import sys
from pathlib import Path
async def run(robot):
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    Path({str(child_pid)!r}).write_text(str(child.pid))
    print("x" * 100000, flush=True)
    while True:
        pass
''')
    callbacks = []
    runtime = SkillRuntime(library, tmp_path / "unused.sock", on_terminal=callbacks.append,
                           output_limit=1024, cancellation_grace_s=0.1)
    try:
        started = runtime.start("busy")
        eventually(child_pid.exists)
        eventually(lambda: runtime.status()["output_dropped_bytes"] > 0)
        child = int(child_pid.read_text())
        before = time.monotonic()
        revoked = runtime.revoke(started["run_id"], "TEST_STOP")
        assert time.monotonic() - before < 0.1
        assert revoked["state"] == "CANCELLED"
        assert not runtime.accepts(started["run_id"])
        result = runtime.stop(started["run_id"])
        assert not runtime.active
        assert result["error"] == "TEST_STOP"
        assert len(result["stdout"].encode()) <= 1024
        assert result["output_dropped_bytes"] > 0
        eventually(lambda: not process_running(child))
        assert len(callbacks) == 1 and callbacks[0]["state"] == "CANCELLED"
    finally:
        runtime.close()


@pytest.mark.parametrize("source,error", [
    ("async def run(robot): raise ValueError('local failure')", "ValueError:local failure"),
    ("import os\nasync def run(robot): os._exit(7)", "WORKER_EXIT:7"),
    ("async def run(robot): return 'x' * 100000", "result exceeded its bound"),
])
def test_worker_errors_and_crash_are_terminal_without_retry(tmp_path, source, error):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("fail_local", source)
    callbacks = []
    runtime = SkillRuntime(library, tmp_path / "unused.sock", on_terminal=callbacks.append)
    try:
        run = runtime.start("fail_local")
        result = complete(runtime, run["run_id"])
        assert result["state"] == "FAILED", result
        assert error in result["error"]
        assert not runtime.accepts(run["run_id"])
        assert len(callbacks) == 1
    finally:
        runtime.close()


def test_terminal_callback_finishes_before_replacement_is_admitted(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("done_local", "async def run(robot): return 1")
    callback_entered, callback_finish = threading.Event(), threading.Event()

    def callback(snapshot):
        callback_entered.set()
        callback_finish.wait(5)

    runtime = SkillRuntime(library, tmp_path / "unused.sock", on_terminal=callback)
    try:
        run = runtime.start("done_local")
        assert callback_entered.wait(5)
        assert runtime.status(run["run_id"])["finalizing"] is True
        assert runtime.active
        assert not runtime.accepts(run["run_id"])
        with pytest.raises(RuntimeError, match="preempted"):
            runtime.start("done_local")
        callback_finish.set()
        assert complete(runtime, run["run_id"])["state"] == "SUCCEEDED"
        second = runtime.start("done_local")
        assert complete(runtime, second["run_id"])["state"] == "SUCCEEDED"
    finally:
        callback_finish.set()
        runtime.close()


def test_host_death_kills_busy_worker_and_its_child_processes(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    worker_path, child_path = tmp_path / "worker_pid", tmp_path / "child_pid"
    library.create("host_death", f'''
import os
import subprocess
import sys
from pathlib import Path
async def run(robot):
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    Path({str(child_path)!r}).write_text(str(child.pid))
    Path({str(worker_path)!r}).write_text(str(os.getpid()))
    while True:
        pass
''')
    host_source = f'''
import time
from pathlib import Path
from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.skill_runtime import SkillRuntime
runtime = SkillRuntime(SkillLibrary(Path({str(library.root)!r})), Path({str(tmp_path / 'unused.sock')!r}))
runtime.start("host_death")
time.sleep(30)
'''
    host = subprocess.Popen([sys.executable, "-c", host_source], cwd=Path(__file__).resolve().parents[3])
    worker = child = None
    try:
        eventually(worker_path.exists)
        worker, child = int(worker_path.read_text()), int(child_path.read_text())
        host.kill()
        host.wait(timeout=2)
        eventually(lambda: not process_running(worker) and not process_running(child))
    finally:
        if host.poll() is None:
            host.kill()
            host.wait(timeout=2)
        if worker is not None:
            try:
                os.killpg(worker, signal.SIGKILL)
            except ProcessLookupError:
                pass
