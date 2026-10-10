"""Common facade for the existing source, evidence and config tool owners."""
from pathlib import Path
import os
import selectors
import signal
import subprocess
import sys
import time

from .agent_source_tools import build_source_tools
from .agent_evidence_tools import build_evidence_tools
from .agent_config_tools import build_config_tools


class SoftwareInterfaceAdapter:
    name = "software"

    def __init__(self, root: Path):
        tools = (*build_source_tools(root), *build_evidence_tools(root), *build_config_tools(root))
        self._tools = {spec.name: (spec, handler) for spec, handler in tools}
        self.capability_names = frozenset(self._tools)

    def capabilities(self):
        return {name: {"name": name, "description": spec.description, "parameters": dict(spec.arguments),
                       "result": {"description": "Existing owning tool result."}, "kind": "action",
                       "supported": True, "available": True, "ready": True, "owner": "host"}
                for name, (spec, _) in self._tools.items()}

    def execute(self, action, **parameters):
        return self._tools[action][1](parameters)

    def read(self, resource):
        raise KeyError(resource)


def test_skill(library, name, *, timeout_s=30.0):
    """Run one optional saved test through the canonical pytest launcher."""
    if type(timeout_s) not in {int, float} or not 0 < timeout_s <= 30:
        raise ValueError("test timeout must be within (0, 30]")
    entry = library.load(name)
    target = library.root / "tests" / f"test_{entry.name}.py"
    if target.is_symlink() or not target.is_file():
        raise ValueError("SKILL_TEST_UNAVAILABLE")
    # Use the canonical launcher function for a library test, including in a
    # temporary development library. This does not add a gate/manifest entry.
    command = [sys.executable, "-c", "from v3.test_runner import _run; import sys; sys.exit(_run([sys.argv[1]]))", str(target)]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join((str(library.root), str(Path(__file__).resolve().parent.parent),
                                                environment.get("PYTHONPATH", "")))
    process = subprocess.Popen(command, cwd=Path(__file__).resolve().parent.parent, env=environment,
                               stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               start_new_session=True, close_fds=True)
    output = b""
    dropped = 0
    timed_out = False
    deadline = time.monotonic() + timeout_s
    selector = selectors.DefaultSelector()
    os.set_blocking(process.stdout.fileno(), False)
    selector.register(process.stdout, selectors.EVENT_READ)
    try:
        exit_seen = None
        while True:
            now = time.monotonic()
            if now >= deadline:
                timed_out = True
                break
            # Keep the leader unreaped until its process group is cleaned up;
            # the PID then cannot be reused for an unrelated group.
            if os.waitid(os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT) is not None:
                exit_seen = exit_seen or now
                if not selector.get_map() or now - exit_seen > 0.1:
                    break
            for key, _ in selector.select(min(0.1, deadline - now)):
                chunk = os.read(key.fd, 8192)
                if not chunk:
                    selector.unregister(key.fileobj)
                    continue
                tail = output + chunk
                dropped += max(0, len(tail) - 16_384)
                output = tail[-16_384:]
    finally:
        selector.close()
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=1)
        process.stdout.close()
    current_hash = library.load(name).source_hash
    return {"status": "PASSED" if process.returncode == 0 and not timed_out and current_hash == entry.source_hash else "FAILED",
            "returncode": process.returncode, "timeout": timed_out,
            "output": output.decode("utf-8", errors="replace"), "output_dropped_bytes": dropped,
            "test_path": str(target), "source_hash": entry.source_hash, "source_changed": current_hash != entry.source_hash}
