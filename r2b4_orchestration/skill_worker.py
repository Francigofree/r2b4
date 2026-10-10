"""One trusted CPython skill invocation, supervised by the public host."""

from __future__ import annotations

import argparse
import asyncio
import inspect
import json
import os
from pathlib import Path
import signal
import sys
import types


MAX_RESULT_BYTES = 32_768
MAX_INPUT_BYTES = 600_000


def _parent_guard(liveness_fd: int, result_fd: int) -> int:
    """EOF from the host kills this invocation's session, even during a busy loop.

    A Python signal handler cannot run in a non-yielding skill. This child only
    waits on a pipe, outside the skill's GIL, and owns no robot capability.
    """
    group = os.getpgrp()
    if group != os.getpid():
        raise RuntimeError("skill worker must own a fresh process group")
    guardian = os.fork()
    if guardian == 0:
        for fd in (0, 1, 2, result_fd):
            if fd != liveness_fd:
                os.close(fd)
        try:
            while os.read(liveness_fd, 1):
                pass
            os.killpg(group, signal.SIGKILL)
        finally:
            os._exit(0)
    os.close(liveness_fd)
    return guardian


def _send(fd: int, value: dict[str, object]) -> None:
    data = json.dumps(value, allow_nan=False, separators=(",", ":")).encode("utf-8") + b"\n"
    if len(data) > MAX_RESULT_BYTES:
        raise ValueError("skill result exceeded its bound; return an asset reference instead")
    while data:
        written = os.write(fd, data)
        data = data[written:]


async def _run(request: dict[str, object], fd: int) -> None:
    from r2b4_orchestration.robot_sdk import AsyncRobotSDK

    source_path = Path(str(request["source_path"]))
    sys.path.insert(0, str(source_path.parent))
    module = types.ModuleType("_r2b4_skill_" + str(request["name"]))
    module.__file__ = str(source_path)
    module.__package__ = ""
    sys.modules[module.__name__] = module
    robot = AsyncRobotSDK(Path(str(request["socket_path"])), invocation_id=str(request["run_id"]),
                          root=Path(str(request["project_root"])))
    current = asyncio.current_task()
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, current.cancel)
    loop.add_signal_handler(signal.SIGINT, current.cancel)
    try:
        # Imports and arbitrary module code execute only in this worker.
        exec(compile(str(request["source"]), str(source_path), "exec"), module.__dict__)
        entry = getattr(module, "run", None)
        if not callable(entry):
            raise ValueError("skill module must expose run(robot, **parameters)")
        _send(fd, {"state": "RUNNING"})
        result = entry(robot, **request["parameters"])
        if inspect.isawaitable(result):
            result = await result
        _send(fd, {"state": "SUCCEEDED", "result": result})
    finally:
        close = getattr(robot, "close", None)
        if close is not None:
            result = close()
            if inspect.isawaitable(result):
                await result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-fd", required=True, type=int)
    parser.add_argument("--liveness-fd", required=True, type=int)
    args = parser.parse_args()
    guardian = None
    try:
        guardian = _parent_guard(args.liveness_fd, args.result_fd)
        data = sys.stdin.buffer.read(MAX_INPUT_BYTES + 1)
        if len(data) > MAX_INPUT_BYTES:
            raise ValueError("skill worker request exceeded its bound")
        request = json.loads(data)
        asyncio.run(_run(request, args.result_fd))
        return 0
    except asyncio.CancelledError:
        _send(args.result_fd, {"state": "CANCELLED", "error": "WORKER_CANCELLED"})
        return 0
    except BaseException as exc:
        _send(args.result_fd, {"state": "FAILED", "error": f"{type(exc).__name__}:{exc}"[:4096]})
        return 1
    finally:
        os.close(args.result_fd)
        if guardian is not None:
            try:
                os.kill(guardian, signal.SIGTERM)
            except ProcessLookupError:
                pass
            os.waitpid(guardian, 0)


if __name__ == "__main__":
    raise SystemExit(main())
