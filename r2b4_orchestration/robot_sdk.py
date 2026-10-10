"""Async client of the common RobotInterface and invocation-owned sessions.

Each request has its own bounded socket. Event waiting never holds the host's
request slots, and STOP can proceed while a motion or media call is waiting.
"""
from __future__ import annotations

import asyncio
import json
import math
import inspect
import types
import sys
from pathlib import Path

from .robot_runtime import MAX_REPLY_BYTES, MAX_REQUEST_BYTES, SCHEMA, socket_path_for


class RobotEvent(dict):
    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc


class AsyncRobotSDK:
    def __init__(self, socket_path=None, *, invocation_id=None, root=None, timeout_s=35.0):
        self.root = Path(root or Path.cwd()).resolve()
        self.socket_path = Path(socket_path or socket_path_for(self.root))
        self.invocation_id = invocation_id
        self.timeout_s = timeout_s
        self._sessions = set()
        self.skill = SkillClient(self)
        self.events = EventClient(self)
        self.media = MediaClient(self)

    async def request(self, operation, *, timeout_s=None, **arguments):
        request = {"schema": SCHEMA, "operation": operation, **arguments}
        if self.invocation_id is not None:
            request["invocation_id"] = self.invocation_id
        payload = json.dumps(request, allow_nan=False, separators=(",", ":")).encode() + b"\n"
        if len(payload) > MAX_REQUEST_BYTES:
            raise ValueError("public robot request exceeded its bound")
        async def exchange():
            reader, writer = await asyncio.open_unix_connection(str(self.socket_path), limit=MAX_REPLY_BYTES)
            try:
                writer.write(payload)
                await writer.drain()
                line = await reader.readline()
                if not line or len(line) > MAX_REPLY_BYTES or not line.endswith(b"\n"):
                    raise ValueError("public robot reply exceeded its bound or was incomplete")
                response = json.loads(line)
                if response.get("schema") != SCHEMA:
                    raise RuntimeError("public robot response schema mismatch")
                if response.get("error"):
                    raise RuntimeError(response["error"])
                return response.get("result")
            finally:
                writer.close()
                await writer.wait_closed()
        return await asyncio.wait_for(exchange(), self.timeout_s if timeout_s is None else timeout_s)

    async def capabilities(self):
        return await self.request("capabilities")

    async def read(self, resource):
        return await self.request("read", resource=resource)

    async def query(self, query=None, **parameters):
        value = query.to_jsonable() if hasattr(query, "to_jsonable") else query if query is not None else parameters
        return await self.request("query", query=value)

    async def spatial_query(self, query=None, **parameters):
        value = query.to_jsonable() if hasattr(query, "to_jsonable") else query if query is not None else parameters
        return await self.request("spatial_query", query=value)

    async def call(self, action, **parameters):
        if action == "vision.observe":
            # Calibrated image bytes go directly producer -> requesting skill,
            # independently of the compact host socket/control interpreter.
            from v3.adapters.vision_media_socket import VisionClient
            if self.invocation_id is not None:
                await self.skill.status(self.invocation_id)
            stream = parameters.pop("stream", "lores")
            if parameters:
                raise ValueError("unknown vision.observe parameters")
            return await asyncio.to_thread(VisionClient(root=self.root).observe, stream_name=stream)
        return await self.request("call", action=action, parameters=parameters)

    execute = call

    async def stop(self):
        return await self.request("stop", timeout_s=5.0)

    def observe(self, *, region=None, object_kind="person", **parameters):
        return Observer(self, {"region": region, "object_kind": object_kind, **parameters})

    async def result(self, run_id):
        return await self.skill.status(run_id)

    async def cancel(self, run_id):
        return await self.skill.stop(run_id)

    async def close(self):
        await asyncio.gather(*(session.close() for session in tuple(self._sessions)), return_exceptions=True)


class SkillClient:
    def __init__(self, robot):
        self.robot = robot

    async def list(self):
        return await self.robot.read("skill.list")

    async def create(self, name, source, description=""):
        return await self.robot.call("skill.create", name=name, source=source, description=description)

    async def update(self, name, source, description=""):
        return await self.robot.call("skill.update", name=name, source=source, description=description)

    async def run(self, name, **parameters):
        return await self.robot.call("skill.run", name=name, parameters=parameters)

    async def invoke(self, name, **parameters):
        """Call a related skill in this worker, retaining the same invocation."""
        source = await self.robot.call("skill.source", name=name)
        module = types.ModuleType("_r2b4_subskill_" + name + "_" + source["source_hash"])
        module.__file__ = source["source_path"]
        module.__package__ = ""
        sys.modules[module.__name__] = module
        exec(compile(source["source"], module.__file__, "exec"), module.__dict__)
        value = module.run(self.robot, **parameters)
        return await value if inspect.isawaitable(value) else value

    async def test(self, name, *, timeout_s=30):
        return await self.robot.call("skill.test", name=name, timeout_s=timeout_s)

    async def status(self, run_id=None):
        return await self.robot.call("skill.status", **({"run_id": run_id} if run_id is not None else {}))

    async def stop(self, run_id=None, reason="STOP"):
        return await self.robot.call("skill.stop", reason=reason, **({"run_id": run_id} if run_id is not None else {}))


class EventClient:
    def __init__(self, robot):
        self.robot = robot
        self.cursor = 0
        self.pending = []

    async def wait(self, source=None, *, timeout_s=None, kinds=None):
        if source is not None:
            return await source.wait(timeout_s=timeout_s)
        deadline = _deadline(timeout_s)
        while True:
            if self.pending:
                return self.pending.pop(0)
            page = await self.robot.request("events", after_sequence=self.cursor, kinds=kinds)
            self.cursor = page["next_sequence"]
            if page["lost_events"]:
                self.pending.append(RobotEvent(kind="coverage_gap", lost_events=page["lost_events"],
                                               sequence=self.cursor))
            self.pending.extend(RobotEvent(row) for row in page["events"])
            if self.pending:
                continue
            if not await _pause(deadline):
                return None


def _deadline(timeout_s):
    if timeout_s is None:
        return None
    if type(timeout_s) not in {int, float} or not math.isfinite(timeout_s) or timeout_s < 0:
        raise ValueError("timeout_s must be finite and non-negative")
    return asyncio.get_running_loop().time() + timeout_s


async def _pause(deadline):
    remaining = None if deadline is None else deadline - asyncio.get_running_loop().time()
    if remaining is not None and remaining <= 0:
        return False
    await asyncio.sleep(0.1 if remaining is None else min(0.1, remaining))
    return True


class Observer:
    def __init__(self, robot, parameters):
        self.robot, self.parameters = robot, parameters
        self.handle = None
        self.cursor = 0
        self.pending = []
        self._owner_offset_ns = 0

    async def __aenter__(self):
        before = asyncio.get_running_loop().time()
        result = await self.robot.call("observe.open", **self.parameters)
        self.handle = result["handle"]
        if result.get("status") != "READY":
            await self.close()
            raise RuntimeError("OBSERVER_NOT_READY")
        # Linux local processes share the monotonic clock. Explicit owner time
        # also makes the deadline translation visible for alternate producers.
        after = asyncio.get_running_loop().time()
        self._owner_offset_ns = (0 if result.get("clock_domain") == "host_monotonic" else
                                result["monotonic_ns"] - int((before + after) * 0.5 * 1e9))
        self.robot._sessions.add(self)
        return self

    async def __aexit__(self, *_):
        await self.close()

    async def close(self):
        if self.handle is not None:
            handle, self.handle = self.handle, None
            await self.robot.call("observe.close", handle=handle)
        self.robot._sessions.discard(self)

    def _ingest(self, page):
        if page.get("lost_events"):
            self.pending.append(RobotEvent(kind="coverage_gap", reason="EVENT_HISTORY_GAP",
                                          lost_events=page["lost_events"]))
        for event in page.get("events", []):
            sequence = event["sequence"]
            if sequence > self.cursor:
                self.pending.append(RobotEvent(event))
                self.cursor = sequence

    async def wait(self, *, timeout_s=None):
        deadline = _deadline(timeout_s)
        while True:
            if self.pending:
                return self.pending.pop(0)
            page = await self.robot.call("observe.events", handle=self.handle, after_sequence=self.cursor)
            self._ingest(page)
            if self.pending:
                continue
            if page.get("status") in {"FAILED", "CLOSED"}:
                return RobotEvent(kind="capability_failed", reason=page.get("reason", page["status"]))
            if not await _pause(deadline):
                return None

    async def finish_events(self, *, until, timeout_s=5):
        deadline = _deadline(timeout_s)
        until_ns = int(until * 1e9) + self._owner_offset_ns
        while True:
            while self.pending:
                event = self.pending.pop(0)
                if event.get("measurement_monotonic_ns", 0) <= until_ns:
                    yield event
            remaining = max(0.0, deadline - asyncio.get_running_loop().time())
            page = await self.robot.call("observe.finish", handle=self.handle, until_monotonic_ns=until_ns,
                                         timeout_s=remaining, after_sequence=self.cursor)
            self._ingest(page)
            while self.pending:
                event = self.pending.pop(0)
                if event.get("measurement_monotonic_ns", 0) <= until_ns:
                    yield event
            if page.get("finished") and self.cursor >= page.get("latest_sequence", self.cursor):
                if page.get("partial"):
                    yield RobotEvent(kind="coverage_gap", reason="INTERVAL_CLOSURE_PARTIAL")
                return
            if not await _pause(deadline):
                yield RobotEvent(kind="coverage_gap", reason="INTERVAL_CLOSURE_TIMEOUT")
                return


class MediaClient:
    def __init__(self, robot):
        self.robot = robot

    def event_recorder(self, observer, **parameters):
        return Recorder(self.robot, observer, parameters)


class Recorder:
    def __init__(self, robot, observer, parameters):
        self.robot, self.observer, self.parameters = robot, observer, parameters
        self.handle = None

    async def __aenter__(self):
        result = await self.robot.call("media.recorder.open", observer=self.observer.handle, **self.parameters)
        self.handle = result["handle"]
        if result.get("status") != "READY":
            await self.__aexit__()
            raise RuntimeError("RECORDER_NOT_READY")
        self.robot._sessions.add(self)
        return self

    async def __aexit__(self, *_):
        await self.close()

    async def close(self):
        if self.handle is not None:
            handle, self.handle = self.handle, None
            await self.robot.call("media.recorder.close", handle=handle)
        self.robot._sessions.discard(self)

    async def start_event_clip(self, event):
        return await self.robot.call("media.recorder.start_event_clip", handle=self.handle, event=dict(event))

    async def finish(self, *, timeout_s=5):
        return await self.robot.call("media.recorder.finish", handle=self.handle, timeout_s=timeout_s)

    async def status(self):
        return await self.robot.call("media.recorder.status", handle=self.handle)


__all__ = ["AsyncRobotSDK", "RobotEvent"]
