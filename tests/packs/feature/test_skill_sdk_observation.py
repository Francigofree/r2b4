"""Async SDK observation/media wrappers through the real bounded socket client."""
from __future__ import annotations

import asyncio
import json
import time

import pytest

from r2b4_orchestration.robot_runtime import SCHEMA
from r2b4_orchestration.robot_sdk import AsyncRobotSDK


class ObservationOwner:
    """Scripted compact producer responses; no vision or control is started."""

    def __init__(self):
        self.calls = []
        self.event_pages = []
        self.finish_pages = []
        self.open_status = "READY"
        self.recorder_status = "READY"
        self.entered = asyncio.Event()

    async def call(self, name, parameters):
        self.calls.append((name, dict(parameters)))
        if name == "observe.open":
            # Startup latency must never shift a shared Linux measurement clock.
            await asyncio.sleep(0.02)
            return {"handle": "observer-a", "status": self.open_status,
                    "clock_domain": "host_monotonic", "monotonic_ns": time.monotonic_ns()}
        if name == "observe.events":
            return self.event_pages.pop(0) if self.event_pages else {
                "events": [], "lost_events": 0, "status": "READY", "latest_sequence": parameters["after_sequence"]}
        if name == "observe.finish":
            return self.finish_pages.pop(0)
        if name == "media.recorder.open":
            self.entered.set()
            return {"handle": "recorder-a", "status": self.recorder_status}
        if name == "media.recorder.start_event_clip":
            return {"clip_id": "clip-a", "status": "ACCEPTED"}
        if name == "media.recorder.finish":
            return {"handle": "recorder-a", "status": "PARTIAL", "partial": True,
                    "clips": [{"clip_id": "clip-a", "status": "RECORDING"}]}
        if name in {"observe.close", "media.recorder.close"}:
            return {"handle": parameters["handle"], "status": "CLOSED"}
        raise AssertionError(name)


async def connected_sdk(path, owner):
    async def handle(reader, writer):
        try:
            request = json.loads(await reader.readline())
            assert request["schema"] == SCHEMA and request["invocation_id"] == "worker-a"
            assert request["operation"] == "call"
            result = await owner.call(request["action"], request["parameters"])
            writer.write(json.dumps({"schema": SCHEMA, "result": result}).encode() + b"\n")
            await writer.drain()
        finally:
            writer.close()
            await writer.wait_closed()

    server = await asyncio.start_unix_server(handle, path=str(path))
    return server, AsyncRobotSDK(socket_path=path, invocation_id="worker-a", timeout_s=1)


def event(sequence, measured, kind="qualified_presence"):
    return {"kind": kind, "sequence": sequence, "event_id": f"observer-a:{sequence}",
            "source_sequence": sequence + 20, "source_frame_sequence": sequence + 30,
            "measurement_monotonic_ns": measured, "owner_generation": "generation-a"}


def test_sdk_ready_event_lineage_duplicate_gap_and_context_cleanup(tmp_path):
    async def scenario():
        owner = ObservationOwner()
        measured = time.monotonic_ns()
        observed = event(1, measured)
        gap = event(2, measured + 1, "coverage_gap")
        owner.event_pages = [{"events": [observed, dict(observed), gap],
                              "lost_events": 0, "status": "READY", "latest_sequence": 2}]
        server, robot = await connected_sdk(tmp_path / "sdk.sock", owner)
        async with server:
            async with robot.observe(region=None, object_kind="person") as observer:
                first = await robot.events.wait(observer, timeout_s=0.1)
                assert first.kind == "qualified_presence"
                assert first.measurement_monotonic_ns == measured
                assert first.source_sequence == 21 and first.owner_generation == "generation-a"
                second = await robot.events.wait(observer, timeout_s=0.1)
                assert second.kind == "coverage_gap" and second.sequence == 2
                assert await robot.events.wait(observer, timeout_s=0) is None
            assert owner.calls[-1] == ("observe.close", {"handle": "observer-a"})
            reads = [parameters for name, parameters in owner.calls if name == "observe.events"]
            assert [item["after_sequence"] for item in reads] == [0, 2]

            # Failed READY entry must release the accepted session handle.
            owner.open_status = "FAILED"
            with pytest.raises(RuntimeError, match="OBSERVER_NOT_READY"):
                async with robot.observe():
                    raise AssertionError("unready observer entered")
            assert owner.calls[-1] == ("observe.close", {"handle": "observer-a"})

    asyncio.run(scenario())


def test_sdk_drains_deadline_pages_with_loss_then_finishes_recorder(tmp_path):
    async def scenario():
        owner = ObservationOwner()
        server, robot = await connected_sdk(tmp_path / "sdk.sock", owner)
        async with server:
            async with robot.observe() as observer:
                async with robot.media.event_recorder(observer, pre_s=1, post_s=2) as recorder:
                    until = asyncio.get_running_loop().time()
                    until_ns = int(until * 1e9)
                    first = event(3, until_ns - 10)
                    last = event(5, until_ns - 1)
                    owner.finish_pages = [
                        {"events": [first, event(4, until_ns + 1)], "lost_events": 2,
                         "finished": True, "partial": True, "latest_sequence": 5},
                        {"events": [last], "lost_events": 0,
                         "finished": True, "partial": True, "latest_sequence": 5},
                    ]
                    drained = [row async for row in observer.finish_events(until=until, timeout_s=1)]
                    presences = [row for row in drained if row.kind == "qualified_presence"]
                    assert [row.measurement_monotonic_ns for row in presences] == [until_ns - 10, until_ns - 1]
                    assert [row.source_sequence for row in presences] == [23, 25]
                    gaps = [row for row in drained if row.kind == "coverage_gap"]
                    assert gaps[0].lost_events == 2
                    assert gaps[-1].reason == "INTERVAL_CLOSURE_PARTIAL"
                    for row in presences:
                        accepted = await recorder.start_event_clip(row)
                        assert accepted["status"] == "ACCEPTED"
                    media = await recorder.finish(timeout_s=0.1)
                    assert media["partial"] and media["clips"][0]["status"] == "RECORDING"
            names = [name for name, _ in owner.calls]
            assert names[-3:] == ["media.recorder.finish", "media.recorder.close", "observe.close"]
            requests = [parameters for name, parameters in owner.calls if name == "observe.finish"]
            assert len(requests) == 2
            assert all(item["until_monotonic_ns"] == until_ns for item in requests)
            assert [item["after_sequence"] for item in requests] == [0, 4]
            assert names.index("media.recorder.finish") > max(index for index, name in enumerate(names) if name == "observe.finish")

    asyncio.run(scenario())


def test_sdk_task_cancellation_closes_recorder_before_observer(tmp_path):
    async def scenario():
        owner = ObservationOwner()
        server, robot = await connected_sdk(tmp_path / "sdk.sock", owner)
        async with server:
            async def monitor():
                async with robot.observe() as observer:
                    async with robot.media.event_recorder(observer):
                        await robot.events.wait(observer, timeout_s=100)

            task = asyncio.create_task(monitor())
            await asyncio.wait_for(owner.entered.wait(), 1)
            # Let the recorder's READY reply reach the worker before cancellation.
            await asyncio.sleep(0.02)
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            assert [name for name, _ in owner.calls][-2:] == ["media.recorder.close", "observe.close"]
            assert not any(name == "media.recorder.finish" for name, _ in owner.calls)

            owner.recorder_status = "FAILED"
            async with robot.observe() as observer:
                with pytest.raises(RuntimeError, match="RECORDER_NOT_READY"):
                    async with robot.media.event_recorder(observer):
                        raise AssertionError("unready recorder entered")
            assert [name for name, _ in owner.calls][-2:] == ["media.recorder.close", "observe.close"]

    asyncio.run(scenario())
