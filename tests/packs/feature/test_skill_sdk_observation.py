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
            if request["operation"] == "events":
                result = await owner.events(request)
            else:
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
            # An expired wait returns without starting another socket read.
            assert [item["after_sequence"] for item in reads] == [0]

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


def test_sdk_wait_retains_post_deadline_measurement_for_interval_finish(tmp_path):
    async def scenario():
        owner = ObservationOwner()
        server, robot = await connected_sdk(tmp_path / "sdk.sock", owner)
        async with server:
            async with robot.observe() as observer:
                async with robot.media.event_recorder(observer) as recorder:
                    loop = asyncio.get_running_loop()
                    deadline = loop.time() + 0.1
                    until_ns = int(deadline * 1e9)
                    late = event(1, until_ns + 1_000_000_000)
                    owner.event_pages = [{"events": [late], "lost_events": 0,
                                          "status": "READY", "latest_sequence": 1}]
                    owner.finish_pages = [{"events": [], "lost_events": 0,
                                           "finished": True, "latest_sequence": 1}]

                    observed = await robot.events.wait(observer, timeout_s=deadline - loop.time())
                    if observed is not None:
                        await recorder.start_event_clip(observed)
                    assert observed is None
                    assert observer.pending == [late]
                    assert await robot.events.wait(observer, timeout_s=0) is None
                    assert observer.pending == [late]

                    drained = [row async for row in observer.finish_events(until=deadline, timeout_s=0.1)]
                    for row in drained:
                        if row.kind == "qualified_presence":
                            await recorder.start_event_clip(row)
                    assert drained == []
                    assert observer.pending == []

            assert not any(name == "media.recorder.start_event_clip" for name, _ in owner.calls)
            finish = [parameters for name, parameters in owner.calls if name == "observe.finish"]
            assert len(finish) == 1
            assert finish[0]["handle"] == "observer-a"
            assert finish[0]["until_monotonic_ns"] == until_ns
            assert finish[0]["after_sequence"] == 1
            assert [name for name, _ in owner.calls][-2:] == ["media.recorder.close", "observe.close"]
            assert robot._sessions == set()

    asyncio.run(scenario())


@pytest.mark.parametrize("stalled_action", ["observe.events", "observe.finish", "events"])
def test_sdk_stalled_event_or_interval_reply_is_bounded_and_sessions_close(tmp_path, stalled_action):
    class StalledObservationOwner(ObservationOwner):
        def __init__(self):
            super().__init__()
            self.events_entered = asyncio.Event()
            self.events_request_task = None

        async def call(self, name, parameters):
            if name == stalled_action:
                self.calls.append((name, dict(parameters)))
                self.events_request_task = asyncio.current_task()
                self.events_entered.set()
                await asyncio.Event().wait()
            return await super().call(name, parameters)

        async def events(self, request):
            assert request["after_sequence"] == 0
            assert request["kinds"] == ["world"]
            return await self.call("events", request)

    async def scenario():
        owner = StalledObservationOwner()
        server, robot = await connected_sdk(tmp_path / "sdk.sock", owner)
        try:
            async with server:
                async with robot.observe() as observer:
                    async with robot.media.event_recorder(observer):
                        if stalled_action == "observe.finish":
                            until = asyncio.get_running_loop().time()

                            async def finish():
                                return [row async for row in observer.finish_events(until=until, timeout_s=0.05)]

                            wait = asyncio.create_task(finish())
                        else:
                            source = None if stalled_action == "events" else observer
                            assert await robot.events.wait(source, timeout_s=0, kinds=["world"]) is None
                            wait = asyncio.create_task(robot.events.wait(source, timeout_s=0.05, kinds=["world"]))
                        await asyncio.wait_for(owner.events_entered.wait(), 0.25)
                        # The caller bounds an in-flight reply even though the
                        # SDK socket's normal request timeout is 1s.
                        result = await asyncio.wait_for(wait, 0.25)
                        if stalled_action == "observe.finish":
                            assert result == [{"kind": "coverage_gap", "reason": "INTERVAL_CLOSURE_TIMEOUT"}]
                            parameters = next(parameters for name, parameters in owner.calls
                                              if name == "observe.finish")
                            assert parameters["until_monotonic_ns"] == int(until * 1e9)
                            assert 0 < parameters["timeout_s"] <= 0.05
                        else:
                            assert result is None
                        assert not owner.events_request_task.done()

                # The owner still has the original events request pending;
                # cleanup completes over its own request connections.
                assert not owner.events_request_task.done()
                assert [name for name, _ in owner.calls][-2:] == ["media.recorder.close", "observe.close"]
                assert robot._sessions == set()
        finally:
            if owner.events_request_task is not None:
                owner.events_request_task.cancel()
                await asyncio.gather(owner.events_request_task, return_exceptions=True)

    asyncio.run(scenario())


def test_sdk_zero_finish_timeout_delivers_buffered_snapshot_and_explicit_gap(tmp_path):
    async def scenario():
        owner = ObservationOwner()
        server, robot = await connected_sdk(tmp_path / "sdk.sock", owner)
        async with server:
            async with robot.observe() as observer:
                loop = asyncio.get_running_loop()
                until = loop.time() + 0.1
                until_ns = int(until * 1e9)
                first = event(1, until_ns - 20)
                second = event(2, until_ns - 10)
                owner.event_pages = [{"events": [first, second, event(3, until_ns + 1_000_000_000)],
                                      "lost_events": 0, "status": "READY", "latest_sequence": 3}]
                assert await robot.events.wait(observer, timeout_s=until - loop.time()) == first

                result = [row async for row in observer.finish_events(until=until, timeout_s=0)]
                assert result == [second, {"kind": "coverage_gap", "reason": "INTERVAL_CLOSURE_TIMEOUT"}]
                assert observer.pending == []
                assert not any(name == "observe.finish" for name, _ in owner.calls)
            assert owner.calls[-1] == ("observe.close", {"handle": "observer-a"})

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
