"""Real mailbox publication races; no resident runtime or hardware is started."""

from __future__ import annotations

import json
import multiprocessing
import os
import threading
import time
from pathlib import Path

import pytest

from v3.adapters.resident_command import (
    AtomicResidentCommandGateway,
    CommandPublicationRejected,
    ResidentCommandClient,
    ResidentCommandMailboxConfig,
)
from v3.contracts import CommandMode, TickContext


TTL_NS = 200_000_000


def _explore(client: ResidentCommandClient, command_id: str = "old-active") -> int:
    return client.publish_explore(
        command_id, max_v_mps=0.3, max_omega_rad_s=0.6, ttl_ns=TTL_NS,
    )


def _in_thread(call, result, done):
    try:
        result["value"] = call()
    except BaseException as exc:
        result["error"] = exc
    finally:
        done.set()


def _stop_process(path, barrier, results, index):
    try:
        client = ResidentCommandClient(ResidentCommandMailboxConfig(Path(path)))
        barrier.wait(timeout=10.0)
        revisions = [client.publish_stop(f"stop-{index}-{n}", ttl_ns=TTL_NS)
                     for n in range(3)]
        results.put(("ok", revisions))
    except BaseException as exc:
        results.put(("error", repr(exc)))


def _hold_stop_at_replace(path, entered, release):
    """Pause the actual commit syscall, while the writer owns its commit lock."""
    original_replace = os.replace

    def blocked_replace(source, target, *args, **kwargs):
        if Path(target) == Path(path):
            entered.set()
            if not release.wait(timeout=10.0):
                raise TimeoutError("test did not release the held publication")
        return original_replace(source, target, *args, **kwargs)

    os.replace = blocked_replace
    try:
        client = ResidentCommandClient(ResidentCommandMailboxConfig(Path(path)))
        client.publish_stop("held-stop", ttl_ns=TTL_NS)
    finally:
        os.replace = original_replace


def _start_held_stop(config):
    context = multiprocessing.get_context("spawn")
    entered, release = context.Event(), context.Event()
    process = context.Process(
        target=_hold_stop_at_replace, args=(str(config.path), entered, release),
    )
    process.start()
    if not entered.wait(timeout=10.0):
        release.set()
        process.terminate()
        process.join(timeout=3.0)
        pytest.fail("publisher never reached the commit syscall")
    return process, release


def _finish_process(process, release):
    release.set()
    process.join(timeout=3.0)
    if process.is_alive():
        process.terminate()
        process.join(timeout=3.0)
        pytest.fail("publication process did not exit")


@pytest.mark.parametrize("initial_publication", [False, True])
def test_stop_commits_while_active_staging_is_delayed(tmp_path, monkeypatch, initial_publication):
    config = ResidentCommandMailboxConfig(tmp_path / "command.json")
    old = ResidentCommandClient(config)
    if not initial_publication:
        _explore(old)

    entered, release = threading.Event(), threading.Event()
    active_done, stop_done = threading.Event(), threading.Event()
    active_result, stop_result = {}, {}
    original_write = os.write

    def delayed_write(descriptor, payload):
        if threading.current_thread() is active_thread:
            entered.set()
            if not release.wait(timeout=5.0):
                raise TimeoutError("test did not release ACTIVE staging")
        return original_write(descriptor, payload)

    active_thread = threading.Thread(
        target=_in_thread, args=(lambda: _explore(old), active_result, active_done),
    )
    monkeypatch.setattr(os, "write", delayed_write)
    active_thread.start()
    stop_thread = None
    try:
        assert entered.wait(timeout=2.0), "ACTIVE publication never entered staging"
        stop = ResidentCommandClient(config)
        stop_thread = threading.Thread(
            target=_in_thread,
            args=(lambda: stop.publish_stop("explicit-stop", ttl_ns=TTL_NS), stop_result, stop_done),
        )
        stop_thread.start()
        assert stop_done.wait(timeout=1.0), "STOP waited for unrelated ACTIVE staging I/O"
        assert "error" not in stop_result, stop_result.get("error")
        stopped_bytes = config.path.read_bytes()
        assert json.loads(stopped_bytes)["mode"] == "STOP"
        assert not active_done.is_set()
    finally:
        release.set()
        active_thread.join(timeout=3.0)
        if stop_thread is not None:
            stop_thread.join(timeout=3.0)

    assert not active_thread.is_alive()
    assert active_done.is_set()
    error = active_result.get("error")
    assert isinstance(error, CommandPublicationRejected)
    assert error.reason == "COMMAND_SUPERSEDED"
    assert config.path.read_bytes() == stopped_bytes
    assert not tuple(tmp_path.glob(".command.json.tmp.*"))
    command = AtomicResidentCommandGateway(config).snapshot(TickContext(0, time.monotonic_ns()))
    assert command.mode is CommandMode.STOP


def test_stop_invalidates_an_admitted_producer_before_its_first_publication(tmp_path):
    config = ResidentCommandMailboxConfig(tmp_path / "command.json")
    admitted = ResidentCommandClient(config)
    stop = ResidentCommandClient(config)
    stop_revision = stop.publish_stop("explicit-stop", ttl_ns=TTL_NS)
    stopped_bytes = config.path.read_bytes()

    with pytest.raises(CommandPublicationRejected, match="COMMAND_SUPERSEDED"):
        _explore(admitted)

    assert stop_revision == 1
    assert config.path.read_bytes() == stopped_bytes


def test_old_renewal_and_conditional_stop_cannot_replace_a_fresh_command(tmp_path):
    config = ResidentCommandMailboxConfig(tmp_path / "command.json")
    old = ResidentCommandClient(config)
    old_revision = _explore(old)
    stop_revision = ResidentCommandClient(config).publish_stop("explicit-stop", ttl_ns=TTL_NS)
    fresh = ResidentCommandClient(config)
    fresh_revision = _explore(fresh, "fresh-active")
    fresh_bytes = config.path.read_bytes()

    with pytest.raises(CommandPublicationRejected, match="COMMAND_SUPERSEDED"):
        _explore(old)
    assert old.publish_stop("old-cleanup-stop", ttl_ns=TTL_NS,
                            only_if_revision=old_revision) is None

    assert old_revision < stop_revision < fresh_revision
    assert config.path.read_bytes() == fresh_bytes
    # Rejection of the old owner must not prevent the fresh producer renewing.
    assert _explore(fresh, "fresh-active") == fresh_revision + 1


def test_conditional_stop_accepts_the_unchanged_empty_mailbox(tmp_path):
    config = ResidentCommandMailboxConfig(tmp_path / "command.json")
    client = ResidentCommandClient(config)
    assert client.publish_stop("initial-failure-stop", ttl_ns=TTL_NS, only_if_revision=0) == 1
    assert json.loads(config.path.read_bytes())["mode"] == "STOP"


def test_concurrent_process_stop_publications_allocate_unique_increasing_revisions(tmp_path):
    config = ResidentCommandMailboxConfig(tmp_path / "command.json")
    context = multiprocessing.get_context("spawn")
    barrier, results = context.Barrier(3), context.Queue()
    processes = [context.Process(
        target=_stop_process, args=(str(config.path), barrier, results, index),
    ) for index in range(3)]
    try:
        for process in processes:
            process.start()
        outcomes = [results.get(timeout=15.0) for _ in processes]
        for process in processes:
            process.join(timeout=3.0)
        assert all(process.exitcode == 0 for process in processes)
        assert all(kind == "ok" for kind, _ in outcomes), outcomes
        assert sorted(revision for _, revisions in outcomes for revision in revisions) == list(range(1, 10))
        payload = json.loads(config.path.read_bytes())
        assert payload["revision"] == 9
        assert payload["mode"] == "STOP"
        assert AtomicResidentCommandGateway(config).snapshot(
            TickContext(0, time.monotonic_ns()),
        ).mode is CommandMode.STOP
    finally:
        for process in processes:
            if process.is_alive():
                process.terminate()
            process.join(timeout=3.0)
        results.close()
        results.join_thread()


def test_stalled_commit_has_bounded_wait_and_does_not_replace_mailbox(tmp_path):
    config = ResidentCommandMailboxConfig(tmp_path / "command.json")
    ResidentCommandClient(config).publish_stop("previous-stop", ttl_ns=TTL_NS)
    before = config.path.read_bytes()
    process, release = _start_held_stop(config)
    result, done = {}, threading.Event()
    contender = threading.Thread(target=_in_thread, args=(
        lambda: ResidentCommandClient(config).publish_stop("contending-stop", ttl_ns=TTL_NS), result, done,
    ))
    try:
        contender.start()
        assert done.wait(timeout=1.5), "publication lock acquisition was unbounded"
        assert isinstance(result.get("error"), TimeoutError), result
        assert config.path.read_bytes() == before
    finally:
        _finish_process(process, release)
        contender.join(timeout=3.0)
    assert process.exitcode == 0
    assert not contender.is_alive()
    assert not tuple(tmp_path.glob(".command.json.tmp.*"))


def test_process_death_releases_the_publication_lock(tmp_path):
    config = ResidentCommandMailboxConfig(tmp_path / "command.json")
    ResidentCommandClient(config).publish_stop("previous-stop", ttl_ns=TTL_NS)
    process, release = _start_held_stop(config)
    try:
        process.terminate()
        process.join(timeout=3.0)
        assert not process.is_alive()
        assert process.exitcode != 0
        assert ResidentCommandClient(config).publish_stop("after-death-stop", ttl_ns=TTL_NS) == 2
        payload = json.loads(config.path.read_bytes())
        assert payload["command_id"] == "after-death-stop"
        assert payload["mode"] == "STOP"
    finally:
        _finish_process(process, release)
