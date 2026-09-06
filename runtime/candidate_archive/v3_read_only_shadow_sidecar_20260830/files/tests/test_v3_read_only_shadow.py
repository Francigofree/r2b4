from dataclasses import FrozenInstanceError
import inspect
import os
from pathlib import Path

import pytest

from v3.adapters.capture_edges import ReplayerV1InputAdapter
from v3.composition import (
    InputShadowComposition,
    ReadOnlyShadowSidecar,
    ShadowSidecarError,
)
from v3.contracts import SafetyDecision


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CAPTURE_EXCERPT = PROJECT_ROOT / "tests" / "fixtures" / "v3_l0_l4_capture_excerpt.jsonl"


def _batches():
    return ReplayerV1InputAdapter().load(CAPTURE_EXCERPT)


def test_spawned_sidecar_matches_local_shadow_without_writer_capability():
    batches = _batches()
    expected = InputShadowComposition().replay(batches)

    with ReadOnlyShadowSidecar() as sidecar:
        worker_pid = sidecar.worker_pid
        actual = sidecar.run_replay(batches)
        assert sidecar.start_method == "spawn"
        assert sidecar.is_alive
        assert not hasattr(sidecar, "write")

    assert worker_pid != os.getpid()
    assert not sidecar.is_alive
    assert tuple(item.trace for item in actual) == expected
    assert all(item.final_actuation == item.trace.layers[-1].output for item in actual)
    assert all(
        item.final_actuation.safety_decision is SafetyDecision.STOP
        and not item.final_actuation.enabled
        and item.final_actuation.left_output == 0.0
        and item.final_actuation.right_output == 0.0
        for item in actual
    )


def test_fresh_spawned_sidecars_are_deterministic_and_results_are_immutable():
    batches = _batches()[:3]

    with ReadOnlyShadowSidecar() as first_sidecar:
        first = first_sidecar.run_replay(batches)
    with ReadOnlyShadowSidecar() as second_sidecar:
        second = second_sidecar.run_replay(batches)

    assert first == second
    with pytest.raises(FrozenInstanceError):
        first[0].trace = second[0].trace


def test_noncontiguous_shadow_input_fails_closed_inside_worker():
    batches = _batches()

    with ReadOnlyShadowSidecar() as sidecar:
        first, skipped = sidecar.run_replay((batches[0], batches[2]))

    assert first.trace.fault_layer is None
    assert skipped.trace.fault_layer == "TickEngine"
    assert skipped.final_actuation.safety_decision is SafetyDecision.FAULT
    assert skipped.final_actuation.reason == "INVALID_TICK_ORDER"
    assert skipped.final_actuation.left_output == 0.0
    assert skipped.final_actuation.right_output == 0.0


def test_sidecar_api_accepts_no_reader_writer_or_legacy_runtime_capability():
    parameters = inspect.signature(ReadOnlyShadowSidecar).parameters

    assert tuple(parameters) == ("config", "response_timeout_s")
    assert not {"reader", "writer", "hal", "runtime", "shared_state"} & set(parameters)

    sidecar = ReadOnlyShadowSidecar()
    try:
        with pytest.raises(TypeError, match="RawDeviceBatch"):
            sidecar.run_tick(object())
        assert sidecar.is_alive
    finally:
        sidecar.close()
        sidecar.close()

    with pytest.raises(ShadowSidecarError, match="closed"):
        sidecar.run_tick(_batches()[0])
