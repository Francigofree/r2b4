from __future__ import annotations

import sys
import time
import types
from dataclasses import dataclass
from pathlib import Path


def _install_r2b4_stubs() -> None:
    capture_encoding = types.ModuleType("v3.capture_encoding")

    class CaptureEncodingError(RuntimeError):
        pass

    def encode_value(value):
        if isinstance(value, dict):
            return {str(k): encode_value(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [encode_value(v) for v in value]
        if value is None or isinstance(value, (str, int, float, bool)):
            return value
        if hasattr(value, "__dict__"):
            return {"__type__": type(value).__name__, **{k: encode_value(v) for k, v in value.__dict__.items()}}
        return str(value)

    def encode_capture_record(record):
        return {
            "record_type": "closed_input_tick",
            "tick_id": record.inputs.context.tick_id,
            "monotonic_ns": record.inputs.context.monotonic_ns,
            "inputs": {"raw_devices": {"samples": []}},
            "expected": {"fault_layer": None, "layers": {}},
        }

    def raw_encoder(_value, _limit):
        raise AssertionError("raw lidar not used in this smoke test")

    capture_encoding.CaptureEncodingError = CaptureEncodingError
    capture_encoding.encode_value = encode_value
    capture_encoding.encode_capture_record = encode_capture_record
    capture_encoding.encode_raw_lidar_snapshot = raw_encoder
    sys.modules["v3.capture_encoding"] = capture_encoding

    execution = types.ModuleType("v3.execution")

    @dataclass
    class Context:
        tick_id: int
        monotonic_ns: int

    @dataclass
    class Inputs:
        context: Context

    class Decision:
        value = "ALLOW"

    class Final:
        safety_decision = Decision()
        reason = ""

    @dataclass
    class Trace:
        fault_layer: str | None = None

    @dataclass
    class Result:
        trace: Trace
        final_actuation: Final

    class ExecutionRecord:
        def __init__(self, tick_id: int, monotonic_ns: int):
            self.inputs = Inputs(Context(tick_id, monotonic_ns))
            self.result = Result(Trace(), Final())
            self.state_checkpoint_after = None

    class EdgeFaultRecord:
        pass

    class WriterFailureRecord:
        pass

    execution.ExecutionRecord = ExecutionRecord
    execution.EdgeFaultRecord = EdgeFaultRecord
    execution.WriterFailureRecord = WriterFailureRecord
    execution.CaptureRecord = object
    execution.REPLAY_STATE_CHECKPOINT_INTERVAL_NS = 1_000_000_000
    sys.modules["v3.execution"] = execution


def test_consumer_streams_triggered_mcap_atomically(tmp_path: Path) -> None:
    _install_r2b4_stubs()
    sys.modules.pop("v3.mcap_capture", None)
    from v3.execution import ExecutionRecord
    from v3.mcap_capture import CaptureState, McapCaptureConfig, McapCaptureConsumer

    target = tmp_path / "capture.mcap"
    consumer = McapCaptureConsumer(
        "smoke",
        target,
        configuration={"test": True},
        config=McapCaptureConfig(
            pre_event_ns=5,
            post_event_ns=2,
            checkpoint_context_ns=0,
            chunk_target_bytes=128,
        ),
    )
    for tick in range(9):
        consumer.observe(
            sequence=tick,
            topic="tick",
            monotonic_ns=tick,
            payload=ExecutionRecord(tick, tick),
        )
    consumer.trigger("MANUAL", monotonic_ns=8)
    for tick in range(9, 12):
        consumer.observe(
            sequence=tick,
            topic="tick",
            monotonic_ns=tick,
            payload=ExecutionRecord(tick, tick),
        )

    result = consumer.finish("PASS")
    assert result is not None
    assert result.complete is True
    assert result.state is CaptureState.FINISHED
    assert result.tick_sequence_gap_count == 0
    assert target.is_file()
    assert target.read_bytes().startswith(b"\x89MCAP0\r\n")
    assert target.read_bytes().endswith(b"\x89MCAP0\r\n")
    assert not list(tmp_path.glob("*.partial*"))


def test_hub_gap_is_never_silently_passed(tmp_path: Path) -> None:
    _install_r2b4_stubs()
    sys.modules.pop("v3.mcap_capture", None)
    from v3.execution import ExecutionRecord
    from v3.mcap_capture import CaptureState, McapCaptureConfig, McapCaptureConsumer

    target = tmp_path / "gap.mcap"
    consumer = McapCaptureConsumer(
        "gap",
        target,
        configuration={},
        config=McapCaptureConfig(pre_event_ns=10, post_event_ns=0, checkpoint_context_ns=0),
    )
    consumer.observe(sequence=0, topic="tick", monotonic_ns=0, payload=ExecutionRecord(0, 0))
    consumer.observe(sequence=2, topic="tick", monotonic_ns=2, payload=ExecutionRecord(2, 2))
    consumer.trigger("MANUAL", monotonic_ns=2)
    result = consumer.finish("PASS")

    assert result is not None
    assert result.complete is False
    assert result.status == "FAIL"
    assert result.state is CaptureState.INTEGRITY_FAILED
    assert result.hub_sequence_gap_count == 1
    assert result.tick_sequence_gap_count == 1
