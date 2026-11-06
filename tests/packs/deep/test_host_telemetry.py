"""Host metrics are bounded passive evidence and never control inputs."""

from __future__ import annotations

import json
from types import SimpleNamespace

from v3.capture_rate import DEFAULT_CAPTURE_HZ
from v3.host_telemetry import HOST_TELEMETRY_TOPIC, HostTelemetrySampler
from v3.mcap_capture import EVENT_TOPIC, McapCaptureConfig
from v3.mcap_reader import McapReader
from v3.process_sidecars import ProcessMcapCaptureSession


def kernel_file(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value)


def test_host_sampling_uses_one_hz_no_catchup_and_explicit_missing_fields(tmp_path, monkeypatch):
    monkeypatch.setattr("v3.host_telemetry.os.cpu_count", lambda: 2)
    proc, sys = tmp_path / "proc", tmp_path / "sys"
    kernel_file(proc / "stat", "cpu 10 0 10 80 0 0 0 0\n")
    kernel_file(proc / "loadavg", "0.1 0.2 0.3 1/200 123\n")
    kernel_file(proc / "meminfo", "MemTotal: 1000 kB\nMemAvailable: 400 kB\nSwapTotal: 20 kB\nSwapFree: 10 kB\n")
    kernel_file(sys / "devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "1500000\n")
    sampler = HostTelemetrySampler(proc_root=proc, sys_root=sys)
    first = sampler.sample(1_000_000_000)
    assert first["sample_sequence"] == 1
    assert first["cpu_utilization_percent"] is None
    assert first["unavailable"]["cpu_utilization_percent"] == "FIRST_SAMPLE"
    assert first["cpu_frequency"] == [{"cpu": 0, "frequency_khz": 1_500_000}]
    assert first["cpu_frequency_missing_count"] == 1
    assert first["unavailable"]["cpu_frequency"] == "PARTIAL"
    assert first["thermal_zone0_celsius"] is None
    assert "thermal_zone0_celsius" in first["unavailable"]
    assert first["memory_bytes"]["MemAvailable"] == 400 * 1024
    kernel_file(proc / "stat", "cpu 50 0 50 100 0 0 0 0\n")
    assert sampler.sample(1_999_999_999) is None
    second = sampler.sample(2_000_000_000)
    assert second["cpu_utilization_percent"] == 80.0
    assert second["sample_sequence"] == 2
    delayed = sampler.sample(20_000_000_000)
    assert delayed["sample_sequence"] == 3
    assert sampler.sample(20_000_000_001) is None


def test_host_metric_files_and_core_payload_are_bounded(tmp_path, monkeypatch):
    monkeypatch.setattr("v3.host_telemetry.os.cpu_count", lambda: 128)
    reads = []

    def read(path, limit=65_536):
        reads.append((str(path), limit))
        if str(path).endswith("scaling_cur_freq"):
            return "1500000"
        if str(path).endswith("stat"):
            raise ValueError("kernel_file_exceeds_bound")
        raise FileNotFoundError()

    monkeypatch.setattr(HostTelemetrySampler, "_read", staticmethod(read))
    sampler = HostTelemetrySampler(proc_root=tmp_path, sys_root=tmp_path)
    row = sampler.sample(1_000_000_000)
    assert len(row["cpu_frequency"]) == 64 and row["cpu_frequency_truncated"]
    assert len(reads) == 68 and max(limit for _, limit in reads) == 65_536
    assert len(json.dumps(row)) < 8192
    assert row["cpu_utilization_percent"] is None
    assert row["unavailable"]["cpu_utilization_percent"] == "ValueError"
    reads.clear()
    assert sampler.sample(1_000_000_001) is None and not reads


def test_five_hz_full_is_user_default_and_explicit_scopes_still_work():
    from v3.interface_cli import _extract_capture_selector
    from v3.operator_controller import DEFAULT_CAPTURE_MODE
    from v3_process_runtime import _parser

    assert DEFAULT_CAPTURE_HZ == 5 and DEFAULT_CAPTURE_MODE == "full"
    assert _extract_capture_selector(["runtime", "start"])[1:3] == ("full", 5)
    assert _extract_capture_selector(["runtime", "start", "c", "alap", "c", "50"])[1:3] == ("alap", 50)
    parsed = _parser().parse_args(["--approval", "test-only"])
    assert parsed.capture_hz == 5 and parsed.capture_mode == "append_only"
    assert McapCaptureConfig(tick_sample_hz=5).sensor_debug is False
    assert McapCaptureConfig(tick_sample_hz=50).sensor_debug is True


def test_production_process_records_host_evidence_without_control_transport(tmp_path):
    from test_flight_recorder import closed_stop_record

    session = ProcessMcapCaptureSession("host", tmp_path / "host.mcap", configuration={},
        project_root=tmp_path, config=McapCaptureConfig(mode="append_only", tick_sample_hz=5))
    session.start()
    try:
        session.observe(closed_stop_record())
    finally:
        output = session.finalize(SimpleNamespace(status=0))
    reader = McapReader(output)
    assert reader.capture_integrity()["integrity"]["complete"]
    rows = [(message, row) for message, row in reader.iter_json_messages(topics=[EVENT_TOPIC])
            if row.get("source_topic") == HOST_TELEMETRY_TOPIC]
    assert rows
    for message, row in rows:
        sample = row["payload"]
        assert sample["sample_period_ns"] == 1_000_000_000
        assert sample["measurement_monotonic_ns"] <= sample["completed_monotonic_ns"]
        assert message.log_time_ns == sample["measurement_monotonic_ns"]
        assert message.publish_time_ns == row["published_monotonic_ns"] >= sample["completed_monotonic_ns"]
        assert sample["capture_process"]["pid"] == session._process.pid
    # Control only enqueued the completed behavior record; host I/O and its
    # publication happened in the already-existing observation process.
    assert session._enqueued_count == 1 and not session.transport_drop_count

    from tools.mcap_evidence.compiler import compile_evidence
    from tools.mcap_evidence.query import query
    from tools.mcap_evidence.verify import verify

    bundle = compile_evidence(output, tmp_path / "host.evidence", workers=1)["output"]
    assert verify(bundle, source=output)["messages"] > 0
    exported = list(query(bundle, topic=EVENT_TOPIC, field="payload.sample_sequence"))
    assert [item["payload"] for item in exported] == [row for _, row in rows]
