from __future__ import annotations

import inspect

import v3.process_sidecars as sidecars
from v3.adapters.process_lidar_port import _put_raw_evidence


def test_raw_lidar_evidence_transport_has_large_bounded_burst_reserve():
    assert sidecars._RAW_LIDAR_CAPACITY >= 512
    assert sidecars._RAW_LIDAR_DRAIN_BATCH >= 128
    assert sidecars._RAW_LIDAR_DRAIN_BATCH <= sidecars._RAW_LIDAR_CAPACITY


def test_raw_lidar_producer_never_blocks_control_or_sensor_owner_on_capture():
    source = inspect.getsource(_put_raw_evidence)
    assert "put_nowait" in source
    assert "target.put(" not in source
    assert "superseded_count += 1" in source


def test_process_capture_exposes_transport_capacity_for_acceptance_evidence():
    assert hasattr(sidecars.ProcessMcapCaptureSession, "raw_lidar_transport_capacity")
