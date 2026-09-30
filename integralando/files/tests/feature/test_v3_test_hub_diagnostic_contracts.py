from __future__ import annotations

from dataclasses import dataclass, fields

from v3.adapters.live_encoder import EncoderEdgeDiagnostics
from v3.contracts.localization import LocalizationQuality
from v3.diagnostic_contracts import contract_from_dataclass, registered_diagnostic_contracts
from v3.test_hub_diagnostic_coverage import analyze_diagnostic_coverage_ticks


def _contract(contract_id: str):
    return next(item for item in registered_diagnostic_contracts() if item.contract_id == contract_id)


def test_production_diagnostic_contracts_follow_dataclasses_and_fingerprint() -> None:
    encoder = _contract("encoder.wheel_velocity")
    localization = _contract("localization.quality")
    assert {item.name for item in fields(EncoderEdgeDiagnostics)} <= set(encoder.field_names)
    assert {
        "left_estimation_timebase", "right_estimation_timebase",
        "left_velocity_uncertainty_mps", "right_velocity_uncertainty_mps",
        "left_quadrature_rejection_delta", "right_quadrature_rejection_delta",
        "left_pending_direction_edges", "right_pending_direction_edges",
    } <= set(encoder.field_names)
    assert set(localization.field_names) == {item.name for item in fields(LocalizationQuality)}

    @dataclass(frozen=True)
    class V1:
        value: float

    @dataclass(frozen=True)
    class V2:
        value: float
        latency_ns: int

    one = contract_from_dataclass(
        contract_id="test", target="SENSOR_SAMPLE", selector="test",
        payload_type=V1, criticality="TEST",
    )
    two = contract_from_dataclass(
        contract_id="test", target="SENSOR_SAMPLE", selector="test",
        payload_type=V2, criticality="TEST",
    )
    assert one.schema_fingerprint != two.schema_fingerprint


def test_future_fields_and_unknown_sources_are_generically_visible() -> None:
    tick = {
        "tick_id": 1, "monotonic_ns": 100,
        "inputs": {"raw_devices": {"samples": [
            {
                "__type__": "DeviceSample", "device_id": "WHEEL_ENCODERS",
                "kind": "wheel_velocity", "sequence": 1, "captured_monotonic_ns": 90,
                "values": [
                    {"__type__": "DataField", "key": "left_mps", "value": 0.1},
                    {"__type__": "DataField", "key": "future_async_latency_ns", "value": 1234},
                ],
            },
            {
                "device_id": "FUTURE_DEVICE", "kind": "future_sensor",
                "sequence": 1, "captured_monotonic_ns": 90,
                "values": [
                    {"key": "confidence", "value": 0.7},
                    {"key": "queue_age_ns", "value": 5000},
                ],
            },
        ]}},
        "expected": {"layers": {}},
    }
    result = analyze_diagnostic_coverage_ticks((tick,))
    wheel = result["sources"]["sensor:wheel_velocity"]
    assert "future_async_latency_ns" in wheel["new_unregistered_fields"]
    assert wheel["fields"]["future_async_latency_ns"]["generic_supported"] is True
    unknown = result["sources"]["sensor:future_sensor"]
    assert unknown["status"] == "GENERIC_ONLY"
    assert unknown["generic_analyzed_field_count"] == 2
