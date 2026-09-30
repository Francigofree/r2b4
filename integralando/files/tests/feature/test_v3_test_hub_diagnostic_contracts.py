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
        "left_estimation_timebase",
        "right_estimation_timebase",
        "left_velocity_uncertainty_mps",
        "right_velocity_uncertainty_mps",
        "left_quadrature_rejection_delta",
        "right_quadrature_rejection_delta",
        "left_pending_direction_edges",
        "right_pending_direction_edges",
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
        contract_id="test",
        target="SENSOR_SAMPLE",
        selector="test",
        payload_type=V1,
        criticality="TEST",
    )
    two = contract_from_dataclass(
        contract_id="test",
        target="SENSOR_SAMPLE",
        selector="test",
        payload_type=V2,
        criticality="TEST",
    )
    assert one.schema_fingerprint != two.schema_fingerprint


def test_future_fields_and_unknown_sources_are_generically_visible_without_root_cause_claim() -> None:
    tick = {
        "tick_id": 1,
        "monotonic_ns": 100,
        "inputs": {
            "raw_devices": {
                "samples": [
                    {
                        "__type__": "DeviceSample",
                        "device_id": "WHEEL_ENCODERS",
                        "kind": "wheel_velocity",
                        "sequence": 1,
                        "captured_monotonic_ns": 90,
                        "values": [
                            {"__type__": "DataField", "key": "left_mps", "value": 0.1},
                            {"__type__": "DataField", "key": "future_async_latency_ns", "value": 1234},
                        ],
                    },
                    {
                        "device_id": "FUTURE_DEVICE",
                        "kind": "future_sensor",
                        "sequence": 1,
                        "captured_monotonic_ns": 90,
                        "values": [
                            {"key": "confidence", "value": 0.7},
                            {"key": "queue_age_ns", "value": 5000},
                        ],
                    },
                ]
            }
        },
        "expected": {"layers": {}},
    }
    result = analyze_diagnostic_coverage_ticks((tick,))

    wheel = result["sources"]["sensor:wheel_velocity"]
    assert "future_async_latency_ns" in wheel["new_unregistered_fields"]
    future = wheel["fields"]["future_async_latency_ns"]
    assert future["generic_supported"] is True
    assert future["domain_analyzed"] is False
    assert future["semantic_role"] == "TIMING"

    unknown = result["sources"]["sensor:future_sensor"]
    assert unknown["status"] == "GENERIC_ONLY"
    assert unknown["generic_analyzed_field_count"] == 2
    assert unknown["production_contract"] is None
    assert result["root_cause_inferred"] is False


def test_triage_uses_production_critical_device_policy_and_collapses_episode() -> None:
    from types import SimpleNamespace

    from v3.mcap_reader import EVENT_TOPIC, TICK_TOPIC
    from v3.test_hub_analysis import analyze_capture

    def tick(tick_id: int, devices: list[dict[str, object]]) -> dict[str, object]:
        ns = tick_id * 20_000_000
        return {
            "tick_id": tick_id,
            "monotonic_ns": ns,
            "record_type": "closed_input_tick",
            "inputs": {"raw_devices": {"device_health": devices}},
            "expected": {
                "fault_layer": None,
                "layers": {
                    "L2": {"rejected": []},
                    "L3": {},
                    "L5": {},
                    "L6": {},
                    "L8": {"v_mps": 0.0, "omega_rad_s": 0.0},
                    "L9": {"allowed_v_mps": 0.0, "allowed_omega_rad_s": 0.0},
                    "L12": {"safety_decision": "ALLOW", "reason": "OK"},
                },
            },
        }

    class Reader:
        def latest_metadata(self, name: str):
            assert name == "r2b4.capture"
            return {"tick_sample_hz": 50}

        def iter_json_messages(self, *, topics):
            if topics == (EVENT_TOPIC,):
                yield SimpleNamespace(log_time_ns=100_000_000), {
                    "event_type": "capture_finalized",
                    "capture_id": "test",
                    "status": "PASS",
                    "integrity": {"complete": True},
                }
                return
            assert topics == (TICK_TOPIC,)
            rows = (
                tick(1, [
                    {"device_id": "CAMERA_FRONT", "state": "UNKNOWN", "reason": "CAMERA_NO_FRAME"},
                    {"device_id": "WHEEL_ENCODERS", "state": "DEGRADED", "reason": "ENCODER_COUNTER_DIAGNOSTIC"},
                ]),
                tick(2, [
                    {"device_id": "CAMERA_FRONT", "state": "UNKNOWN", "reason": "CAMERA_NO_FRAME"},
                    {"device_id": "WHEEL_ENCODERS", "state": "DEGRADED", "reason": "ENCODER_COUNTER_DIAGNOSTIC"},
                ]),
                tick(3, []),
            )
            for row in rows:
                yield SimpleNamespace(log_time_ns=row["monotonic_ns"]), row

    result = analyze_capture(Reader())
    sensors = result["sensors"]
    assert sensors["device_non_ok_count"] == 4
    assert sensors["critical_device_non_ok_count"] == 2
    assert sensors["optional_device_non_ok_count"] == 2
    device_incidents = [item for item in result["incidents"] if item["category"] == "DEVICE_HEALTH"]
    assert len(device_incidents) == 1
    assert device_incidents[0]["evidence"]["devices"] == [
        {"device_id": "WHEEL_ENCODERS", "state": "DEGRADED", "reason": "ENCODER_COUNTER_DIAGNOSTIC"}
    ]
