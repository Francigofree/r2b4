from __future__ import annotations

from pathlib import Path
import queue

from v3.adapters.process_lidar_port import _put_raw_end
from v3.layers.l11_actuator_control import WheelPiConfig
from v3.test_hub_facts import analyze_ticks
from v3.wheel_motion import velocity_quality


FORBIDDEN_EVIDENCE_KEYS = {
    "root_cause", "root_cause_candidate", "physical_root_cause",
    "priority_evidence", "severity", "diagnosis_status", "finding_count",
}


def _walk_keys(value: object) -> set[str]:
    result: set[str] = set()
    if isinstance(value, dict):
        for key, child in value.items():
            result.add(str(key))
            result |= _walk_keys(child)
    elif isinstance(value, (list, tuple)):
        for child in value:
            result |= _walk_keys(child)
    return result


def test_fact_compiler_emits_measurements_and_production_states_only() -> None:
    tick = {
        "tick_id": 1,
        "monotonic_ns": 20_000_000,
        "inputs": {
            "raw_devices": {
                "device_health": [
                    {"device_id": "WHEEL_ENCODERS", "state": "OK", "reason": None},
                ],
                "samples": [{
                    "__type__": "DeviceSample",
                    "device_id": "WHEEL_ENCODERS",
                    "kind": "wheel_velocity",
                    "sequence": 1,
                    "captured_monotonic_ns": 19_000_000,
                    "values": [
                        {"__type__": "DataField", "key": "left_mps", "value": 0.14},
                        {"__type__": "DataField", "key": "right_mps", "value": 0.15},
                        {"__type__": "DataField", "key": "left_estimation_window_ns", "value": 42_000_000},
                        {"__type__": "DataField", "key": "left_quadrature_rejection_delta", "value": 1},
                    ],
                }],
            },
        },
        "expected": {
            "fault_layer": None,
            "layers": {
                "L2": {"rejected": []},
                "L3": {
                    "x_m": 0.0, "y_m": 0.0, "v_mps": 0.145, "omega_rad_s": 0.0,
                    "localization_quality": {
                        "local_translation": "GOOD", "heading": "GOOD",
                        "global_position": "DEGRADED", "local_pose_continuous": True,
                        "pose_discontinuity": False, "slip_suspected": False,
                        "local_sigma_m": 0.01, "global_sigma_m": 0.2,
                        "yaw_sigma_rad": 0.01, "observability": 0.5,
                    },
                },
                "L8": {"requested_v_mps": 0.15, "requested_omega_rad_s": 0.0},
                "L9": {"allowed_v_mps": 0.15, "allowed_omega_rad_s": 0.0},
                "L10": {"left_mps": 0.15, "right_mps": 0.15},
                "L11": {"left_normalized": 0.2, "right_normalized": 0.2},
                "L12": {"safety_decision": "ALLOW", "reason": "OK"},
            },
        },
    }
    result = analyze_ticks((tick,))
    assert result["encoder"]["fields"]["left_estimation_window_ns"]["numeric"]["count"] == 1
    assert result["encoder"]["fields"]["left_quadrature_rejection_delta"]["numeric"]["max"] == 1
    assert result["motion"]["signals"]["requested_v_mps"]["mean"] == 0.15
    assert result["localization"]["state_counts"]["global_position"]["DEGRADED"] == 1
    assert not (FORBIDDEN_EVIDENCE_KEYS & _walk_keys(result))


def test_l11_encoder_transition_contract_is_current_source() -> None:
    fields = WheelPiConfig.__dataclass_fields__
    assert "minimum_reliable_speed_mps" in fields
    assert "velocity_unreliable_below_mps" in fields
    assert velocity_quality(0.15, 0.15, 0.13) == 1.0
    assert 0.0 < velocity_quality(0.14, 0.15, 0.13) < 1.0


def test_raw_lidar_terminal_marker_is_reliable_or_explicit_failure() -> None:
    target: queue.Queue[object] = queue.Queue(maxsize=1)
    target.put(("occupied",))
    try:
        _put_raw_end(
            target, last_revision=10, produced_count=10,
            superseded_count=0, timeout_s=0.001,
        )
    except RuntimeError:
        pass
    else:
        raise AssertionError("terminal marker saturation must not fail silently")


def test_legacy_testhub_modules_and_robot_adapter_are_removed() -> None:
    root = Path(__file__).resolve().parents[2]
    keep = {
        "test_hub_runtime.py", "test_hub_diagnostic_coverage.py",
        "test_hub_compiler.py", "test_hub_facts.py",
    }
    leftovers = [
        path.name for path in (root / "v3").glob("test_hub_*.py")
        if path.name not in keep
    ]
    assert leftovers == []
    assert not (root / "v3/adapters/testhub.py").exists()
    interface = (root / "v3/interface_adapters.py").read_text(encoding="utf-8")
    assert "TestHubInterfaceAdapter" not in interface
    assert "adapters.testhub" not in interface
