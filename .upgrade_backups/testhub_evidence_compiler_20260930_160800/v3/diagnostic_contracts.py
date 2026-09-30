"""Shared diagnostic schema contracts derived from production V3 types.

This module is deliberately read-only.  It does not own runtime, safety, motor,
mission or capture authority.  Test Hub uses these contracts to discover schema
changes automatically instead of copying production field lists by hand.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, fields, is_dataclass
from typing import Mapping, Sequence


_GENERIC_SCHEMA_VERSION = 1


@dataclass(frozen=True, slots=True)
class DiagnosticFieldContract:
    name: str
    semantic_role: str
    unit: str | None
    generic_checks: tuple[str, ...]
    origin: str

    def as_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "semantic_role": self.semantic_role,
            "unit": self.unit,
            "generic_checks": list(self.generic_checks),
            "origin": self.origin,
        }


@dataclass(frozen=True, slots=True)
class DiagnosticContract:
    """One production-observability contract consumed by offline diagnostics."""

    contract_id: str
    target: str
    selector: str
    criticality: str
    fields: tuple[DiagnosticFieldContract, ...]
    domain_analyzed_fields: tuple[str, ...] = ()

    @property
    def field_names(self) -> tuple[str, ...]:
        return tuple(item.name for item in self.fields)

    @property
    def schema_fingerprint(self) -> str:
        payload = {
            "version": _GENERIC_SCHEMA_VERSION,
            "contract_id": self.contract_id,
            "target": self.target,
            "selector": self.selector,
            "criticality": self.criticality,
            "fields": [item.as_dict() for item in self.fields],
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def as_dict(self) -> dict[str, object]:
        return {
            "contract_id": self.contract_id,
            "target": self.target,
            "selector": self.selector,
            "criticality": self.criticality,
            "schema_fingerprint": self.schema_fingerprint,
            "field_count": len(self.fields),
            "fields": [item.as_dict() for item in self.fields],
            "domain_analyzed_fields": list(self.domain_analyzed_fields),
        }


def infer_field_contract(name: str, *, origin: str = "OBSERVED_GENERIC") -> DiagnosticFieldContract:
    """Infer bounded generic semantics from a scalar field name.

    The result is descriptive metadata only.  It never creates a fault or root
    cause verdict.  Unknown future fields therefore remain visible immediately.
    """

    low = name.lower()
    role = "GENERIC"
    unit: str | None = None
    checks: tuple[str, ...] = ("TYPE_STABILITY", "PRESENCE_RATE")

    if low.endswith("_ns") or "timestamp_ns" in low:
        role = "TIMING"
        unit = "ns"
        checks = checks + ("NONNEGATIVE", "CHANGE_RATE")
    elif low.endswith("_ms"):
        role = "TIMING"
        unit = "ms"
        checks = checks + ("NONNEGATIVE", "CHANGE_RATE")
    elif low.endswith("_mps"):
        role = "MEASUREMENT"
        unit = "m/s"
        checks = checks + ("FINITE_RANGE", "CHANGE_RATE")
    elif low.endswith("_rad_s"):
        role = "MEASUREMENT"
        unit = "rad/s"
        checks = checks + ("FINITE_RANGE", "CHANGE_RATE")
    elif low.endswith("_rad"):
        role = "MEASUREMENT"
        unit = "rad"
        checks = checks + ("FINITE_RANGE", "CHANGE_RATE")
    elif low.endswith("_m"):
        role = "MEASUREMENT"
        unit = "m"
        checks = checks + ("FINITE_RANGE", "CHANGE_RATE")
    elif any(token in low for token in ("trust", "confidence", "observability", "coverage", "ratio", "margin")):
        role = "QUALITY"
        unit = "ratio"
        checks = checks + ("FINITE_RANGE", "UNIT_INTERVAL_CANDIDATE")
    elif any(token in low for token in ("count", "errors", "rejections", "revision", "generation", "candidate_id", "sequence")):
        role = "COUNTER"
        checks = checks + ("NONNEGATIVE", "MONOTONICITY_OBSERVATION")
    elif any(token in low for token in ("stale", "valid", "ready", "timed_out", "degenerate", "continuous", "discontinuity", "suspected", "usable", "running")):
        role = "STATE"
        checks = checks + ("STATE_DISTRIBUTION", "TRANSITION_COUNT")
    elif any(token in low for token in ("reason", "code", "timebase", "health", "state", "frame_id", "direction")):
        role = "STATE"
        checks = checks + ("STATE_DISTRIBUTION", "TRANSITION_COUNT")

    return DiagnosticFieldContract(name, role, unit, checks, origin)


def contract_from_dataclass(
    *,
    contract_id: str,
    target: str,
    selector: str,
    payload_type: type[object],
    criticality: str,
    exclude_fields: Sequence[str] = (),
    aliases: Mapping[str, str] | None = None,
    prepend_fields: Sequence[str] = (),
    append_fields: Sequence[str] = (),
    domain_analyzed_fields: Sequence[str] = (),
) -> DiagnosticContract:
    """Build a contract from the live production dataclass definition.

    New dataclass fields are incorporated automatically unless explicitly
    excluded.  This is the core anti-drift mechanism.
    """

    if not is_dataclass(payload_type):
        raise TypeError("payload_type must be a dataclass type")
    alias_map = dict(aliases or {})
    excluded = set(exclude_fields)
    names: list[tuple[str, str]] = []
    for name in prepend_fields:
        names.append((name, "ADAPTER_DERIVED"))
    for item in fields(payload_type):
        if item.name in excluded:
            continue
        names.append((alias_map.get(item.name, item.name), f"{payload_type.__name__}.{item.name}"))
    for name in append_fields:
        names.append((name, "ADAPTER_DERIVED"))

    unique: list[DiagnosticFieldContract] = []
    seen: set[str] = set()
    for name, origin in names:
        if name in seen:
            continue
        seen.add(name)
        unique.append(infer_field_contract(name, origin=origin))

    domain = tuple(name for name in domain_analyzed_fields if name in seen)
    return DiagnosticContract(
        contract_id=contract_id,
        target=target,
        selector=selector,
        criticality=criticality,
        fields=tuple(unique),
        domain_analyzed_fields=domain,
    )


def registered_diagnostic_contracts() -> tuple[DiagnosticContract, ...]:
    """Return contracts derived from the current production source types."""

    from .adapters.live_encoder import EncoderEdgeDiagnostics
    from .adapters.live_imu import ImuHeadingReading
    from .adapters.live_lidar import (
        LidarMatcherDiagnostics,
        LidarPoseReading,
        LidarRelativeMotionReading,
    )
    from .contracts.localization import LocalizationQuality

    encoder_domain = (
        "left_mps",
        "right_mps",
        "left_distance_delta_m",
        "right_distance_delta_m",
        "raw_left_distance_m",
        "raw_right_distance_m",
        "left_measurement_trust",
        "right_measurement_trust",
        "left_velocity_uncertainty_mps",
        "right_velocity_uncertainty_mps",
        "rejection_code",
        "left_read_error_delta",
        "right_read_error_delta",
        "left_invalid_alert_delta",
        "right_invalid_alert_delta",
        "left_quadrature_rejection_delta",
        "right_quadrature_rejection_delta",
    )
    imu_domain = (
        "yaw_rad",
        "omega_rad_s",
        "confidence",
        "calibration",
        "omega_confidence",
        "omega_calibration",
    )
    matcher_domain = (
        "tracking_ready",
        "matcher_timed_out",
        "matcher_degenerate",
        "matcher_reason",
        "matcher_runtime_ms",
        "matcher_queue_delay_ms",
        "matcher_input_age_ns",
        "matcher_confidence",
        "inlier_ratio",
        "robust_rmse_m",
        "sector_coverage",
        "observability_score",
        "ambiguity_margin",
        "degeneracy_reasons",
    )

    return (
        contract_from_dataclass(
            contract_id="encoder.wheel_velocity",
            target="SENSOR_SAMPLE",
            selector="wheel_velocity",
            payload_type=EncoderEdgeDiagnostics,
            criticality="PRODUCTION_CRITICAL",
            prepend_fields=(
                "left_mps",
                "right_mps",
                "trust",
                "measurement_stale",
                "measurement_timing_valid",
                "left_velocity_quality",
                "right_velocity_quality",
            ),
            domain_analyzed_fields=encoder_domain,
        ),
        contract_from_dataclass(
            contract_id="imu.ekf_heading",
            target="SENSOR_SAMPLE",
            selector="ekf_heading",
            payload_type=ImuHeadingReading,
            criticality="PRODUCTION_CRITICAL",
            exclude_fields=("sequence", "captured_monotonic_ns"),
            aliases={"stale": "measurement_stale", "timing_valid": "measurement_timing_valid"},
            domain_analyzed_fields=imu_domain,
        ),
        contract_from_dataclass(
            contract_id="lidar.matcher_diagnostics",
            target="SENSOR_SAMPLE",
            selector="lidar_matcher_diagnostics",
            payload_type=LidarMatcherDiagnostics,
            criticality="PRODUCTION_CRITICAL",
            exclude_fields=("scan_to_map_seed",),
            append_fields=(
                "source_scan_revision",
                "seed_pose_x_m",
                "seed_pose_y_m",
                "seed_pose_yaw_rad",
            ),
            domain_analyzed_fields=matcher_domain,
        ),
        contract_from_dataclass(
            contract_id="lidar.relative_motion",
            target="SENSOR_SAMPLE",
            selector="lidar_relative_motion",
            payload_type=LidarRelativeMotionReading,
            criticality="PRODUCTION_CRITICAL",
        ),
        contract_from_dataclass(
            contract_id="lidar.pose",
            target="SENSOR_SAMPLE",
            selector="lidar_pose",
            payload_type=LidarPoseReading,
            criticality="PRODUCTION_CRITICAL",
            prepend_fields=("frame_id",),
            append_fields=("confidence",),
            domain_analyzed_fields=("x_m", "y_m", "yaw_rad", "confidence", "r_scale"),
        ),
        contract_from_dataclass(
            contract_id="localization.quality",
            target="LAYER_MAPPING",
            selector="L3.localization_quality",
            payload_type=LocalizationQuality,
            criticality="CONTROL_CRITICAL",
        ),
    )


__all__ = [
    "DiagnosticContract",
    "DiagnosticFieldContract",
    "contract_from_dataclass",
    "infer_field_contract",
    "registered_diagnostic_contracts",
]
