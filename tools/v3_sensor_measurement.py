#!/usr/bin/env python3
"""Finite, zero-output concrete V3 sensor and state-estimation measurement."""

from __future__ import annotations

import argparse
import json
import signal
import sys
from collections.abc import Mapping
from pathlib import Path

import lgpio
import serial
import smbus2

from v3.adapters.native_lidar_port import (
    NativeLidarPort,
    load_native_lidar_port_config,
    open_native_lidar_port,
)
from v3.adapters.bounded_command import BoundedTeleopProfile
from v3.contracts import AcquisitionFrame, AdmittedFrame, RobotEstimate
from v3_bounded_config import (
    NativeSensorPolicyConfig,
    load_bounded_physical_runtime_config,
)
from v3_hardware_runtime import (
    FiniteSensorMeasurementConfig,
    SensorMeasurementReport,
    run_finite_sensor_measurement,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class _SignalStop:
    def __init__(self) -> None:
        self.requested = False

    def handle(self, _signum, _frame) -> None:
        self.requested = True

    def __call__(self) -> bool:
        return self.requested


def native_sensor_policy() -> NativeSensorPolicyConfig:
    """Return the explicit first-hardware validation policy."""

    return NativeSensorPolicyConfig(
        encoder_maximum_sample_interval_ns=100_000_000,
        encoder_maximum_abs_velocity_mps=1.5,
        encoder_minimum_trust=0.5,
        imu_maximum_sample_age_ns=100_000_000,
        imu_heading_clockwise_positive=True,
        imu_yaw_rate_axis=2,
        imu_yaw_rate_clockwise_positive=False,
        imu_yaw_offset_rad=0.0,
        imu_minimum_confidence=0.5,
        imu_minimum_calibration=2,
        imu_allow_rate_only=True,
        lidar_maximum_result_age_ns=250_000_000,
        lidar_maximum_future_skew_ns=10_000_000,
        lidar_pose_r_scale=1.0,
        lidar_minimum_confidence=0.2,
        lidar_maximum_measurement_age_ns=250_000_000,
    )


def _layer(result, layer: str):
    for record in result.trace.layers:
        if record.layer == layer:
            return record.output
    return None


def _sample_values(sample) -> dict[str, object]:
    return {field.key: field.value for field in sample.values}


def _json_value(value):
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    return value


def summarize_report(report: SensorMeasurementReport) -> dict[str, object]:
    """Serialize only measurement diagnostics; no value feeds control back."""

    source_state_counts: dict[str, dict[str, int]] = {}
    sample_ranges: dict[str, dict[str, list[float]]] = {}
    rejected_counts: dict[str, int] = {}
    estimates = []
    tick_summaries = []
    for result in report.ticks:
        acquisition = _layer(result, "L1")
        admitted = _layer(result, "L2")
        estimate = _layer(result, "L3")
        health_summary = []
        if isinstance(acquisition, AcquisitionFrame):
            for health in acquisition.io_health:
                states = source_state_counts.setdefault(health.device_id, {})
                states[health.state.value] = states.get(health.state.value, 0) + 1
                health_summary.append(
                    {
                        "device_id": health.device_id,
                        "state": health.state.value,
                        "reason": health.reason,
                    }
                )
            for sample in acquisition.samples:
                numeric = {
                    key: float(value)
                    for key, value in _sample_values(sample).items()
                    if isinstance(value, (int, float)) and not isinstance(value, bool)
                }
                numeric["captured_minus_tick_ns"] = float(
                    sample.captured_monotonic_ns - acquisition.context.monotonic_ns
                )
                ranges = sample_ranges.setdefault(sample.kind, {})
                for key, value in numeric.items():
                    bounds = ranges.setdefault(key, [value, value])
                    bounds[0] = min(bounds[0], value)
                    bounds[1] = max(bounds[1], value)
        accepted_kinds = []
        if isinstance(admitted, AdmittedFrame):
            accepted_kinds = [item.kind for item in admitted.accepted]
            for item in admitted.rejected:
                key = item.reason.value
                rejected_counts[key] = rejected_counts.get(key, 0) + 1
        if isinstance(estimate, RobotEstimate):
            estimates.append(
                {
                    "tick_id": estimate.context.tick_id,
                    "x_m": estimate.x_m,
                    "y_m": estimate.y_m,
                    "yaw_rad": estimate.yaw_rad,
                    "v_mps": estimate.v_mps,
                    "omega_rad_s": estimate.omega_rad_s,
                }
            )
        tick_summaries.append(
            {
                "tick_id": result.trace.context.tick_id,
                "monotonic_ns": result.trace.context.monotonic_ns,
                "health": health_summary,
                "accepted_kinds": accepted_kinds,
                "fault_layer": result.trace.fault_layer,
                "safety_decision": result.final_actuation.safety_decision.value,
                "safety_reason": result.final_actuation.reason,
            }
        )
    passed = (
        bool(report.ticks)
        and report.healthy_tick_count > 0
        and bool(report.l3_estimates)
        and report.all_commits_zero
        and not report.operator_stopped
    )
    return {
        "schema": "R2B4_V3_SENSOR_MEASUREMENT_V1",
        "status": "PASS" if passed else "FAIL",
        "tick_count": len(report.ticks),
        "healthy_tick_count": report.healthy_tick_count,
        "fault_tick_count": report.fault_tick_count,
        "l3_estimate_count": len(report.l3_estimates),
        "operator_stopped": report.operator_stopped,
        "all_commits_zero": report.all_commits_zero,
        "source_state_counts": source_state_counts,
        "rejected_counts": rejected_counts,
        "sample_ranges": sample_ranges,
        "first_estimate": estimates[0] if estimates else None,
        "last_estimate": estimates[-1] if estimates else None,
        "ticks": tick_summaries,
    }


def _open_lidar_port(danger_zone_m: float, pose_provider) -> NativeLidarPort:
    config = load_native_lidar_port_config(
        PROJECT_ROOT / "conf" / "hardver.json",
        PROJECT_ROOT / "conf" / "vezerles.json",
        danger_zone_m=danger_zone_m,
    )
    return open_native_lidar_port(config, pose_provider, serial.Serial)


def _lidar_diagnostics(service: NativeLidarPort | None) -> dict[str, object] | None:
    if service is None:
        return None
    status = service.get_runtime_status()
    result = service.get_matcher_result()
    result_summary = getattr(result, "summary", None) if result is not None else None
    selected_status_keys = (
        "running",
        "health",
        "driver_connected",
        "raw_scan_rate_hz",
        "raw_scan_latest_age_s",
        "scan_seq",
        "queue_drops",
        "result_queue_drops",
        "stale_result_drops",
        "matcher_process_errors",
        "matcher_processed_scans",
        "matcher_latency_ms_latest",
        "matcher_contract_id",
        "matcher_confidence_model",
        "matcher_transport",
    )
    return {
        "runtime_status": {
            key: status.get(key)
            for key in selected_status_keys
            if key in status
        },
        "last_result": (
            {
                "matcher_result_id": getattr(result, "matcher_result_id", None),
                "source_raw_scan_id": getattr(result, "source_raw_scan_id", None),
                "summary": _json_value(result_summary)
                if isinstance(result_summary, Mapping)
                else None,
            }
            if result is not None
            else None
        ),
    }


def _measurement_config(tick_count: int) -> FiniteSensorMeasurementConfig:
    runtime = load_bounded_physical_runtime_config(
        PROJECT_ROOT / "conf" / "hardver.json",
        PROJECT_ROOT / "conf" / "fizika.json",
        PROJECT_ROOT / "conf" / "speed_map.json",
        BoundedTeleopProfile(
            command_id="v3-sensor-measurement-zero-only",
            start_tick_id=2,
            active_tick_count=1,
            v_mps=0.01,
            omega_rad_s=0.0,
            max_v_mps=0.01,
            max_omega_rad_s=0.01,
        ),
        sensor_policy=native_sensor_policy(),
    )
    return FiniteSensorMeasurementConfig.from_runtime(runtime, tick_count=tick_count)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a finite V3 encoder/IMU/lidar/L3 measurement with zero output",
    )
    parser.add_argument("--ticks", type=int, default=250)
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    stop = _SignalStop()
    old_handlers = {
        signum: signal.signal(signum, stop.handle)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    lidar_service: NativeLidarPort | None = None

    def open_lidar(pose_provider) -> NativeLidarPort:
        nonlocal lidar_service
        lidar_service = _open_lidar_port(
            config.sensors.lidar_danger_zone_m,
            pose_provider,
        )
        return lidar_service

    try:
        config = _measurement_config(args.ticks)
        report = run_finite_sensor_measurement(
            lgpio,
            smbus2.SMBus,
            open_lidar,
            config,
            stop_requested=stop,
        )
        summary = summarize_report(report)
        summary["lidar_diagnostics"] = _lidar_diagnostics(lidar_service)
    except Exception as exc:
        summary = {
            "schema": "R2B4_V3_SENSOR_MEASUREMENT_V1",
            "status": "ERROR",
            "error_type": type(exc).__name__,
            "error": str(exc),
        }
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)

    rendered = json.dumps(summary, indent=2, sort_keys=True)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    console_summary = dict(summary)
    ticks = console_summary.pop("ticks", [])
    console_summary["tick_details_written"] = len(ticks) if args.output else 0
    print(json.dumps(console_summary, indent=2, sort_keys=True))
    return 0 if summary.get("status") == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
