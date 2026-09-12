#!/usr/bin/env python3
"""Finite STOP-only RPi measurement through the canonical resident process.

The private command directory stays empty, so AtomicResidentCommandGateway
supplies STOP throughout. Hardware ownership, shutdown and motor writes remain
entirely in the normal runtime. This tool grants no positive motion command.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
import time

from v3.adapters.resident_command import AtomicResidentCommandGateway, ResidentCommandMailboxConfig
from v3.mcap_capture import McapCaptureConfig
from v3.mcap_reader import EVENT_TOPIC, McapReader, RAW_LIDAR_TOPIC, TICK_TOPIC
from v3.test_hub_v2 import verify_evidence
from v3_process_runtime import (
    PROJECT_ROOT, AsyncResidentStatusPublisher, McapCaptureSession, ResidentStatusConfig,
    _capture_configuration, load_resident_runtime_config, native_lidar_factory,
    run_v3_resident_process,
)


def measure(duration_s: float = 10.0) -> dict[str, object]:
    if not 1 <= duration_s <= 60:
        raise ValueError('duration must be between 1 and 60 seconds')
    import lgpio
    import serial
    import smbus2

    destination = Path(tempfile.mkdtemp(prefix='r2b4-mcap-measurement-', dir='/tmp'))
    command_dir = destination / 'stop-only-command'
    command_dir.mkdir(mode=0o700)
    config = load_resident_runtime_config(PROJECT_ROOT)
    gateway = AtomicResidentCommandGateway(ResidentCommandMailboxConfig(command_dir / 'absent.json'))
    session = McapCaptureSession(
        destination.name, destination / 'capture.mcap',
        configuration=_capture_configuration(PROJECT_ROOT, config),
        metadata={'purpose': 'finite STOP-only RPi5 MCAP measurement'},
        config=McapCaptureConfig(mode='append_only'),
    )
    publisher = AsyncResidentStatusPublisher(ResidentStatusConfig(destination / 'status.json'))
    started = time.monotonic()
    report = run_v3_resident_process(
        lgpio, smbus2.SMBus, native_lidar_factory(config.sensor_inputs, serial.Serial), lgpio,
        gateway, config, publisher, approval='native-resident-v3',
        stop_requested=lambda: time.monotonic() - started >= duration_s,
        capture_session=session,
    )
    capture = session.worker.result
    if capture is None:
        raise RuntimeError(f'no capture produced; measurement directory: {destination}')
    reader = McapReader(capture.path)
    final = reader.last_json(EVENT_TOPIC)[1]
    ticks = 0
    all_zero = True
    for _, row in reader.iter_json_messages(topics=[TICK_TOPIC]):
        ticks += 1
        output = row['expected']['layers'].get('L12', {})
        all_zero = all_zero and output.get('enabled') is False and output.get('left_output') == 0 and output.get('right_output') == 0
    raw_count = max_points = truncated = 0
    for _, row in reader.iter_json_messages(topics=[RAW_LIDAR_TOPIC]):
        raw_count += 1
        max_points = max(max_points, row['source_point_count'])
        truncated += int(row['points_truncated'])
    metrics = final['metrics']
    size = capture.path.stat().st_size
    evidence_verification = verify_evidence(session.evidence['evidence_index'])
    payload = {
        'status': 'PASS' if report.status == 0 and all_zero and session.evidence['status'] == 'PASS' and evidence_verification['status'] == 'PASS' else 'FAIL',
        'output_dir': str(destination), 'capture_path': str(capture.path),
        'runtime_report': report.as_dict(), 'all_final_motor_outputs_zero': all_zero,
        'captured_tick_count': ticks, 'capture_size_bytes': size,
        'capture_metrics': metrics,
        'mean_capture_bytes_per_second': size / (metrics['elapsed_ns'] / 1e9),
        'process_cpu_percent_one_core': 100 * metrics['process_cpu_ns'] / metrics['elapsed_ns'],
        'raw_lidar': {'scan_count': raw_count, 'max_points': max_points, 'truncated': truncated},
        'test_hub': session.evidence, 'evidence_verification': evidence_verification['status'],
        'scope': 'stationary resident runtime; no physical movement or loaded motor behavior demonstrated',
    }
    (destination / 'measurement.json').write_text(json.dumps(payload, indent=2) + '\n')
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=10)
    args = parser.parse_args()
    payload = measure(args.duration_s)
    print(json.dumps(payload, indent=2))
    return 0 if payload['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
