#!/usr/bin/env python3
"""Motor-output-free raw A/B encoder probe.

Stop the resident runtime before using this tool. It claims only encoder GPIOs,
never imports a motor writer, and writes the bounded raw A/B callback decisions
plus final counter summaries to NDJSON.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from v3.adapters.gpio_counter import (
    GpioCounterChannelConfig,
    GpioCounterPairConfig,
    NativeGpioSignedCounterPair,
)


def load_pair(path: Path, gpio_chip: int):
    raw = json.loads(path.read_text(encoding="utf-8"))
    enc = raw["encoderek"]
    common = dict(
        forward_b_level=int(enc["forward_b_level"]),
        pull_up=bool(enc["input_pull_up"]),
        a_debounce_micros=int(enc["a_debounce_micros"]),
        direction_guard_micros=int(enc.get("direction_guard_micros", 50)),
        direction_change_confirm_edges=int(enc.get("direction_change_confirm_edges", 3)),
        direction_change_confirm_window_micros=int(
            enc.get("direction_change_confirm_window_micros", 250_000)
        ),
    )
    return GpioCounterPairConfig(
        left=GpioCounterChannelConfig(
            int(enc["bal_a_pin"]), int(enc["bal_b_pin"]),
            invert=bool(enc["invert_bal"]), **common,
        ),
        right=GpioCounterChannelConfig(
            int(enc["jobb_a_pin"]), int(enc["jobb_b_pin"]),
            invert=bool(enc["invert_jobb"]), **common,
        ),
        gpio_chip=gpio_chip,
        diagnostic_event_capacity=16384,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware", default="conf/hardver.json")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--gpio-chip", type=int, default=0)
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    try:
        import lgpio  # type: ignore
    except ImportError as exc:
        raise SystemExit("lgpio is required on the Raspberry Pi") from exc

    hardware = Path(args.hardware).resolve()
    config = load_pair(hardware, args.gpio_chip)
    output = Path(args.output) if args.output else Path("runtime") / (
        "encoder_ab_probe_" + time.strftime("%Y%m%d_%H%M%S") + ".ndjson"
    )
    output.parent.mkdir(parents=True, exist_ok=True)

    owner = NativeGpioSignedCounterPair(lgpio, config)
    started = time.monotonic_ns()
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            time.sleep(0.05)
    finally:
        ended = time.monotonic_ns()
        left_snapshot = owner.left_counter.snapshot()
        right_snapshot = owner.right_counter.snapshot()
        left_events = owner.diagnostic_events("left")
        right_events = owner.diagnostic_events("right")
        owner.close()

    with output.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps({
            "record_type": "header",
            "schema": "R2B4_ENCODER_AB_PROBE_V1",
            "motor_output_authority": False,
            "hardware": str(hardware),
            "started_monotonic_ns": started,
            "ended_monotonic_ns": ended,
            "config": {
                "left": asdict(config.left),
                "right": asdict(config.right),
                "gpio_chip": config.gpio_chip,
            },
        }, sort_keys=True) + "\n")
        merged = [("left", e) for e in left_events] + [("right", e) for e in right_events]
        merged.sort(key=lambda item: (item[1].callback_timestamp_ns, item[0]))
        for side, event in merged:
            handle.write(json.dumps({
                "record_type": "ab_event",
                "side": side,
                **asdict(event),
            }, sort_keys=True) + "\n")
        for side, snapshot in (("left", left_snapshot), ("right", right_snapshot)):
            handle.write(json.dumps({
                "record_type": "summary",
                "side": side,
                "pulse_count": snapshot.pulse_count,
                "invalid_alerts": snapshot.invalid_alerts,
                "quadrature_rejections": snapshot.quadrature_rejections,
                "direction_change_candidates": snapshot.direction_change_candidates,
                "direction_changes_confirmed": snapshot.direction_changes_confirmed,
                "confirmed_direction": snapshot.confirmed_direction,
                "pending_direction": snapshot.pending_direction,
                "pending_direction_edges": snapshot.pending_direction_edges,
                "edge_history_count": len(snapshot.edge_history),
            }, sort_keys=True) + "\n")

    print(output)
    print("No motor writer was created. Rotate wheels manually during the probe.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
