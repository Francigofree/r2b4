#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from v3.capture_compaction import compact_checkpoint_row, compact_tick_row
from v3.mcap_reader import CHECKPOINT_TOPIC, McapReader, TICK_TOPIC


def main() -> int:
    parser = argparse.ArgumentParser(description="R2B4 MCAP lossless compaction size audit")
    parser.add_argument("capture")
    args = parser.parse_args()

    reader = McapReader(Path(args.capture))
    stored = {}
    projected = {}
    counts = {}
    for message in reader.iter_messages():
        stored[message.topic] = stored.get(message.topic, 0) + len(message.data)
        counts[message.topic] = counts.get(message.topic, 0) + 1
        value = message.json()
        if message.topic == TICK_TOPIC and isinstance(value, dict):
            value = compact_tick_row(value)
        elif message.topic == CHECKPOINT_TOPIC and isinstance(value, dict):
            value = compact_checkpoint_row(value)
        encoded = json.dumps(
            value, ensure_ascii=False, sort_keys=True,
            separators=(",", ":"), allow_nan=False,
        ).encode("utf-8")
        projected[message.topic] = projected.get(message.topic, 0) + len(encoded)

    total_stored = sum(stored.values())
    total_projected = sum(projected.values())
    result = {
        "capture": str(Path(args.capture).resolve()),
        "message_payload_bytes_before": total_stored,
        "message_payload_bytes_projected_after": total_projected,
        "saved_bytes": total_stored - total_projected,
        "saved_fraction": (
            (total_stored - total_projected) / total_stored if total_stored else 0.0
        ),
        "topics": {
            topic: {
                "count": counts[topic],
                "before_bytes": stored[topic],
                "projected_after_bytes": projected[topic],
                "saved_bytes": stored[topic] - projected[topic],
            }
            for topic in sorted(stored)
        },
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
