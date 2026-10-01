"""Small analyzer helpers without runtime authority."""

from __future__ import annotations


def topic_message_counts(reader: object) -> dict[str, int]:
    raw = reader.statistics.get("channel_message_counts") if isinstance(reader.statistics, dict) else None
    counts = raw if isinstance(raw, dict) else {}
    result: dict[str, int] = {}
    for channel_id, count in counts.items():
        try:
            channel = reader.channels.get(int(channel_id))
            number = int(count)
        except (TypeError, ValueError):
            continue
        if channel is not None:
            result[channel.topic] = number
    return dict(sorted(result.items()))
