"""Strict native MCAP inspection shared by replay and developer scripts."""
from dataclasses import asdict
from pathlib import Path

from .mcap_reader import McapReader, McapReadError


def inspect_mcap(capture_path: str | Path, *, deep: bool = False) -> dict[str, object]:
    reader = McapReader(capture_path)
    structure = reader.inspect(verify_chunks=deep)
    final = None
    integrity_error = None
    if deep and structure.valid:
        try:
            final = reader.capture_integrity()
        except McapReadError as exc:
            integrity_error = str(exc)
    return {
        'status': 'PASS' if structure.valid and integrity_error is None else 'FAIL',
        'capture_path': str(reader.path.resolve()),
        'structure': asdict(structure),
        'integrity_error': integrity_error,
        'capture_metadata': reader.latest_metadata('r2b4.capture'),
        'final_metadata': reader.latest_metadata('r2b4.capture.final'),
        'final_event': final,
        'topics': {c.topic: {'channel_id': c.channel_id, 'encoding': c.message_encoding,
                             'metadata': dict(c.metadata)} for c in reader.channels.values()},
    }
