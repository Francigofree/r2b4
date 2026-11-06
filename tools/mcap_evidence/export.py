"""Deterministic JSON and size-bounded NDJSON shards (never split a row)."""
import hashlib
import json
from pathlib import Path


def encoded(value) -> bytes:
    return (json.dumps(value, ensure_ascii=True, separators=(',', ':'), sort_keys=True,
                       allow_nan=False) + '\n').encode('utf-8')


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded(value))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


class Shards:
    def __init__(self, root: Path, unit: int, max_bytes: int):
        self.root, self.unit, self.max_bytes = root, unit, max_bytes
        self.files = {}
        self.parts = {}

    def write(self, name: str, row) -> tuple[str, int, int]:
        data = encoded(row)
        part = self.parts.get(name, 0)
        current = self.files.get(name)
        if current and current[1].tell() and current[1].tell() + len(data) > self.max_bytes:
            current[1].close()
            part += 1
            current = None
        if current is None:
            relative = f'{name}_{self.unit:05d}_{part:05d}.ndjson'
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            current = (relative, path.open('wb'))
            self.files[name] = current
            self.parts[name] = part
        offset = current[1].tell()
        current[1].write(data)
        return current[0], offset, len(data)

    def close(self):
        for _, stream in self.files.values():
            stream.close()


def manifest_digest(manifest) -> str:
    """SHA256 of canonical JSON + LF, omitting only the self_sha256 field."""
    return hashlib.sha256(encoded({k: v for k, v in manifest.items() if k != 'self_sha256'})).hexdigest()
