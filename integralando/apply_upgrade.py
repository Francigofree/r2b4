#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
from datetime import datetime, timezone
from pathlib import Path

TARGET_HEAD = "bc188d1895d334dd9d10bd5b57fb8798522494bc"
PACKAGE_ROOT = Path(__file__).resolve().parent
FILES_ROOT = PACKAGE_ROOT / "files"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source anchor, found {count}")
    return text.replace(old, new, 1)


def git_head(root: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return None


def atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    temp = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as handle:
            handle.write(content)
        os.replace(temp, path)
    finally:
        temp.unlink(missing_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", nargs="?", default="/home/alba/project_r2b4")
    parser.add_argument("--allow-source-drift", action="store_true")
    args = parser.parse_args()

    root = Path(args.root).resolve()
    if not (root / "v3").is_dir():
        raise RuntimeError(f"not an R2B4 repo: {root}")

    head = git_head(root)
    if not args.allow_source_drift and head != TARGET_HEAD:
        raise RuntimeError(
            f"target HEAD is {TARGET_HEAD}, current HEAD is {head}; "
            "use --allow-source-drift only after reviewing source anchors"
        )

    capture_path = root / "v3/mcap_capture.py"
    reader_path = root / "v3/mcap_reader.py"
    bridge_path = root / "v3/mcap_replay_bridge.py"

    capture = capture_path.read_text(encoding="utf-8")
    reader = reader_path.read_text(encoding="utf-8")
    bridge = bridge_path.read_text(encoding="utf-8")

    capture = replace_once(
        capture,
        "from .capture_ipc import IPC_CHECKPOINT_KEY, IPC_TRIGGER_REASON_KEY\n",
        "from .capture_compaction import compact_checkpoint_row, compact_tick_row\n"
        "from .capture_ipc import IPC_CHECKPOINT_KEY, IPC_TRIGGER_REASON_KEY\n",
        "mcap_capture import",
    )

    old = (
        "            referenced = tuple(sorted(_referenced_lidar_revisions(row)))\n"
        "            if len(referenced) > self._config.max_referenced_revisions_per_tick:\n"
        "                raise CaptureEncodingError(\"tick exceeds referenced LiDAR revision bound\")\n"
        "            tick = EncodedRecord(\n"
    )
    new = (
        "            referenced = tuple(sorted(_referenced_lidar_revisions(row)))\n"
        "            if len(referenced) > self._config.max_referenced_revisions_per_tick:\n"
        "                raise CaptureEncodingError(\"tick exceeds referenced LiDAR revision bound\")\n"
        "            stored_row = compact_tick_row(row)\n"
        "            tick = EncodedRecord(\n"
    )
    capture = replace_once(capture, old, new, "mcap_capture stored row")

    # IMPORTANT: target the TICK block specifically.
    # The generic line "payload=_json_bytes(row)" also exists in the raw LiDAR
    # block and must remain unchanged.
    old = (
        "                mcap_topic=TICK_TOPIC,\n"
        "                monotonic_ns=monotonic_ns,\n"
        "                sequence=tick_id,\n"
        "                payload=_json_bytes(row),\n"
        "                tick_id=tick_id,\n"
    )
    new = (
        "                mcap_topic=TICK_TOPIC,\n"
        "                monotonic_ns=monotonic_ns,\n"
        "                sequence=tick_id,\n"
        "                payload=_json_bytes(stored_row),\n"
        "                tick_id=tick_id,\n"
    )
    capture = replace_once(
        capture,
        old,
        new,
        "mcap_capture tick payload",
    )

    capture = replace_once(
        capture,
        "                        payload=_json_bytes(checkpoint_row),\n",
        "                        payload=_json_bytes(compact_checkpoint_row(checkpoint_row)),\n",
        "mcap_capture checkpoint payload",
    )

    reader = replace_once(
        reader,
        "from typing import BinaryIO, Iterable, Iterator, Mapping, Sequence\n",
        "from typing import BinaryIO, Iterable, Iterator, Mapping, Sequence\n\n"
        "from .capture_compaction import expand_checkpoint_row, expand_tick_row\n",
        "mcap_reader import",
    )
    old = (
        "    def iter_json_messages(self, **kwargs: object) -> Iterator[tuple[McapMessage, object]]:\n"
        "        for message in self.iter_messages(**kwargs):\n"
        "            yield message, message.json()\n"
    )
    new = (
        "    def iter_json_messages(self, **kwargs: object) -> Iterator[tuple[McapMessage, object]]:\n"
        "        for message in self.iter_messages(**kwargs):\n"
        "            payload = message.json()\n"
        "            if isinstance(payload, dict):\n"
        "                if message.topic == TICK_TOPIC:\n"
        "                    payload = expand_tick_row(payload)\n"
        "                elif message.topic == CHECKPOINT_TOPIC:\n"
        "                    payload = expand_checkpoint_row(payload)\n"
        "            yield message, payload\n"
    )
    reader = replace_once(reader, old, new, "mcap_reader expansion")

    old = (
        "    for message in reader.iter_messages(\n"
        "        topics=(TICK_TOPIC,),\n"
        "        start_ns=checkpoint_message.log_time_ns if checkpoint_message else None,\n"
        "        end_ns=last_target_msg.log_time_ns,\n"
        "    ):\n"
    )
    new = (
        "    for message, payload in reader.iter_json_messages(\n"
        "        topics=(TICK_TOPIC,),\n"
        "        start_ns=checkpoint_message.log_time_ns if checkpoint_message else None,\n"
        "        end_ns=last_target_msg.log_time_ns,\n"
        "    ):\n"
    )
    bridge = replace_once(bridge, old, new, "mcap_replay_bridge tick iterator")
    bridge = replace_once(
        bridge,
        "        payload = message.json()\n"
        "        if not isinstance(payload, dict):\n",
        "        if not isinstance(payload, dict):\n",
        "mcap_replay_bridge expanded payload",
    )

    planned = {
        capture_path: capture,
        reader_path: reader,
        bridge_path: bridge,
        root / "v3/capture_compaction.py": (
            FILES_ROOT / "v3/capture_compaction.py"
        ).read_text(encoding="utf-8"),
        root / "tests/test_v3_capture_compaction.py": (
            FILES_ROOT / "tests/test_v3_capture_compaction.py"
        ).read_text(encoding="utf-8"),
        root / "tools/r2b4_capture_size_audit.py": (
            FILES_ROOT / "tools/r2b4_capture_size_audit.py"
        ).read_text(encoding="utf-8"),
    }

    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    backup = root / "runtime/upgrade_backups" / f"capture_economy_refactor_{stamp}"
    backup.mkdir(parents=True, exist_ok=False)

    written: list[Path] = []
    try:
        for path, content in planned.items():
            if path.exists():
                rel = path.relative_to(root)
                dest = backup / rel
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, dest)
            atomic_write(path, content)
            written.append(path)
    except Exception:
        for path in reversed(written):
            rel = path.relative_to(root)
            saved = backup / rel
            if saved.exists():
                shutil.copy2(saved, path)
            else:
                path.unlink(missing_ok=True)
        raise

    print("Applied R2B4 lossless capture economy refactor.")
    print(f"Backup: {backup}")
    print("No robot movement was started.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
