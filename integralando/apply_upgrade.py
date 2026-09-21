#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

EXPECTED_BASE = "6c785ec4497944481c285578240277528fa6a18c"
TARGETS = (
    "v3/process_sidecars.py",
    "v3/adapters/process_lidar_port.py",
    "v3/mcap_capture.py",
    "v3/mcap_reader.py",
    "v3/mcap_replay_bridge.py",
    "v3/test_hub_portable.py",
    "v3_process_runtime.py",
)
NEW_FILES = (
    "v3/capture_ipc.py",
    "tests/test_v3_capture_refactor_p0.py",
)


def run(*args: str, cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, cwd=cwd, check=True, text=True, capture_output=True)


def replace_once(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one {label} anchor, found {count}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def insert_before_once(path: Path, marker: str, addition: str, label: str) -> None:
    replace_once(path, marker, addition + marker, label)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--project-root", default=".")
    ap.add_argument("--allow-newer", action="store_true")
    args = ap.parse_args()
    root = Path(args.project_root).resolve()
    if not (root / ".git").is_dir():
        raise SystemExit("project-root is not a Git working tree")
    head = run("git", "rev-parse", "HEAD", cwd=root).stdout.strip()
    if head != EXPECTED_BASE and not args.allow_newer:
        raise SystemExit(
            f"HEAD is {head}; inspected base is {EXPECTED_BASE}. "
            "Re-review before using --allow-newer."
        )
    dirty = run("git", "status", "--short", "--", *TARGETS, cwd=root).stdout.strip()
    if dirty:
        raise SystemExit("Refusing to overwrite modified target files:\n" + dirty)

    package = Path(__file__).resolve().parent
    backup = Path(tempfile.mkdtemp(prefix="r2b4_capture_refactor_backup_"))
    for rel in TARGETS:
        dst = backup / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / rel, dst)
    for rel in NEW_FILES:
        if (root / rel).exists():
            dst = backup / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(root / rel, dst)

    try:
        for rel in NEW_FILES:
            dst = root / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(package / "files" / rel, dst)

        # ---- v3/process_sidecars.py ----
        p = root / "v3/process_sidecars.py"
        replace_once(
            p,
            "from v3.execution import CaptureRecord\n",
            "from v3.execution import CaptureRecord\n"
            "from v3.capture_ipc import (\n"
            "    CaptureCoreExpander, CaptureCoreProjector, CaptureIpcProjectionError,\n"
            ")\n",
            "capture IPC import",
        )
        replace_once(
            p,
            "    project_root: str,\n    worker_cpu: int | None,\n",
            "    expect_raw_lidar_end: bool,\n    project_root: str,\n    worker_cpu: int | None,\n",
            "sidecar raw-end argument",
        )
        replace_once(
            p,
            '            topics=("v3.capture_record", "v3.raw_lidar"),\n',
            '            topics=(\n'
            '                "v3.capture_record", "v3.raw_lidar",\n'
            '                "v3.raw_lidar_transport", "v3.capture_transport",\n'
            '            ),\n',
            "capture topics",
        )
        replace_once(
            p,
            "        consumer.start()\n        ready_event.set()\n\n"
            "        processed = 0\n"
            "        finish_request: tuple[str, bool, int] | None = None\n",
            "        consumer.start()\n"
            "        expander = CaptureCoreExpander()\n"
            "        ready_event.set()\n\n"
            "        processed = 0\n"
            "        raw_end_received = not expect_raw_lidar_end\n"
            "        finish_request: tuple[str, bool, int, int] | None = None\n",
            "capture sidecar state",
        )
        replace_once(
            p,
            '''            for _ in range(_RAW_LIDAR_CAPACITY):
                try:
                    raw_kind, raw_wire = raw_lidar_queue.get_nowait()
                except queue.Empty:
                    break
                if raw_kind != "raw":
                    raise RuntimeError(f"unknown LiDAR evidence kind: {raw_kind!r}")
                snapshot = _unwire_raw(raw_wire)
                if snapshot is not None:
                    hub.publish(snapshot, topic="v3.raw_lidar")
''',
            '''            for _ in range(_RAW_LIDAR_CAPACITY):
                try:
                    raw_message = raw_lidar_queue.get_nowait()
                except queue.Empty:
                    break
                if not isinstance(raw_message, tuple) or not raw_message:
                    raise RuntimeError("invalid LiDAR evidence message")
                raw_kind = raw_message[0]
                if raw_kind == "raw":
                    if len(raw_message) < 2:
                        raise RuntimeError("raw LiDAR evidence lacks payload")
                    snapshot = _unwire_raw(raw_message[1])
                    if snapshot is not None:
                        hub.publish(snapshot, topic="v3.raw_lidar")
                elif raw_kind == "raw_end":
                    if len(raw_message) != 4:
                        raise RuntimeError("invalid raw LiDAR end marker")
                    raw_end_received = True
                    hub.publish(
                        {
                            "event_type": "raw_lidar_transport_end",
                            "last_revision": int(raw_message[1]),
                            "produced_count": int(raw_message[2]),
                            "superseded_count": int(raw_message[3]),
                        },
                        topic="v3.raw_lidar_transport",
                    )
                else:
                    raise RuntimeError(f"unknown LiDAR evidence kind: {raw_kind!r}")
''',
            "raw receive loop",
        )
        replace_once(
            p,
            '                    finish_request = (str(command[1]), bool(command[2]), int(command[3]))\n',
            '                    finish_request = (\n'
            '                        str(command[1]), bool(command[2]), int(command[3]), int(command[4])\n'
            '                    )\n',
            "finish request shape",
        )
        replace_once(
            p,
            "            if finish_request is not None and processed >= finish_request[2]:\n"
            "                hub.close()\n",
            "            if (\n"
            "                finish_request is not None\n"
            "                and processed >= finish_request[2]\n"
            "                and raw_end_received\n"
            "            ):\n"
            "                if finish_request[3]:\n"
            "                    hub.publish(\n"
            "                        {\"event_type\": \"capture_core_transport_loss\", \"drop_count\": finish_request[3]},\n"
            "                        topic=\"v3.capture_transport\",\n"
            "                    )\n"
            "                hub.close()\n",
            "finish raw-end gate",
        )
        replace_once(
            p,
            '''            if kind == "record":
                hub.publish(payload, topic="v3.capture_record")
            elif kind == "raw_lidar":
''',
            '''            if kind == "core":
                hub.publish(expander.expand(payload), topic="v3.capture_record")
            elif kind == "record":
                hub.publish(payload, topic="v3.capture_record")
            elif kind == "raw_lidar":
''',
            "core ingress",
        )
        replace_once(
            p,
            '        "_strict_affinity",\n        "_transport_capacity",\n        "_worker_cpu",\n',
            '        "_strict_affinity",\n        "_transport_capacity",\n        "_worker_cpu",\n'
            '        "_projector",\n        "_projection_drop_count",\n        "_expect_raw_lidar_end",\n',
            "session slots",
        )
        replace_once(
            p,
            "        project_root: str | Path,\n        worker_cpu: int | None = None,\n        strict_affinity: bool = False,\n    ) -> None:\n",
            "        project_root: str | Path,\n        worker_cpu: int | None = None,\n        strict_affinity: bool = False,\n        expect_raw_lidar_end: bool = False,\n    ) -> None:\n",
            "session raw-end arg",
        )
        replace_once(
            p,
            "        self._worker_cpu = worker_cpu\n"
            "        self._strict_affinity = strict_affinity\n"
            "        self._process = context.Process(\n",
            "        self._worker_cpu = worker_cpu\n"
            "        self._strict_affinity = strict_affinity\n"
            "        self._projector = CaptureCoreProjector()\n"
            "        self._projection_drop_count = 0\n"
            "        self._expect_raw_lidar_end = bool(expect_raw_lidar_end)\n"
            "        self._process = context.Process(\n",
            "projector init",
        )
        replace_once(
            p,
            "                self._ready_event,\n"
            "                self._failed_event,\n"
            "                str(self._project_root),\n",
            "                self._ready_event,\n"
            "                self._failed_event,\n"
            "                self._expect_raw_lidar_end,\n"
            "                str(self._project_root),\n",
            "process args",
        )
        replace_once(
            p,
            "    def failed(self) -> bool:\n"
            "        return bool(self._failed_event.is_set() or self._drop_count)\n",
            "    def failed(self) -> bool:\n"
            "        return bool(self._failed_event.is_set() or self._drop_count or self._projection_drop_count)\n",
            "failed property",
        )
        replace_once(
            p,
            "    def observe(self, record: CaptureRecord) -> None:\n"
            '        self._enqueue("record", record)\n',
            "    def observe(self, record: CaptureRecord) -> None:\n"
            "        try:\n"
            "            projected = self._projector.project(record)\n"
            "        except CaptureIpcProjectionError:\n"
            "            self._projection_drop_count += 1\n"
            "            return\n"
            '        self._enqueue("core", projected)\n',
            "observe projection",
        )
        replace_once(
            p,
            '            ("finish", status, True, self._enqueued_count),\n',
            '            ("finish", status, True, self._enqueued_count, self._drop_count + self._projection_drop_count),\n',
            "finish loss count",
        )

        # ---- v3/adapters/process_lidar_port.py ----
        p = root / "v3/adapters/process_lidar_port.py"
        insert_before_once(
            p,
            "\ndef _lidar_owner_process_main(\n",
            '''\ndef _put_raw_evidence(target: Any, payload: object, superseded_count: int) -> int:
    try:
        target.put_nowait(payload)
        return superseded_count
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    else:
        superseded_count += 1
    try:
        target.put_nowait(payload)
    except queue.Full:
        superseded_count += 1
    return superseded_count


def _put_raw_end(target: Any, *, last_revision: int, produced_count: int, superseded_count: int) -> None:
    marker = ("raw_end", int(last_revision), int(produced_count), int(superseded_count))
    try:
        target.put(marker, timeout=0.5)
        return
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    else:
        superseded_count += 1
    marker = ("raw_end", int(last_revision), int(produced_count), int(superseded_count))
    try:
        target.put(marker, timeout=0.5)
    except queue.Full:
        return

''',
            "raw helper insertion",
        )
        replace_once(
            p,
            "    port = None\n    try:\n",
            "    port = None\n"
            "    raw_produced_count = 0\n"
            "    raw_superseded_count = 0\n"
            "    last_raw_revision = 0\n"
            "    try:\n",
            "raw accounting init",
        )
        replace_once(
            p,
            '''        if initial_raw is not None:
            _put_latest(raw_queue, ("raw", _wire_raw(initial_raw)))
        ready_event.set()
        last_raw_revision = int(getattr(initial_raw, "raw_scan_id", 0) or 0)
''',
            '''        if initial_raw is not None:
            raw_produced_count += 1
            raw_superseded_count = _put_raw_evidence(
                raw_queue, ("raw", _wire_raw(initial_raw)), raw_superseded_count
            )
        ready_event.set()
        last_raw_revision = int(getattr(initial_raw, "raw_scan_id", 0) or 0)
''',
            "initial raw accounting",
        )
        replace_once(
            p,
            '''            if raw_changed and raw is not None:
                _put_latest(raw_queue, ("raw", _wire_raw(raw)))
                control_raw = _wire_control_raw(
''',
            '''            if raw_changed and raw is not None:
                raw_produced_count += 1
                raw_superseded_count = _put_raw_evidence(
                    raw_queue, ("raw", _wire_raw(raw)), raw_superseded_count
                )
                control_raw = _wire_control_raw(
''',
            "changed raw accounting",
        )
        replace_once(
            p,
            '''    finally:
        if port is not None:
            try:
                port.stop()
            except BaseException as exc:
                _put_latest(state_queue, ("error", type(exc).__name__, str(exc)))
''',
            '''    finally:
        if port is not None:
            try:
                port.stop()
            except BaseException as exc:
                _put_latest(state_queue, ("error", type(exc).__name__, str(exc)))
        _put_raw_end(
            raw_queue,
            last_revision=last_raw_revision,
            produced_count=raw_produced_count,
            superseded_count=raw_superseded_count,
        )
''',
            "raw end",
        )
        replace_once(
            p,
            '''        if not isinstance(newest, tuple) or len(newest) != 2 or newest[0] != "raw":
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        snapshot = _unwire_raw(newest[1])
''',
            '''        if not isinstance(newest, tuple) or not newest:
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        if newest[0] == "raw_end":
            if len(newest) == 4:
                self._status["capture_raw_transport_end"] = True
                self._status["capture_raw_produced_count"] = int(newest[2])
                self._status["capture_raw_superseded_count"] = int(newest[3])
                return
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        if len(newest) != 2 or newest[0] != "raw":
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        snapshot = _unwire_raw(newest[1])
''',
            "raw end local handling",
        )

        # ---- v3/mcap_capture.py ----
        p = root / "v3/mcap_capture.py"
        replace_once(p, "    max_consecutive_raw_lidar_missing: int = 2\n",
                     "    max_consecutive_raw_lidar_missing: int = 2\n    require_raw_lidar_transport_end: bool = False\n",
                     "raw end config")
        replace_once(p, '        if self.mode not in {"triggered", "append_only"}:\n',
                     '        if type(self.require_raw_lidar_transport_end) is not bool:\n'
                     '            raise TypeError("require_raw_lidar_transport_end must be bool")\n\n'
                     '        if self.mode not in {"triggered", "append_only"}:\n',
                     "raw end config validation")
        replace_once(p, "    raw_lidar_missing_revisions: tuple[int, ...]\n",
                     "    raw_lidar_missing_revisions: tuple[int, ...]\n    replay_complete: bool\n    raw_evidence_complete: bool\n",
                     "CaptureResult integrity fields")
        replace_once(p, '        "_latest_capacity_eviction_ns",\n',
                     '        "_latest_capacity_eviction_ns",\n        "_raw_capacity_eviction_count",\n        "_core_capacity_eviction_count",\n        "_latest_raw_capacity_eviction_ns",\n',
                     "capacity slots")
        replace_once(p, '        "_last_raw_revision",\n',
                     '        "_last_raw_revision",\n        "_raw_transport_end",\n        "_raw_transport_produced_count",\n        "_raw_transport_superseded_count",\n',
                     "raw transport slots")
        replace_once(
            p,
            "        encoded_metadata = encode_value(metadata or {})\n"
            "        if not isinstance(encoded_configuration, dict) or not isinstance(encoded_metadata, dict):\n",
            "        encoded_metadata = encode_value(metadata or {})\n"
            "        if isinstance(encoded_metadata, dict):\n"
            "            encoded_metadata = {**encoded_metadata, \"capture_policy\": encode_value(settings)}\n"
            "        if not isinstance(encoded_configuration, dict) or not isinstance(encoded_metadata, dict):\n",
            "capture policy metadata",
        )
        replace_once(p,
                     "        self._capacity_eviction_count = 0\n        self._latest_capacity_eviction_ns: int | None = None\n",
                     "        self._capacity_eviction_count = 0\n        self._latest_capacity_eviction_ns: int | None = None\n        self._raw_capacity_eviction_count = 0\n        self._core_capacity_eviction_count = 0\n        self._latest_raw_capacity_eviction_ns: int | None = None\n",
                     "capacity init")
        replace_once(p,
                     "        self._last_raw_revision: int | None = None\n",
                     "        self._last_raw_revision: int | None = None\n        self._raw_transport_end = False\n        self._raw_transport_produced_count = 0\n        self._raw_transport_superseded_count = 0\n",
                     "raw transport init")
        replace_once(p, "        encoded = self._encode_observation(item)\n",
                     "        self._consume_transport_integrity(item)\n        encoded = self._encode_observation(item)\n",
                     "transport integrity call")
        insert_before_once(
            p,
            "    def _encode_observation(self, item: ObservationFrame) -> tuple[EncodedRecord, ...]:\n",
            '''    def _consume_transport_integrity(self, item: ObservationFrame) -> None:
        payload = item.payload
        if _looks_like_capture_transport_topic(item.topic):
            if isinstance(payload, Mapping):
                drops = payload.get("drop_count", 0)
                if isinstance(drops, int) and not isinstance(drops, bool) and drops > 0:
                    self._integrity_reasons.add("CORE_TRANSPORT_LOSS")
            return
        if not _looks_like_raw_lidar_transport_topic(item.topic):
            return
        if not isinstance(payload, Mapping):
            self._integrity_reasons.add("RAW_LIDAR_TRANSPORT_INVALID")
            return
        if payload.get("event_type") != "raw_lidar_transport_end":
            return
        self._raw_transport_end = True
        produced = payload.get("produced_count", 0)
        superseded = payload.get("superseded_count", 0)
        if isinstance(produced, int) and not isinstance(produced, bool) and produced >= 0:
            self._raw_transport_produced_count = produced
        else:
            self._integrity_reasons.add("RAW_LIDAR_TRANSPORT_INVALID")
        if isinstance(superseded, int) and not isinstance(superseded, bool) and superseded >= 0:
            self._raw_transport_superseded_count = superseded
            if superseded:
                self._integrity_reasons.add("RAW_LIDAR_TRANSPORT_SUPERSEDE")
        else:
            self._integrity_reasons.add("RAW_LIDAR_TRANSPORT_INVALID")

''',
            "transport integrity method",
        )
        replace_once(
            p,
            '''        if isinstance(payload, (ExecutionRecord, EdgeFaultRecord, WriterFailureRecord)):
            row = encode_capture_record(payload)
''',
            '''        if item.topic == "v3.capture_record" and isinstance(payload, Mapping):
            row = dict(payload)
        elif isinstance(payload, (ExecutionRecord, EdgeFaultRecord, WriterFailureRecord)):
            row = encode_capture_record(payload)
        else:
            row = None
        if row is not None:
''',
            "preencoded core row",
        )
        replace_once(
            p,
            '''    def _enforce_capacities(self) -> None:
        while self._ring_tick_count > self._config.max_tick_count:
            self._drop_oldest_matching(TICK_TOPIC)
        while self._ring_raw_count > self._config.max_raw_lidar_scans:
            self._drop_oldest_matching(RAW_LIDAR_TOPIC)
        while self._ring_bytes > self._config.max_byte_capacity:
            self._drop_oldest(capacity=True)

    def _drop_oldest_matching(self, topic: str) -> None:
''',
            '''    def _enforce_capacities(self) -> None:
        while self._ring_tick_count > self._config.max_tick_count:
            self._drop_oldest_matching(TICK_TOPIC, capacity=True)
        while self._ring_raw_count > self._config.max_raw_lidar_scans:
            self._drop_oldest_matching(RAW_LIDAR_TOPIC, capacity=True)
        while self._ring_bytes > self._config.max_byte_capacity:
            if self._ring_raw_count:
                self._drop_oldest_matching(RAW_LIDAR_TOPIC, capacity=True)
            else:
                self._drop_oldest(capacity=True)

    def _drop_oldest_matching(self, topic: str, *, capacity: bool = False) -> None:
''',
            "raw-first ring capacity",
        )
        replace_once(
            p,
            '''            self._capacity_eviction_count += 1
            self._latest_capacity_eviction_ns = item.monotonic_ns
            return
        raise RuntimeError(f"ring count inconsistent for {topic}")
''',
            '''            if capacity:
                self._capacity_eviction_count += 1
                if topic == RAW_LIDAR_TOPIC:
                    self._raw_capacity_eviction_count += 1
                    self._latest_raw_capacity_eviction_ns = item.monotonic_ns
                else:
                    self._core_capacity_eviction_count += 1
                    self._latest_capacity_eviction_ns = item.monotonic_ns
            return
        raise RuntimeError(f"ring count inconsistent for {topic}")
''',
            "matching capacity accounting",
        )
        replace_once(
            p,
            '''        if capacity:
            self._capacity_eviction_count += 1
            self._latest_capacity_eviction_ns = item.monotonic_ns
''',
            '''        if capacity:
            self._capacity_eviction_count += 1
            if item.mcap_topic == RAW_LIDAR_TOPIC:
                self._raw_capacity_eviction_count += 1
                self._latest_raw_capacity_eviction_ns = item.monotonic_ns
            else:
                self._core_capacity_eviction_count += 1
                self._latest_capacity_eviction_ns = item.monotonic_ns
''',
            "generic capacity accounting",
        )
        replace_once(
            p,
            '''        if raw_lidar_missing_count:
            if raw_lidar_loss_within_tolerance:
                integrity_warnings.append("RAW_LIDAR_SPARSE_LOSS_TOLERATED")
            else:
                if gap_missing_raw:
                    self._integrity_reasons.add("RAW_LIDAR_REVISION_GAP")
                if missing_raw:
                    self._integrity_reasons.add("REFERENCED_RAW_LIDAR_MISSING")

        complete = not self._integrity_reasons
''',
            '''        if raw_lidar_missing_count:
            self._integrity_reasons.add("RAW_LIDAR_EVIDENCE_INCOMPLETE")
            if raw_lidar_loss_within_tolerance:
                integrity_warnings.append("RAW_LIDAR_SPARSE_LOSS_TOLERATED")
            if gap_missing_raw:
                self._integrity_reasons.add("RAW_LIDAR_REVISION_GAP")
            if missing_raw:
                self._integrity_reasons.add("REFERENCED_RAW_LIDAR_MISSING")

        if self._config.require_raw_lidar_transport_end and not self._raw_transport_end:
            self._integrity_reasons.add("RAW_LIDAR_TRANSPORT_END_MISSING")
        raw_capacity_missing = bool(
            self._latest_raw_capacity_eviction_ns is not None
            and self._latest_raw_capacity_eviction_ns >= lower
        )
        if raw_capacity_missing:
            self._integrity_reasons.add("RAW_LIDAR_CAPACITY_EVICTION")

        raw_integrity_reasons = sorted(
            reason for reason in self._integrity_reasons if _is_raw_integrity_reason(reason)
        )
        replay_integrity_reasons = sorted(
            reason for reason in self._integrity_reasons if not _is_raw_integrity_reason(reason)
        )
        replay_complete = not replay_integrity_reasons
        raw_evidence_complete = not raw_integrity_reasons
        complete = replay_complete and raw_evidence_complete
''',
            "split integrity",
        )
        replace_once(p, '            "complete": complete,\n',
                     '            "complete": complete,\n            "replay_complete": replay_complete,\n            "raw_evidence_complete": raw_evidence_complete,\n            "replay_integrity_reasons": replay_integrity_reasons,\n            "raw_integrity_reasons": raw_integrity_reasons,\n',
                     "integrity fields")
        replace_once(p, '            "capacity_eviction_count": self._capacity_eviction_count,\n',
                     '            "capacity_eviction_count": self._capacity_eviction_count,\n            "core_capacity_eviction_count": self._core_capacity_eviction_count,\n            "raw_capacity_eviction_count": self._raw_capacity_eviction_count,\n            "raw_transport_end": self._raw_transport_end,\n            "raw_transport_produced_count": self._raw_transport_produced_count,\n            "raw_transport_superseded_count": self._raw_transport_superseded_count,\n',
                     "integrity transport metrics")
        replace_once(p, '                "complete": "true" if complete else "false",\n',
                     '                "complete": "true" if complete else "false",\n                "replay_complete": "true" if replay_complete else "false",\n                "raw_evidence_complete": "true" if raw_evidence_complete else "false",\n',
                     "final metadata split")
        replace_once(p, '            raw_lidar_missing_revisions=missing_raw,\n        )\n',
                     '            raw_lidar_missing_revisions=missing_raw,\n            replay_complete=replay_complete,\n            raw_evidence_complete=raw_evidence_complete,\n        )\n',
                     "CaptureResult values")
        insert_before_once(
            p,
            "\ndef _looks_like_raw_lidar_topic(topic: str) -> bool:\n",
            '''\ndef _looks_like_capture_transport_topic(topic: str) -> bool:
    return topic.strip().lower().replace("-", "_") in {"v3.capture_transport", "capture_transport"}


def _looks_like_raw_lidar_transport_topic(topic: str) -> bool:
    return topic.strip().lower().replace("-", "_") in {"v3.raw_lidar_transport", "raw_lidar_transport"}


def _is_raw_integrity_reason(reason: str) -> bool:
    return reason.startswith("RAW_LIDAR_") or reason == "REFERENCED_RAW_LIDAR_MISSING"

''',
            "integrity helpers",
        )

        # ---- v3/mcap_reader.py ----
        p = root / "v3/mcap_reader.py"
        replace_once(p, "    def capture_integrity(self) -> dict[str, object]:\n",
                     "    def capture_integrity(self, *, require_raw_evidence: bool = True) -> dict[str, object]:\n",
                     "reader signature")
        replace_once(
            p,
            '''        if (integrity.get("complete") is not True or metadata.get("complete") != "true"
                or integrity.get("integrity_reasons") or not subscription_ok
                or not counts.get(TICK_TOPIC)):
            raise McapReadError("CAPTURE_INCOMPLETE: required evidence is incomplete")
''',
            '''        replay_complete = (
            integrity.get("replay_complete", integrity.get("complete")) is True
            and metadata.get("replay_complete", metadata.get("complete")) == "true"
        )
        raw_complete = (
            integrity.get("raw_evidence_complete", integrity.get("complete")) is True
            and metadata.get("raw_evidence_complete", metadata.get("complete")) == "true"
        )
        required_complete = replay_complete and (raw_complete if require_raw_evidence else True)
        replay_reasons = integrity.get(
            "replay_integrity_reasons", integrity.get("integrity_reasons")
        )
        if not required_complete or replay_reasons or not subscription_ok or not counts.get(TICK_TOPIC):
            raise McapReadError("CAPTURE_INCOMPLETE: required evidence is incomplete")
''',
            "reader split integrity",
        )

        # ---- v3/mcap_replay_bridge.py ----
        p = root / "v3/mcap_replay_bridge.py"
        replace_once(p, "            else reader.capture_integrity()\n",
                     "            else reader.capture_integrity(require_raw_evidence=False)\n",
                     "replay reader policy")
        replace_once(
            p,
            "    original_complete = replay_eligible = True\n",
            "    final_integrity = final.get(\"integrity\") if isinstance(final, Mapping) else None\n"
            "    if not isinstance(final_integrity, Mapping):\n"
            "        raise McapReplayBridgeError(\"MCAP capture lacks final integrity\")\n"
            "    replay_eligible = bool(final_integrity.get(\"replay_complete\", final_integrity.get(\"complete\")))\n"
            "    original_complete = bool(final_integrity.get(\"complete\"))\n"
            "    if not replay_eligible:\n"
            "        raise McapReplayBridgeError(\"CAPTURE_INCOMPLETE: replay core is incomplete\")\n",
            "replay eligibility",
        )

        # ---- v3/test_hub_portable.py ----
        p = root / "v3/test_hub_portable.py"
        replace_once(
            p,
            "            verified_final = reader.capture_integrity()\n",
            "            verified_final = reader.capture_integrity(require_raw_evidence=False)\n",
            "portable replay raw-independent integrity",
        )

        # ---- v3_process_runtime.py ----
        p = root / "v3_process_runtime.py"
        replace_once(
            p,
            "                    mode=args.capture_mode,\n                ),\n",
            "                    mode=args.capture_mode,\n                    require_raw_lidar_transport_end=True,\n                ),\n",
            "production raw-end policy",
        )
        replace_once(
            p,
            "                strict_affinity=(\n"
            "                    affinity_config.strict if affinity_config.enabled else False\n"
            "                ),\n"
            "            )\n"
            "            if capture_path is not None\n",
            "                strict_affinity=(\n"
            "                    affinity_config.strict if affinity_config.enabled else False\n"
            "                ),\n"
            "                expect_raw_lidar_end=True,\n"
            "            )\n"
            "            if capture_path is not None\n",
            "production sidecar raw-end expectation",
        )

        compile_targets = [str(root / rel) for rel in TARGETS + NEW_FILES if rel.endswith(".py")]
        subprocess.run([sys.executable, "-m", "py_compile", *compile_targets], cwd=root, check=True)
    except BaseException:
        print(f"Apply failed. Temporary rollback backup: {backup}", file=sys.stderr)
        raise

    print("Capture refactor P0 applied.")
    print(f"Inspected base: {EXPECTED_BASE}")
    print(f"Temporary rollback backup: {backup}")
    print("No robot motion was started.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
