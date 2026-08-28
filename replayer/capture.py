"""Immutable, hash-chained capture writer and fail-closed verifier."""

from __future__ import annotations

import json
import os
import queue
import shutil
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from controller.motion_platform_contract import MOTION_PLATFORM_CONTRACT_ID
from replayer.adapters import (
    CONFIG_FILES,
    build_source_manifest,
    layer_boundaries_from_frame,
)
from replayer.contracts import (
    ADAPTER_ID,
    CAPTURE_SCHEMA,
    CAPTURE_SCHEMA_V2,
    CAPTURE_SCHEMA_V21,
    CAPTURE_STATUS_ACTIVE,
    CAPTURE_STATUS_COMPLETE,
    CAPTURE_STATUS_INVALID,
    FRAME_SCHEMA,
    FRAME_SCHEMA_V2,
    FRAME_SCHEMA_V21,
    LAYER_BOUNDARIES_SCHEMA_V21,
    LAYER_BOUNDARY_ORDER_V21,
    LAYER_L6_INTENT_RESOLVER,
    LAYER_L7A_MOTION_GUIDANCE,
    LAYER_L10B_SAFETY_GATE,
    LAYER_L8_MOTION_CONTROLLER,
    LAYER_L9_MOTION_EXECUTOR,
    LAYER_SERVICE_ACTUATION,
    MATCHER_REPLAY_EVIDENCE_REF_SCHEMA,
    MATCHER_REPLAY_EVIDENCE_SCHEMA,
    PIPELINE_ADAPTER_ID,
    PIPELINE_ADAPTER_ID_V21,
    PIPELINE_FRAME_SCHEMA_V2,
    PIPELINE_STAGE_ORDER,
    SOURCE_MANIFEST_SCHEMA,
    SUPPORTED_CAPTURE_SCHEMAS,
    ZERO_HASH,
    ReplayerError,
    canonical_bytes,
    finite_float,
    seal_payload,
    sha256_bytes,
    sha256_file,
    utc_now,
    verify_sealed_payload,
)
from replayer.storage import (
    artifact_inventory,
    capture_dir,
    create_capture_dir,
    make_tree_read_only,
    read_json,
    verify_artifact_inventory,
    write_json_atomic,
)


_STOP = object()
MIN_COMPLETE_FRAMES = 2
MAX_RECORDED_DT_S = 5.0


def _relative_or_absolute(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except (OSError, ValueError):
        return str(path.resolve())


def _external_file_lineage(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        return {"path": str(path), "present": False, "sha256": "", "size_bytes": 0}
    return {
        "path": str(path),
        "present": True,
        "sha256": sha256_file(path),
        "size_bytes": path.stat().st_size,
    }


class CaptureRecorder:
    """Bounded non-blocking producer with a dedicated JSONL writer thread."""

    def __init__(
        self,
        *,
        project_root: Path,
        executor_contract: Dict[str, Any],
        pipeline_contract: Dict[str, Any] | None = None,
        data_root: str | Path | None = None,
        capture_id: str | None = None,
        runtime_session_dir: str | Path | None = None,
        queue_size: int = 4096,
        seal_read_only: bool = True,
    ) -> None:
        self.project_root = Path(project_root).resolve()
        self.capture_id, self.capture_path = create_capture_dir(data_root, capture_id=capture_id)
        self.runtime_session_dir = (
            Path(runtime_session_dir).resolve() if str(runtime_session_dir or "").strip() else None
        )
        self.executor_contract = dict(executor_contract)
        if str(self.executor_contract.get("adapter_id", "")) != ADAPTER_ID:
            raise ReplayerError("capture_executor_adapter_mismatch")
        self.pipeline_contract = dict(pipeline_contract or {})
        if self.pipeline_contract:
            pipeline_adapter = str(self.pipeline_contract.get("adapter_id", ""))
            if pipeline_adapter not in {PIPELINE_ADAPTER_ID, PIPELINE_ADAPTER_ID_V21}:
                raise ReplayerError("capture_pipeline_adapter_mismatch")
            if list(self.pipeline_contract.get("stage_order") or []) != list(PIPELINE_STAGE_ORDER):
                raise ReplayerError("capture_pipeline_stage_order_mismatch")
            if (
                pipeline_adapter == PIPELINE_ADAPTER_ID_V21
                and list(self.pipeline_contract.get("layer_order") or [])
                != list(LAYER_BOUNDARY_ORDER_V21)
            ):
                raise ReplayerError("capture_layer_boundary_order_mismatch")
        is_v21 = bool(
            self.pipeline_contract
            and str(self.pipeline_contract.get("adapter_id", ""))
            == PIPELINE_ADAPTER_ID_V21
        )
        self.capture_schema = (
            CAPTURE_SCHEMA_V21
            if is_v21
            else (CAPTURE_SCHEMA_V2 if self.pipeline_contract else CAPTURE_SCHEMA)
        )
        self.frame_schema = (
            FRAME_SCHEMA_V21
            if is_v21
            else (FRAME_SCHEMA_V2 if self.pipeline_contract else FRAME_SCHEMA)
        )
        self.adapter_id = (
            PIPELINE_ADAPTER_ID_V21
            if is_v21
            else (PIPELINE_ADAPTER_ID if self.pipeline_contract else ADAPTER_ID)
        )
        self.production_component = (
            "sealed_motion_platform_layers"
            if self.capture_schema == CAPTURE_SCHEMA_V21
            else (
                "production_motion_command_pipeline"
                if self.pipeline_contract
                else "motion_executor.MotionExecutor"
            )
        )
        self._seal_read_only = bool(seal_read_only)
        self._queue: queue.Queue[Any] = queue.Queue(maxsize=max(16, int(queue_size)))
        self._lock = threading.Lock()
        self._closing = False
        self._closed = False
        self._attempt_count = 0
        self._written_count = 0
        self._dropped_count = 0
        self._writer_errors: List[str] = []
        self._previous_hash = ZERO_HASH
        self._first_mono_ns: Optional[int] = None
        self._last_mono_ns: Optional[int] = None
        self._last_cycle_id: Optional[int] = None
        self._dt_sum = 0.0
        self._dt_min: Optional[float] = None
        self._dt_max: Optional[float] = None
        self._non_monotonic_count = 0
        self._cycle_gap_count = 0
        self._capture_seq_gap_count = 0
        self._last_capture_seq: Optional[int] = None
        self._executor_reset_marker_mode: Optional[bool] = None
        self._executor_reset_count = 0
        self._first_executor_reset_generation: Optional[int] = None
        self._last_executor_reset_generation: Optional[int] = None
        self._started_at_utc = utc_now()
        self._close_timing: Dict[str, Any] = {
            "state": "NOT_STARTED",
            "close_duration_s": None,
            "writer_drain_s": None,
            "manifest_close_s": None,
            "seal_read_only_s": None,
            "queue_depth_at_close": None,
            "frames_flush_fsync_complete": False,
            "terminal_manifest_written": False,
        }

        self._snapshot_configuration()
        write_json_atomic(
            self.capture_path / "source_manifest.json",
            build_source_manifest(
                self.project_root,
                adapter_id=self.adapter_id,
                component=self.production_component,
            ),
        )
        self._write_manifest(status=CAPTURE_STATUS_ACTIVE, reason="capture_in_progress")

        self._thread = threading.Thread(
            target=self._writer_loop,
            name=f"r2b4-replayer-{self.capture_id}",
            daemon=True,
        )
        self._thread.start()

    def _snapshot_configuration(self) -> None:
        for rel in CONFIG_FILES:
            source = self.project_root / rel
            if not source.is_file():
                raise ReplayerError(f"capture_config_missing:{rel}")
            target = self.capture_path / "config" / Path(rel).name
            if Path(rel).name == "speed_map.json":
                runtime_speed_map = dict(
                    (self.executor_contract.get("runtime_bound_config") or {}).get("speed_map") or {}
                )
                try:
                    source_speed_map = json.loads(source.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError) as exc:
                    raise ReplayerError(f"capture_speed_map_read_failed:{exc}") from exc
                if not runtime_speed_map:
                    raise ReplayerError("capture_runtime_speed_map_missing")
                if runtime_speed_map != source_speed_map:
                    text = json.dumps(
                        runtime_speed_map,
                        ensure_ascii=False,
                        indent=2,
                        sort_keys=True,
                        allow_nan=False,
                    ) + "\n"
                    with target.open("x", encoding="utf-8") as dst:
                        dst.write(text)
                        dst.flush()
                        os.fsync(dst.fileno())
                    continue
            with source.open("rb") as src, target.open("xb") as dst:
                shutil.copyfileobj(src, dst)
                dst.flush()
                os.fsync(dst.fileno())

    def _runtime_lineage(self) -> Dict[str, Any]:
        if self.runtime_session_dir is None:
            return {
                "session_dir": "",
                "session_manifest": {"present": False, "path": "", "sha256": "", "size_bytes": 0},
                "runtime_summary": {"present": False, "path": "", "sha256": "", "size_bytes": 0},
            }
        return {
            "session_dir": _relative_or_absolute(self.runtime_session_dir, self.project_root),
            "session_manifest": _external_file_lineage(self.runtime_session_dir / "session_manifest.json"),
            "runtime_summary": _external_file_lineage(self.runtime_session_dir / "runtime" / "summary.json"),
        }

    def _timing_summary(self) -> Dict[str, Any]:
        count = int(self._written_count)
        duration_s = 0.0
        if self._first_mono_ns is not None and self._last_mono_ns is not None:
            duration_s = max(0.0, (self._last_mono_ns - self._first_mono_ns) / 1_000_000_000.0)
        summary = {
            "frame_count": count,
            "first_monotonic_ns": self._first_mono_ns,
            "last_monotonic_ns": self._last_mono_ns,
            "duration_s": round(duration_s, 12),
            "dt_min_s": None if self._dt_min is None else round(self._dt_min, 12),
            "dt_max_s": None if self._dt_max is None else round(self._dt_max, 12),
            "dt_mean_s": None if count == 0 else round(self._dt_sum / count, 12),
            "non_monotonic_count": int(self._non_monotonic_count),
            "cycle_gap_count": int(self._cycle_gap_count),
            "capture_seq_gap_count": int(self._capture_seq_gap_count),
        }
        if self._executor_reset_marker_mode:
            summary.update(
                {
                    "executor_reset_count": int(self._executor_reset_count),
                    "first_executor_reset_generation": self._first_executor_reset_generation,
                    "last_executor_reset_generation": self._last_executor_reset_generation,
                }
            )
        return summary

    def _manifest_payload(self, *, status: str, reason: str) -> Dict[str, Any]:
        inventory = (
            artifact_inventory(self.capture_path, exclude=("capture_manifest.json",))
            if status != CAPTURE_STATUS_ACTIVE
            else []
        )
        payload = {
            "schema": self.capture_schema,
            "capture_id": self.capture_id,
            "status": str(status),
            "status_reason": str(reason),
            "immutable_reference": status != CAPTURE_STATUS_ACTIVE,
            "started_at_utc": self._started_at_utc,
            "closed_at_utc": utc_now() if status != CAPTURE_STATUS_ACTIVE else None,
            "adapter_id": self.adapter_id,
            "production_component": self.production_component,
            "executor_contract": self.executor_contract,
            "config_scope": list(CONFIG_FILES),
            "source_manifest_path": "source_manifest.json",
            "frames_path": "frames.jsonl",
            "runtime_lineage": self._runtime_lineage(),
            "capture_attempt_count": int(self._attempt_count),
            "frame_count": int(self._written_count),
            "dropped_frame_count": int(self._dropped_count),
            "writer_errors": list(self._writer_errors),
            "frame_chain_head": self._previous_hash,
            "timing": self._timing_summary(),
            "artifact_integrity": inventory,
            "acceptance_contract": {
                "minimum_frames": MIN_COMPLETE_FRAMES,
                "dropped_frames_must_equal": 0,
                "writer_errors_must_equal": [],
                "monotonic_timestamps_required": True,
                "contiguous_capture_sequence_required": True,
                "contiguous_runtime_cycle_ids_required": True,
                "executor_reset_boundaries_replayed_when_present": True,
                "valid_capture_required_for_match": True,
            },
        }
        if self.pipeline_contract:
            payload["pipeline_contract"] = self.pipeline_contract
            payload["acceptance_contract"].update(
                {
                    "complete_pipeline_stage_chain_required": True,
                    "first_divergence_required_for_mismatch": True,
                    "physical_plant_model_required": False,
                }
            )
        if self.capture_schema == CAPTURE_SCHEMA_V21:
            payload["acceptance_contract"].update(
                {
                    "sealed_layer_boundaries_required": list(
                        LAYER_BOUNDARY_ORDER_V21
                    ),
                    "partial_replay_prefix_warmup_required": True,
                    "legacy_semantic_pipeline_replay_required": False,
                }
            )
        return seal_payload(payload, "manifest_sha256")

    def _write_manifest(self, *, status: str, reason: str) -> None:
        write_json_atomic(
            self.capture_path / "capture_manifest.json",
            self._manifest_payload(status=status, reason=reason),
        )

    def record(self, frame: Dict[str, Any]) -> bool:
        """Queue one frame without waiting; a full queue invalidates the capture."""
        with self._lock:
            if self._closing or self._closed:
                return False
            self._attempt_count += 1
            capture_seq = self._attempt_count
        item = dict(frame)
        item["schema"] = self.frame_schema
        item["capture_seq"] = int(capture_seq)
        if self.capture_schema == CAPTURE_SCHEMA_V21:
            try:
                item["layer_boundaries"] = layer_boundaries_from_frame(item)
            except Exception as exc:
                with self._lock:
                    self._writer_errors.append(
                        f"layer_boundary_capture_failed:{type(exc).__name__}:{exc}"
                    )
                return False
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            with self._lock:
                self._dropped_count += 1
            return False

    def mark_invalid(self, reason: str) -> None:
        normalized = str(reason or "capture_invalidated").strip() or "capture_invalidated"
        with self._lock:
            self._writer_errors.append(normalized)

    def _observe_timing(self, record: Dict[str, Any]) -> None:
        mono_ns = int(record["monotonic_ns"])
        cycle_id = int(record["cycle_id"])
        capture_seq = int(record["capture_seq"])
        dt_s = finite_float(record["dt_s"], field="dt_s")
        if dt_s <= 0.0 or dt_s > MAX_RECORDED_DT_S:
            raise ReplayerError(f"dt_out_of_range:{dt_s}")
        marker_present = "executor_reset_generation" in record
        if self._executor_reset_marker_mode is None:
            self._executor_reset_marker_mode = marker_present
        elif marker_present != self._executor_reset_marker_mode:
            raise ReplayerError("executor_reset_marker_presence_changed")
        if marker_present:
            raw_generation = record["executor_reset_generation"]
            if isinstance(raw_generation, bool):
                raise ReplayerError("executor_reset_generation_invalid")
            reset_generation = int(raw_generation)
            if reset_generation < 0:
                raise ReplayerError("executor_reset_generation_invalid")
            if self._first_executor_reset_generation is None:
                self._first_executor_reset_generation = reset_generation
            previous_generation = self._last_executor_reset_generation
            if previous_generation is not None and reset_generation < previous_generation:
                raise ReplayerError("executor_reset_generation_decreased")
            if previous_generation is not None and reset_generation != previous_generation:
                self._executor_reset_count += 1
            self._last_executor_reset_generation = reset_generation
        if self._first_mono_ns is None:
            self._first_mono_ns = mono_ns
        if self._last_mono_ns is not None and mono_ns <= self._last_mono_ns:
            self._non_monotonic_count += 1
        if self._last_cycle_id is not None and cycle_id != self._last_cycle_id + 1:
            self._cycle_gap_count += 1
        if self._last_capture_seq is not None and capture_seq != self._last_capture_seq + 1:
            self._capture_seq_gap_count += 1
        self._last_mono_ns = mono_ns
        self._last_cycle_id = cycle_id
        self._last_capture_seq = capture_seq
        self._dt_sum += dt_s
        self._dt_min = dt_s if self._dt_min is None else min(self._dt_min, dt_s)
        self._dt_max = dt_s if self._dt_max is None else max(self._dt_max, dt_s)

    def _writer_loop(self) -> None:
        frames_path = self.capture_path / "frames.jsonl"
        try:
            with frames_path.open("x", encoding="utf-8") as handle:
                while True:
                    item = self._queue.get()
                    try:
                        if item is _STOP:
                            break
                        record = dict(item)
                        self._observe_timing(record)
                        record["prev_hash"] = self._previous_hash
                        record["frame_hash"] = sha256_bytes(canonical_bytes(record))
                        line = json.dumps(
                            record,
                            ensure_ascii=False,
                            sort_keys=True,
                            separators=(",", ":"),
                            allow_nan=False,
                        )
                        handle.write(line + "\n")
                        self._previous_hash = str(record["frame_hash"])
                        self._written_count += 1
                        if self._written_count % 50 == 0:
                            handle.flush()
                    except Exception as exc:
                        self._writer_errors.append(f"frame_write_failed:{type(exc).__name__}:{exc}")
                    finally:
                        self._queue.task_done()
                handle.flush()
                os.fsync(handle.fileno())
        except Exception as exc:
            self._writer_errors.append(f"writer_failed:{type(exc).__name__}:{exc}")

    def close(
        self,
        *,
        timeout_s: float = 10.0,
        invalid_reason: str = "",
    ) -> Dict[str, Any]:
        close_started = time.monotonic()
        with self._lock:
            if self._closed:
                return self.status()
            self._closing = True
        close_timeout_s = max(0.5, float(timeout_s))
        queue_depth_at_close = int(self._queue.qsize())
        self._close_timing = {
            "state": "DRAINING",
            "close_duration_s": None,
            "writer_drain_s": None,
            "manifest_close_s": None,
            "seal_read_only_s": None,
            "queue_depth_at_close": queue_depth_at_close,
            "close_timeout_s": close_timeout_s,
            "frames_flush_fsync_complete": False,
            "terminal_manifest_written": False,
        }
        deadline = close_started + close_timeout_s
        inserted = False
        while time.monotonic() < deadline and not inserted:
            try:
                self._queue.put(_STOP, timeout=min(0.25, max(0.01, deadline - time.monotonic())))
                inserted = True
            except queue.Full:
                continue
        if inserted:
            self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if not inserted or self._thread.is_alive():
            self._writer_errors.append("writer_close_timeout")
            self._close_timing.update(
                {
                    "state": "TIMEOUT",
                    "close_duration_s": round(time.monotonic() - close_started, 6),
                    "writer_drain_s": round(time.monotonic() - close_started, 6),
                }
            )
            # Deliberately leave ACTIVE: an interrupted capture can never verify as complete.
            return self.status()

        writer_drained_at = time.monotonic()
        self._close_timing.update(
            {
                "state": "FINALIZING_MANIFEST",
                "writer_drain_s": round(writer_drained_at - close_started, 6),
                "frames_flush_fsync_complete": True,
            }
        )

        invalid_reasons: List[str] = []
        if str(invalid_reason or "").strip():
            invalid_reasons.append(str(invalid_reason).strip())
        if self._written_count < MIN_COMPLETE_FRAMES:
            invalid_reasons.append("insufficient_frames")
        if self._dropped_count:
            invalid_reasons.append("dropped_frames")
        if self._writer_errors:
            invalid_reasons.append("writer_errors")
        if self._attempt_count != self._written_count:
            invalid_reasons.append("attempt_count_mismatch")
        timing = self._timing_summary()
        if timing["non_monotonic_count"]:
            invalid_reasons.append("non_monotonic_timestamps")
        if timing["cycle_gap_count"]:
            invalid_reasons.append("runtime_cycle_gap")
        if timing["capture_seq_gap_count"]:
            invalid_reasons.append("capture_sequence_gap")

        status = CAPTURE_STATUS_INVALID if invalid_reasons else CAPTURE_STATUS_COMPLETE
        reason = ",".join(invalid_reasons) if invalid_reasons else "capture_complete"
        manifest_started = time.monotonic()
        self._write_manifest(status=status, reason=reason)
        manifest_finished = time.monotonic()
        self._close_timing.update(
            {
                "state": "SEALING",
                "manifest_close_s": round(manifest_finished - manifest_started, 6),
                "terminal_manifest_written": True,
            }
        )
        with self._lock:
            self._closed = True
        seal_started = time.monotonic()
        if self._seal_read_only:
            make_tree_read_only(self.capture_path)
        close_finished = time.monotonic()
        self._close_timing.update(
            {
                "state": status,
                "seal_read_only_s": round(close_finished - seal_started, 6),
                "close_duration_s": round(close_finished - close_started, 6),
            }
        )
        return self.status()

    def status(self) -> Dict[str, Any]:
        return {
            "capture_id": self.capture_id,
            "capture_path": str(self.capture_path),
            "attempt_count": int(self._attempt_count),
            "frame_count": int(self._written_count),
            "dropped_frame_count": int(self._dropped_count),
            "writer_errors": list(self._writer_errors),
            "closing": bool(self._closing),
            "closed": bool(self._closed),
            "close_timing": dict(self._close_timing),
        }

    def __enter__(self) -> "CaptureRecorder":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close(invalid_reason="capture_context_exception" if exc_type is not None else "")


def _pipeline_frame_errors(record: Dict[str, Any], line_no: int) -> List[str]:
    errors: List[str] = []
    pipeline = record.get("pipeline")
    if not isinstance(pipeline, dict):
        return [f"pipeline_frame_missing:{line_no}"]
    if str(pipeline.get("schema", "")) != PIPELINE_FRAME_SCHEMA_V2:
        errors.append(f"pipeline_frame_schema_invalid:{line_no}")
    if list(pipeline.get("stage_order") or []) != list(PIPELINE_STAGE_ORDER):
        errors.append(f"pipeline_stage_order_invalid:{line_no}")
    stages = pipeline.get("stages")
    if not isinstance(stages, dict):
        return errors + [f"pipeline_stages_missing:{line_no}"]
    if set(stages) != set(PIPELINE_STAGE_ORDER):
        errors.append(f"pipeline_stage_set_invalid:{line_no}")
    for stage_name in PIPELINE_STAGE_ORDER:
        stage = stages.get(stage_name)
        if not isinstance(stage, dict):
            errors.append(f"pipeline_stage_missing:{line_no}:{stage_name}")
            continue
        if not isinstance(stage.get("input"), dict):
            errors.append(f"pipeline_stage_input_missing:{line_no}:{stage_name}")
        if not isinstance(stage.get("recorded_output"), dict):
            errors.append(f"pipeline_stage_output_missing:{line_no}:{stage_name}")
    executor_stage = dict(stages.get("motion_executor") or {})
    if dict(executor_stage.get("input") or {}) != dict(record.get("executor_call") or {}):
        errors.append(f"pipeline_executor_input_lineage_mismatch:{line_no}")
    if dict(executor_stage.get("recorded_output") or {}) != dict(
        record.get("recorded_executor_output") or {}
    ):
        errors.append(f"pipeline_executor_output_lineage_mismatch:{line_no}")
    pwm_stage = dict(stages.get("pwm") or {})
    pwm_output = dict(pwm_stage.get("recorded_output") or {})
    executor_output = dict(record.get("recorded_executor_output") or {})
    try:
        if (
            float(pwm_output["pwm_l"]) != float(executor_output["pwm_l"])
            or float(pwm_output["pwm_r"]) != float(executor_output["pwm_r"])
        ):
            errors.append(f"pipeline_pwm_lineage_mismatch:{line_no}")
    except (KeyError, TypeError, ValueError):
        errors.append(f"pipeline_pwm_output_invalid:{line_no}")
    plant = pipeline.get("plant")
    if not isinstance(plant, dict) or str(plant.get("adapter_id", "")) != "NONE":
        errors.append(f"pipeline_plant_boundary_invalid:{line_no}")
    return errors


def _layer_boundary_frame_errors(record: Dict[str, Any], line_no: int) -> List[str]:
    errors: List[str] = []
    boundaries = record.get("layer_boundaries")
    if not isinstance(boundaries, dict):
        return [f"layer_boundaries_missing:{line_no}"]
    if str(boundaries.get("schema", "")) != LAYER_BOUNDARIES_SCHEMA_V21:
        errors.append(f"layer_boundaries_schema_invalid:{line_no}")
    if list(boundaries.get("layer_order") or []) != list(LAYER_BOUNDARY_ORDER_V21):
        errors.append(f"layer_boundary_order_invalid:{line_no}")
    layers = boundaries.get("layers")
    if not isinstance(layers, dict):
        return errors + [f"layer_boundaries_layers_missing:{line_no}"]
    if set(layers) != set(LAYER_BOUNDARY_ORDER_V21):
        errors.append(f"layer_boundary_set_invalid:{line_no}")
    for layer_name in LAYER_BOUNDARY_ORDER_V21:
        boundary = layers.get(layer_name)
        if not isinstance(boundary, dict):
            errors.append(f"layer_boundary_missing:{line_no}:{layer_name}")
            continue
        if not isinstance(boundary.get("available"), bool):
            errors.append(f"layer_boundary_availability_invalid:{line_no}:{layer_name}")
        expected_replayable = layer_name != LAYER_L10B_SAFETY_GATE
        if boundary.get("replayable") is not expected_replayable:
            errors.append(f"layer_boundary_replayability_invalid:{line_no}:{layer_name}")
        if not isinstance(boundary.get("input"), dict):
            errors.append(f"layer_boundary_input_missing:{line_no}:{layer_name}")
        if not isinstance(boundary.get("recorded_output"), dict):
            errors.append(f"layer_boundary_output_missing:{line_no}:{layer_name}")

    l6 = dict(layers.get(LAYER_L6_INTENT_RESOLVER) or {})
    l7 = dict(layers.get(LAYER_L7A_MOTION_GUIDANCE) or {})
    l8 = dict(layers.get(LAYER_L8_MOTION_CONTROLLER) or {})
    l9 = dict(layers.get(LAYER_L9_MOTION_EXECUTOR) or {})
    service = dict(layers.get(LAYER_SERVICE_ACTUATION) or {})
    safety = dict(layers.get(LAYER_L10B_SAFETY_GATE) or {})
    if l8.get("available") is not True:
        errors.append(f"l8_boundary_unavailable:{line_no}")
    if l6.get("available") is not True:
        errors.append(f"l6_boundary_unavailable:{line_no}")
    if l7.get("available") is not True:
        errors.append(f"l7a_boundary_unavailable:{line_no}")
    normal_available = l9.get("available") is True
    service_available = service.get("available") is True
    if normal_available == service_available:
        errors.append(f"actuation_boundary_cardinality_invalid:{line_no}")
    active_actuation = l9 if normal_available else service
    if dict(active_actuation.get("input") or {}) != dict(record.get("executor_call") or {}):
        errors.append(f"actuation_boundary_input_lineage_mismatch:{line_no}")
    safety_input = dict(safety.get("input") or {})
    if dict(safety_input.get("candidate_output") or {}) != dict(
        active_actuation.get("recorded_output") or {}
    ):
        errors.append(f"safety_candidate_lineage_mismatch:{line_no}")
    if dict(safety_input.get("safety_lineage") or {}) != dict(
        record.get("safety_lineage") or {}
    ):
        errors.append(f"safety_decision_lineage_mismatch:{line_no}")
    if dict(safety.get("recorded_output") or {}) != dict(record.get("final_output") or {}):
        errors.append(f"safety_output_lineage_mismatch:{line_no}")

    def exact_keys(payload: Any, expected: set[str], label: str) -> None:
        if not isinstance(payload, dict) or set(payload) != expected:
            errors.append(f"layer_contract_fields_invalid:{line_no}:{label}")

    l8_input = dict(l8.get("input") or {})
    l6_input = dict(l6.get("input") or {})
    l6_output = dict(l6.get("recorded_output") or {})
    l7_input = dict(l7.get("input") or {})
    l7_output = dict(l7.get("recorded_output") or {})
    exact_keys(
        l6_input,
        {"requested_motion", "resolver", "cycle_context"},
        "l6_input",
    )
    exact_keys(
        l6_output,
        {"resolved_motion", "resolution_status", "resolved_intent"},
        "l6_output",
    )
    exact_keys(
        l7_input,
        {
            "resolved_intent",
            "pose",
            "world",
            "cycle_context",
            "drive_capabilities",
            "executed_left_mps",
            "executed_right_mps",
            "actual_linear_mps",
            "actual_angular_dps",
        },
        "l7a_input",
    )
    exact_keys(
        l7_output,
        {"physical_command", "diagnostics"},
        "l7a_output",
    )
    if dict(l6_output.get("resolved_intent") or {}) != dict(
        l7_input.get("resolved_intent") or {}
    ):
        errors.append(f"l6_l7a_intent_lineage_mismatch:{line_no}")
    if dict(l7_output.get("physical_command") or {}) != dict(
        l8_input.get("physical_command") or {}
    ):
        errors.append(f"l7a_l8_physical_lineage_mismatch:{line_no}")
    exact_keys(
        l8_input,
        {"cycle_context", "physical_command", "motion_envelope", "drive_capabilities"},
        "l8_input",
    )
    exact_keys(
        l8_input.get("cycle_context"),
        {
            "cycle_id",
            "monotonic_time",
            "dt_observed_s",
            "dt_control_s",
            "timing_valid",
            "timing_reason",
        },
        "l8_cycle_context",
    )
    exact_keys(
        l8_input.get("physical_command"),
        {
            "contract_id",
            "physical_command_id",
            "resolved_id",
            "cycle_id",
            "valid_until_monotonic",
            "physical_mode",
            "v_mps",
            "omega_rad_s",
            "left_mps",
            "right_mps",
            "guidance_reason",
            "trace_metadata",
        },
        "l8_physical_command",
    )
    exact_keys(
        l8_input.get("motion_envelope"),
        {
            "cycle_id",
            "physical_command_id",
            "stop_required",
            "stop_reason",
            "max_abs_v_mps",
            "max_abs_omega_rad_s",
            "max_abs_wheel_mps",
            "max_wheel_accel_mps2",
            "max_wheel_decel_mps2",
            "capability_version",
        },
        "l8_motion_envelope",
    )
    exact_keys(
        l8_input.get("drive_capabilities"),
        {
            "track_width_m",
            "calibrated_wheel_min_mps",
            "calibrated_wheel_max_mps",
            "max_wheel_accel_mps2",
            "max_wheel_decel_mps2",
            "capability_version",
        },
        "l8_drive_capabilities",
    )
    l8_output = dict(l8.get("recorded_output") or {})
    exact_keys(
        l8_output,
        {
            "contract_id",
            "wheel_setpoint_id",
            "physical_command_id",
            "resolved_id",
            "cycle_id",
            "left_target_mps",
            "right_target_mps",
            "feasible",
            "reason",
            "applied_limits",
        },
        "l8_output",
    )
    if str(l8_output.get("contract_id", "")) != MOTION_PLATFORM_CONTRACT_ID:
        errors.append(f"l8_output_contract_id_invalid:{line_no}")
    if normal_available:
        l9_input = dict(l9.get("input") or {})
        exact_keys(
            l9_input,
            {"method", "cycle_context", "wheel_setpoint", "wheel_feedback"},
            "l9_input",
        )
        if str(l9_input.get("method", "")) != "compute":
            errors.append(f"l9_method_invalid:{line_no}")
        if dict(l9_input.get("wheel_setpoint") or {}) != l8_output:
            errors.append(f"l8_l9_setpoint_lineage_mismatch:{line_no}")
        exact_keys(
            l9.get("recorded_output"),
            {
                "schema",
                "contract_id",
                "candidate_output_id",
                "wheel_setpoint_id",
                "physical_command_id",
                "resolved_id",
                "cycle_id",
                "left_pwm",
                "right_pwm",
                "output_reason",
            },
            "l9_output",
        )
    if service_available:
        service_input = dict(service.get("input") or {})
        exact_keys(
            service_input,
            {"method", "request", "monotonic_time"},
            "service_input",
        )
        if str(service_input.get("method", "")) != "service_compute":
            errors.append(f"service_method_invalid:{line_no}")
        exact_keys(
            service.get("recorded_output"),
            {"schema", "left_pwm", "right_pwm", "accepted", "reason"},
            "service_output",
        )
    exact_keys(
        safety.get("recorded_output"),
        {"pwm_l", "pwm_r"},
        "l10b_output",
    )
    return errors


def _read_and_verify_frames(
    path: Path,
    *,
    expected_frame_schema: str = FRAME_SCHEMA,
    pipeline_required: bool = False,
    layer_boundaries_required: bool = False,
) -> Dict[str, Any]:
    errors: List[str] = []
    previous_hash = ZERO_HASH
    previous_mono: Optional[int] = None
    previous_cycle: Optional[int] = None
    previous_seq: Optional[int] = None
    frame_count = 0
    dt_sum = 0.0
    dt_min: Optional[float] = None
    dt_max: Optional[float] = None
    first_mono: Optional[int] = None
    last_mono: Optional[int] = None
    reset_marker_mode: Optional[bool] = None
    first_reset_generation: Optional[int] = None
    previous_reset_generation: Optional[int] = None
    reset_count = 0
    supported_methods = {"compute", "service_compute"}
    matcher_evidence_ids = set()
    try:
        with path.open("r", encoding="utf-8") as handle:
            for line_no, raw in enumerate(handle, start=1):
                if not raw.strip():
                    errors.append(f"blank_frame_line:{line_no}")
                    continue
                try:
                    record = json.loads(raw)
                except json.JSONDecodeError:
                    errors.append(f"invalid_frame_json:{line_no}")
                    continue
                if not isinstance(record, dict):
                    errors.append(f"frame_object_required:{line_no}")
                    continue
                claimed_hash = str(record.get("frame_hash", "") or "")
                unsigned = dict(record)
                unsigned.pop("frame_hash", None)
                actual_hash = sha256_bytes(canonical_bytes(unsigned))
                if claimed_hash != actual_hash:
                    errors.append(f"frame_hash_mismatch:{line_no}")
                if str(record.get("prev_hash", "") or "") != previous_hash:
                    errors.append(f"frame_chain_break:{line_no}")
                if str(record.get("schema", "") or "") != str(expected_frame_schema):
                    errors.append(f"frame_schema_invalid:{line_no}")
                if pipeline_required:
                    errors.extend(_pipeline_frame_errors(record, line_no))
                elif layer_boundaries_required:
                    errors.extend(_layer_boundary_frame_errors(record, line_no))
                elif "pipeline" in record:
                    errors.append(f"v1_frame_contains_v2_pipeline:{line_no}")
                matcher_evidence = record.get("matcher_evidence")
                if matcher_evidence is not None:
                    if not isinstance(matcher_evidence, dict):
                        errors.append(f"matcher_evidence_not_object:{line_no}")
                    else:
                        matcher_schema = str(matcher_evidence.get("schema", ""))
                        try:
                            matcher_result_id = int(
                                matcher_evidence.get("matcher_result_id", 0) or 0
                            )
                        except (TypeError, ValueError):
                            matcher_result_id = 0
                        if matcher_result_id <= 0:
                            errors.append(f"matcher_evidence_id_invalid:{line_no}")
                        elif matcher_schema == MATCHER_REPLAY_EVIDENCE_SCHEMA:
                            matcher_evidence_ids.add(matcher_result_id)
                            if not isinstance(matcher_evidence.get("available"), bool):
                                errors.append(
                                    f"matcher_evidence_availability_invalid:{line_no}"
                                )
                            if bool(matcher_evidence.get("available", False)) and (
                                not isinstance(matcher_evidence.get("input"), dict)
                                or not isinstance(
                                    matcher_evidence.get("recorded_output"), dict
                                )
                                or not isinstance(
                                    matcher_evidence.get("map_lineage"), dict
                                )
                            ):
                                errors.append(
                                    f"matcher_evidence_payload_incomplete:{line_no}"
                                )
                        elif matcher_schema == MATCHER_REPLAY_EVIDENCE_REF_SCHEMA:
                            if matcher_result_id not in matcher_evidence_ids:
                                errors.append(
                                    f"matcher_evidence_reference_without_source:{line_no}"
                                )
                        else:
                            errors.append(f"matcher_evidence_schema_invalid:{line_no}")
                try:
                    mono_ns = int(record["monotonic_ns"])
                    cycle_id = int(record["cycle_id"])
                    capture_seq = int(record["capture_seq"])
                    dt_s = finite_float(record["dt_s"], field="dt_s")
                    if dt_s <= 0.0 or dt_s > MAX_RECORDED_DT_S:
                        errors.append(f"frame_dt_out_of_range:{line_no}")
                    marker_present = "executor_reset_generation" in record
                    if reset_marker_mode is None:
                        reset_marker_mode = marker_present
                    elif marker_present != reset_marker_mode:
                        errors.append(f"frame_executor_reset_marker_presence_changed:{line_no}")
                    if marker_present:
                        raw_generation = record["executor_reset_generation"]
                        if isinstance(raw_generation, bool):
                            raise ReplayerError("executor_reset_generation_invalid")
                        reset_generation = int(raw_generation)
                        if reset_generation < 0:
                            raise ReplayerError("executor_reset_generation_invalid")
                        if first_reset_generation is None:
                            first_reset_generation = reset_generation
                        if (
                            previous_reset_generation is not None
                            and reset_generation < previous_reset_generation
                        ):
                            errors.append(f"frame_executor_reset_generation_decreased:{line_no}")
                        if (
                            previous_reset_generation is not None
                            and reset_generation != previous_reset_generation
                        ):
                            reset_count += 1
                        previous_reset_generation = reset_generation
                    if previous_mono is not None and mono_ns <= previous_mono:
                        errors.append(f"frame_time_not_monotonic:{line_no}")
                    if previous_cycle is not None and cycle_id != previous_cycle + 1:
                        errors.append(f"frame_cycle_gap:{line_no}")
                    if previous_seq is not None and capture_seq != previous_seq + 1:
                        errors.append(f"frame_sequence_gap:{line_no}")
                    if previous_seq is None and capture_seq != 1:
                        errors.append("frame_sequence_does_not_start_at_one")
                    method = str(((record.get("executor_call") or {}).get("method", "")) or "")
                    if method not in supported_methods:
                        errors.append(f"frame_method_unsupported:{line_no}:{method or 'MISSING'}")
                    if not isinstance(record.get("recorded_executor_output"), dict):
                        errors.append(f"frame_executor_output_missing:{line_no}")
                    if not isinstance(record.get("final_output"), dict):
                        errors.append(f"frame_final_output_missing:{line_no}")
                    if first_mono is None:
                        first_mono = mono_ns
                    last_mono = mono_ns
                    previous_mono = mono_ns
                    previous_cycle = cycle_id
                    previous_seq = capture_seq
                    dt_sum += dt_s
                    dt_min = dt_s if dt_min is None else min(dt_min, dt_s)
                    dt_max = dt_s if dt_max is None else max(dt_max, dt_s)
                except (KeyError, TypeError, ValueError, ReplayerError) as exc:
                    errors.append(f"frame_contract_invalid:{line_no}:{exc}")
                previous_hash = claimed_hash
                frame_count += 1
    except OSError as exc:
        errors.append(f"frames_read_failed:{exc}")

    duration_s = 0.0 if first_mono is None or last_mono is None else (last_mono - first_mono) / 1e9
    timing = {
        "frame_count": frame_count,
        "first_monotonic_ns": first_mono,
        "last_monotonic_ns": last_mono,
        "duration_s": round(max(0.0, duration_s), 12),
        "dt_min_s": None if dt_min is None else round(dt_min, 12),
        "dt_max_s": None if dt_max is None else round(dt_max, 12),
        "dt_mean_s": None if frame_count == 0 else round(dt_sum / frame_count, 12),
        "non_monotonic_count": sum(1 for error in errors if error.startswith("frame_time_not_monotonic:")),
        "cycle_gap_count": sum(1 for error in errors if error.startswith("frame_cycle_gap:")),
        "capture_seq_gap_count": sum(1 for error in errors if error.startswith("frame_sequence_gap:")),
    }
    if reset_marker_mode:
        timing.update(
            {
                "executor_reset_count": int(reset_count),
                "first_executor_reset_generation": first_reset_generation,
                "last_executor_reset_generation": previous_reset_generation,
            }
        )
    return {"errors": errors, "frame_count": frame_count, "chain_head": previous_hash, "timing": timing}


def _verify_captured_configuration(path: Path, manifest: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    payloads: Dict[str, Dict[str, Any]] = {}
    for rel in CONFIG_FILES:
        name = Path(rel).name
        try:
            payloads[name] = read_json(path / "config" / name)
        except ReplayerError as exc:
            errors.append(str(exc))
    if errors:
        return errors

    contract = dict(manifest.get("executor_contract") or {})
    constructor = dict(contract.get("constructor") or {})
    runtime_bound = dict(contract.get("runtime_bound_config") or {})
    if str(payloads["control_mode.json"].get("control_mode", "")) != "UNIFIED":
        errors.append("captured_control_mode_not_unified")
    if str(contract.get("control_mode", "")) != "UNIFIED":
        errors.append("executor_contract_control_mode_not_unified")
    try:
        finite_float(
            payloads["fizika.json"].get("nyomtav_szelesseg_m"),
            field="captured_track_width",
        )
    except ReplayerError as exc:
        errors.append(str(exc))
    speed_map = payloads["speed_map.json"]
    if str(speed_map.get("schema", "")) != "R2B4_WHEEL_SPEED_MAP_V2":
        errors.append("captured_speed_map_schema_invalid")
    if str(speed_map.get("map_state", "")).strip().upper() != "ACTIVE":
        errors.append("captured_speed_map_not_active")
    if speed_map != dict(runtime_bound.get("speed_map") or {}):
        errors.append("captured_speed_map_runtime_contract_mismatch")
    if str(manifest.get("schema", "")) in {CAPTURE_SCHEMA_V2, CAPTURE_SCHEMA_V21}:
        pipeline_contract = dict(manifest.get("pipeline_contract") or {})
        expected_pipeline_adapter = (
            PIPELINE_ADAPTER_ID_V21
            if str(manifest.get("schema", "")) == CAPTURE_SCHEMA_V21
            else PIPELINE_ADAPTER_ID
        )
        if str(pipeline_contract.get("adapter_id", "")) != expected_pipeline_adapter:
            errors.append("pipeline_contract_adapter_invalid")
        if list(pipeline_contract.get("stage_order") or []) != list(PIPELINE_STAGE_ORDER):
            errors.append("pipeline_contract_stage_order_invalid")
        if (
            str(manifest.get("schema", "")) == CAPTURE_SCHEMA_V21
            and list(pipeline_contract.get("layer_order") or [])
            != list(LAYER_BOUNDARY_ORDER_V21)
        ):
            errors.append("pipeline_contract_layer_order_invalid")
        motion_constructor = dict(
            pipeline_contract.get("motion_controller_constructor") or {}
        )
        if set(motion_constructor) != {"enable_slew"}:
            errors.append("pipeline_motion_controller_constructor_invalid")
    return errors


def verify_capture(capture_id: str, *, data_root: str | Path | None = None) -> Dict[str, Any]:
    path = capture_dir(data_root, capture_id)
    errors: List[str] = []
    manifest: Dict[str, Any] = {}
    if not path.is_dir() or path.is_symlink():
        errors.append("capture_directory_missing_or_symlink")
    else:
        manifest_path = path / "capture_manifest.json"
        if manifest_path.is_symlink():
            errors.append("capture_manifest_symlink_forbidden")
        try:
            manifest = read_json(manifest_path)
        except ReplayerError as exc:
            errors.append(str(exc))

    if manifest:
        manifest_schema = str(manifest.get("schema", ""))
        if manifest_schema not in SUPPORTED_CAPTURE_SCHEMAS:
            errors.append("capture_schema_invalid")
        is_v2 = manifest_schema == CAPTURE_SCHEMA_V2
        is_v21 = manifest_schema == CAPTURE_SCHEMA_V21
        expected_adapter = (
            PIPELINE_ADAPTER_ID_V21
            if is_v21
            else (PIPELINE_ADAPTER_ID if is_v2 else ADAPTER_ID)
        )
        if str(manifest.get("adapter_id", "")) != expected_adapter:
            errors.append("capture_adapter_id_invalid")
        if str(manifest.get("capture_id", "")) != str(capture_id):
            errors.append("capture_id_mismatch")
        if str(manifest.get("status", "")) != CAPTURE_STATUS_COMPLETE:
            errors.append(f"capture_not_complete:{manifest.get('status', 'MISSING')}")
        if manifest.get("immutable_reference") is not True:
            errors.append("capture_not_marked_immutable")
        if not verify_sealed_payload(manifest, "manifest_sha256"):
            errors.append("capture_manifest_hash_invalid")
        errors.extend(
            verify_artifact_inventory(
                path,
                manifest.get("artifact_integrity"),
                exclude=("capture_manifest.json",),
            )
        )

        try:
            source_manifest = read_json(path / str(manifest.get("source_manifest_path", "source_manifest.json")))
            if str(source_manifest.get("schema", "")) != SOURCE_MANIFEST_SCHEMA:
                errors.append("source_manifest_schema_invalid")
            if str(source_manifest.get("adapter_id", "")) != expected_adapter:
                errors.append("source_manifest_adapter_mismatch")
            if str(source_manifest.get("component", "")) != str(
                manifest.get("production_component", "")
            ):
                errors.append("source_manifest_component_mismatch")
            if not verify_sealed_payload(source_manifest, "source_manifest_sha256"):
                errors.append("source_manifest_hash_invalid")
            if source_manifest.get("missing"):
                errors.append("source_manifest_incomplete")
        except ReplayerError as exc:
            errors.append(str(exc))

        expected_config = sorted(Path(rel).name for rel in CONFIG_FILES)
        actual_config = sorted(path.name for path in (path / "config").glob("*.json")) if (path / "config").is_dir() else []
        if actual_config != expected_config:
            errors.append("captured_config_set_invalid")
        else:
            errors.extend(_verify_captured_configuration(path, manifest))

        frame_report = _read_and_verify_frames(
            path / str(manifest.get("frames_path", "frames.jsonl")),
            expected_frame_schema=(
                FRAME_SCHEMA_V21
                if is_v21
                else (FRAME_SCHEMA_V2 if is_v2 else FRAME_SCHEMA)
            ),
            pipeline_required=is_v2,
            layer_boundaries_required=is_v21,
        )
        errors.extend(frame_report["errors"])
        if int(manifest.get("frame_count", -1)) != int(frame_report["frame_count"]):
            errors.append("frame_count_manifest_mismatch")
        if int(manifest.get("capture_attempt_count", -1)) != int(frame_report["frame_count"]):
            errors.append("capture_attempt_count_mismatch")
        if int(manifest.get("dropped_frame_count", -1)) != 0:
            errors.append("capture_contains_dropped_frames")
        if list(manifest.get("writer_errors") or []):
            errors.append("capture_contains_writer_errors")
        if int(frame_report["frame_count"]) < MIN_COMPLETE_FRAMES:
            errors.append("capture_has_insufficient_frames")
        if str(manifest.get("frame_chain_head", "")) != str(frame_report["chain_head"]):
            errors.append("frame_chain_head_mismatch")
        if dict(manifest.get("timing") or {}) != dict(frame_report["timing"]):
            errors.append("timing_summary_mismatch")
    else:
        frame_report = {"frame_count": 0, "chain_head": ZERO_HASH, "timing": {}}

    unique_errors = list(dict.fromkeys(errors))
    valid = not unique_errors
    return {
        "schema": (
            "R2B4_REPLAYER_CAPTURE_VERIFICATION_V2_1"
            if str(manifest.get("schema", "")) == CAPTURE_SCHEMA_V21
            else (
                "R2B4_REPLAYER_CAPTURE_VERIFICATION_V2"
                if str(manifest.get("schema", "")) == CAPTURE_SCHEMA_V2
                else "R2B4_REPLAYER_CAPTURE_VERIFICATION_V1"
            )
        ),
        "capture_id": str(capture_id),
        "capture_path": str(path),
        "valid": valid,
        "status": "VALID" if valid else "INVALID",
        "errors": unique_errors,
        "frame_count": int(frame_report.get("frame_count", 0)),
        "timing": dict(frame_report.get("timing") or {}),
        "manifest": manifest,
        "gates": {
            "manifest_integrity": "PASS" if not any("manifest" in error for error in unique_errors) else "FAIL",
            "artifact_integrity": "PASS" if not any(error.startswith("artifact_") for error in unique_errors) else "FAIL",
            "frame_chain": "PASS" if not any("frame_hash" in error or "frame_chain" in error for error in unique_errors) else "FAIL",
            "completeness": "PASS" if valid else "FAIL",
            "timing": "PASS" if not any("time" in error or "cycle_gap" in error or "dt_" in error for error in unique_errors) else "FAIL",
        },
    }


def inspect_capture(capture_id: str, *, data_root: str | Path | None = None) -> Dict[str, Any]:
    """Read only the sealed manifest for a fast, explicitly non-accepting summary."""
    path = capture_dir(data_root, capture_id)
    errors: List[str] = []
    manifest: Dict[str, Any] = {}
    if not path.is_dir() or path.is_symlink():
        errors.append("capture_directory_missing_or_symlink")
    else:
        try:
            manifest = read_json(path / "capture_manifest.json")
        except ReplayerError as exc:
            errors.append(str(exc))
    schema = str(manifest.get("schema", "") or "")
    manifest_hash_valid = bool(
        manifest and verify_sealed_payload(manifest, "manifest_sha256")
    )
    if manifest and schema not in SUPPORTED_CAPTURE_SCHEMAS:
        errors.append("capture_schema_invalid")
    if manifest and not manifest_hash_valid:
        errors.append("capture_manifest_hash_invalid")
    pipeline_contract = dict(manifest.get("pipeline_contract") or {})
    available_layers = (
        list(pipeline_contract.get("layer_order") or [])
        if schema == CAPTURE_SCHEMA_V21
        else list(pipeline_contract.get("stage_order") or [])
    )
    timing = dict(manifest.get("timing") or {})
    return {
        "schema": "R2B4_REPLAYER_INSPECT_V2_1",
        "capture_id": str(capture_id),
        "capture_path": str(path),
        "capture_schema": schema,
        "capture_status": str(manifest.get("status", "MISSING") or "MISSING"),
        "status_reason": str(manifest.get("status_reason", "") or ""),
        "frame_count": int(manifest.get("frame_count", 0) or 0),
        "dropped_frame_count": int(manifest.get("dropped_frame_count", 0) or 0),
        "writer_error_count": len(list(manifest.get("writer_errors") or [])),
        "timing": {
            "first_monotonic_ns": timing.get("first_monotonic_ns"),
            "last_monotonic_ns": timing.get("last_monotonic_ns"),
            "duration_s": timing.get("duration_s"),
            "dt_mean_s": timing.get("dt_mean_s"),
        },
        "available_layers": available_layers,
        "replayable_layers": list(pipeline_contract.get("replayable_layers") or []),
        "manifest_integrity": "VALID" if manifest_hash_valid and not errors else "INVALID",
        "errors": list(dict.fromkeys(errors)),
        "verification_scope": "MANIFEST_ONLY",
        "replay_acceptance": "NOT_EVALUATED_USE_VERIFY_OR_REPLAY",
    }
