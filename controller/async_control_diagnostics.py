#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Latest-only asynchronous publisher for non-critical control diagnostics."""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

from controller.runtime_affinity import apply_active_service_thread_affinity
from log.control_snapshot import compact_control_snapshot_sections
from log.unified_logger import (
    CHANNEL_CONTROL,
    get_unified_logger,
    write_ekf_diag,
    write_encoder_diag,
    write_imu_diag,
    write_timing,
)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


class AsyncControlDiagnosticsPublisher:
    """Run telemetry and structured diagnostic publication off the 50 Hz thread."""

    SCHEMA = "ASYNC_CONTROL_DIAGNOSTICS_PUBLISHER_V1"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pending = None
        self._started = False
        self._submitted = 0
        self._processed = 0
        self._failed = 0
        self._dropped = 0
        self._lock_miss = 0
        self._last_error = ""
        self._last_processing_ms = 0.0
        self._max_processing_ms = 0.0
        self._last_telemetry_ts = 0.0
        self._last_timing_diag_ts = 0.0
        self._last_ekf_diag_ts = 0.0
        self._last_encoder_diag_ts = 0.0
        self._last_imu_diag_ts = 0.0
        self._last_snapshot_min_ts = 0.0
        self._last_snapshot_full_ts = 0.0
        self._last_snapshot_state = None
        self._last_housekeeping_ts = 0.0

    def submit(self, ctrl, sample: Dict[str, Any]) -> bool:
        acquired = self._lock.acquire(blocking=False)
        if not acquired:
            self._lock_miss += 1
            return False
        try:
            if self._pending is not None:
                self._dropped += 1
            self._pending = (ctrl, sample)
            self._submitted += 1
            if not self._started:
                self._thread = threading.Thread(
                    target=self._worker,
                    name="r2b4-control-diagnostics",
                    daemon=True,
                )
                self._thread.start()
                self._started = True
            stats = self._stats_locked()
        finally:
            self._lock.release()
        try:
            ctrl.control_diagnostics_publisher_status = stats
        except Exception:
            pass
        self._wake.set()
        return True

    def stop(self, *, timeout_s: float = 1.0) -> None:
        self._stop.set()
        self._wake.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.01, float(timeout_s)))

    def status(self) -> dict:
        acquired = self._lock.acquire(blocking=False)
        if not acquired:
            return {
                "schema": self.SCHEMA,
                "mode": "latest_only_async_control_diagnostics",
                "latest_only": True,
                "queue_capacity": 1,
                "status_lock_busy": True,
                "lock_miss_count": int(self._lock_miss),
            }
        try:
            return self._stats_locked()
        finally:
            self._lock.release()

    def _stats_locked(self) -> dict:
        thread = self._thread
        return {
            "schema": self.SCHEMA,
            "mode": "latest_only_async_control_diagnostics",
            "thread_started": bool(self._started),
            "running": bool(thread is not None and thread.is_alive()),
            "latest_only": True,
            "queue_capacity": 1,
            "pending_count": int(1 if self._pending is not None else 0),
            "submitted": int(self._submitted),
            "processed": int(self._processed),
            "failed": int(self._failed),
            "dropped_superseded": int(self._dropped),
            "lock_miss_count": int(self._lock_miss),
            "last_error": str(self._last_error),
            "last_processing_ms": float(self._last_processing_ms),
            "max_processing_ms": float(self._max_processing_ms),
        }

    def _take_latest(self):
        with self._lock:
            item = self._pending
            self._pending = None
            return item

    def _worker(self) -> None:
        apply_active_service_thread_affinity(role="status_writer")
        while not self._stop.is_set():
            self._wake.wait(timeout=1.0)
            self._wake.clear()
            item = self._take_latest()
            if item is None:
                continue
            ctrl, sample = item
            start = time.perf_counter()
            ok = False
            error = ""
            try:
                self._publish(ctrl, dict(sample or {}))
                ok = True
            except Exception as exc:
                error = str(exc)
            dt_ms = max(0.0, (time.perf_counter() - start) * 1000.0)
            with self._lock:
                self._last_processing_ms = float(dt_ms)
                self._max_processing_ms = max(float(self._max_processing_ms), float(dt_ms))
                if ok:
                    self._processed += 1
                    self._last_error = ""
                else:
                    self._failed += 1
                    self._last_error = error or "control_diagnostics_publish_failed"
                stats = self._stats_locked()
            try:
                ctrl.control_diagnostics_publisher_status = stats
            except Exception:
                pass

    def _publish(self, ctrl, sample: Dict[str, Any]) -> None:
        now = _safe_float(sample.get("now"), time.perf_counter())
        cycle_id = _safe_int(sample.get("cycle_id"), 0)
        ekf_state = sample.get("ekf_state") if isinstance(sample.get("ekf_state"), dict) else {}
        l_sum = sample.get("l_sum") if isinstance(sample.get("l_sum"), dict) else {}
        loop_result = sample.get("loop_result") if isinstance(sample.get("loop_result"), dict) else {}
        safety_state = sample.get("safety_state") if isinstance(sample.get("safety_state"), dict) else {}
        motion_command_semantics = (
            sample.get("motion_command_semantics")
            if isinstance(sample.get("motion_command_semantics"), dict)
            else {}
        )
        state_name = (
            ctrl.sm.get_current_state_name()
            if getattr(ctrl, "sm", None)
            else str(sample.get("state_name") or "NONE")
        )
        pwm_l = _safe_float(sample.get("pwm_l"), 0.0)
        pwm_r = _safe_float(sample.get("pwm_r"), 0.0)
        v_l_raw = _safe_float(sample.get("v_l_raw"), 0.0)
        v_r_raw = _safe_float(sample.get("v_r_raw"), 0.0)
        self._publish_control_snapshots(
            ctrl,
            now=now,
            state_name=state_name,
            ekf_state=ekf_state,
            motion_command_semantics=motion_command_semantics,
            safety_state=safety_state,
            sample=sample,
            loop_result=loop_result,
            pwm_l=pwm_l,
            pwm_r=pwm_r,
            v_l_raw=v_l_raw,
            v_r_raw=v_r_raw,
        )
        self._publish_periodic_telemetry(
            ctrl,
            now=now,
            state_name=state_name,
            ekf_state=ekf_state,
            l_sum=l_sum,
            safety_state=safety_state,
            sample=sample,
            pwm_l=pwm_l,
            pwm_r=pwm_r,
            v_l_raw=v_l_raw,
            v_r_raw=v_r_raw,
        )
        self._publish_structured_diagnostics(
            ctrl,
            now=now,
            cycle_id=cycle_id,
            ekf_state=ekf_state,
            loop_result=loop_result,
            sample=sample,
            pwm_l=pwm_l,
            pwm_r=pwm_r,
            v_l_raw=v_l_raw,
            v_r_raw=v_r_raw,
        )
        self._publish_housekeeping(ctrl, now=now)

    def _publish_periodic_telemetry(
        self,
        ctrl,
        *,
        now: float,
        state_name: str,
        ekf_state: Dict[str, Any],
        l_sum: Dict[str, Any],
        safety_state: Dict[str, Any],
        sample: Dict[str, Any],
        pwm_l: float,
        pwm_r: float,
        v_l_raw: float,
        v_r_raw: float,
    ) -> None:
        log_hz = max(0.1, _safe_float(getattr(ctrl, "log_hz", 5.0), 5.0))
        if (now - self._last_telemetry_ts) <= (1.0 / log_hz):
            return
        self._last_telemetry_ts = float(now)
        telemetry_kw = {}
        motion_src = getattr(ctrl, "motion_command_source", None)
        if motion_src:
            telemetry_kw["motion_src"] = motion_src
        telemetry_kw["control_mode"] = getattr(ctrl, "control_mode", None)
        telemetry_kw["safety_allow"] = bool((safety_state or {}).get("allow", True))
        telemetry_kw["safety_reason"] = (safety_state or {}).get("reason", "OK")
        telemetry_kw["encoder_enabled"] = getattr(ctrl, "encoder_enabled", None)
        telemetry_kw["encoder_gain"] = getattr(ctrl, "encoder_usage_gain", None)
        telemetry_kw["quality_state"] = str(
            (getattr(ctrl, "motion_quality_status", {}) or {}).get("quality_state", "N/A")
        )
        telemetry_kw["side_ratio"] = (
            (getattr(ctrl, "encoder_reliability_status", {}) or {}).get("side_ratio_lr_abs")
        )
        telemetry_kw["stop_residual"] = (
            (getattr(ctrl, "motion_quality_status", {}) or {}).get("stop_residual_mps")
        )
        telemetry_kw["vel_stability"] = (
            (getattr(ctrl, "motion_quality_status", {}) or {}).get("velocity_stability_mps")
        )
        telemetry_kw["estimator_confidence"] = getattr(ctrl, "estimator_confidence", None)
        logger = getattr(ctrl, "logger", None)
        if logger is None:
            return
        logger.log_telemetry(
            _safe_float(sample.get("elapsed"), 0.0),
            state_name,
            ekf_state.get("x"),
            ekf_state.get("y"),
            ekf_state.get("theta_deg"),
            l_sum,
            v_l_raw,
            v_r_raw,
            pwm_l,
            pwm_r,
            getattr(ctrl, "speed_level", 0),
            getattr(ctrl, "v_target", 0.0),
            getattr(ctrl, "v_cmd", 0.0),
            getattr(ctrl, "omega_target", 0.0),
            getattr(ctrl, "turn_level", 0),
            **telemetry_kw,
        )

    def _publish_control_snapshots(
        self,
        ctrl,
        *,
        now: float,
        state_name: str,
        ekf_state: Dict[str, Any],
        motion_command_semantics: Dict[str, Any],
        safety_state: Dict[str, Any],
        sample: Dict[str, Any],
        loop_result: Dict[str, Any],
        pwm_l: float,
        pwm_r: float,
        v_l_raw: float,
        v_r_raw: float,
    ) -> None:
        ul = get_unified_logger()
        if ul is None:
            return
        command_type = str(motion_command_semantics.get("command_type", "") or "")
        final_pwm_zero_reason = str(sample.get("final_pwm_zero_reason") or "")
        snapshot_state_key = (
            str(state_name),
            str(getattr(ctrl, "motion_command_source", "") or ""),
            command_type,
            final_pwm_zero_reason,
        )
        emit_min = snapshot_state_key != self._last_snapshot_state or (
            now - self._last_snapshot_min_ts
        ) >= _safe_float(getattr(ctrl, "_control_snapshot_min_interval_s", 0.25), 0.25)
        if emit_min:
            ul.log_event(
                CHANNEL_CONTROL,
                "control_loop",
                "control_snapshot_min",
                {
                    "state": state_name,
                    "x": ekf_state.get("x"),
                    "y": ekf_state.get("y"),
                    "theta_deg": ekf_state.get("theta_deg"),
                    "v_target": getattr(ctrl, "v_target", 0.0),
                    "v_cmd": getattr(ctrl, "v_cmd", 0.0),
                    "omega_target": getattr(ctrl, "omega_target", 0.0),
                    "pwm_l": pwm_l,
                    "pwm_r": pwm_r,
                    "speed_level": getattr(ctrl, "speed_level", 0),
                    "turn_level": getattr(ctrl, "turn_level", 0),
                    "motion_src": getattr(ctrl, "motion_command_source", None),
                    "command_type": command_type,
                    "safety_allow": bool((safety_state or {}).get("allow", True)),
                    "safety_reason": str((safety_state or {}).get("reason", "OK") or "OK"),
                    "final_pwm_zero_reason": final_pwm_zero_reason,
                    "loop_budget_total_ema_ms": float(
                        (getattr(ctrl, "loop_budget_status", {}) or {}).get("total_ema_ms", 0.0)
                    ),
                },
                level="INFO",
            )
            self._last_snapshot_min_ts = float(now)
            self._last_snapshot_state = snapshot_state_key

        full_log_active = bool(getattr(ctrl, "log_capture_active", False))
        full_interval_s = (
            _safe_float(getattr(ctrl, "_control_snapshot_full_capture_interval_s", 0.2), 0.2)
            if full_log_active
            else _safe_float(getattr(ctrl, "_control_snapshot_full_interval_s", 1.0), 1.0)
        )
        if (now - self._last_snapshot_full_ts) < full_interval_s:
            return
        sections = compact_control_snapshot_sections(
            motion_command=dict(motion_command_semantics or {}),
            motion_resolution=dict(getattr(ctrl, "motion_resolution_status", {}) or {}),
            motion_semantics=dict(getattr(ctrl, "motion_semantics_status", {}) or {}),
            motion_quality=dict(getattr(ctrl, "motion_quality_status", {}) or {}),
        )
        enc_snap = loop_result.get("encoder_snapshot")
        enc_diag = (
            {
                "l_pulses": int(getattr(enc_snap, "left_pulses", 0)),
                "r_pulses": int(getattr(enc_snap, "right_pulses", 0)),
                "l_dist": round(float(getattr(enc_snap, "left_distance", 0.0)), 5),
                "r_dist": round(float(getattr(enc_snap, "right_distance", 0.0)), 5),
                "health": str(getattr(enc_snap, "health", "")),
            }
            if enc_snap is not None
            else {}
        )
        ul.log_event(
            CHANNEL_CONTROL,
            "control_loop",
            "control_snapshot",
            {
                "snapshot_schema": "CONTROL_SNAPSHOT_COMPACT_V2",
                "compacted": True,
                "state": state_name,
                "x": ekf_state.get("x"),
                "y": ekf_state.get("y"),
                "theta_deg": ekf_state.get("theta_deg"),
                "v_target": getattr(ctrl, "v_target", 0.0),
                "v_cmd": getattr(ctrl, "v_cmd", 0.0),
                "omega_target": getattr(ctrl, "omega_target", 0.0),
                "v_l_raw": v_l_raw,
                "v_r_raw": v_r_raw,
                "pwm_l": pwm_l,
                "pwm_r": pwm_r,
                "motion_src": getattr(ctrl, "motion_command_source", None),
                "command_type": command_type,
                "motion_command": sections["motion_command"],
                "motion_resolution": sections["motion_resolution"],
                "motion_semantics": sections["motion_semantics"],
                "motion_quality": sections["motion_quality"],
                "safety": safety_state,
                "final_pwm_zero_reason": final_pwm_zero_reason,
                "pid": sample.get("pid_diag") or {},
                "encoder": enc_diag,
                "loop_budget": dict(getattr(ctrl, "loop_budget_status", {}) or {}),
            },
            level="INFO",
        )
        self._last_snapshot_full_ts = float(now)

    def _publish_structured_diagnostics(
        self,
        ctrl,
        *,
        now: float,
        cycle_id: int,
        ekf_state: Dict[str, Any],
        loop_result: Dict[str, Any],
        sample: Dict[str, Any],
        pwm_l: float,
        pwm_r: float,
        v_l_raw: float,
        v_r_raw: float,
    ) -> None:
        full_log_active = bool(getattr(ctrl, "log_capture_active", False))
        if bool(sample.get("log_timing")):
            interval = _safe_float(
                sample.get("timing_diag_capture_interval" if full_log_active else "timing_diag_interval"),
                0.2,
            )
            if (now - self._last_timing_diag_ts) >= interval:
                self._last_timing_diag_ts = float(now)
                write_timing(
                    now,
                    cycle_id,
                    _safe_float(sample.get("dt_loop"), 0.0),
                    _safe_float(sample.get("dt_target"), 0.02),
                    _safe_float(sample.get("sleep_time"), 0.0),
                    bool(sample.get("overrun_flag", False)),
                    None,
                    dt_loop_observed_raw=sample.get("dt_loop_observed_raw"),
                    dt_loop_clamped=sample.get("dt_loop_clamped"),
                )
        if bool(sample.get("log_ekf_diag")):
            interval = _safe_float(
                sample.get("ekf_diag_capture_interval" if full_log_active else "ekf_diag_interval"),
                0.2,
            )
            if (now - self._last_ekf_diag_ts) >= interval:
                self._last_ekf_diag_ts = float(now)
                q_diag = None
                try:
                    q_val = getattr(getattr(getattr(ctrl, "control_loop", None), "ekf", None), "_Q_current", None)
                    if q_val is not None and hasattr(q_val, "shape"):
                        q_diag = [float(q_val[i, i]) for i in range(min(5, q_val.shape[0]))]
                except Exception:
                    q_diag = None
                write_ekf_diag(
                    now,
                    cycle_id,
                    loop_result.get("dt_ekf") or _safe_float(sample.get("dt_target"), 0.02),
                    loop_result.get("dt_ekf_source") or "loop",
                    None,
                    None,
                    False,
                    q_diag,
                    ekf_state.get("innovation_theta"),
                    bool(loop_result.get("still_for_zupt", False)),
                )
        if bool(sample.get("log_encoder_diag")):
            interval = _safe_float(
                sample.get("encoder_diag_capture_interval" if full_log_active else "encoder_diag_interval"),
                0.1,
            )
            if (now - self._last_encoder_diag_ts) >= interval:
                self._last_encoder_diag_ts = float(now)
                enc_snap = loop_result.get("encoder_snapshot")
                enc_rel = dict(getattr(ctrl, "encoder_reliability_status", {}) or {})
                pulses_delta = dict(enc_rel.get("pulses_delta") or {})
                if enc_snap is not None:
                    canonical_velocity = dict(enc_rel.get("canonical_velocity") or {})
                    write_encoder_diag(
                        now,
                        cycle_id,
                        getattr(enc_snap, "left_pulses", 0),
                        getattr(enc_snap, "right_pulses", 0),
                        int(pulses_delta.get("left", 0) or 0),
                        int(pulses_delta.get("right", 0) or 0),
                        v_l_raw,
                        v_r_raw,
                        pwm_l,
                        pwm_r,
                        str(enc_rel.get("canonical_state", "")).upper() == "IDLE",
                        {},
                        {},
                        float(
                            canonical_velocity.get("left_mps")
                            if canonical_velocity.get("left_mps") is not None
                            else v_l_raw
                        ),
                        float(
                            canonical_velocity.get("right_mps")
                            if canonical_velocity.get("right_mps") is not None
                            else v_r_raw
                        ),
                        str(enc_rel.get("pipeline_model", "KIT0085_QUADRATURE")),
                        str(enc_rel.get("ekf_usage_mode", "NORMAL")),
                        float(enc_rel.get("combined_trust", 0.0) or 0.0),
                        str(enc_rel.get("ekf_usage_reason", "")),
                        bool(enc_rel.get("symmetry_violation_instant", False)),
                        bool(enc_rel.get("symmetry_fault_active", False)),
                        str(enc_rel.get("symmetry_fault_side", "NONE")),
                    )
        if bool(sample.get("log_imu_diag")):
            interval = _safe_float(
                sample.get("imu_diag_capture_interval" if full_log_active else "imu_diag_interval"),
                0.2,
            )
            if (now - self._last_imu_diag_ts) >= interval:
                self._last_imu_diag_ts = float(now)
                imu_snap = loop_result.get("imu_snapshot")
                write_imu_diag(
                    now,
                    cycle_id,
                    loop_result.get("gyro_z_dps"),
                    loop_result.get("accel_x_g"),
                    loop_result.get("gyro_z_rad"),
                    loop_result.get("accel_x_mps2"),
                    ekf_state.get("gyro_bias"),
                    getattr(imu_snap, "health", "OK") if imu_snap else "N/A",
                )

    def _publish_housekeeping(self, ctrl, *, now: float) -> None:
        if (now - self._last_housekeeping_ts) < 1.0:
            return
        self._last_housekeeping_ts = float(now)
        ul = get_unified_logger()
        if ul is None:
            return
        hk = ul.run_housekeeping(now_ts=now)
        if not isinstance(hk, dict):
            return
        ctrl.logger_runtime_stats = {
            "queue_depth": int(hk.get("queued_messages", 0)),
            "dropped_messages": int(hk.get("dropped_messages", 0)),
            "write_errors": int(hk.get("write_errors", 0)),
            "last_flush_time": float(hk.get("last_flush_time", 0.0)),
            "last_flush_duration_ms": float(hk.get("last_flush_duration_ms", 0.0)),
            "max_flush_duration_ms": float(hk.get("max_flush_duration_ms", 0.0)),
            "last_immediate_write_duration_ms": float(
                hk.get("last_immediate_write_duration_ms", 0.0)
            ),
            "max_immediate_write_duration_ms": float(
                hk.get("max_immediate_write_duration_ms", 0.0)
            ),
            "total_immediate_jsonl": int(hk.get("total_immediate_jsonl", 0)),
            "updated_ts": float(now),
        }
