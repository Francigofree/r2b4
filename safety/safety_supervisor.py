# -*- coding: utf-8 -*-

from dataclasses import dataclass
import math
import time

from config_manager import config as global_config
from middleware.peripheral_usage import get_cached_peripherals
from state import RobotState

DEFAULT_AMR_LIDAR_GUARD_SOURCES = ("STATE", "ADAPTIVE", "AI", "CORE")


def _is_lidar_enabled(controller) -> bool:
    """LIDAR BE/KI SSOT: runtime/peripherals_enabled.json."""
    return bool(
        get_cached_peripherals(status_path=getattr(controller, "status_path", None)).get(
            "lidar",
            True,
        )
    )


def _safe_float(value, default):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


@dataclass
class SafetyDecision:
    allow: bool = True
    action: str = "OK"  # OK | STOP
    reason: str = ""
    v_target: float = 0.0
    omega_target: float = 0.0


class SafetySupervisor:
    """
    Globális safety réteg.
    Folyamatosan ellenőriz és felülírhatja a mozgást.
    """
    def __init__(self, controller):
        self.controller = controller
        self._last_reason = ""
        self._last_log = 0.0
        self._last_emergency_ts = 0.0
        self._last_recovery_lidar_stale_warn = 0.0
        self._last_decision = SafetyDecision()
        self._last_decision_ts = time.monotonic()
        self._front_block_count = 0
        self._back_block_count = 0
        self._amr_lidar_bad_count = 0
        self._amr_lidar_last_observation_id = None
        self._amr_lidar_last_observation_new = False
        self._amr_lidar_last_quality_reason = "UNINITIALIZED"
        self._sensor_health_source = "UNINITIALIZED"
        self._last_sensor_health_diag = {
            "ok": False,
            "reason": "UNINITIALIZED",
            "source": self._sensor_health_source,
            "duration_us": 0,
        }
        self._last_lidar_stale_diag = {
            "checked": False,
            "stale_by_last_update": False,
            "stale_by_odom": False,
            "stale_confirmed": False,
            "last_update_age_s": None,
            "lidar_odom_latest_age_s": None,
            "threshold_s": None,
            "source": "NONE",
        }
        self._last_lidar_adapter_snapshot = {
            "summary": dict(getattr(controller, "lidar_summary", {}) or {}),
            "timestamp": getattr(controller, "lidar_last_update", None),
            "health": str(getattr(controller, "lidar_health", "UNKNOWN") or "UNKNOWN"),
            "lock_busy": False,
            "source": "initial",
        }
        self._lidar_adapter_lock_busy_count = 0
        self._lidar_adapter_last_lock_busy = False

        # Config alapértékek
        self.lidar_stale_sec = 0.6
        self.stop_on_lidar_stale = True
        self.obstacle_confirm_ticks = 2
        self.emergency_cooldown_sec = 1.0

        # Hardver configból
        hw = global_config.get("hardver", default={})
        lidar = hw.get("lidar", {})
        self.danger_zone = lidar.get("biztonsagi_zona_m", 0.30)
        safety_cfg = (global_config.get("vezerles", default={}) or {}).get("safety", {})
        self.obstacle_confirm_ticks = max(1, int(safety_cfg.get("akadaly_megerosites_tick", self.obstacle_confirm_ticks)))
        self.emergency_cooldown_sec = float(safety_cfg.get("emergency_cooldown_sec", self.emergency_cooldown_sec))
        self.amr_lidar_guard_enabled = bool(safety_cfg.get("amr_lidar_guard_enabled", True))
        raw_sources = safety_cfg.get("amr_lidar_guard_sources", DEFAULT_AMR_LIDAR_GUARD_SOURCES) or DEFAULT_AMR_LIDAR_GUARD_SOURCES
        self.amr_lidar_guard_sources = tuple(
            str(src or "").strip().upper() for src in raw_sources if str(src or "").strip()
        ) or DEFAULT_AMR_LIDAR_GUARD_SOURCES
        self.amr_lidar_min_scan_points = max(1, int(safety_cfg.get("amr_lidar_min_scan_points", 10)))
        self.amr_lidar_min_confidence = max(0.0, float(safety_cfg.get("amr_lidar_min_confidence", 0.25)))
        self.amr_lidar_max_candidate_age_s = max(0.05, float(safety_cfg.get("amr_lidar_max_candidate_age_s", 0.5)))
        self.amr_lidar_max_latest_age_s = max(0.1, float(safety_cfg.get("amr_lidar_max_latest_age_s", 1.5)))
        self.amr_lidar_bad_confirm_ticks = max(1, int(safety_cfg.get("amr_lidar_bad_confirm_ticks", 3)))

    def evaluate(self, now=None) -> SafetyDecision:
        if now is None:
            now = time.monotonic()

        effective_v_target = float(getattr(self.controller, "v_target", 0.0) or 0.0)

        # Kalibráció alatt nincs mozgás
        if self.controller.sm.current_enum == RobotState.CALIBRATING:
            decision = SafetyDecision(
                allow=False,
                action="STOP",
                reason="Kalibráció aktív",
                v_target=0.0,
                omega_target=0.0
            )
            self._last_decision = decision
            self._last_decision_ts = now
            return decision

        # --- SENSOR HEALTH CHECK ---
        health_ok, health_reason = self._check_sensor_health(now=now)
        if not health_ok:
            decision = SafetyDecision(
                allow=False,
                action="STOP",
                reason=health_reason,
                v_target=0.0,
                omega_target=0.0
            )
            self._last_decision = decision
            self._last_decision_ts = now
            return decision
        # ---------------------------

        lidar_enabled = _is_lidar_enabled(self.controller)

        # Lidar frissesség és akadály csak BE állapotban
        recovery_lidar_stale = False
        if lidar_enabled:
            lidar_adapter = self._lidar_adapter_snapshot()
            # Lidar frissesség ellenőrzés
            if self.stop_on_lidar_stale and abs(float(effective_v_target)) > 1e-4:
                last_ts = lidar_adapter["timestamp"]
                stale_age = None
                stale_by_last_update = False
                if last_ts is not None:
                    try:
                        stale_age = max(0.0, float(now) - float(last_ts))
                        stale_by_last_update = stale_age > float(self.lidar_stale_sec)
                    except (TypeError, ValueError):
                        stale_age = None

                lidar_odom = dict(getattr(self.controller, "lidar_odom_runtime_status", {}) or {})
                odom_latest_age_s = _safe_float(lidar_odom.get("latest_age_s"), math.nan)
                odom_age_valid = math.isfinite(float(odom_latest_age_s))
                stale_by_odom = bool(odom_age_valid and odom_latest_age_s > float(self.lidar_stale_sec))
                stale_confirmed = bool(
                    stale_by_last_update
                    and ((not odom_age_valid) or stale_by_odom)
                )
                self._last_lidar_stale_diag = {
                    "checked": True,
                    "stale_by_last_update": bool(stale_by_last_update),
                    "stale_by_odom": bool(stale_by_odom),
                    "stale_confirmed": bool(stale_confirmed),
                    "last_update_age_s": (None if stale_age is None else round(float(stale_age), 3)),
                    "lidar_odom_latest_age_s": (None if not odom_age_valid else round(float(odom_latest_age_s), 3)),
                    "threshold_s": round(float(self.lidar_stale_sec), 3),
                    "source": ("lidar_last_update+lidar_odom" if odom_age_valid else "lidar_last_update"),
                }
                if stale_confirmed:
                    if self._in_recovery_mobility_mode():
                        recovery_lidar_stale = True
                        self._emit_recovery_lidar_stale_warning(now=now, stale_age=float(stale_age or 0.0))
                    else:
                        decision = SafetyDecision(
                            allow=False,
                            action="STOP",
                            reason="LIDAR adat elavult",
                            v_target=0.0,
                            omega_target=0.0
                        )
                        self._last_decision = decision
                        self._last_decision_ts = now
                        return decision
            else:
                self._last_lidar_stale_diag = {
                    "checked": False,
                    "stale_by_last_update": False,
                    "stale_by_odom": False,
                    "stale_confirmed": False,
                    "last_update_age_s": None,
                    "lidar_odom_latest_age_s": None,
                    "threshold_s": round(float(self.lidar_stale_sec), 3),
                    "source": "inactive_or_idle",
                }

            # Lidar akadály ellenőrzés (FOLLOW/adaptive motion is also subject to front/back block)
            lidar = lidar_adapter["summary"]
            if effective_v_target > 0 and lidar.get("blocked_front", False):
                self._front_block_count += 1
            else:
                self._front_block_count = 0

            if effective_v_target < 0 and lidar.get("blocked_back", False):
                self._back_block_count += 1
            else:
                self._back_block_count = 0

            if effective_v_target > 0 and self._front_block_count >= self.obstacle_confirm_ticks:
                decision = SafetyDecision(
                    allow=False,
                    action="STOP",
                    reason=f"Akadaly elol ({self._front_block_count} tick)",
                    v_target=0.0,
                    omega_target=0.0
                )
                self._last_decision = decision
                self._last_decision_ts = now
                return decision

            if effective_v_target < 0 and self._back_block_count >= self.obstacle_confirm_ticks:
                decision = SafetyDecision(
                    allow=False,
                    action="STOP",
                    reason=f"Akadaly hatul ({self._back_block_count} tick)",
                    v_target=0.0,
                    omega_target=0.0
                )
                self._last_decision = decision
                self._last_decision_ts = now
                return decision

            if recovery_lidar_stale:
                decision = SafetyDecision(
                    allow=True,
                    action="OK",
                    reason="LIDAR adat elavult (RECOVERY_TOLERATED)",
                    v_target=float(effective_v_target),
                    omega_target=float(getattr(self.controller, "omega_target", 0.0) or 0.0),
                )
                self._last_decision = decision
                self._last_decision_ts = now
                return decision
        else:
            self._front_block_count = 0
            self._back_block_count = 0
            self._last_lidar_stale_diag = {
                "checked": False,
                "stale_by_last_update": False,
                "stale_by_odom": False,
                "stale_confirmed": False,
                "last_update_age_s": None,
                "lidar_odom_latest_age_s": None,
                "threshold_s": round(float(self.lidar_stale_sec), 3),
                "source": "lidar_disabled",
            }

        # AMR source-ok (STATE/ADAPTIVE/AI/CORE): csak jó LiDAR minőség mellett mozoghatnak.
        if self._amr_lidar_guard_required(effective_v_target):
            lidar_ok, lidar_reason, lidar_evidence = self._evaluate_amr_lidar_quality()
            observation_id = self._amr_lidar_observation_id(
                lidar_reason=lidar_reason,
                evidence=lidar_evidence,
            )
            observation_new = bool(
                observation_id is None
                or observation_id != self._amr_lidar_last_observation_id
            )
            self._amr_lidar_last_observation_new = observation_new
            self._amr_lidar_last_quality_reason = str(lidar_reason or "")

            # A matcher eredmenye tipikusan tobb 50 Hz-es control tickig azonos.
            # Egyetlen rossz scan nem szamolhato harom fuggetlen meresnek. Ha az
            # observation azonosito hianyzik, fail-closed modon megmarad a
            # tickenkenti megerosites.
            if observation_new:
                self._amr_lidar_last_observation_id = observation_id
                if not lidar_ok:
                    self._amr_lidar_bad_count += 1
                else:
                    self._amr_lidar_bad_count = 0

            if self._amr_lidar_bad_count >= self.amr_lidar_bad_confirm_ticks:
                decision = SafetyDecision(
                    allow=False,
                    action="STOP",
                    reason=(
                        f"AMR_LIDAR_GUARD {lidar_reason} "
                        f"({self._amr_lidar_bad_count} scan)"
                    ),
                    v_target=0.0,
                    omega_target=0.0,
                )
                self._last_decision = decision
                self._last_decision_ts = now
                return decision
        else:
            self._amr_lidar_bad_count = 0
            self._amr_lidar_last_observation_id = None
            self._amr_lidar_last_observation_new = False
            self._amr_lidar_last_quality_reason = "NOT_REQUIRED"

        decision = SafetyDecision(allow=True, action="OK", reason="OK")
        self._last_decision = decision
        self._last_decision_ts = now
        return decision

    def _in_recovery_mobility_mode(self) -> bool:
        return bool(getattr(self.controller, "recovery_mobility_mode", False))

    def _emit_recovery_lidar_stale_warning(self, now: float, stale_age: float) -> None:
        if (now - self._last_recovery_lidar_stale_warn) < 1.0:
            return

        reason = "LIDAR adat elavult (RECOVERY_TOLERATED)"
        if hasattr(self.controller, "logger"):
            self.controller.logger.warn(f"[SAFETY] {reason} age={stale_age:.2f}s")
        if hasattr(self.controller, "telemetry"):
            self.controller.telemetry.emit_audit(
                "SAFETY_WARN",
                "SAFETY",
                severity="WARN",
                details={
                    "reason": "LIDAR adat elavult",
                    "recovery_tolerated": True,
                    "stale_age_s": round(float(stale_age), 3),
                },
            )
        self._last_recovery_lidar_stale_warn = now

    def apply(self, decision: SafetyDecision):
        if decision.allow:
            return

        # Vészleállítás okkal (cooldown: azonos okot ne lőjünk minden tickben újra).
        now = time.monotonic()
        should_trigger = (
            decision.reason != self._last_reason or
            (now - self._last_emergency_ts) >= self.emergency_cooldown_sec
        )
        if should_trigger:
            self.controller._emergency_stop(reason=decision.reason)
            self._last_emergency_ts = now
        if hasattr(self.controller, "telemetry"):
            self.controller.telemetry.emit_audit(
                "SAFETY_STOP",
                "SAFETY",
                severity="WARN",
                details={"reason": decision.reason, "triggered": should_trigger}
            )

        # Log throttling
        if decision.reason != self._last_reason or (now - self._last_log) > 1.0:
            self.controller.logger.warn(f"[SAFETY] {decision.reason}")
            self._last_reason = decision.reason
            self._last_log = now

    def status(self) -> dict:
        return {
            "allow": self._last_decision.allow,
            "action": self._last_decision.action,
            "reason": self._last_decision.reason,
            "ts": self._last_decision_ts,
            "lidar_stale": dict(self._last_lidar_stale_diag or {}),
            "sensor_health": dict(self._last_sensor_health_diag or {}),
            "amr_lidar_guard": {
                "bad_observation_count": int(self._amr_lidar_bad_count),
                "confirm_observations": int(self.amr_lidar_bad_confirm_ticks),
                "last_observation_id": self._amr_lidar_last_observation_id,
                "last_observation_new": bool(self._amr_lidar_last_observation_new),
                "last_quality_reason": str(self._amr_lidar_last_quality_reason or ""),
                "confirmation_unit": "distinct_lidar_observation",
            },
            "lidar_adapter": {
                "lock_busy_count": int(self._lidar_adapter_lock_busy_count),
                "last_lock_busy": bool(self._lidar_adapter_last_lock_busy),
            },
        }

    def notify_control_mode_change(self, old_mode: str, new_mode: str, state: str) -> None:
        self._last_reason = f"CONTROL_MODE {old_mode} -> {new_mode} ({state})"

    def _check_sensor_health(self, now=None):
        start = time.perf_counter()
        self._sensor_health_source = "CONFIG_AND_CACHED_SENSOR_STATE"
        try:
            ok, reason = self._check_sensor_health_impl(now=now)
            return bool(ok), str(reason)
        except Exception as exc:
            ok, reason = False, f"SENSOR HEALTH HIBA ({exc})"
            return ok, reason
        finally:
            self._last_sensor_health_diag = {
                "ok": bool(locals().get("ok", False)),
                "reason": str(locals().get("reason", "SENSOR_HEALTH_EXCEPTION")),
                "source": str(self._sensor_health_source),
                "duration_us": int(max(0.0, (time.perf_counter() - start) * 1_000_000.0)),
            }

    def _check_sensor_health_impl(self, now=None):
        """IMU és Encoder validáció a konfigurált határértékekhez."""
        ctrl = self.controller
        now_mono = time.monotonic() if now is None else float(now)
        
        # 1. KIT0085 quadrature encoder check
        enc_l = getattr(ctrl, "enc_l", None)
        enc_r = getattr(ctrl, "enc_r", None)
        if enc_l and enc_r:
            health_l = str(getattr(enc_l, "health", "ERROR") or "ERROR").upper()
            health_r = str(getattr(enc_r, "health", "ERROR") or "ERROR").upper()
            if health_l not in ("OK", "DEGRADED") or health_r not in ("OK", "DEGRADED"):
                return False, f"KIT0085 ENCODER HIBA (L:{health_l} R:{health_r})"
            for enc in (enc_l, enc_r):
                if int(getattr(enc, "pin_a", -1)) == int(getattr(enc, "pin_b", -1)):
                    return False, "KIT0085 ENCODER GPIO A/B ÜTKÖZÉS"

        # 2. The BNO055 service snapshot is the only IMU health surface.
        self._sensor_health_source = "IMU_SERVICE_SNAPSHOT"
        try:
            imu_driver = getattr(ctrl, "imu_driver", None)
            if str(getattr(imu_driver, "provider", "") or "").strip().lower() != "bno055":
                return False, "BNO055 IMU DRIVER HIÁNYZIK"
            imu_service = getattr(ctrl, "imu_service", None)
            snapshot = imu_service.get_snapshot() if imu_service is not None else None
            if snapshot is None:
                return False, "BNO055 IMU SNAPSHOT HIÁNYZIK"
            snapshot_health = str(getattr(snapshot, "health", "ERROR") or "ERROR").upper()
            if snapshot_health not in ("OK", "DEGRADED"):
                return False, "BNO055 IMU SNAPSHOT HIBA"
            snapshot_ts = float(getattr(snapshot, "timestamp", 0.0) or 0.0)
            snapshot_age_s = max(0.0, float(now_mono) - snapshot_ts)
            if snapshot_ts <= 0.0 or snapshot_age_s > 0.5:
                return False, f"BNO055 IMU SNAPSHOT ELAVULT ({snapshot_age_s:.3f}s)"
            gyro_values = tuple(getattr(snapshot, "gyro", ()) or ())
            if len(gyro_values) < 3 or not all(
                isinstance(v, (int, float)) and math.isfinite(float(v))
                for v in gyro_values[:3]
            ):
                return False, "BNO055 GYRO ADAT INVALID"
            if snapshot_health == "DEGRADED":
                consecutive_errors = int(getattr(snapshot, "consecutive_errors", 0) or 0)
                return True, (
                    "BNO055 IMU DEGRADED "
                    f"({consecutive_errors} olvasási hiba, age={snapshot_age_s:.3f}s)"
                )
        except Exception as e:
            return False, f"BNO055 GYRO HIBA ({e})"

        return True, "OK"

    def _amr_lidar_guard_required(self, effective_v_target: float) -> bool:
        if not bool(self.amr_lidar_guard_enabled):
            return False
        source = str(getattr(self.controller, "motion_command_source", "") or "").strip().upper()
        if source not in self.amr_lidar_guard_sources:
            return False
        omega_target = float(getattr(self.controller, "omega_target", 0.0) or 0.0)
        moving = abs(float(effective_v_target)) > 1e-4 or abs(float(omega_target)) > 1e-4
        return bool(moving)

    def _lidar_adapter_snapshot(self):
        lock = getattr(self.controller, "lidar_lock", None)
        if lock is None:
            snapshot = {
                "summary": dict(getattr(self.controller, "lidar_summary", {}) or {}),
                "timestamp": getattr(self.controller, "lidar_last_update", None),
                "health": str(getattr(self.controller, "lidar_health", "UNKNOWN") or "UNKNOWN"),
                "lock_busy": False,
                "source": "direct_no_lock",
            }
            self._last_lidar_adapter_snapshot = dict(snapshot)
            self._lidar_adapter_last_lock_busy = False
            return snapshot

        acquired = False
        try:
            acquire = getattr(lock, "acquire", None)
            if callable(acquire):
                acquired = bool(acquire(blocking=False))
            if not acquired:
                self._lidar_adapter_lock_busy_count += 1
                self._lidar_adapter_last_lock_busy = True
                cached = dict(getattr(self, "_last_lidar_adapter_snapshot", {}) or {})
                summary = dict(cached.get("summary") or {})
                return {
                    "summary": summary,
                    "timestamp": cached.get("timestamp"),
                    "health": str(cached.get("health", "UNKNOWN") or "UNKNOWN"),
                    "lock_busy": True,
                    "source": "cached_lock_busy",
                }
            snapshot = {
                "summary": dict(getattr(self.controller, "lidar_summary", {}) or {}),
                "timestamp": getattr(self.controller, "lidar_last_update", None),
                "health": str(getattr(self.controller, "lidar_health", "UNKNOWN") or "UNKNOWN"),
                "lock_busy": False,
                "source": "locked_latest",
            }
            self._last_lidar_adapter_snapshot = dict(snapshot)
            self._lidar_adapter_last_lock_busy = False
            return snapshot
        finally:
            if acquired:
                try:
                    lock.release()
                except Exception:
                    pass

    @staticmethod
    def _evidence_id(payload, explicit_keys, legacy_key):
        for key in explicit_keys:
            if key not in payload:
                continue
            try:
                value = int(payload.get(key) or 0)
            except (TypeError, ValueError):
                value = 0
            return value if value > 0 else 0
        try:
            value = int(payload.get(legacy_key, 0) or 0)
        except (TypeError, ValueError):
            value = 0
        return value if value > 0 else 0

    def _amr_lidar_evidence_snapshot(self):
        adapter = self._lidar_adapter_snapshot()
        lidar_summary = dict(adapter["summary"] or {})
        odom = dict(getattr(self.controller, "lidar_odom_runtime_status", {}) or {})
        candidate_age_s = _safe_float(odom.get("candidate_age_s"), math.inf)
        latest_age_s = _safe_float(odom.get("latest_age_s"), math.inf)
        candidate_fresh = bool(
            odom.get("candidate_available", False)
            and math.isfinite(candidate_age_s)
            and candidate_age_s <= float(self.amr_lidar_max_candidate_age_s)
        )
        latest_fresh = bool(
            math.isfinite(latest_age_s)
            and latest_age_s <= float(self.amr_lidar_max_latest_age_s)
        )
        # ``candidate_confidence`` is the odometry-promotion signal.  During
        # tracking reacquisition it is deliberately clamped just below the
        # estimator threshold even when the independent matcher measurement
        # is high quality.  Safety owns the measurement-confidence contract,
        # so prefer its dedicated signal and retain the legacy field only for
        # older producers which do not publish the split metric yet.
        candidate_measurement_conf = _safe_float(
            odom.get("candidate_measurement_confidence"),
            math.nan,
        )
        if math.isfinite(candidate_measurement_conf):
            candidate_conf = float(candidate_measurement_conf)
            candidate_conf_source = "candidate_measurement_confidence"
        else:
            candidate_conf = _safe_float(odom.get("candidate_confidence"), 0.0)
            candidate_conf_source = "candidate_confidence_legacy"

        latest_measurement_conf = _safe_float(
            odom.get("latest_measurement_confidence"),
            math.nan,
        )
        if math.isfinite(latest_measurement_conf):
            latest_conf = float(latest_measurement_conf)
            latest_conf_source = "latest_measurement_confidence"
        else:
            latest_conf = _safe_float(odom.get("latest_confidence"), 0.0)
            latest_conf_source = "latest_confidence_legacy"
        candidate_integrity = _safe_float(
            odom.get("candidate_integrity_score"),
            candidate_conf,
        )
        latest_integrity = _safe_float(
            odom.get("latest_integrity_score"),
            latest_conf,
        )
        candidate_integrity_state = str(
            odom.get("candidate_integrity_state", "LEGACY") or "LEGACY"
        ).upper()
        latest_integrity_state = str(
            odom.get("latest_integrity_state", "LEGACY") or "LEGACY"
        ).upper()

        selected_kind = None
        selected_id = 0
        selected_confidence = 0.0
        selected_confidence_source = "none"
        selected_integrity_score = 0.0
        selected_integrity_state = "INCOMPLETE"
        # A fresh matcher candidate is the newest independent localization
        # observation. Never mask it with an older accepted measurement merely
        # because the retained measurement has a larger scalar confidence.
        if candidate_fresh:
            selected_kind = "matcher_result"
            selected_id = self._evidence_id(
                odom,
                ("candidate_id", "matcher_result_id"),
                "candidate_created",
            )
            selected_confidence = float(candidate_conf)
            selected_confidence_source = str(candidate_conf_source)
            selected_integrity_score = float(candidate_integrity)
            selected_integrity_state = str(candidate_integrity_state)
        elif latest_fresh:
            selected_kind = "lidar_odometry_measurement"
            selected_id = self._evidence_id(
                odom,
                ("lidar_odometry_measurement_id",),
                "accepted",
            )
            selected_confidence = float(latest_conf)
            selected_confidence_source = str(latest_conf_source)
            selected_integrity_score = float(latest_integrity)
            selected_integrity_state = str(latest_integrity_state)

        return {
            "summary": lidar_summary,
            "timestamp": adapter["timestamp"],
            "health": adapter["health"],
            "odom": odom,
            "raw_scan_id": self._evidence_id(lidar_summary, ("raw_scan_id",), "scan_seq"),
            "candidate_id": self._evidence_id(
                odom,
                ("candidate_id", "matcher_result_id"),
                "candidate_created",
            ),
            "measurement_id": self._evidence_id(
                odom,
                ("lidar_odometry_measurement_id",),
                "accepted",
            ),
            "candidate_fresh": bool(candidate_fresh),
            "latest_fresh": bool(latest_fresh),
            "selected_kind": selected_kind,
            "selected_id": int(selected_id),
            "selected_confidence": float(selected_confidence),
            "selected_confidence_source": str(selected_confidence_source),
            "selected_integrity_score": float(selected_integrity_score),
            "selected_integrity_state": str(selected_integrity_state),
        }

    def _amr_lidar_observation_id(self, *, lidar_reason: str = "", evidence=None):
        """Return an ID derived from the evidence used by the AMR quality gate.

        Raw scans, matcher results and accepted odometry measurements are
        different observations.  In particular, a periodically republished
        raw timestamp must never turn one retained matcher result into several
        confidence observations.
        """
        evidence = dict(evidence or self._amr_lidar_evidence_snapshot())
        raw_scan_id = int(evidence.get("raw_scan_id", 0) or 0)
        candidate_id_value = int(evidence.get("candidate_id", 0) or 0)
        measurement_id_value = int(evidence.get("measurement_id", 0) or 0)

        reason = str(lidar_reason or "").strip().upper()
        raw_id = ("raw_scan", int(raw_scan_id)) if raw_scan_id > 0 else None
        candidate_id = (
            ("matcher_result", int(candidate_id_value))
            if candidate_id_value > 0
            else None
        )
        measurement_id = (
            ("lidar_odometry_measurement", int(measurement_id_value))
            if measurement_id_value > 0
            else None
        )

        # Point count and matcher-called are properties of the raw scan result.
        if reason.startswith("LOW_SCAN_POINTS") or reason == "NO_MATCHER_SIGNAL":
            return raw_id

        # Device health is not a retained matcher/odometry observation.  Use a
        # real raw scan ID if one exists; otherwise None keeps the established
        # fail-closed confirmation behaviour.
        if reason == "LIDAR_DISABLED" or reason.startswith("LIDAR_HEALTH_"):
            return raw_id

        if reason == "ODOM_STALE":
            stale_ids = tuple(
                item for item in (candidate_id, measurement_id) if item is not None
            )
            return ("freshness", stale_ids) if stale_ids else None

        selected_kind = str(evidence.get("selected_kind") or "")
        selected_id = int(evidence.get("selected_id", 0) or 0)
        confidence_id = (
            (selected_kind, selected_id)
            if selected_kind and selected_id > 0
            else None
        )
        if confidence_id is None:
            return None
        return (
            "quality",
            ("confidence", confidence_id),
            ("freshness", (confidence_id,)),
        )

    def _evaluate_amr_lidar_quality(self):
        evidence = self._amr_lidar_evidence_snapshot()
        if not _is_lidar_enabled(self.controller):
            return False, "LIDAR_DISABLED", evidence

        lidar_health = str(evidence.get("health", "UNKNOWN") or "UNKNOWN").strip().upper()
        if lidar_health != "OK":
            return False, f"LIDAR_HEALTH_{lidar_health}", evidence

        lidar_summary = dict(evidence.get("summary") or {})
        scan_count_filtered = int(_safe_float(lidar_summary.get("scan_count_filtered", 0.0), 0.0))
        if scan_count_filtered < int(self.amr_lidar_min_scan_points):
            return False, f"LOW_SCAN_POINTS({scan_count_filtered})", evidence

        if not (bool(evidence.get("candidate_fresh")) or bool(evidence.get("latest_fresh"))):
            return False, "ODOM_STALE", evidence

        signal_conf = float(evidence.get("selected_confidence", 0.0) or 0.0)
        if signal_conf < float(self.amr_lidar_min_confidence):
            return False, f"LOW_CONF({signal_conf:.3f})", evidence

        integrity_state = str(
            evidence.get("selected_integrity_state", "INCOMPLETE") or "INCOMPLETE"
        ).upper()
        integrity_score = float(
            evidence.get("selected_integrity_score", 0.0) or 0.0
        )
        if integrity_state not in ("OK", "LEGACY"):
            return (
                False,
                f"LOCALIZATION_INTEGRITY_{integrity_state}({integrity_score:.3f})",
                evidence,
            )
        if integrity_score < float(self.amr_lidar_min_confidence):
            return False, f"LOW_INTEGRITY({integrity_score:.3f})", evidence

        matcher_called = bool(lidar_summary.get("matcher_called", False))
        odom = dict(evidence.get("odom") or {})
        accepted_total = int(_safe_float(odom.get("accepted", 0.0), 0.0))
        if not matcher_called and not (accepted_total > 0 and bool(evidence.get("latest_fresh"))):
            return False, "NO_MATCHER_SIGNAL", evidence

        return True, "OK", evidence

    def _check_amr_lidar_quality(self) -> tuple[bool, str]:
        ok, reason, _evidence = self._evaluate_amr_lidar_quality()
        return bool(ok), str(reason)
