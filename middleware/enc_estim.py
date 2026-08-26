#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Enkóder becslő: sebesség, távolság és impulzus-alapú yaw (theta_enc).
Alapelv: a pulse-delta az elsődleges igazságforrás, a PWM csak vezérlési kontextus.
"""

import math
import time
from dataclasses import dataclass, field
from config_manager import config as global_config


@dataclass
class EstimData:
    raw_velocity: float = 0.0
    velocity: float = 0.0
    unsigned_velocity: float = 0.0
    distance: float = 0.0
    unsigned_distance: float = 0.0
    distance_delta: float = 0.0
    unsigned_distance_delta: float = 0.0
    pulses: int = 0
    dp: int = 0
    dt: float = 0.0
    direction: float = 0.0
    direction_source: str = "INIT"
    direction_confident: bool = False
    unresolved_pulses: int = 0
    last_pulse_time: float = field(default_factory=time.perf_counter)


@dataclass(frozen=True)
class _CounterSnapshot:
    pulse_count: int


class EncoderEstimator:
    """
    Sebesség és távolság becslő az enkóderek alapján.
    RAW-first alapelv:
    - a pulse-delta a forrásigazság,
    - minimális, indokolt utófeldolgozás (idle deadband + irány-bizonytalanság jelzése),
    - nincs rejtett domináns-oldal vagy extra simítás.

    A KIT0085 quadrature driver signed pulse countot ad, ezért az irány
    közvetlenül az A/B fázisviszonyból származik. A PWM csak diagnosztikai
    kontextus; nem módosítja a mért irányt vagy távolságot.
    """
    def __init__(self, enc_l, enc_r):
        self.enc_l = enc_l
        self.enc_r = enc_r

        fizika = global_config.get("fizika", default={})
        szuro = global_config.get("vezerles", "becslo_szuro", default={})
        hw = global_config.get("hardver", default={})

        try:
            self.step_distance = float(fizika["lepes_hossz_m"])
            self.wheel_base = float(fizika["nyomtav_szelesseg_m"])
            wheel_radius_m = float(fizika["kerek_sugar_m"])
            configured_cpr = int(fizika["encoder_impulzus_per_fordulat"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("KIT0085 physics configuration is incomplete") from exc
        if min(self.step_distance, self.wheel_base, wheel_radius_m) <= 0.0:
            raise ValueError("KIT0085 physics values must be positive")
        if configured_cpr <= 0:
            raise ValueError("KIT0085 encoder_impulzus_per_fordulat must be positive")
        expected_step_m = 2.0 * math.pi * wheel_radius_m / float(configured_cpr)
        if not math.isclose(self.step_distance, expected_step_m, rel_tol=1e-4, abs_tol=1e-9):
            raise ValueError(
                "KIT0085 lepes_hossz_m is inconsistent with wheel radius and CPR"
            )
        for encoder in (enc_l, enc_r):
            driver_cpr = getattr(encoder, "counts_per_revolution", None)
            if driver_cpr is not None and int(driver_cpr) != configured_cpr:
                raise ValueError("KIT0085 driver CPR differs from physics configuration")
        self.left_step_scale = self._safe_scale(
            fizika.get("lepes_hossz_bal_szorzo", fizika.get("lepes_hossz_bal_scale", 1.0)),
            default=1.0,
        )
        self.right_step_scale = self._safe_scale(
            fizika.get("lepes_hossz_jobb_szorzo", fizika.get("lepes_hossz_jobb_scale", 1.0)),
            default=1.0,
        )
        self._refresh_step_distances()
        self.noise_floor = szuro.get("zajszures_kuszob", 0.025)
        # A KIT0085 driver már maga alkalmazza az oldalankénti invert beállítást.
        self.inv_m_l = hw.get("motorok", {}).get("bal_oldal", {}).get("invert", False)
        self.inv_m_r = hw.get("motorok", {}).get("jobb_oldal", {}).get("invert", False)
        self.inv_e_l = hw.get("encoderek", {}).get("invert_bal", False)
        self.inv_e_r = hw.get("encoderek", {}).get("invert_jobb", False)
        self._signed_counts_l = bool(getattr(enc_l, "signed_counts", False))
        self._signed_counts_r = bool(getattr(enc_r, "signed_counts", False))
        if not (self._signed_counts_l and self._signed_counts_r):
            raise ValueError("EncoderEstimator requires signed KIT0085 quadrature counters")

        # Yaw: impulzus akkumulátor; küszöb alacsonyabb = gyakoribb theta_enc frissítés
        # Use refined threshold from config if available
        self._min_distance_threshold = szuro.get("theta_enc_min_delta_m", self.step_distance * 0.5)
        self._ds_l_acc = 0.0
        self._ds_r_acc = 0.0
        self.theta_enc = 0.0  # [rad], impulzus-alapú yaw

        initial_left, initial_right = self._read_driver_snapshots()
        self._last_t = time.perf_counter()
        # Timestamp of the counter sample that produced the public estimator
        # state.  This is deliberately distinct from EncoderService publish
        # time: a worker iteration that is too early must not make old counter
        # data look fresh.
        self._measurement_timestamp = float(self._last_t)
        self._last_pl = int(initial_left.pulse_count)
        self._last_pr = int(initial_right.pulse_count)
        self.left = EstimData()
        self.right = EstimData()
        self.left.last_pulse_time = self._last_t
        self.right.last_pulse_time = self._last_t

        self._last_unresolved_dp_l = 0
        self._last_unresolved_dp_r = 0
        self._last_update = {
            "dt": 0.0,
            "left_direction_source": "INIT",
            "right_direction_source": "INIT",
            "left_unresolved_dp": 0,
            "right_unresolved_dp": 0,
        }

    @staticmethod
    def _safe_scale(value, default: float = 1.0) -> float:
        try:
            out = float(value)
        except Exception:
            out = float(default)
        if out <= 1e-6:
            out = float(default)
        return max(0.05, min(20.0, float(out)))

    @staticmethod
    def _driver_snapshot(encoder):
        snapshot = getattr(encoder, "snapshot", None)
        if callable(snapshot):
            return snapshot()

        return _CounterSnapshot(pulse_count=int(getattr(encoder, "pulse_count", 0)))

    def _read_driver_snapshots(self):
        return self._driver_snapshot(self.enc_l), self._driver_snapshot(self.enc_r)

    def _refresh_step_distances(self) -> None:
        self.step_distance_left = float(self.step_distance) * float(self.left_step_scale)
        self.step_distance_right = float(self.step_distance) * float(self.right_step_scale)

    def update_physics(
        self,
        wheel_base: float = None,
        step_distance: float = None,
        left_step_scale: float = None,
        right_step_scale: float = None,
    ):
        """
        Fizikai paraméterek frissítése futásidőben (pl. config reload).
        """
        refresh_scale = False
        if wheel_base is not None and wheel_base > 1e-6:
            self.wheel_base = float(wheel_base)
        if step_distance is not None and step_distance > 1e-9:
            self.step_distance = float(step_distance)
            refresh_scale = True
            # Threshold update logic should ideally follow config if available
            szuro = global_config.get("vezerles", "becslo_szuro", default={})
            self._min_distance_threshold = szuro.get("theta_enc_min_delta_m", self.step_distance * 0.5)
        if left_step_scale is not None:
            self.left_step_scale = self._safe_scale(left_step_scale, default=self.left_step_scale)
            refresh_scale = True
        if right_step_scale is not None:
            self.right_step_scale = self._safe_scale(right_step_scale, default=self.right_step_scale)
            refresh_scale = True
        if refresh_scale:
            self._refresh_step_distances()

    @staticmethod
    def _sign(value: float, eps: float = 1e-9) -> float:
        if value > eps:
            return 1.0
        if value < -eps:
            return -1.0
        return 0.0

    def update(self, pwm_l: float = 0.0, pwm_r: float = 0.0):
        """
        Frissítés: impulzus delta → sebesség/távolság/yaw.
        KIT0085 esetén a pulse delta signed, ezért a PWM nem irányforrás.
        """
        left_snapshot, right_snapshot = self._read_driver_snapshots()
        now = time.perf_counter()
        dt = now - self._last_t

        if dt < 0.002:
            return False

        pl = int(left_snapshot.pulse_count)
        pr = int(right_snapshot.pulse_count)

        dpl_raw = pl - self._last_pl
        dpr_raw = pr - self._last_pr
        
        dpl = dpl_raw
        dpr = dpr_raw

        if abs(dpl) > 0:
            self.left.last_pulse_time = now
        if abs(dpr) > 0:
            self.right.last_pulse_time = now

        dir_l = self._sign(dpl)
        dir_r = self._sign(dpr)
        dir_src_l = "QUADRATURE_AB" if dpl else "QUADRATURE_IDLE"
        dir_src_r = "QUADRATURE_AB" if dpr else "QUADRATURE_IDLE"
        dir_conf_l = True
        dir_conf_r = True

        ds_l_u = abs(float(dpl)) * self.step_distance_left
        ds_r_u = abs(float(dpr)) * self.step_distance_right
        ds_l = float(dpl) * self.step_distance_left
        ds_r = float(dpr) * self.step_distance_right
        v_l_raw = ds_l / dt
        v_r_raw = ds_r / dt
        v_l_u = ds_l_u / dt
        v_r_u = ds_r_u / dt

        # Impulzus alapú yaw akkumulátor (kvantálási zaj csökkentés).
        self._ds_l_acc += ds_l
        self._ds_r_acc += ds_r

        # Minimális zajszűrés: csak pulse-mentes mintán deadband.
        if abs(dpl) == 0 and abs(v_l_raw) < self.noise_floor:
            v_l_raw = 0.0
        if abs(dpr) == 0 and abs(v_r_raw) < self.noise_floor:
            v_r_raw = 0.0
        if abs(dpl) == 0 and abs(v_l_u) < self.noise_floor:
            v_l_u = 0.0
        if abs(dpr) == 0 and abs(v_r_u) < self.noise_floor:
            v_r_u = 0.0

        unresolved_dp_l = int(abs(dpl)) if (abs(dpl) > 0 and not dir_conf_l) else 0
        unresolved_dp_r = int(abs(dpr)) if (abs(dpr) > 0 and not dir_conf_r) else 0
        self._last_unresolved_dp_l = unresolved_dp_l
        self._last_unresolved_dp_r = unresolved_dp_r
        if unresolved_dp_l > 0:
            self.left.unresolved_pulses += unresolved_dp_l
        if unresolved_dp_r > 0:
            self.right.unresolved_pulses += unresolved_dp_r

        # STANDSTILL FIX: ne nullázzuk azonnal az akkumulátort.
        if (
            abs(self._ds_l_acc) >= self._min_distance_threshold
            or abs(self._ds_r_acc) >= self._min_distance_threshold
        ):
            dtheta = (self._ds_r_acc - self._ds_l_acc) / self.wheel_base
            self.theta_enc += dtheta
            self._ds_l_acc = 0.0
            self._ds_r_acc = 0.0

        self.left.raw_velocity = v_l_raw
        self.right.raw_velocity = v_r_raw
        self.left.velocity = v_l_raw
        self.right.velocity = v_r_raw
        self.left.unsigned_velocity = v_l_u
        self.right.unsigned_velocity = v_r_u
        self.left.pulses = pl
        self.right.pulses = pr
        self.left.dp = int(dpl)
        self.right.dp = int(dpr)
        self.left.dt = dt
        self.right.dt = dt
        self.left.direction = float(dir_l)
        self.right.direction = float(dir_r)
        self.left.direction_source = str(dir_src_l)
        self.right.direction_source = str(dir_src_r)
        self.left.direction_confident = bool(dir_conf_l)
        self.right.direction_confident = bool(dir_conf_r)
        self.left.distance_delta = float(ds_l)
        self.right.distance_delta = float(ds_r)
        self.left.unsigned_distance_delta = float(ds_l_u)
        self.right.unsigned_distance_delta = float(ds_r_u)

        # Távolság: közvetlen pulse-delta integrálás (ne szűrt v*dt).
        self.left.distance += ds_l
        self.right.distance += ds_r
        self.left.unsigned_distance += ds_l_u
        self.right.unsigned_distance += ds_r_u

        self._last_update = {
            "dt": float(dt),
            "left_direction_source": str(dir_src_l),
            "right_direction_source": str(dir_src_r),
            "left_unresolved_dp": int(unresolved_dp_l),
            "right_unresolved_dp": int(unresolved_dp_r),
            "left_dp": int(dpl),
            "right_dp": int(dpr),
            "left_signed_ds_m": float(ds_l),
            "right_signed_ds_m": float(ds_r),
            "left_unsigned_ds_m": float(ds_l_u),
            "right_unsigned_ds_m": float(ds_r_u),
            "step_distance_left_m": float(self.step_distance_left),
            "step_distance_right_m": float(self.step_distance_right),
            "step_scale_left": float(self.left_step_scale),
            "step_scale_right": float(self.right_step_scale),
        }

        self._last_pl, self._last_pr, self._last_t = pl, pr, now
        self._measurement_timestamp = float(now)
        return True

    @property
    def measurement_timestamp(self) -> float:
        """Monotonic time of the counter sample represented by ``left/right``."""
        return float(self._measurement_timestamp)
