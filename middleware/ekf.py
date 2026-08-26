#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ExtendedKalmanFilter: magnetométer nélküli, zárt hurkú mozgásvezérlésre alkalmas állapotbecslő.
Állapot: x = [px, py, theta, v, gyro_bias].
Predict: IMU (gyro_z, accel_x). Update: enkóder (v_enc, theta_enc), ZUPT (v=0), LIDAR (x,y,theta).
Adaptivitási réteg: gain scheduling Q/R, csúszás, innováció, bias gyorsító, theta_hold álló helyzetben.
Szenzor bemenetet nem módosítjuk; minden korrekció explicit EKF update.

--- Koordináta-rendszer (frame) konvenció ---
Robot (body) frame:
  +X = előre (haladási irány)
  +Y = balra
  +Z = felfelé
Yaw (theta) konvenció:
  pozitív yaw = balra fordulás (counter-clockwise, felülnézetből)
IMU leképezés:
  gyro_z [rad/s] -> yaw rate (omega)
  accel_x [m/s²] -> előre irányú gyorsulás (robot X tengely)
"""

import math
from typing import Any, Dict, Optional

import numpy as np

from middleware.robot_frame import POSE_FRAME_ID, POSE_FRAME_OWNER, pose_frame_contract

# Módok a gain schedulinghez (diszkrét Q/R profilok)
MODE_STILL = "still"
MODE_LINEAR = "linear"
MODE_ROTATING = "rotating"


def _default_ekf_config():
    """Alapértelmezett EKF paraméterek (stabil, nem adaptív)."""
    return {
        "Q_diag": [0.002, 0.002, 0.001, 0.02, 1e-5],   # px, py, theta, v, gyro_bias
        "R_enc": [0.015, 0.012],                        # R_v, R_theta (enkóder)
        "R_zupt": 0.01,                                 # ZUPT v=0 mérési zaj
        "zupt_threshold": 0.02,                         # |v_l|,|v_r|,|v_cmd| < ez → ZUPT
        "R_lidar": [0.05, 0.05, 0.02],                 # LIDAR pozíció mérési zaj: x, y, theta (variancia)
        "R_v_lidar": 0.03,                             # LIDAR delta alapján számolt v mérési zaj (variancia)
        "lidar_confidence_threshold": 0.3,              # LIDAR update csak ha confidence > ez
        "adaptive_q": False,                             # régi: mozgásfüggő Q
        "q_scale_accel": 0.0,
        "P_min_diag": 1e-8,                              # P diagonál minimum (numerikus stabilitás)
        "use_joseph_form": False,                        # True: Joseph form P update (stabilabb, lassabb)
        # Innováció kapu (NIS) - outlier mérések védelme.
        "innovation_gating": {
            "enabled": True,
            "enc_nis_max": 25.0,    # 2 DoF, lazább felső küszöb
            "lidar_nis_max": 35.0,  # 3 DoF, lazább felső küszöb
            "v_lidar_nis_max": 18.0,  # 1 DoF (v) outlier kapu
            "lidar_soft_nis_ratio_max": 3.8,  # köztes tartomány: soft apply
            "lidar_soft_r_scale_max": 10.0,   # max R_lidar skála soft ágon
        },
        # Adaptivitási réteg (gain scheduling + egyéb)
        "adaptivity": {
            "enabled": False,
            "still_threshold": 0.02,
            "rotate_threshold": 0.15,
            "Q_still": [0.001, 0.001, 0.0008, 0.015, 2e-4],   # Q_bias magas állóban
            "Q_linear": [0.002, 0.002, 0.001, 0.02, 1e-5],
            "Q_rotate": [0.002, 0.002, 0.003, 0.02, 1e-5],    # Q_theta nagy forgatásnál
            "R_enc_still": [0.015, 0.025],              # R_theta nagy (kevésbé bízunk encoder theta-ban)
            "R_enc_linear": [0.015, 0.012],
            "R_enc_rotate": [0.015, 0.018],             # közepes
            "slip_velocity_threshold": 0.25,            # |v_r - v_l| [m/s]
            "slip_accel_min": 1.5,                      # |accel_x| [m/s²] - csúszás ha nagy accel
            "slip_R_scale": 5.0,                        # R_enc *= ez csúszásnál
            "innovation_theta_threshold_rad": 0.08,    # ~4.5°
            "innovation_R_theta_scale": 2.0,
            "bias_accel_k": 0.02,                      # gyro_bias += k * gyro_z * dt állóban
            "R_theta_hold": 0.008,                     # theta_k = theta_{k-1} mérési zaj
            "online_learning": True,                  # folyamatos Q/R és csúszás/innováció adaptáció
            "alpha_slip_ema": 0.05,                    # csúszás indikátor EMA (0–1)
            "alpha_inno_ema": 0.08,                    # innováció theta magnitúdó EMA [rad]
            "Q_online_gamma": 0.4,                     # Q skála = 1 + gamma * inno_ema (modellbizonytalanság)
            "Q_online_max": 0.25,                      # max Q növekmény
        },
    }


class ExtendedKalmanFilter:
    """
    Kiterjesztett Kalman szűrő: [px, py, theta, v, gyro_bias].
    Predict: omega = gyro_z - gyro_bias; theta += omega*dt; v += accel_x*dt; px, py.
    Update: enkóder (v_enc, theta_enc), ZUPT (v=0), opcionális theta_hold (állóban).
    Adaptivitás: egyszer per ciklus update_adaptivity() → mód + Q/R beállítás, majd fix mátrixokkal fut az EKF.
    """

    IX_PX = 0
    IX_PY = 1
    IX_THETA = 2
    IX_V = 3
    IX_GYRO_BIAS = 4
    N = 5

    def __init__(self, wheel_base_or_config=None, config: dict = None, **kwargs):
        # Shadow EKF contract compatibility:
        # - ExtendedKalmanFilter(config)
        # - ExtendedKalmanFilter(wheel_base, config)
        # - ExtendedKalmanFilter(wheel_base=..., config=...)
        kw_wheel_base = kwargs.pop("wheel_base", None)
        kw_config = kwargs.pop("config", None)
        if kwargs:
            unknown = ", ".join(sorted(kwargs.keys()))
            raise TypeError(f"Unexpected keyword argument(s): {unknown}")
        if kw_config is not None:
            if config is not None:
                raise TypeError("Multiple values for config")
            config = kw_config
        if kw_wheel_base is not None:
            if wheel_base_or_config is not None and not isinstance(wheel_base_or_config, dict):
                raise TypeError("Multiple values for wheel_base")
            wheel_base_or_config = kw_wheel_base
        if isinstance(wheel_base_or_config, dict) and config is None:
            wheel_base = 0.175
            config = wheel_base_or_config
        else:
            wheel_base = 0.175 if wheel_base_or_config is None else wheel_base_or_config
        self.wheel_base = max(1e-3, float(wheel_base))
        cfg = dict(_default_ekf_config())
        if config:
            cfg.update(config)

        self._zupt_threshold = float(cfg.get("zupt_threshold", 0.02))
        self._adaptive_q = bool(cfg.get("adaptive_q", False))
        self._q_scale_accel = float(cfg.get("q_scale_accel", 0.0))

        q_diag = list(cfg.get("Q_diag", cfg["Q_diag"]))
        if len(q_diag) < self.N:
            q_diag = (q_diag + [1e-5] * self.N)[:self.N]
        self._Q_base = np.diag([float(x) for x in q_diag[:self.N]])

        r_enc = cfg.get("R_enc", cfg["R_enc"])
        self._R_enc = np.diag([float(r_enc[0]) if len(r_enc) > 0 else 0.015,
                               float(r_enc[1]) if len(r_enc) > 1 else 0.012])

        self._R_zupt = float(cfg.get("R_zupt", cfg["R_zupt"]))
        self._R_gyro = float(cfg.get("R_gyro", 0.0))

        # LIDAR abszolút pozíciókorrekció
        r_lidar = cfg.get("R_lidar", [0.05, 0.05, 0.02])
        rl = [float(r_lidar[0]), float(r_lidar[1]) if len(r_lidar) > 1 else 0.05,
              float(r_lidar[2]) if len(r_lidar) > 2 else 0.02]
        self._R_lidar = np.diag(rl)
        self._R_v_lidar = float(cfg.get("R_v_lidar", 0.03))
        self._lidar_confidence_threshold = float(cfg.get("lidar_confidence_threshold", 0.3))

        # --- Adaptivitási réteg: előre számolt Q/R profilok ---
        ad = cfg.get("adaptivity") or {}
        self._adaptivity_enabled = bool(ad.get("enabled", False))
        self._still_th = float(ad.get("still_threshold", self._zupt_threshold))
        self._rotate_th = float(ad.get("rotate_threshold", 0.15))
        self._slip_v_th = float(ad.get("slip_velocity_threshold", 0.25))
        self._slip_a_min = float(ad.get("slip_accel_min", 1.5))
        self._slip_R_scale = float(ad.get("slip_R_scale", 5.0))
        self._inno_th_rad = float(ad.get("innovation_theta_threshold_rad", 0.08))
        self._inno_R_scale = float(ad.get("innovation_R_theta_scale", 2.0))
        self._bias_accel_k = float(ad.get("bias_accel_k", 0.01))
        self._bias_accel_gyro_max = float(ad.get("bias_accel_gyro_max_rad_s", 0.05))
        self._R_theta_hold = float(ad.get("R_theta_hold", 0.004))
        # Álló helyzetben (főleg encoder nélkül) erősebb yaw-hold a drift csökkentésére.
        self._theta_hold_strong_gyro_max = max(
            0.0,
            float(ad.get("theta_hold_strong_gyro_max_rad_s", 0.03)),
        )
        self._theta_hold_disable_gyro_min = max(
            0.0,
            float(ad.get("theta_hold_disable_gyro_min_rad_s", 0.20)),
        )
        self._theta_hold_strong_r_scale = min(
            1.0,
            max(1e-4, float(ad.get("theta_hold_strong_r_scale", 0.05))),
        )
        self._online_learning = bool(ad.get("online_learning", True))
        self._alpha_slip_ema = float(ad.get("alpha_slip_ema", 0.05))
        self._alpha_inno_ema = float(ad.get("alpha_inno_ema", 0.08))
        self._Q_online_gamma = float(ad.get("Q_online_gamma", 0.4))
        self._Q_online_max = float(ad.get("Q_online_max", 0.25))

        def diag5(arr, default=None):
            if default is None:
                default = [0.002, 0.002, 0.001, 0.02, 1e-5]
            a = list(arr) if isinstance(arr, (list, tuple)) else default
            return np.diag([float(a[i]) if i < len(a) else default[i] for i in range(self.N)])

        qs = ad.get("Q_still", [0.001, 0.001, 0.0008, 0.015, 2e-4])
        ql = ad.get("Q_linear", cfg.get("Q_diag", [0.002, 0.002, 0.001, 0.02, 1e-5]))
        qr = ad.get("Q_rotate", [0.002, 0.002, 0.003, 0.02, 1e-5])
        self._Q_still = diag5(qs)
        self._Q_linear = diag5(ql)
        self._Q_rotate = diag5(qr)

        def r2(arr):
            a = list(arr) if isinstance(arr, (list, tuple)) else [0.015, 0.012]
            return np.diag([float(a[0]) if len(a) > 0 else 0.015, float(a[1]) if len(a) > 1 else 0.012])

        self._R_enc_still = r2(ad.get("R_enc_still", [0.015, 0.025]))
        self._R_enc_linear = r2(ad.get("R_enc_linear", cfg.get("R_enc", [0.015, 0.012])))
        self._R_enc_rotate = r2(ad.get("R_enc_rotate", [0.015, 0.018]))

        self.reset(0.0, 0.0, 0.0, 0.0, 0.0)

        self._theta_gyro = 0.0
        self._theta_enc = 0.0
        self._last_v_enc = 0.0
        self._last_omega_rad_s = 0.0  # gyro - bias, belső hurok EKF feedbackhoz
        # Párhuzamos „statikus” EKF: fix Q_base, R_enc (adaptivitás nélkül) – finomhangolás loghoz
        self._x_static = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self._P_static = np.diag([0.01, 0.01, 0.01, 0.01, 0.0001])

        # Adaptivitás: aktuális mód és ciklusra beállított Q/R
        self._mode = MODE_LINEAR
        self._Q_current = self._Q_base.copy()
        self._R_enc_current = self._R_enc.copy()
        self._last_innovation_theta = 0.0
        self._last_innovation_v = 0.0
        self._last_S_enc_diag = [0.0, 0.0]
        self._last_innov_zupt = 0.0
        self._last_innov_theta_hold = 0.0
        self._last_innov_v_lidar = 0.0
        self._zupt_applied_last = False
        self._theta_hold_applied_last = False
        self._slip_this_cycle = False
        self._still_this_cycle = False
        self._slip_ema = 0.0
        self._inno_theta_ema = 0.0
        self._last_R_theta_hold_eff = self._R_theta_hold

        # P numerikus stabilitás: minimális diagonál, szimmetria
        self._P_min_diag = float(cfg.get("P_min_diag", 1e-8))
        self._use_joseph_form = bool(cfg.get("use_joseph_form", False))

        gate = cfg.get("innovation_gating") or {}
        self._gate_enabled = bool(gate.get("enabled", True))
        self._enc_nis_max = float(gate.get("enc_nis_max", 25.0))
        self._lidar_nis_max = float(gate.get("lidar_nis_max", 35.0))
        self._v_lidar_nis_max = float(gate.get("v_lidar_nis_max", 18.0))
        self._lidar_soft_nis_ratio_max = float(gate.get("lidar_soft_nis_ratio_max", 3.8))
        self._lidar_soft_r_scale_max = float(gate.get("lidar_soft_r_scale_max", 10.0))
        # NIS-reject esetén egyszeri, nagyobb zajú sebességmérés fallback
        # (stabilizálja a v állapotot enyhe outlier esetén is).
        self._v_lidar_soft_nis_ratio_max = 4.5
        self._v_lidar_soft_r_scale_max = 12.0
        self._enc_gate_reject = False
        self._lidar_gate_reject = False
        self._v_lidar_gate_reject = False
        self._enc_nis_last = 0.0
        self._lidar_nis_last = 0.0
        self._v_lidar_nis_last = 0.0
        self._lidar_soft_applied_last = False
        self._lidar_soft_r_scale_last = 1.0
        self._v_lidar_soft_applied_last = False
        self._v_lidar_soft_r_scale_last = 1.0
        self._last_lidar_update["confidence_threshold"] = float(self._lidar_confidence_threshold)
        self._last_lidar_update["nis_threshold"] = float(self._lidar_nis_max)

        trust_cfg = cfg.get("trust_adaptation") or {}
        self._trust_adaptation_enabled = bool(trust_cfg.get("enabled", True))
        self._trust_asymmetry_warn = float(trust_cfg.get("asymmetry_warn", 0.35))
        self._trust_asymmetry_critical = float(trust_cfg.get("asymmetry_critical", 0.65))
        self._trust_idle_r_scale = float(trust_cfg.get("idle_r_scale", 6.0))
        self._trust_asymmetry_r_scale = float(trust_cfg.get("asymmetry_r_scale", 2.5))
        self._trust_anomaly_r_scale = float(trust_cfg.get("anomaly_r_scale", 3.5))
        self._trust_skip_threshold = float(trust_cfg.get("skip_threshold", 0.12))
        self._encoder_r_scale_external = 1.0
        self._encoder_update_skip_reason = ""
        self._encoder_trust_mode = "NORMAL"
        self._encoder_context_flags = {}
        self._encoder_enabled = True
        self._encoder_usage_gain = 1.0
        self._encoder_confidence_hint = 1.0
        self._encoder_covariance_hint = 1.0

    def _ensure_P_valid(self, P: np.ndarray) -> None:
        """P szimmetrikus és pozitív definit diagonál (clamp)."""
        P[:] = 0.5 * (P + P.T)
        for i in range(self.N):
            if P[i, i] < self._P_min_diag:
                P[i, i] = self._P_min_diag

    def reset(self, px=0.0, py=0.0, theta=0.0, v=0.0, gyro_bias=0.0):
        """Állapot és kovariancia alaphelyzet."""
        self.x = np.array([
            float(px), float(py), self._wrap(float(theta)),
            float(v), float(gyro_bias)
        ], dtype=float)
        self.P = np.diag([0.01, 0.01, 0.01, 0.01, 0.0001])
        self._theta_gyro = float(theta)
        self._theta_enc = float(theta)
        self._last_v_enc = 0.0
        self._last_innovation_theta = 0.0
        self._last_innovation_v = 0.0
        self._last_S_enc_diag = [0.0, 0.0]
        self._last_innov_zupt = 0.0
        self._last_innov_theta_hold = 0.0
        self._last_innov_v_lidar = 0.0
        self._zupt_applied_last = False
        self._theta_hold_applied_last = False
        self._slip_this_cycle = False
        self._slip_ema = 0.0
        self._inno_theta_ema = 0.0
        self._last_R_theta_hold_eff = self._R_theta_hold
        self._last_omega_rad_s = 0.0
        self._enc_gate_reject = False
        self._lidar_gate_reject = False
        self._v_lidar_gate_reject = False
        self._enc_nis_last = 0.0
        self._lidar_nis_last = 0.0
        self._v_lidar_nis_last = 0.0
        self._lidar_soft_applied_last = False
        self._lidar_soft_r_scale_last = 1.0
        self._v_lidar_soft_applied_last = False
        self._v_lidar_soft_r_scale_last = 1.0
        self._encoder_r_scale_external = 1.0
        self._encoder_update_skip_reason = ""
        self._encoder_trust_mode = "NORMAL"
        self._encoder_context_flags = {}
        self._encoder_enabled = True
        self._encoder_usage_gain = 1.0
        self._encoder_confidence_hint = 1.0
        self._encoder_covariance_hint = 1.0
        self._last_lidar_update = {
            "status": "idle",
            "applied": False,
            "reject_reason": "",
            "confidence": None,
            "r_scale": 1.0,
            "confidence_threshold": float(getattr(self, "_lidar_confidence_threshold", 0.0)),
            "nis": None,
            "nis_threshold": float(getattr(self, "_lidar_nis_max", 0.0)),
            "innovation": None,
            "gate_reject": False,
            "soft_applied": False,
            "soft_r_scale": 1.0,
        }
        self._x_static = np.array([
            float(px), float(py), self._wrap(float(theta)), float(v), float(gyro_bias)
        ])
        self._P_static = np.diag([0.01, 0.01, 0.01, 0.01, 0.0001])

    def _wrap(self, a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _innovation_nis(self, innov: np.ndarray, S: np.ndarray) -> float:
        """NIS = innov^T S^-1 innov. Hiba esetén +végtelen."""
        try:
            return float(innov.T @ np.linalg.inv(S) @ innov)
        except Exception:
            return float("inf")

    def set_lidar_confidence_threshold(self, threshold: float) -> None:
        self._lidar_confidence_threshold = max(0.0, float(threshold))
        if isinstance(getattr(self, "_last_lidar_update", None), dict):
            self._last_lidar_update["confidence_threshold"] = float(self._lidar_confidence_threshold)

    def set_encoder_theta_suppression(self, enabled: bool) -> None:
        self._suppress_encoder_theta = bool(enabled)

    def _set_lidar_update_result(
        self,
        *,
        applied: bool,
        reject_reason: str = "",
        confidence: Optional[float] = None,
        r_scale: float = 1.0,
        innovation: Optional[np.ndarray] = None,
        nis: Optional[float] = None,
        soft_applied: bool = False,
        soft_r_scale: float = 1.0,
    ) -> Dict[str, Any]:
        result = {
            "status": (
                "soft_applied"
                if (applied and bool(soft_applied))
                else ("applied" if applied else str(reject_reason or "idle"))
            ),
            "applied": bool(applied),
            "reject_reason": "" if applied else str(reject_reason or ""),
            "confidence": None if confidence is None else float(confidence),
            "r_scale": float(r_scale),
            "confidence_threshold": float(self._lidar_confidence_threshold),
            "nis": None if nis is None else float(nis),
            "nis_threshold": float(self._lidar_nis_max),
            "innovation": None if innovation is None else [float(v) for v in innovation],
            "gate_reject": bool(self._lidar_gate_reject),
            "soft_applied": bool(soft_applied),
            "soft_r_scale": float(soft_r_scale),
        }
        self._last_lidar_update = result
        return dict(result)

    def _update_mode(self, v_l: float, v_r: float, v_cmd: float) -> str:
        """Mód meghatározása: STILL, ROTATING vagy LINEAR. Csak küszöbök, nincs extra mátrix."""
        th = self._still_th
        if abs(v_l) < th and abs(v_r) < th and abs(v_cmd) < th:
            return MODE_STILL
        if abs(v_r - v_l) > self._rotate_th:
            return MODE_ROTATING
        return MODE_LINEAR

    def _set_Q_R(self, mode: str, slip: bool, innovation_theta_large: bool):
        """Diszkrét Q/R profil alap: pointer csere. Csúszás/innováció folyamatos skálázás az EMA-kkal történik _get_R_enc/_get_Q-ban."""
        if mode == MODE_STILL:
            self._Q_current = self._Q_still.copy()
            self._R_enc_current = self._R_enc_still.copy()
        elif mode == MODE_ROTATING:
            self._Q_current = self._Q_rotate.copy()
            self._R_enc_current = self._R_enc_rotate.copy()
        else:
            self._Q_current = self._Q_linear.copy()
            self._R_enc_current = self._R_enc_linear.copy()
        if not self._online_learning:
            if slip:
                self._R_enc_current = self._R_enc_current * self._slip_R_scale
            if innovation_theta_large:
                self._R_enc_current[1, 1] *= self._inno_R_scale

    def update_adaptivity(self, v_l: float, v_r: float, v_cmd: float,
                         accel_x: float, gyro_z_rad: float, dt: float):
        """
        Adaptivitási réteg: ciklus elején egyszer hívandó.
        Mód + Q/R beállítás, csúszás, innováció alapú R_theta, bias gyorsító állóban.
        """
        if not self._adaptivity_enabled:
            self._slip_this_cycle = False
            self._still_this_cycle = (
                abs(v_l) < self._zupt_threshold and
                abs(v_r) < self._zupt_threshold and
                abs(v_cmd) < self._zupt_threshold
            )
            return

        self._mode = self._update_mode(v_l, v_r, v_cmd)
        self._still_this_cycle = self._mode == MODE_STILL

        slip = (abs(v_r - v_l) > self._slip_v_th and
                abs(accel_x) > self._slip_a_min)
        self._slip_this_cycle = bool(slip)
        inno_large = abs(self._last_innovation_theta) > self._inno_th_rad

        if self._online_learning:
            self._slip_ema = (1.0 - self._alpha_slip_ema) * self._slip_ema + self._alpha_slip_ema * (1.0 if slip else 0.0)
            inno_mag = abs(self._last_innovation_theta)
            self._inno_theta_ema = (1.0 - self._alpha_inno_ema) * self._inno_theta_ema + self._alpha_inno_ema * inno_mag

        self._set_Q_R(self._mode, slip, inno_large)

        # 4) Bias gyorsító csak álló helyzetben, és csak ha gyro plausibilis (zajszűrő)
        if self._still_this_cycle and self._bias_accel_k > 0 and dt > 0:
            # Csak akkor gyorsítunk, ha a nyers gyro plausibilis (nem rúgtak bele a robotba)
            if abs(gyro_z_rad) < getattr(self, "_bias_accel_gyro_max", 0.05):
                # Maradék omega = gyro - bias. Álló helyzetben ennek 0-nak kellene lennie.
                # Ami marad, az a bias hiba, amit integrálunk.
                omega_residual = gyro_z_rad - self.x[self.IX_GYRO_BIAS]
                self.x[self.IX_GYRO_BIAS] += self._bias_accel_k * omega_residual * dt

    def _get_Q(self, accel_x: float) -> np.ndarray:
        """Q: adaptivitás be → _Q_current, opcionálisan online skála (innováció EMA); különben régi logika."""
        if self._adaptivity_enabled:
            Q = self._Q_current.copy()
            if self._online_learning and self._Q_online_gamma > 0:
                scale = 1.0 + min(self._Q_online_max, self._Q_online_gamma * self._inno_theta_ema)
                for i in range(self.N):
                    Q[i, i] *= scale
            return Q
        Q = self._Q_base.copy()
        if self._adaptive_q and self._q_scale_accel > 0 and abs(accel_x) > 1e-6:
            scale = 1.0 + self._q_scale_accel * min(abs(accel_x), 5.0)
            for i in (self.IX_PX, self.IX_PY, self.IX_THETA, self.IX_V):
                Q[i, i] *= scale
        return Q

    def _get_R_enc(self) -> np.ndarray:
        """R enkóder: adaptivitás be → _R_enc_current; online: csúszás és innováció EMA alapú folyamatos skála."""
        if self._adaptivity_enabled:
            R = self._R_enc_current.copy()
            if self._online_learning:
                slip_scale = 1.0 + (self._slip_R_scale - 1.0) * self._slip_ema
                inno_ratio = min(1.0, self._inno_theta_ema / max(1e-6, self._inno_th_rad))
                inno_scale = 1.0 + (self._inno_R_scale - 1.0) * inno_ratio
                R[0, 0] *= slip_scale
                R[1, 1] *= slip_scale * inno_scale
        else:
            R = self._R_enc.copy()
        ext = float(getattr(self, "_encoder_r_scale_external", 1.0) or 1.0)
        if ext > 1.0:
            R[0, 0] *= ext
            R[1, 1] *= ext
        return R

    def predict(self, accel_x: float, gyro_z_rad: float, dt: float):
        """Predikció: IMU (gyro_z [rad/s], accel_x [m/s²])."""
        if dt <= 0:
            return self.get_state()

        px, py, theta, v, gyro_bias = self.x
        self._theta_before_predict = theta

        omega = gyro_z_rad - gyro_bias
        self._last_omega_rad_s = float(omega)
        new_theta = self._wrap(theta + omega * dt)
        new_v = v + accel_x * dt
        # Sanity clamp: fizikailag lehetetlen sebesség esetén exponenciális dekay.
        # Gravitáció vetület (ax_g * 9.81 állóban) jellegzetes mellékhatása.
        if abs(new_v) > 1.5:
            new_v = v * 0.9  # Convergál 0 felé, ZUPT elvégzi a pontos korrrekciót
        v_avg = (v + new_v) / 2.0
        # Álló módban ne integráljuk a sebességet a pozícióba (drift csökkentése)
        if getattr(self, "_mode", None) == MODE_STILL:
            new_px, new_py = px, py
        else:
            new_px = px + v_avg * math.cos(theta) * dt
            new_py = py + v_avg * math.sin(theta) * dt

        self.x = np.array([new_px, new_py, new_theta, new_v, gyro_bias])

        F = np.eye(self.N)
        F[self.IX_PX, self.IX_THETA] = -v_avg * math.sin(theta) * dt
        F[self.IX_PX, self.IX_V] = math.cos(theta) * dt
        F[self.IX_PY, self.IX_THETA] = v_avg * math.cos(theta) * dt
        F[self.IX_PY, self.IX_V] = math.sin(theta) * dt
        F[self.IX_THETA, self.IX_GYRO_BIAS] = -dt

        Q = self._get_Q(accel_x)
        if self._R_gyro > 0.0:
            Q[self.IX_THETA, self.IX_THETA] += self._R_gyro
        self.P = F @ self.P @ F.T + Q
        self._ensure_P_valid(self.P)

        # Statikus EKF: ugyanaz a kinematika, fix Q_base
        px_s, py_s, theta_s, v_s, gb_s = self._x_static
        omega_s = gyro_z_rad - gb_s
        new_theta_s = self._wrap(theta_s + omega_s * dt)
        new_v_s = v_s + accel_x * dt
        v_avg_s = (v_s + new_v_s) / 2.0
        if getattr(self, "_mode", None) == MODE_STILL:
            new_px_s, new_py_s = px_s, py_s
        else:
            new_px_s = px_s + v_avg_s * math.cos(theta_s) * dt
            new_py_s = py_s + v_avg_s * math.sin(theta_s) * dt
        self._x_static = np.array([new_px_s, new_py_s, new_theta_s, new_v_s, gb_s])
        self._P_static = F @ self._P_static @ F.T + self._Q_base
        self._ensure_P_valid(self._P_static)

        self._theta_gyro = float(self.x[self.IX_THETA])
        return self.get_state()

    def update_encoders(
        self,
        v_l: float,
        v_r: float,
        dt: float,
        theta_enc_rad: Optional[float] = None,
    ):
        """
        Enkóder korrekció: z = [v_enc, theta_enc].
        theta_enc_rad: ha megadott, impulzus-alapú yaw a becslőből (PART 2+3: nem sebesség/dt).
        Ha None, visszaesés régi omega_enc*dt számításra.
        """
        if dt <= 0:
            return self.get_state()

        # --- Encoder Health Cross-Check ---
        # Ha a két enkóder sebessége között nagy az eltérés (v_err > 0.2 m/s), 
        # DE a giroszkóp szerint szinte egyenesen megyünk (gyro < 0.1 rad/s), 
        # akkor valószínűleg az egyik enkóder kihagy. Ilyenkor a nagyobb sebességet vesszük alapul.
        v_diff = abs(v_r - v_l)
        gyro_z_rad_abs = abs(getattr(self, "_last_omega_rad_s", 0.0)) # A predict-ből jövő legfrissebb omega
        if v_diff > 0.2 and gyro_z_rad_abs < 0.1:
            if abs(v_l) > abs(v_r):
                v_r = v_l
            else:
                v_l = v_r

        v_enc = (v_l + v_r) / 2.0
        update_theta = True

        if theta_enc_rad is not None:
            theta_enc = self._wrap(float(theta_enc_rad))
        else:
            # Check if we should suppress relative theta update from encoders
            if getattr(self, "_suppress_encoder_theta", False):
                theta_enc = self.x[self.IX_THETA]
                update_theta = False
            else:
                theta_prev = getattr(self, "_theta_before_predict", self.x[self.IX_THETA])
                omega_enc = (v_r - v_l) / self.wheel_base
                theta_enc = self._wrap(theta_prev + omega_enc * dt)

        # Álló módban az enkóder sebességét ne bízzuk meg (zaj → drift), v mérés = 0
        if getattr(self, "_mode", None) == MODE_STILL:
            z = np.array([0.0, theta_enc])
        else:
            z = np.array([v_enc, theta_enc])
            
        hx = np.array([self.x[self.IX_V], self.x[self.IX_THETA]])
        H = np.zeros((2, self.N))
        H[0, self.IX_V] = 1.0
        if update_theta:
            H[1, self.IX_THETA] = 1.0
        else:
            # If skipping theta update, set measurement to the current state to zero innovation
            z[1] = hx[1]

        innov = z - hx
        innov[1] = self._wrap(innov[1])
        self._last_innovation_theta = float(innov[1])
        self._last_innovation_v = float(innov[0])

        R_enc = self._get_R_enc()
        # Állóban R_v nagy, hogy az enkóder v ne húzza fel a v állapotot (ZUPT intézi)
        if getattr(self, "_mode", None) == MODE_STILL:
            R_enc = R_enc.copy()
            R_enc[0, 0] = max(R_enc[0, 0], 0.5)
        S = H @ self.P @ H.T + R_enc
        enc_nis = self._innovation_nis(innov, S)
        self._enc_nis_last = enc_nis
        self._enc_gate_reject = bool(self._gate_enabled and enc_nis > self._enc_nis_max)
        self._last_S_enc_diag = [float(S[0, 0]), float(S[1, 1])]
        if self._enc_gate_reject:
            self._theta_enc = theta_enc
            self._last_v_enc = v_enc
            return self.get_state()
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ innov).flatten()
        self.x[self.IX_THETA] = self._wrap(self.x[self.IX_THETA])
        if self._use_joseph_form:
            IKH = np.eye(self.N) - K @ H
            self.P = IKH @ self.P @ IKH.T + K @ R_enc @ K.T
        else:
            self.P = (np.eye(self.N) - K @ H) @ self.P
        self._ensure_P_valid(self.P)

        self._theta_enc = theta_enc
        self._last_v_enc = v_enc

        # Statikus EKF: ugyanaz z, H; R = fix _R_enc
        hx_s = np.array([self._x_static[self.IX_V], self._x_static[self.IX_THETA]])
        innov_s = z - hx_s
        innov_s[1] = self._wrap(innov_s[1])
        S_s = H @ self._P_static @ H.T + self._R_enc
        K_s = self._P_static @ H.T @ np.linalg.inv(S_s)
        self._x_static = self._x_static + (K_s @ innov_s).flatten()
        self._x_static[self.IX_THETA] = self._wrap(self._x_static[self.IX_THETA])
        self._P_static = (np.eye(self.N) - K_s @ H) @ self._P_static
        self._ensure_P_valid(self._P_static)

        return self.get_state()

    def update_zupt(self):
        """ZUPT: v=0 mérés."""
        self._zupt_applied_last = True
        H = np.zeros((1, self.N))
        H[0, self.IX_V] = 1.0
        z = np.array([0.0])
        R = np.array([[self._R_zupt]])
        hx = np.array([self.x[self.IX_V]])
        innov = z - hx
        self._last_innov_zupt = float(innov[0])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ innov).flatten()
        self.P = (np.eye(self.N) - K @ H) @ self.P
        self._ensure_P_valid(self.P)
        # Statikus EKF
        hx_s = np.array([self._x_static[self.IX_V]])
        innov_s = z - hx_s
        S_s = H @ self._P_static @ H.T + R
        K_s = self._P_static @ H.T @ np.linalg.inv(S_s)
        self._x_static = self._x_static + (K_s @ innov_s).flatten()
        self._P_static = (np.eye(self.N) - K_s @ H) @ self._P_static
        self._ensure_P_valid(self._P_static)
        return self.get_state()

    def update_theta_hold(self):
        """
        Álló helyzetben explicit mérés: theta_k = theta_{k-1} (omega_measured = 0 ekvivalens).
        A hívó csak álló helyzetben hívja (pl. ZUPT után).
        """
        self._theta_hold_applied_last = True
        theta_prev = getattr(self, "_theta_before_predict", self.x[self.IX_THETA])
        H = np.zeros((1, self.N))
        H[0, self.IX_THETA] = 1.0
        z = np.array([self._wrap(theta_prev)])
        hx = np.array([self.x[self.IX_THETA]])
        innov = z - hx
        innov[0] = self._wrap(innov[0])
        self._last_innov_theta_hold = float(innov[0])
        r_theta_hold_eff = float(self._R_theta_hold)
        encoder_inactive = (not bool(getattr(self, "_encoder_enabled", True))) or float(
            getattr(self, "_encoder_usage_gain", 1.0)
        ) < 0.15
        enc_ctx = dict(getattr(self, "_encoder_context_flags", {}) or {})
        canonical_state = str(enc_ctx.get("canonical_state", "") or "").upper()
        theta_measurement_reliable = bool(enc_ctx.get("theta_measurement_reliable", True))
        idle_theta_unreliable = canonical_state == "IDLE" and (not theta_measurement_reliable)
        if (
            (encoder_inactive or idle_theta_unreliable)
            and abs(float(getattr(self, "_last_omega_rad_s", 0.0))) <= self._theta_hold_strong_gyro_max
        ):
            r_theta_hold_eff *= self._theta_hold_strong_r_scale
        r_theta_hold_eff = max(1e-8, r_theta_hold_eff)
        self._last_R_theta_hold_eff = r_theta_hold_eff
        R = np.array([[r_theta_hold_eff]])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ innov).flatten()
        self.x[self.IX_THETA] = self._wrap(self.x[self.IX_THETA])
        self.P = (np.eye(self.N) - K @ H) @ self.P
        self._ensure_P_valid(self.P)
        # Statikus EKF
        hx_s = np.array([self._x_static[self.IX_THETA]])
        innov_s = z - hx_s
        innov_s[0] = self._wrap(innov_s[0])
        S_s = H @ self._P_static @ H.T + R
        K_s = self._P_static @ H.T @ np.linalg.inv(S_s)
        self._x_static = self._x_static + (K_s @ innov_s).flatten()
        self._x_static[self.IX_THETA] = self._wrap(self._x_static[self.IX_THETA])
        self._P_static = (np.eye(self.N) - K_s @ H) @ self._P_static
        self._ensure_P_valid(self._P_static)
        return self.get_state()

    def update_velocity_measurement(self, v_meas: float, r_var: Optional[float] = None):
        """
        Külső lineáris sebesség mérés (pl. LIDAR pose delta / dt) frissítése.
        Csak a v állapotot korrigálja, így LIDAR_FIRST módban csökkenti az
        accel-integrációból adódó driftet encoder integráció nélkül.
        """
        self._v_lidar_soft_applied_last = False
        self._v_lidar_soft_r_scale_last = 1.0
        try:
            z_v = float(v_meas)
        except (TypeError, ValueError):
            self._v_lidar_gate_reject = False
            self._v_lidar_nis_last = 0.0
            return self.get_state()
        if not math.isfinite(z_v):
            self._v_lidar_gate_reject = False
            self._v_lidar_nis_last = 0.0
            return self.get_state()

        r_eff = float(self._R_v_lidar if r_var is None else r_var)
        if not math.isfinite(r_eff):
            r_eff = float(self._R_v_lidar)
        r_eff = max(1e-6, r_eff)

        H = np.zeros((1, self.N))
        H[0, self.IX_V] = 1.0
        z = np.array([z_v])
        hx = np.array([self.x[self.IX_V]])
        innov = z - hx
        self._last_innov_v_lidar = float(innov[0])

        R = np.array([[r_eff]])
        S = H @ self.P @ H.T + R
        v_nis = self._innovation_nis(innov, S)
        self._v_lidar_nis_last = float(v_nis)
        self._v_lidar_gate_reject = bool(self._gate_enabled and v_nis > self._v_lidar_nis_max)
        if self._v_lidar_gate_reject:
            nis_limit = max(1e-9, float(self._v_lidar_nis_max))
            nis_ratio = float(v_nis) / nis_limit if math.isfinite(v_nis) else float("inf")
            if math.isfinite(nis_ratio) and nis_ratio <= self._v_lidar_soft_nis_ratio_max:
                soft_scale = min(self._v_lidar_soft_r_scale_max, max(1.0, nis_ratio))
                R = np.array([[r_eff * soft_scale]])
                S = H @ self.P @ H.T + R
                self._v_lidar_nis_last = float(self._innovation_nis(innov, S))
                self._v_lidar_gate_reject = False
                self._v_lidar_soft_applied_last = True
                self._v_lidar_soft_r_scale_last = float(soft_scale)
            else:
                return self.get_state()

        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ innov).flatten()
        self.P = (np.eye(self.N) - K @ H) @ self.P
        self._ensure_P_valid(self.P)

        # Statikus EKF ág
        hx_s = np.array([self._x_static[self.IX_V]])
        innov_s = z - hx_s
        S_s = H @ self._P_static @ H.T + R
        K_s = self._P_static @ H.T @ np.linalg.inv(S_s)
        self._x_static = self._x_static + (K_s @ innov_s).flatten()
        self._P_static = (np.eye(self.N) - K_s @ H) @ self._P_static
        self._ensure_P_valid(self._P_static)
        return self.get_state()

    def update_lidar(
        self,
        x_lidar: float,
        y_lidar: float,
        theta_lidar: float,
        confidence: float,
        r_scale: Optional[float] = None,
        preserve_theta: bool = False,
        preserve_position: bool = False,
    ):
        """
        LIDAR alapú abszolút pozíciókorrekció.
        z_lidar = [x_lidar, y_lidar, theta_lidar], h(x) = [px, py, theta].
        Structured resultot ad vissza, hogy a hívó lássa: alkalmazva lett-e
        a mérés, vagy elutasítottuk és miért.
        A px, py (és theta) bizonytalanságot (P00, P11, P22) csökkenti.
        A pontfelhőt ne ide add: a már kiszámolt LIDAR pozíciót (pl. scan matching + előző pose).
        """
        self._lidar_soft_applied_last = False
        self._lidar_soft_r_scale_last = 1.0
        try:
            x = float(x_lidar)
            y = float(y_lidar)
            theta = self._wrap(float(theta_lidar))
            confidence_value = float(confidence)
            if r_scale is None:
                r_scale_value = 1.0
            else:
                r_scale_value = float(r_scale)
        except (TypeError, ValueError):
            self._lidar_gate_reject = False
            self._lidar_nis_last = 0.0
            return self._set_lidar_update_result(applied=False, reject_reason="rejected_invalid", r_scale=1.0)
        if not math.isfinite(r_scale_value) or r_scale_value <= 0.0:
            r_scale_value = 1.0
        r_scale_value = min(20.0, max(0.05, r_scale_value))

        if not all(math.isfinite(v) for v in (x, y, theta, confidence_value)):
            self._lidar_gate_reject = False
            self._lidar_nis_last = 0.0
            return self._set_lidar_update_result(
                applied=False,
                reject_reason="rejected_invalid",
                confidence=confidence_value if math.isfinite(confidence_value) else None,
                r_scale=r_scale_value,
            )

        if confidence_value < self._lidar_confidence_threshold:
            self._lidar_gate_reject = False
            self._lidar_nis_last = 0.0
            return self._set_lidar_update_result(
                applied=False,
                reject_reason="rejected_low_confidence",
                confidence=confidence_value,
                r_scale=r_scale_value,
            )

        theta_before_update = float(self.x[self.IX_THETA])
        theta_static_before_update = float(self._x_static[self.IX_THETA])
        position_before_update = (float(self.x[self.IX_PX]), float(self.x[self.IX_PY]))
        position_static_before_update = (
            float(self._x_static[self.IX_PX]),
            float(self._x_static[self.IX_PY]),
        )
        z = np.array([x, y, theta])
        hx = np.array([
            self.x[self.IX_PX],
            self.x[self.IX_PY],
            self.x[self.IX_THETA],
        ])
        H = np.zeros((3, self.N))
        H[0, self.IX_PX] = 1.0
        H[1, self.IX_PY] = 1.0
        H[2, self.IX_THETA] = 1.0

        innov = z - hx
        innov[2] = self._wrap(innov[2])
        if not np.isfinite(innov).all():
            self._lidar_gate_reject = False
            self._lidar_nis_last = 0.0
            return self._set_lidar_update_result(
                applied=False,
                reject_reason="rejected_invalid",
                confidence=confidence_value,
                r_scale=r_scale_value,
            )

        R_lidar = self._R_lidar * r_scale_value
        S = H @ self.P @ H.T + R_lidar
        lidar_nis = self._innovation_nis(innov, S)
        self._lidar_nis_last = lidar_nis
        self._lidar_gate_reject = bool(self._gate_enabled and lidar_nis > self._lidar_nis_max)
        if self._lidar_gate_reject:
            nis_limit = max(1e-9, float(self._lidar_nis_max))
            nis_ratio = float(lidar_nis) / nis_limit if math.isfinite(lidar_nis) else float("inf")
            if math.isfinite(nis_ratio) and nis_ratio <= self._lidar_soft_nis_ratio_max:
                soft_scale = min(self._lidar_soft_r_scale_max, max(1.0, nis_ratio))
                self._lidar_soft_applied_last = True
                self._lidar_soft_r_scale_last = float(soft_scale)
                r_scale_value = float(r_scale_value * soft_scale)
                R_lidar = self._R_lidar * r_scale_value
                S = H @ self.P @ H.T + R_lidar
                lidar_nis = self._innovation_nis(innov, S)
                self._lidar_nis_last = float(lidar_nis)
                self._lidar_gate_reject = False
            else:
                return self._set_lidar_update_result(
                    applied=False,
                    reject_reason="rejected_nis",
                    confidence=confidence_value,
                    r_scale=r_scale_value,
                    innovation=innov,
                    nis=lidar_nis,
                )
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ innov).flatten()
        if preserve_position:
            self.x[self.IX_PX], self.x[self.IX_PY] = position_before_update
        if preserve_theta:
            self.x[self.IX_THETA] = theta_before_update
        self.x[self.IX_THETA] = self._wrap(self.x[self.IX_THETA])
        self.P = (np.eye(self.N) - K @ H) @ self.P
        self._ensure_P_valid(self.P)

        # Statikus EKF: ugyanaz a LIDAR update
        hx_s = np.array([
            self._x_static[self.IX_PX],
            self._x_static[self.IX_PY],
            self._x_static[self.IX_THETA],
        ])
        innov_s = z - hx_s
        innov_s[2] = self._wrap(innov_s[2])
        S_s = H @ self._P_static @ H.T + R_lidar
        K_s = self._P_static @ H.T @ np.linalg.inv(S_s)
        self._x_static = self._x_static + (K_s @ innov_s).flatten()
        if preserve_position:
            self._x_static[self.IX_PX], self._x_static[self.IX_PY] = position_static_before_update
        if preserve_theta:
            self._x_static[self.IX_THETA] = theta_static_before_update
        self._x_static[self.IX_THETA] = self._wrap(self._x_static[self.IX_THETA])
        self._P_static = (np.eye(self.N) - K_s @ H) @ self._P_static
        self._ensure_P_valid(self._P_static)

        return self._set_lidar_update_result(
            applied=True,
            confidence=confidence_value,
            r_scale=r_scale_value,
            innovation=innov,
            nis=lidar_nis,
            soft_applied=bool(self._lidar_soft_applied_last),
            soft_r_scale=float(self._lidar_soft_r_scale_last),
        )

    def get_state(self):
        """Állapot + telemetria; opcionálisan adaptivity_mode; statikus + Q/R a teljes EKF loghoz."""
        d = {
            "pose_frame_id": POSE_FRAME_ID,
            "pose_frame_owner": POSE_FRAME_OWNER,
            "pose_frame_contract": pose_frame_contract(),
            "x": float(self.x[self.IX_PX]),
            "y": float(self.x[self.IX_PY]),
            "theta": float(self.x[self.IX_THETA]),
            "theta_deg": math.degrees(float(self.x[self.IX_THETA])),
            "v": float(self.x[self.IX_V]),
            "omega_rad_s": float(getattr(self, "_last_omega_rad_s", 0.0)),
            "P": self.P.tolist(),
            "theta_gyro": math.degrees(self._theta_gyro),
            "theta_enc": math.degrees(self._theta_enc),
            "theta_fused": math.degrees(float(self.x[self.IX_THETA])),
            "v_enc": self._last_v_enc,
            "v_fused": float(self.x[self.IX_V]),
            "gyro_bias": float(self.x[self.IX_GYRO_BIAS]),
            # Statikus EKF (fix Q/R) és adaptív réteg paraméterek – finomhangolás log
            "theta_fused_static": math.degrees(float(self._x_static[self.IX_THETA])),
            "v_fused_static": float(self._x_static[self.IX_V]),
            "Q_bias": float(self._Q_current[self.IX_GYRO_BIAS, self.IX_GYRO_BIAS]),
            "R_theta_enc": float(self._R_enc_current[1, 1]),
            "R_enc_v": float(self._R_enc_current[0, 0]),
            "R_zupt": self._R_zupt,
            "R_theta_hold": self._R_theta_hold,
            "R_theta_hold_eff": float(getattr(self, "_last_R_theta_hold_eff", self._R_theta_hold)),
            "EKF_mode": self._mode,
            # Innovációk és konzisztencia (finomhangolás, gating validáció)
            "innovation_v": self._last_innovation_v,
            "innovation_theta": self._last_innovation_theta,
            "S_enc_diag": list(self._last_S_enc_diag),
            "innovation_zupt": self._last_innov_zupt,
            "innovation_theta_hold": self._last_innov_theta_hold,
            "innovation_v_lidar": self._last_innov_v_lidar,
            "zupt_applied": bool(self._zupt_applied_last),
            "theta_hold_applied": bool(self._theta_hold_applied_last),
            "slip_this_cycle": self._slip_this_cycle,
            "enc_gate_reject": self._enc_gate_reject,
            "lidar_gate_reject": self._lidar_gate_reject,
            "lidar_soft_applied": bool(self._lidar_soft_applied_last),
            "lidar_soft_r_scale": float(self._lidar_soft_r_scale_last),
            "v_lidar_gate_reject": self._v_lidar_gate_reject,
            "v_lidar_soft_applied": bool(self._v_lidar_soft_applied_last),
            "v_lidar_soft_r_scale": float(self._v_lidar_soft_r_scale_last),
            "enc_nis": self._enc_nis_last,
            "lidar_nis": self._lidar_nis_last,
            "v_lidar_nis": self._v_lidar_nis_last,
            "lidar_confidence_threshold": float(self._lidar_confidence_threshold),
            "lidar_update": dict(getattr(self, "_last_lidar_update", {}) or {}),
            "encoder_trust_mode": str(getattr(self, "_encoder_trust_mode", "NORMAL")),
            "encoder_r_scale": float(getattr(self, "_encoder_r_scale_external", 1.0)),
            "encoder_update_skip_reason": str(getattr(self, "_encoder_update_skip_reason", "")),
            "encoder_context_flags": dict(getattr(self, "_encoder_context_flags", {}) or {}),
            "encoder_enabled": bool(getattr(self, "_encoder_enabled", True)),
            "encoder_usage_gain": float(getattr(self, "_encoder_usage_gain", 1.0)),
            "encoder_confidence_hint": float(getattr(self, "_encoder_confidence_hint", 1.0)),
            "encoder_covariance_hint": float(getattr(self, "_encoder_covariance_hint", 1.0)),
        }
        if self._adaptivity_enabled:
            d["adaptivity_mode"] = self._mode
            if self._online_learning:
                d["slip_ema"] = round(self._slip_ema, 4)
                d["inno_theta_ema_rad"] = round(self._inno_theta_ema, 5)
        return d

    @property
    def zupt_threshold(self):
        return self._zupt_threshold

    @property
    def still_this_cycle(self):
        """True ha az aktuális ciklusban still mód van (theta_hold hívható)."""
        return self._still_this_cycle

    def get_covariance(self):
        """Returns the current covariance matrix P."""
        return self.P.copy()

    def get_innovation(self):
        """Returns the last innovation vector [v, theta]."""
        return np.array([self._last_innovation_v, self._last_innovation_theta])

    def _apply_encoder_trust_context(
        self,
        *,
        trust_ctx: dict,
        v_l: float,
        v_r: float,
        v_cmd: float,
        v_target: float,
    ) -> tuple:
        """
        Context-aware encoder trust adaptation.
        Returns: (v_l_eff, v_r_eff, skip_encoder_update, skip_reason, usage_mode)
        """
        self._encoder_r_scale_external = 1.0
        self._encoder_update_skip_reason = ""
        self._encoder_trust_mode = "NORMAL"
        self._encoder_context_flags = dict(trust_ctx or {})
        self._encoder_confidence_hint = 1.0
        self._encoder_covariance_hint = 1.0

        if not self._trust_adaptation_enabled or not isinstance(trust_ctx, dict):
            return v_l, v_r, False, "", "NORMAL"

        stale = bool(trust_ctx.get("snapshot_stale", False))
        motor_off = bool(trust_ctx.get("motor_off", False))
        cmd_idle = bool(trust_ctx.get("cmd_idle", False))
        idle_false_pulse = bool(trust_ctx.get("idle_false_pulse", False))
        asym = float(trust_ctx.get("side_asymmetry", 0.0) or 0.0)
        anomaly = bool(trust_ctx.get("anomaly_active", False))
        trust = float(trust_ctx.get("combined_trust", 1.0) or 1.0)
        self._encoder_confidence_hint = max(0.0, min(1.0, trust))
        cov_hint = float(trust_ctx.get("ekf_covariance_scale_hint", 1.0) or 1.0)
        cov_hint = max(1.0, min(30.0, cov_hint))
        self._encoder_covariance_hint = cov_hint
        self._encoder_r_scale_external *= cov_hint
        usage_mode = str(trust_ctx.get("ekf_usage_mode", "NORMAL") or "NORMAL").upper()
        usage_reason = str(trust_ctx.get("ekf_usage_reason", "") or "")

        if usage_mode == "REJECT":
            self._encoder_trust_mode = "CONTEXT_REJECT"
            reason = usage_reason or "CONTEXT_REJECT"
            self._encoder_update_skip_reason = reason
            return v_l, v_r, True, reason, usage_mode
        if usage_mode == "DEGRADED":
            self._encoder_trust_mode = "CONTEXT_DEGRADED"
            self._encoder_r_scale_external *= max(1.0, self._trust_anomaly_r_scale)
        if usage_mode == "THETA_ONLY":
            self._encoder_trust_mode = "THETA_ONLY"
            self._encoder_r_scale_external *= max(1.0, self._trust_asymmetry_r_scale)

        if stale:
            self._encoder_trust_mode = "STALE_SKIP"
            self._encoder_update_skip_reason = "ENCODER_STALE"
            return v_l, v_r, True, "ENCODER_STALE", usage_mode

        if motor_off and cmd_idle:
            self._encoder_trust_mode = "IDLE_CONTEXT"
            self._encoder_r_scale_external *= max(1.0, self._trust_idle_r_scale)
            if idle_false_pulse:
                # Motor-off noise: treat as static context.
                v_l = 0.0
                v_r = 0.0
                self._encoder_update_skip_reason = "IDLE_NOISE_CLAMP"

        if asym >= self._trust_asymmetry_warn:
            self._encoder_trust_mode = "ASYMMETRY_DEGRADED"
            asym_ratio = 1.0 + min(2.0, max(0.0, asym - self._trust_asymmetry_warn))
            self._encoder_r_scale_external *= max(1.0, self._trust_asymmetry_r_scale * asym_ratio)
            if asym >= self._trust_asymmetry_critical and abs(v_cmd) < 0.05 and abs(v_target) < 0.05:
                v_l = 0.0
                v_r = 0.0
                self._encoder_update_skip_reason = "ASYMMETRY_IDLE_CLAMP"

        if anomaly:
            self._encoder_trust_mode = "ANOMALY_DEGRADED"
            self._encoder_r_scale_external *= max(1.0, self._trust_anomaly_r_scale)

        if trust <= self._trust_skip_threshold and abs(v_cmd) < 0.03 and abs(v_target) < 0.03:
            self._encoder_trust_mode = "LOW_TRUST_SKIP"
            self._encoder_update_skip_reason = "LOW_TRUST_IDLE"
            return v_l, v_r, True, "LOW_TRUST_IDLE", usage_mode

        self._encoder_r_scale_external = max(1.0, min(self._encoder_r_scale_external, 30.0))

        return v_l, v_r, False, self._encoder_update_skip_reason, usage_mode

    def update(self, imu, encoder, dt):
        """
        Unified update method for Shadow EKF architecture.
        imu: {'accel_x': float, 'gyro_z': float}
        encoder: {'v_l': float, 'v_r': float, 'v_cmd': float, 'v_target': float, 'theta_enc': float}
        """
        if dt <= 0:
            return self.get_state()

        self._zupt_applied_last = False
        self._theta_hold_applied_last = False
        self._v_lidar_soft_applied_last = False
        self._v_lidar_soft_r_scale_last = 1.0

        accel_x = imu.get('accel_x', 0.0)
        gyro_z = imu.get('gyro_z', 0.0)
        v_l = encoder.get('v_l', 0.0)
        v_r = encoder.get('v_r', 0.0)
        v_cmd = encoder.get('v_cmd', 0.0)
        v_target = encoder.get('v_target', 0.0)
        theta_enc = encoder.get('theta_enc')
        encoder_enabled = bool(encoder.get("enabled", True))
        encoder_usage_gain = max(0.0, min(1.0, float(encoder.get("usage_gain", 1.0) or 0.0)))
        self._encoder_enabled = bool(encoder_enabled)
        self._encoder_usage_gain = float(encoder_usage_gain)
        trust_ctx = encoder.get("trust") if isinstance(encoder, dict) else {}
        theta_measurement_reliable = True
        if isinstance(trust_ctx, dict):
            theta_measurement_reliable = bool(trust_ctx.get("theta_measurement_reliable", True))

        # 1. Adaptivity
        v_cmd_for_ekf = v_cmd
        if abs(v_target) > abs(v_cmd_for_ekf):
            v_cmd_for_ekf = v_target

        v_l_eff, v_r_eff, skip_encoder_update, skip_reason, usage_mode = self._apply_encoder_trust_context(
            trust_ctx=(trust_ctx if isinstance(trust_ctx, dict) else {}),
            v_l=v_l,
            v_r=v_r,
            v_cmd=v_cmd_for_ekf,
            v_target=v_target,
        )
        if encoder_usage_gain < 0.999:
            v_l_eff *= encoder_usage_gain
            v_r_eff *= encoder_usage_gain
            blend_r_scale = 1.0 / max(0.2, encoder_usage_gain)
            self._encoder_r_scale_external *= max(1.0, blend_r_scale)
            if self._encoder_trust_mode == "NORMAL":
                self._encoder_trust_mode = "BLEND"
        if (not encoder_enabled) and encoder_usage_gain <= 1e-3:
            skip_encoder_update = True
            skip_reason = "ENCODER_DISABLED"
            usage_mode = "DISABLED"
            theta_measurement_reliable = False
            self._encoder_trust_mode = "DISABLED"

        th = self._still_th
        cmd_still = abs(v_cmd_for_ekf) < th and abs(v_target) < th
        canonical_state = str((trust_ctx or {}).get("canonical_state", "") or "").upper()
        idle_noise = bool((trust_ctx or {}).get("idle_false_pulse", False))
        if canonical_state == "IDLE":
            v_l_ekf, v_r_ekf = 0.0, 0.0
        elif cmd_still and idle_noise and abs(v_l_eff) < 0.12 and abs(v_r_eff) < 0.12:
            v_l_ekf, v_r_ekf = 0.0, 0.0
        else:
            v_l_ekf, v_r_ekf = v_l_eff, v_r_eff

        self.update_adaptivity(v_l_ekf, v_r_ekf, v_cmd_for_ekf, accel_x, gyro_z, dt)

        # 2. Predict
        self.predict(accel_x, gyro_z, dt)

        # 3. Update Encoders
        still_for_zupt = (
            abs(v_l_ekf) < th
            and abs(v_r_ekf) < th
            and abs(v_cmd_for_ekf) < th
            and abs(v_target) < th
        )

        if still_for_zupt:
            if not skip_encoder_update:
                if theta_measurement_reliable:
                    self.update_encoders(0.0, 0.0, dt, theta_enc_rad=theta_enc)
                else:
                    # Fontos: ha theta mérés explicit tiltott, ne fusson le az
                    # encoder update fallback theta (theta_prev + omega_enc*dt),
                    # mert ez kézi (motor-off) forgatásnál visszahúzza a yaw-t.
                    self._encoder_update_skip_reason = "THETA_SUPPRESSED"
            self.update_zupt()
            if abs(float(getattr(self, "_last_omega_rad_s", 0.0))) <= self._theta_hold_disable_gyro_min:
                self.update_theta_hold()
        else:
            if not skip_encoder_update:
                theta_input = theta_enc if theta_measurement_reliable else None
                if usage_mode == "THETA_ONLY":
                    self.update_encoders(0.0, 0.0, dt, theta_enc_rad=theta_input)
                    self._encoder_update_skip_reason = "THETA_ONLY" if theta_measurement_reliable else "THETA_SUPPRESSED"
                else:
                    self.update_encoders(v_l_ekf, v_r_ekf, dt, theta_enc_rad=theta_input)
                    if not theta_measurement_reliable:
                        self._encoder_update_skip_reason = "THETA_SUPPRESSED"
            else:
                self._encoder_update_skip_reason = skip_reason or "TRUST_SKIP"

        return self.get_state()
