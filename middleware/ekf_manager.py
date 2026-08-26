from __future__ import annotations

from collections import deque
from typing import Any, Dict, Tuple

import numpy as np

from middleware.ekf import ExtendedKalmanFilter


class EKFManager:
    """
    Dual EKF manager (live + shadow).

    Data flow and safety isolation:
    - Same IMU/encoder snapshots are fed into both filters in the same tick.
    - Live EKF is always updated; control uses only live state.
    - Shadow update can be skipped if loop budget is exceeded.
    - Shadow cannot write back into control outputs unless explicit validated apply happens.
    """

    def __init__(self, wheel_base: float, live_config: dict, shadow_config: dict):
        self.wheel_base = float(wheel_base)
        self.live_config = dict(live_config or {})
        self.shadow_config = dict(shadow_config or {})

        self.ekf_live = ExtendedKalmanFilter(self.wheel_base, self.live_config)
        self.ekf_shadow = ExtendedKalmanFilter(self.wheel_base, self.shadow_config)

        val_cfg = (self.live_config.get("shadow_validation") or {}) if isinstance(self.live_config, dict) else {}
        self._window_frames = int(max(10, val_cfg.get("window_frames", 60)))
        self._th_cov_det = float(val_cfg.get("cov_det_threshold", 1e-4))
        self._th_innov_mean = float(val_cfg.get("innovation_mean_threshold", 0.08))
        self._th_cov_growth = float(val_cfg.get("cov_growth_rate_threshold", 4e-3))
        self._th_yaw_step = float(val_cfg.get("yaw_step_std_deg_threshold", 8.0))
        self._stable_required = int(max(10, val_cfg.get("stable_frames_required", 40)))
        self._shadow_live_yaw_divergence_deg = float(val_cfg.get("shadow_live_yaw_divergence_deg", 45.0))
        self._shadow_live_pos_divergence_m = float(val_cfg.get("shadow_live_pos_divergence_m", 1.5))
        self._shadow_divergence_frames_required = int(max(1, val_cfg.get("shadow_divergence_frames_required", 20)))
        self._auto_resync_shadow = bool(val_cfg.get("auto_resync_shadow", True))
        self._shadow_divergence_frames = 0
        self._last_shadow_divergence = {
            "yaw_diff_deg": 0.0,
            "pos_diff_m": 0.0,
            "resynced": False,
        }

        self.validation_status = "initializing"
        self.validation_state: Dict[str, Any] = {
            "error_code": "",
            "stable_frames": 0,
            "mean_innovation": 0.0,
            "covariance_growth_rate": 0.0,
            "yaw_stability": 0.0,
            "cov_det": 0.0,
        }

        self._innov_abs_hist = deque(maxlen=self._window_frames)
        self._cov_det_hist = deque(maxlen=self._window_frames)
        self._yaw_hist = deque(maxlen=self._window_frames)

        # Diagnosztika a get_telemetry / check_ekf_tune_ready számára
        self._last_dt_stats = None
        self._last_noise_stats = None
        self._last_sensor_ok = True

    def set_diagnostics(
        self,
        dt_stats: dict = None,
        noise_stats: dict = None,
        sensor_ok: bool = True,
    ) -> None:
        """Control loop által hívva: dt és zaj statisztika, szenzor véges flag."""
        if dt_stats is not None:
            self._last_dt_stats = dt_stats
        if noise_stats is not None:
            self._last_noise_stats = noise_stats
        self._last_sensor_ok = bool(sensor_ok)

    def update(self, imu: dict, encoder: dict, dt: float, loop_duration: float = 0.0):
        """
        50Hz loop: live és shadow mindig ugyanazzal a bemenettel frissül (determinisztikus tuning).
        """
        live_state = self.ekf_live.update(imu, encoder, dt)
        shadow_state = self.ekf_shadow.update(imu, encoder, dt)
        shadow_state = self._guard_shadow_divergence(live_state, shadow_state)
        self._update_validation_metrics(shadow_state)
        return live_state, shadow_state, False

    @staticmethod
    def _yaw_diff_deg_wrapped(live_yaw_deg: float, shadow_yaw_deg: float) -> float:
        diff = float(live_yaw_deg) - float(shadow_yaw_deg)
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0
        return diff

    def resync_shadow(self) -> None:
        """Public: resync shadow EKF from current live state (call after live reset)."""
        live_state = self.ekf_live.get_state()
        self._resync_shadow_from_live(live_state)

    def _resync_shadow_from_live(self, live_state: Dict[str, Any]) -> Dict[str, Any]:
        self.ekf_shadow = ExtendedKalmanFilter(self.wheel_base, self.shadow_config)
        self.ekf_shadow.reset(
            px=float(live_state.get("x", 0.0)),
            py=float(live_state.get("y", 0.0)),
            theta=float(live_state.get("theta", 0.0)),
            v=float(live_state.get("v", 0.0)),
            gyro_bias=float(live_state.get("gyro_bias", 0.0)),
        )
        self.validation_status = "initializing"
        self.validation_state["stable_frames"] = 0
        self.validation_state["error_code"] = "SHADOW_RESYNC"
        self._innov_abs_hist.clear()
        self._cov_det_hist.clear()
        self._yaw_hist.clear()
        return self.ekf_shadow.get_state()

    def _guard_shadow_divergence(self, live_state: Dict[str, Any], shadow_state: Dict[str, Any]) -> Dict[str, Any]:
        live_yaw = float(live_state.get("theta_deg", 0.0))
        shadow_yaw = float(shadow_state.get("theta_deg", 0.0))
        yaw_diff = self._yaw_diff_deg_wrapped(live_yaw, shadow_yaw)

        live_x = float(live_state.get("x", 0.0))
        live_y = float(live_state.get("y", 0.0))
        shadow_x = float(shadow_state.get("x", 0.0))
        shadow_y = float(shadow_state.get("y", 0.0))
        pos_diff = float(np.hypot(live_x - shadow_x, live_y - shadow_y))

        diverged = (
            abs(yaw_diff) > self._shadow_live_yaw_divergence_deg
            or pos_diff > self._shadow_live_pos_divergence_m
        )
        self._last_shadow_divergence = {
            "yaw_diff_deg": float(yaw_diff),
            "pos_diff_m": float(pos_diff),
            "resynced": False,
        }

        if not diverged:
            self._shadow_divergence_frames = 0
            return shadow_state

        self._shadow_divergence_frames += 1
        self.validation_status = "unstable"
        self.validation_state["error_code"] = "SHADOW_LIVE_DIVERGENCE"
        self.validation_state["stable_frames"] = 0

        if not self._auto_resync_shadow:
            return shadow_state
        if self._shadow_divergence_frames < self._shadow_divergence_frames_required:
            return shadow_state

        self._shadow_divergence_frames = 0
        self._last_shadow_divergence["resynced"] = True
        return self._resync_shadow_from_live(live_state)

    def _update_validation_metrics(self, shadow_state: dict) -> None:
        P = self.ekf_shadow.get_covariance()

        theta = float(shadow_state.get("theta", 0.0))
        innov_v = abs(float(shadow_state.get("innovation_v", 0.0)))
        innov_t = abs(float(shadow_state.get("innovation_theta", 0.0)))
        innov_mag = max(innov_v, innov_t)

        cov_det = float(np.linalg.det(P))

        self._innov_abs_hist.append(innov_mag)
        self._cov_det_hist.append(cov_det)
        self._yaw_hist.append(theta)

        if len(self._innov_abs_hist) < self._window_frames:
            self.validation_status = "initializing"
            self.validation_state["error_code"] = ""
            return

        if not np.isfinite(P).all():
            self._mark_unstable("COVARIANCE_UNSTABLE")
            return
        if not all(np.isfinite(float(shadow_state.get(k, 0.0))) for k in ("x", "y", "theta", "v", "gyro_bias")):
            self._mark_unstable("EKF_DIVERGENCE")
            return
        if not np.isfinite(cov_det) or cov_det <= 0.0 or cov_det > self._th_cov_det:
            self._mark_unstable("COVARIANCE_UNSTABLE")
            return

        mean_innov = float(np.mean(self._innov_abs_hist))
        cov_growth = float(np.mean(np.diff(self._cov_det_hist))) if len(self._cov_det_hist) > 1 else 0.0
        yaw_steps = np.diff(np.array(self._yaw_hist, dtype=float))
        yaw_steps = ((yaw_steps + np.pi) % (2.0 * np.pi)) - np.pi
        yaw_stability_deg = float(np.std(np.degrees(yaw_steps))) if len(yaw_steps) else 0.0

        self.validation_state.update(
            {
                "mean_innovation": mean_innov,
                "covariance_growth_rate": cov_growth,
                "yaw_stability": yaw_stability_deg,
                "cov_det": cov_det,
            }
        )

        if mean_innov > self._th_innov_mean:
            self._mark_unstable("INNOVATION_SPIKE")
            return
        if cov_growth > self._th_cov_growth:
            self._mark_unstable("COVARIANCE_UNSTABLE")
            return
        if not np.isfinite(yaw_stability_deg) or yaw_stability_deg > self._th_yaw_step:
            self._mark_unstable("EKF_DIVERGENCE")
            return

        self.validation_state["error_code"] = ""
        self.validation_state["stable_frames"] = int(self.validation_state.get("stable_frames", 0)) + 1
        self.validation_status = "stable" if self.validation_state["stable_frames"] >= self._stable_required else "initializing"

    def _mark_unstable(self, error_code: str) -> None:
        self.validation_status = "unstable"
        self.validation_state["error_code"] = str(error_code)
        self.validation_state["stable_frames"] = 0

    def check_ekf_tune_ready(self) -> dict:
        """
        EKF tuning készültségi ellenőrzés.
        Visszaadja: ready, timing_ok, noise_stats_ok, shadow_ok, telemetry_ok.
        """
        dt_stats = self._last_dt_stats or {}
        noise_stats = self._last_noise_stats or {}
        std_dt = float(dt_stats.get("std_dt", 1.0))
        timing_ok = std_dt < 0.002  # 2 ms
        sensor_finite = self._last_sensor_ok
        noise_stats_ok = (
            noise_stats.get("gyro_var") is not None
            and noise_stats.get("accel_var") is not None
            and noise_stats.get("encoder_var") is not None
        )
        live = self.ekf_live.get_state()
        shadow = self.ekf_shadow.get_state()
        live_yaw_deg = float(live.get("theta_deg", 0.0))
        shadow_yaw_deg = float(shadow.get("theta_deg", 0.0))
        yaw_diff_deg = abs(live_yaw_deg - shadow_yaw_deg)
        # normalizálás -180..180
        if yaw_diff_deg > 180:
            yaw_diff_deg = 360 - yaw_diff_deg
        shadow_ok = yaw_diff_deg < 2.0  # 2 fok küszöb
        telemetry_ok = True  # ha itt vagyunk, a pipeline fut
        ready = timing_ok and sensor_finite and noise_stats_ok and shadow_ok and telemetry_ok
        return {
            "ready": ready,
            "timing_ok": timing_ok,
            "noise_stats_ok": noise_stats_ok,
            "shadow_ok": shadow_ok,
            "telemetry_ok": telemetry_ok,
        }

    def get_telemetry(self):
        live = self.ekf_live.get_state()
        shadow = self.ekf_shadow.get_state()

        P = self.ekf_live.get_covariance()
        live_cov = np.diag(P).tolist()
        shadow_cov = np.diag(self.ekf_shadow.get_covariance()).tolist()

        live_yaw = float(live.get("theta_deg", 0.0))
        shadow_yaw = float(shadow.get("theta_deg", 0.0))
        live_x = float(live.get("x", 0.0))
        live_y = float(live.get("y", 0.0))
        shadow_x = float(shadow.get("x", 0.0))
        shadow_y = float(shadow.get("y", 0.0))
        pos_diff = float(np.hypot(live_x - shadow_x, live_y - shadow_y))

        dt_stats = self._last_dt_stats or {}
        noise_stats = self._last_noise_stats or {}

        # Kovariancia egyes elemek (P: px, py, theta, v, gyro_bias -> index 2=theta, 3=v)
        covariance_trace = float(np.trace(P))
        covariance_theta = float(P[2, 2]) if P.shape[0] > 2 else 0.0
        covariance_velocity = float(P[3, 3]) if P.shape[0] > 3 else 0.0

        q_diag = list(self.shadow_config.get("Q_diag", [0.0, 0.0, 0.0, 0.0, 0.0]))
        r_enc = list(self.shadow_config.get("R_enc", [0.0, 0.0]))

        ekf_tune_ready = self.check_ekf_tune_ready()

        return {
            "live": {
                "yaw": live_yaw,
                "position": [live_x, live_y],
                "velocity": float(live.get("v", 0.0)),
                "cov_diag": live_cov,
                "innovation": [float(live.get("innovation_v", 0.0)), float(live.get("innovation_theta", 0.0))],
            },
            "shadow": {
                "yaw": shadow_yaw,
                "position": [shadow_x, shadow_y],
                "velocity": float(shadow.get("v", 0.0)),
                "cov_diag": shadow_cov,
                "innovation": [float(shadow.get("innovation_v", 0.0)), float(shadow.get("innovation_theta", 0.0))],
            },
            "yaw_diff": live_yaw - shadow_yaw,
            "shadow_live_yaw_diff": live_yaw - shadow_yaw,
            "shadow_live_pos_diff": pos_diff,
            "validation_status": self.validation_status,
            "shadow_params": {
                "Q_yaw": float(q_diag[2] if len(q_diag) > 2 else 0.0),
                "Q_velocity": float(q_diag[3] if len(q_diag) > 3 else 0.0),
                "R_gyro": float(self.shadow_config.get("R_gyro", 0.0)),
                "R_encoder": float(r_enc[0] if len(r_enc) > 0 else 0.0),
                "ZUPT_threshold": float(self.shadow_config.get("zupt_threshold", 0.0)),
            },
            "validation_metrics": {
                "mean_innovation": float(self.validation_state.get("mean_innovation", 0.0)),
                "covariance_growth_rate": float(self.validation_state.get("covariance_growth_rate", 0.0)),
                "yaw_stability": float(self.validation_state.get("yaw_stability", 0.0)),
                "stable_frames": int(self.validation_state.get("stable_frames", 0)),
                "error_code": str(self.validation_state.get("error_code", "")),
            },
            "shadow_divergence": {
                "yaw_diff_deg": float(self._last_shadow_divergence.get("yaw_diff_deg", 0.0)),
                "pos_diff_m": float(self._last_shadow_divergence.get("pos_diff_m", 0.0)),
                "frames": int(self._shadow_divergence_frames),
                "yaw_threshold_deg": float(self._shadow_live_yaw_divergence_deg),
                "pos_threshold_m": float(self._shadow_live_pos_divergence_m),
                "auto_resync": bool(self._auto_resync_shadow),
                "resynced": bool(self._last_shadow_divergence.get("resynced", False)),
            },
            # EKF diagnosztika a GUI-hoz
            "ekf_yaw": live_yaw,
            "ekf_position_x": live_x,
            "ekf_position_y": live_y,
            "ekf_velocity": float(live.get("v", 0.0)),
            "innovation_v": float(live.get("innovation_v", 0.0)),
            "innovation_theta": float(live.get("innovation_theta", 0.0)),
            "covariance_trace": covariance_trace,
            "covariance_theta": covariance_theta,
            "covariance_velocity": covariance_velocity,
            "dt_mean": dt_stats.get("mean_dt"),
            "dt_std": dt_stats.get("std_dt"),
            "ekf_timing_warning": dt_stats.get("ekf_timing_warning", False),
            "gyro_var": noise_stats.get("gyro_var"),
            "accel_var": noise_stats.get("accel_var"),
            "encoder_var": noise_stats.get("encoder_var"),
            "ekf_tune_ready": ekf_tune_ready,
        }

    def _translated_shadow_params(self, params: Dict[str, Any]) -> Dict[str, Any]:
        cfg = dict(self.shadow_config)
        q_diag = list(cfg.get("Q_diag", [0.002, 0.002, 0.001, 0.02, 1e-5]))
        if len(q_diag) < 5:
            q_diag = (q_diag + [1e-5] * 5)[:5]
        r_enc = list(cfg.get("R_enc", [0.015, 0.012]))
        if len(r_enc) < 2:
            r_enc = (r_enc + [0.012])[:2]

        if "Q_yaw" in params:
            q_diag[2] = float(params["Q_yaw"])
        if "Q_velocity" in params:
            q_diag[3] = float(params["Q_velocity"])
        if "R_encoder" in params:
            r_enc[0] = float(params["R_encoder"])
            r_enc[1] = float(params["R_encoder"])
        if "R_gyro" in params:
            cfg["R_gyro"] = float(params["R_gyro"])
        if "ZUPT_threshold" in params:
            cfg["zupt_threshold"] = float(params["ZUPT_threshold"])

        cfg["Q_diag"] = q_diag
        cfg["R_enc"] = r_enc
        return cfg

    def update_shadow_params(self, params: dict):
        self.shadow_config = self._translated_shadow_params(dict(params or {}))

        curr_state = self.ekf_shadow.get_state()
        self.ekf_shadow = ExtendedKalmanFilter(self.wheel_base, self.shadow_config)
        self.ekf_shadow.reset(
            px=float(curr_state.get("x", 0.0)),
            py=float(curr_state.get("y", 0.0)),
            theta=float(curr_state.get("theta", 0.0)),
            v=float(curr_state.get("v", 0.0)),
            gyro_bias=float(curr_state.get("gyro_bias", 0.0)),
        )

        self.validation_status = "initializing"
        self.validation_state["stable_frames"] = 0
        self.validation_state["error_code"] = ""
        self._innov_abs_hist.clear()
        self._cov_det_hist.clear()
        self._yaw_hist.clear()

    def apply_shadow_to_live(self) -> Tuple[bool, str]:
        if self.validation_status != "stable":
            code = str(self.validation_state.get("error_code") or "EKF_DIVERGENCE")
            return False, code

        shadow_state = self.ekf_shadow.get_state()
        if not all(np.isfinite(float(shadow_state.get(k, 0.0))) for k in ("x", "y", "theta", "v", "gyro_bias")):
            return False, "EKF_DIVERGENCE"

        self.live_config = dict(self.shadow_config)
        self.ekf_live = ExtendedKalmanFilter(self.wheel_base, self.live_config)
        self.ekf_live.reset(
            px=float(shadow_state.get("x", 0.0)),
            py=float(shadow_state.get("y", 0.0)),
            theta=float(shadow_state.get("theta", 0.0)),
            v=float(shadow_state.get("v", 0.0)),
            gyro_bias=float(shadow_state.get("gyro_bias", 0.0)),
        )
        return True, "OK"
