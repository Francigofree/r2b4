#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF integritás és kalibrációs sanity teszt.

Futtatás:
    python tests/test_ekf_integrity.py

Cél:
- ExtendedKalmanFilter magellenőrzés (Q/R méret, predict/update ágak, gating).
- 1 m lineáris mozgás sanity.
- Skálahiba diagnózis (dt mismatch).
- Strukturált riport: problémák, javasolt fixek, config irányok.
"""

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np

# Projekt gyökér felvétele importhoz
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from middleware.ekf import ExtendedKalmanFilter  # noqa: E402
from control_loop import ControlLoop  # noqa: E402


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str


class _DummyEncoderSnapshot:
    def __init__(self, ts: float, v_l: float, v_r: float, theta_enc: float):
        self.timestamp = ts
        self.left_velocity = v_l
        self.right_velocity = v_r
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.left_pulses = 0
        self.right_pulses = 0
        self.theta_enc = theta_enc
        self.health = "OK"


class _DummyIMUSnapshot:
    def __init__(self, ts: float, gz_dps: float, ax_g: float):
        self.timestamp = ts
        self.gyro = (0.0, 0.0, gz_dps)
        self.accel = (ax_g, 0.0, 0.0)
        self.health = "OK"


class _DummyEncoderService:
    def __init__(self, snapshots):
        self._snapshots = list(snapshots)
        self._idx = 0

    def get_snapshot(self):
        if self._idx >= len(self._snapshots):
            return self._snapshots[-1]
        s = self._snapshots[self._idx]
        self._idx += 1
        return s


class _DummyIMUService:
    def __init__(self, snapshots):
        self._snapshots = list(snapshots)
        self._idx = 0

    def get_snapshot(self):
        if self._idx >= len(self._snapshots):
            return self._snapshots[-1]
        s = self._snapshots[self._idx]
        self._idx += 1
        return s


class _DummyCore:
    def tick(self):
        return None


class _DummyRobot:
    def __init__(self):
        self.v_target = 0.0
        self.omega_target = 0.0


class _DummySM:
    def __init__(self):
        self.robot = _DummyRobot()

    def update(self, dt):
        return None


class _DummyEKFManager:
    def __init__(self, ekf):
        self.ekf_live = ekf
        self.ekf_shadow = ExtendedKalmanFilter(getattr(ekf, "wheel_base", 0.175), {})

    def set_diagnostics(self, dt_stats=None, noise_stats=None, sensor_ok=True):
        return None

    def update(self, imu, encoder, dt, loop_duration=0.0):
        live_state = self.ekf_live.update(imu, encoder, dt)
        shadow_state = self.ekf_shadow.update(imu, encoder, dt)
        return live_state, shadow_state, False


class _DummyCtrl:
    def __init__(self):
        self.cfg = {
            "vezerles": {
                "ekf_use_loop_dt": False,
                "gyro_zero_hold_rad_s": 0.05,
            }
        }
        self._prev_pwm_l = 0.0
        self._prev_pwm_r = 0.0
        self.v_target = 0.0
        self.omega_target = 0.0
        self.v_cmd = 0.0
        self.turn_level = 0
        self.speed_level = 0
        self.motion_command_source = "STATE"
        self.input_vector = None
        self.turn_omega_levels = {}
        self.turn_mix = 1.0
        self.turn_min_level = 0
        self.speeds_fwd = [0.0] * 10
        self.speeds_rev = [0.0] * 10


def _simulate_linear(
    with_enc_update: bool = True,
    dt: float = 0.02,
    total_t: float = 5.0,
    v_cmd: float = 0.2,
    theta0: float = 0.0,
    dt_for_ekf: float = None,
) -> dict:
    ekf = ExtendedKalmanFilter(wheel_base=0.175, config={})
    ekf.reset(theta=theta0)
    steps = int(total_t / dt)
    dt_used = dt if dt_for_ekf is None else dt_for_ekf
    for _ in range(steps):
        ekf.update_adaptivity(v_cmd, v_cmd, v_cmd, 0.0, 0.0, dt_used)
        ekf.predict(0.0, 0.0, dt_used)
        if with_enc_update:
            ekf.update_encoders(v_cmd, v_cmd, dt_used, theta_enc_rad=theta0)
    return ekf.get_state()


def run_checks() -> Tuple[List[CheckResult], List[str], List[str]]:
    checks: List[CheckResult] = []
    issues: List[str] = []
    fixes: List[str] = []

    # 1) Q/R mátrix konzisztencia
    ekf = ExtendedKalmanFilter(wheel_base=0.175, config={})
    ok_mats = (
        ekf._Q_base.shape == (5, 5)
        and ekf._R_enc.shape == (2, 2)
        and ekf._R_lidar.shape == (3, 3)
        and np.array([[ekf._R_zupt]]).shape == (1, 1)
        and np.array([[ekf._R_theta_hold]]).shape == (1, 1)
    )
    checks.append(CheckResult("Q/R méretek", ok_mats, "Q=5x5, R_enc=2x2, R_lidar=3x3, R_zupt/R_theta_hold=1x1"))
    if not ok_mats:
        issues.append("Q/R mátrixméret hiba.")
        fixes.append("Ellenőrizd a vezerles.ekf konfiguráció listahosszait (Q_diag>=5, R_enc>=2, R_lidar>=3).")

    # 2) Predict + 3) Encoder update + fizikai 1m sanity
    s = _simulate_linear(with_enc_update=True, dt=0.02, total_t=5.0, v_cmd=0.2, theta0=0.0)
    x_ok = abs(s["x"] - 1.0) < 0.02
    y_ok = abs(s["y"]) < 0.01
    th_ok = abs(s["theta_deg"]) < 0.5
    checks.append(CheckResult("1m lineáris teszt", x_ok and y_ok and th_ok, f"x={s['x']:.4f}, y={s['y']:.4f}, th={s['theta_deg']:.3f}"))
    if not (x_ok and y_ok and th_ok):
        issues.append(f"1m lineáris sanity eltérés: x={s['x']:.4f}, y={s['y']:.4f}, theta={s['theta_deg']:.3f}")
        fixes.append("Nézd a dt_ekf forrást, encoder gatinget és fizika paramétereket (nyomtáv/lépéshossz).")

    # Trigonometria ellenőrzés (theta=90° -> y tengely)
    s90 = _simulate_linear(with_enc_update=True, dt=0.02, total_t=5.0, v_cmd=0.2, theta0=math.pi / 2)
    trig_ok = abs(s90["x"]) < 0.01 and abs(s90["y"] - 1.0) < 0.02
    checks.append(CheckResult("Trigonometria (90°)", trig_ok, f"x={s90['x']:.4f}, y={s90['y']:.4f}"))
    if not trig_ok:
        issues.append("cos/sin tengelyhasználat eltérés gyanú.")
        fixes.append("Ellenőrizd predict-ben: px += v_avg*cos(theta)*dt, py += v_avg*sin(theta)*dt.")

    # 4) ZUPT/theta_hold trigger csak still állapotban (ControlLoop gate)
    dt = 0.02
    enc_still = [_DummyEncoderSnapshot(ts=i * dt, v_l=0.0, v_r=0.0, theta_enc=0.0) for i in range(10)]
    imu_still = [_DummyIMUSnapshot(ts=i * dt, gz_dps=0.0, ax_g=0.0) for i in range(10)]
    cl_still = ControlLoop(
        _DummyEncoderService(enc_still),
        _DummyIMUService(imu_still),
        _DummyEKFManager(ExtendedKalmanFilter(0.175, {})),
        _DummySM(),
        _DummyCore(),
        loop_hz=50.0,
    )
    ctrl = _DummyCtrl()
    lr_still = cl_still.tick(0.02, ctrl)

    enc_move = [_DummyEncoderSnapshot(ts=i * dt, v_l=0.2, v_r=0.2, theta_enc=0.0) for i in range(10)]
    imu_move = [_DummyIMUSnapshot(ts=i * dt, gz_dps=0.0, ax_g=0.0) for i in range(10)]
    cl_move = ControlLoop(
        _DummyEncoderService(enc_move),
        _DummyIMUService(imu_move),
        _DummyEKFManager(ExtendedKalmanFilter(0.175, {})),
        _DummySM(),
        _DummyCore(),
        loop_hz=50.0,
    )
    ctrl2 = _DummyCtrl()
    ctrl2.v_target = 0.2
    ctrl2.v_cmd = 0.2
    lr_move = cl_move.tick(0.02, ctrl2)

    still_gate_ok = lr_still["zupt_applied"] and lr_still["theta_hold_applied"] and (not lr_move["zupt_applied"]) and (not lr_move["theta_hold_applied"])
    checks.append(CheckResult("ZUPT/theta_hold gate", still_gate_ok, f"still={lr_still['zupt_applied']}/{lr_still['theta_hold_applied']}, move={lr_move['zupt_applied']}/{lr_move['theta_hold_applied']}"))
    if not still_gate_ok:
        issues.append("ZUPT/theta_hold nem csak állóban aktiválódik.")
        fixes.append("Szigorítsd a still feltételt, és egységesítsd v_cmd/v_target gate logikát.")

    # 5) LIDAR confidence és gate reject
    ekf_l = ExtendedKalmanFilter(0.175, {"innovation_gating": {"enabled": True, "lidar_nis_max": 10.0}})
    before = ekf_l.get_state()
    low_conf_res = ekf_l.update_lidar(10.0, 10.0, 0.0, confidence=0.1)  # confidence threshold alatt
    after_low_conf = ekf_l.get_state()
    low_conf_ok = (
        abs(after_low_conf["x"] - before["x"]) < 1e-12
        and abs(after_low_conf["y"] - before["y"]) < 1e-12
        and low_conf_res["reject_reason"] == "rejected_low_confidence"
    )

    # extrém outlier -> NIS reject várható
    nis_res = ekf_l.update_lidar(1000.0, 1000.0, 0.0, confidence=1.0)
    out = ekf_l.get_state()
    gate_ok = bool(out["lidar_gate_reject"]) and nis_res["reject_reason"] == "rejected_nis"
    checks.append(CheckResult("LIDAR threshold+gating", low_conf_ok and gate_ok, f"low_conf_no_update={low_conf_ok}, gate_reject={gate_ok}"))
    if not (low_conf_ok and gate_ok):
        issues.append("LIDAR confidence vagy NIS gate viselkedés eltér.")
        fixes.append("Ellenőrizd lidar_confidence_threshold és innovation_gating.lidar_nis_max értékeket.")

    # 5/b) LIDAR R skálázás: kisebb r_scale -> erősebb korrekció
    ekf_rs = ExtendedKalmanFilter(0.175, {"innovation_gating": {"enabled": True, "lidar_nis_max": 1e9}})
    ekf_rs.reset()
    base_res = ekf_rs.update_lidar(1.0, 0.0, 0.0, confidence=1.0, r_scale=1.0)
    x_base = float(ekf_rs.get_state()["x"])
    ekf_rs.reset()
    scaled_res = ekf_rs.update_lidar(1.0, 0.0, 0.0, confidence=1.0, r_scale=0.35)
    x_scaled = float(ekf_rs.get_state()["x"])
    r_scale_ok = (
        abs(float(base_res.get("r_scale", 0.0)) - 1.0) < 1e-9
        and abs(float(scaled_res.get("r_scale", 0.0)) - 0.35) < 1e-9
        and x_scaled > x_base
    )
    checks.append(CheckResult("LIDAR R-scale hatás", r_scale_ok, f"x_base={x_base:.6f}, x_scaled={x_scaled:.6f}"))
    if not r_scale_ok:
        issues.append("LIDAR r_scale nem módosítja elvárt irányban a korrekció erősségét.")
        fixes.append("Ellenőrizd az update_lidar R_lidar skálázását és a visszaadott r_scale mezőt.")

    # 6) Adaptív Q/R + online learning skálázás
    ekf_ad = ExtendedKalmanFilter(
        0.175,
        {
            "adaptivity": {
                "enabled": True,
                "online_learning": True,
                "slip_velocity_threshold": 0.1,
                "slip_accel_min": 0.5,
                "slip_R_scale": 5.0,
                "innovation_theta_threshold_rad": 0.05,
                "innovation_R_theta_scale": 2.0,
                "Q_online_gamma": 0.4,
                "Q_online_max": 0.25,
                "Q_still": [0.001, 0.001, 0.0008, 0.015, 2e-4],
                "Q_linear": [0.002, 0.002, 0.001, 0.02, 1e-5],
                "Q_rotate": [0.002, 0.002, 0.003, 0.02, 1e-5],
                "R_enc_still": [0.015, 0.025],
                "R_enc_linear": [0.015, 0.012],
                "R_enc_rotate": [0.015, 0.018],
            }
        },
    )
    ekf_ad._last_innovation_theta = 0.2
    ekf_ad.update_adaptivity(v_l=0.0, v_r=0.6, v_cmd=0.2, accel_x=2.0, gyro_z_rad=0.0, dt=0.02)
    Rscaled = ekf_ad._get_R_enc()
    Qscaled = ekf_ad._get_Q(2.0)
    adapt_ok = Rscaled[1, 1] >= ekf_ad._R_enc_current[1, 1] and Qscaled[2, 2] >= ekf_ad._Q_current[2, 2]
    checks.append(CheckResult("Adaptív Q/R + online scaling", adapt_ok, f"R_theta={Rscaled[1,1]:.6f}, Q_theta={Qscaled[2,2]:.6f}"))
    if not adapt_ok:
        issues.append("Adaptív Q/R online skálázás nem látszik.")
        fixes.append("Ellenőrizd adaptivity.enabled + online_learning + küszöb paramétereket.")

    # 7) Statikus másolat konzisztens frissítés
    ekf_s = ExtendedKalmanFilter(0.175, {})
    ekf_s.predict(0.0, 0.0, 0.02)
    ekf_s.update_encoders(0.2, 0.2, 0.02, theta_enc_rad=0.0)
    stat_ok = np.isfinite(ekf_s._x_static).all() and ekf_s._P_static.shape == (5, 5) and abs(ekf_s._x_static[0]) > 0
    checks.append(CheckResult("Statikus EKF ág", stat_ok, f"x_static={ekf_s._x_static[0]:.6f}"))
    if not stat_ok:
        issues.append("Statikus (_x_static/_P_static) ág frissítés hibás.")
        fixes.append("Ellenőrizd predict/update ágak párhuzamos számítását a statikus filterre is.")

    # Skálahiba diagnózis: dt mismatch
    s_dt_half = _simulate_linear(with_enc_update=True, dt=0.02, total_t=5.0, v_cmd=0.2, theta0=0.0, dt_for_ekf=0.01)
    dt_mismatch = s_dt_half["x"] < 0.7
    checks.append(CheckResult("dt mismatch érzékenység", dt_mismatch, f"x_half_dt={s_dt_half['x']:.4f}"))
    if dt_mismatch:
        issues.append("A becsült elmozdulás erősen dt-függő: rossz dt esetén skálahiba várható.")
        fixes.append("Logold és validáld folyamatosan a dt_ekf értéket; hiba esetén átmenetileg használd az ekf_use_loop_dt=true módot.")

    return checks, issues, fixes


def main() -> int:
    checks, issues, fixes = run_checks()

    print("=" * 72)
    print("EKF INTEGRITY REPORT")
    print("=" * 72)
    for c in checks:
        status = "OK" if c.ok else "HIBA"
        print(f"[{status}] {c.name}: {c.detail}")

    print("\n--- Found issues ---")
    if issues:
        for i, item in enumerate(issues, start=1):
            print(f"{i}. {item}")
    else:
        print("Nincs kritikus eltérés a futtatott sanity tesztekben.")

    print("\n--- Suggested fixes ---")
    if fixes:
        for i, item in enumerate(fixes, start=1):
            print(f"{i}. {item}")
    else:
        print("Nincs kötelező azonnali beavatkozás.")

    print("\n--- Suggested config directions ---")
    print("1. Ha predikció alulskáláz: dt_ekf forrás (sensor/loop), timestamp monotónia, then step_distance/nyomtáv.")
    print("2. Ha sok gate reject: emeld óvatosan enc_nis_max / lidar_nis_max értéket.")
    print("3. Ha túl agresszív still viselkedés: szigorítsd a still_for_zupt feltételt (v_cmd ÉS v_target).")
    print("4. Ha heading drift állóban: csökkentsd R_theta_hold-ot vagy növeld bias_accel_k-t kis lépésekben.")

    # Sikerkritérium: a fő integritási check-ek menjenek át.
    must_pass = [
        "Q/R méretek",
        "1m lineáris teszt",
        "Trigonometria (90°)",
        "LIDAR threshold+gating",
    ]
    ok_map = {c.name: c.ok for c in checks}
    overall_ok = all(ok_map.get(name, False) for name in must_pass)
    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
