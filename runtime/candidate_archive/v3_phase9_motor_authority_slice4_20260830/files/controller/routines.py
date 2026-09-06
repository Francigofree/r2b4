#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Specifikus működési rutinok és biztonsági eljárások.
Ide tartozik a vészleállítás, kalibráció, kamera fotó, videó felvétel, és előre programozott mozgások.

VÉSZLEÁLLÍTÁS BELÉPÉSI PONTOK (mind emergency_stop(ctrl, reason=...) hívás):
- Billentyűzet SPACE: middleware/keyboard.py dispatch_commands() → robot._emergency_stop(EMERGENCY_STOP_REASON_SPACE)
- GUI Stop gomb: controller/commands.py poll (ctype=="stop") → emergency_stop(ctrl, EMERGENCY_STOP_REASON_GUI_STOP)
- Full reset (SPACE/RESET): full_reset() belül emergency_stop(ctrl, reason)
- Safety / watchdog / LIDAR / CORE: saját reason stringgel hívják _emergency_stop() vagy emergency_stop()
"""

import os
import sys
import time
import threading
from log.runtime_debug import set_log_switch, write_text_atomic
from log.unified_logger import CHANNEL_TELEMETRY, get_unified_logger
from middleware.peripheral_usage import set_peripheral_enabled

from state import RobotState
from core.task_model import RobotTask, TaskType, TaskPriority
from config_manager import config as global_config
from middleware.ekf import ExtendedKalmanFilter
from middleware.ffp import PIDConfig, active_wheel_speed_range
from core.control_strategies import load_control_mode, save_control_mode, normalize_control_mode
from middleware.lidar_estim import LidarEstimator
from motion_executor import MotionExecutor

# Vészleállítás ok konstansok – audit és log konzisztencia
EMERGENCY_STOP_REASON_SPACE = "SPACE"       # Billentyűzet SPACE (dedikált vészleállító gomb)
EMERGENCY_STOP_REASON_GUI_STOP = "GUI_STOP" # GUI Stop gomb (commands.jsonl "stop")
EMERGENCY_STOP_REASON_FULL_RESET = "FULL_RESET"

def emergency_stop(ctrl, reason="UNKNOWN"):
    """
    AZONNALI VÉSZLEÁLLÍTÁS – IPARI MEGOLDÁS.
    - Motor: azonnal 0 PWM, rámpa nélkül (akárki, akármit csinál).
    - Összes futó folyamat leáll (core queue, executor, követés, keresés).
    - Kamera: minden kamerát használó folyamat leáll, kamera KI.
    - Pozíció + PID reset, robot IDLE.
    Biztonsági kritikus funkció.
    """
    try:
        if hasattr(ctrl, "logger") and ctrl.logger:
            ctrl.logger.error(f"[!!!] VÉSZMEGÁLLÍTÁS AKTIVÁLVA ({reason}) [!!!]")
    except Exception:
        pass

    # Diagnosztika: utolsó vészok mentése, hogy GUI-ból egyértelműen látszódjon.
    try:
        ctrl.last_emergency_reason = str(reason)
        ctrl.last_emergency_ts = time.time()
        ctrl.emergency_stop_count = int(getattr(ctrl, "emergency_stop_count", 0)) + 1
        ctrl.stop_status = {
            "active": True,
            "type": "EMERGENCY_STOP",
            "reason": str(reason),
            "source": "SAFETY",
            "ts": ctrl.last_emergency_ts,
        }
        if hasattr(ctrl, "telemetry"):
            reason_code = "E_EMERGENCY_STOP"
            rr = str(reason).upper()
            if "WATCHDOG" in rr:
                reason_code = "E_WATCHDOG"
            elif "GUI_STOP" in rr or rr == "SPACE":
                reason_code = "E_OPERATOR_STOP"
            elif "CALIBRATION" in rr:
                reason_code = "E_CALIBRATION_WINDOW"
            ctrl.telemetry.emit_audit(
                "EMERGENCY_STOP",
                "SAFETY",
                severity="ERROR",
                details={
                    "reason": str(reason),
                    "reason_code": reason_code,
                    "count": ctrl.emergency_stop_count,
                },
            )
    except Exception:
        pass

    # 1. MOTOR: stop-only capability, rámpa nélkül (első lépés, mindenki előtt)
    try:
        if hasattr(ctrl, "motor_l") and ctrl.motor_l:
            try:
                ctrl.motor_l.stop()
            except Exception:
                pass
    except Exception:
        pass
    try:
        if hasattr(ctrl, "motor_r") and ctrl.motor_r:
            try:
                ctrl.motor_r.stop()
            except Exception:
                pass
    except Exception:
        pass

    # 2. HARDVERES FALLBACK (ha stop hibázott, közvetlen GPIO 0)
    try:
        if hasattr(ctrl, "motor_l") and ctrl.motor_l:
            try:
                ctrl.motor_l.stop()
            except Exception:
                # Fallback: közvetlen GPIO 0-ra állítás
                try:
                    import lgpio
                    cfg = getattr(ctrl, "cfg", {}) or {}
                    motor_cfg = cfg.get("hardver", {}).get("motorok", {}).get("bal_oldal", {})
                    if motor_cfg:
                        handle = lgpio.gpiochip_open(0)
                        lgpio.tx_pwm(handle, motor_cfg.get("gpio_in1"), 8000, 0)
                        lgpio.tx_pwm(handle, motor_cfg.get("gpio_in2"), 8000, 0)
                        lgpio.gpiochip_close(handle)
                except Exception:
                    pass
    except Exception:
        pass
    
    try:
        if hasattr(ctrl, "motor_r") and ctrl.motor_r:
            try:
                ctrl.motor_r.stop()
            except Exception:
                # Fallback: közvetlen GPIO 0-ra állítás
                try:
                    import lgpio
                    cfg = getattr(ctrl, "cfg", {}) or {}
                    motor_cfg = cfg.get("hardver", {}).get("motorok", {}).get("jobb_oldal", {})
                    if motor_cfg:
                        handle = lgpio.gpiochip_open(0)
                        lgpio.tx_pwm(handle, motor_cfg.get("gpio_in1"), 8000, 0)
                        lgpio.tx_pwm(handle, motor_cfg.get("gpio_in2"), 8000, 0)
                        lgpio.gpiochip_close(handle)
                except Exception:
                    pass
    except Exception:
        pass
    
    # 3. Vezérlési változók nullázása (ha ctrl rendben van)
    try:
        ctrl.v_target = 0.0
        ctrl.v_cmd = 0.0
        ctrl.omega_target = 0.0
        if hasattr(ctrl, "requested_motion_intent"):
            ctrl.requested_motion_intent = {"v": 0.0, "omega": 0.0}
        if hasattr(ctrl, "requested_track_reference"):
            ctrl.requested_track_reference = {"left_mps": 0.0, "right_mps": 0.0}
        if hasattr(ctrl, "state_track_reference"):
            ctrl.state_track_reference = {"left_mps": 0.0, "right_mps": 0.0}
        ctrl.speed_level = 0
        if getattr(ctrl, "speed_limits", None):
            try:
                ctrl.speed_limits.set_gear_from_level(ctrl.speed_level)
            except Exception:
                pass
        ctrl.turn_level = 0
        ctrl.dock_active = False
        if hasattr(ctrl, "service_pwm_command"):
            ctrl.service_pwm_command = {
                "active": False,
                "command_type": "",
                "source": "",
                "left_pwm": 0.0,
                "right_pwm": 0.0,
                "v_hint": 0.0,
                "omega_hint": 0.0,
            }
        if hasattr(ctrl, "service_motion_active"):
            ctrl.service_motion_active = False
        if hasattr(ctrl, "motion_executor") and ctrl.motion_executor:
            ctrl.motion_executor.reset()
    except Exception:
        pass

    # 4. Állapotgép: vészhelyzeti mód (FAILSAFE) – feloldás: full_reset / strong_reset → IDLE
    try:
        if hasattr(ctrl, "sm") and ctrl.sm:
            ctrl.sm.transition_to(RobotState.FAILSAFE)
    except Exception:
        pass
    
    # 5. AI/Core sor törlése (ha rendben van)
    try:
        if hasattr(ctrl, 'core') and ctrl.core:
            if hasattr(ctrl.core, 'queue'):
                ctrl.core.queue.clear()
            if hasattr(ctrl.core, 'executor'):
                ctrl.core.executor.is_running = False
    except Exception:
        pass

    # 6. Összes kamerát használó folyamat leáll + kamera KI
    try:
        set_peripheral_enabled("camera", False, status_path=getattr(ctrl, "status_path", None))
    except Exception:
        pass
    try:
        from controller.tasks.follower import stop_following, _release_camera
        from controller.tasks.search_person import stop_search_person, _release_search_camera
        if getattr(ctrl, "following_active", False):
            stop_following(ctrl)
        if getattr(ctrl, "searching_person", False):
            stop_search_person(ctrl)
        _release_camera(ctrl)
        _release_search_camera(ctrl)
    except Exception:
        pass

    # 7. Pozíció reset NINCS itt: csak a fejléc "Pozíció 0 (R)" gomb és az R billentyű
    #     hívhatja reset_position(ctrl)-t (commands.jsonl reset_pos / keyboard R).
    # try:
    #     reset_position(ctrl)
    # except Exception:
    #     pass

    # 8. Formális forrásállapot: MANUAL, arbiter szinkron (következő input konzisztens)
    try:
        ctrl.motion_command_source = "MANUAL"
        if hasattr(ctrl, "arbiter") and ctrl.arbiter:
            prev_active = getattr(ctrl.arbiter, "active", None)
            ts_now = time.monotonic()
            ctrl.arbiter.active = "MANUAL"
            ctrl.arbiter.last_ts["MANUAL"] = ts_now
            ctrl.arbiter.last_switch = {"ts": ts_now, "from": prev_active, "to": "MANUAL", "reason": "emergency_stop"}
    except Exception:
        pass

def capture_photo(ctrl, resolution_preset="kozepes"):
    """
    Közepes (vagy megadott) felbontású fotó készítése, Pic/ mappába timestamp-elt névvel.
    A legutolsó kép relatív útvonalát runtime/latest_photo.txt-be írja (app megjelenítéshez).
    Kamera modul lazy import: libcamera/picamera2 csak P megnyomásakor töltődik.
    """
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    runtime_dir = os.path.join(project_root, "runtime")
    os.makedirs(runtime_dir, exist_ok=True)
    try:
        from driver.cam import Camera, pic_dir, ts_base
        cam = Camera(resolution_preset=resolution_preset)
        cam.start()
        folder = pic_dir()
        base = ts_base()
        cfg = getattr(ctrl, "cfg", {}) or {}
        fmt = (cfg.get("cam") or {}).get("kep") or {}
        ext_cfg = (cfg.get("cam") or {}).get("kamera") or {}
        ext = "jpg" if (ext_cfg.get("fotó_formátum") or "jpg").lower() in ("jpg", "jpeg") else "jpg"
        filename = f"{base}.{ext}"
        path_abs = os.path.join(folder, filename)
        cam.capture(path_abs)
        cam.stop()
        rel = os.path.relpath(path_abs, project_root).replace("\\", "/")
        latest_file = os.path.join(runtime_dir, "latest_photo.txt")
        write_text_atomic(latest_file, rel + "\n")
        ul = get_unified_logger()
        if ul is not None:
            ul.log_event(CHANNEL_TELEMETRY, "camera", "photo_captured", {"path": rel, "preset": resolution_preset}, level="INFO")
        if hasattr(ctrl, "logger"):
            ctrl.logger.info(f"[CAM] Fotó mentve: {rel}")
        return rel
    except ModuleNotFoundError as e:
        if hasattr(ctrl, "logger"):
            mod = str(e.name) if hasattr(e, "name") else str(e).replace("No module named ", "").strip("'")
            ctrl.logger.warn(f"[CAM] Fotó hiba: {e}")
            if mod in ("libcamera", "picamera2"):
                ctrl.logger.warn(
                    "[CAM] Diagnosztika: A kamera a libcamera/picamera2-t használja. "
                    "Raspberry Pi OS-on: sudo apt install -y libcamera-dev python3-libcamera. "
                    "Vagy pip install picamera2 (RPi környezetben)."
                )
        return None
    except Exception as e:
        import traceback
        if hasattr(ctrl, "logger"):
            ctrl.logger.warn(f"[CAM] Fotó hiba: {e}")
            ctrl.logger.warn(traceback.format_exc())
        return None


def _video_max_seconds(ctrl):
    """Max felvétel idő (mp) a conf video.max_ido_mp_manual alapján (alapértelmezett 300 = 5 perc)."""
    cfg = getattr(ctrl, "cfg", {}) or {}
    v = (cfg.get("cam") or {}).get("video") or {}
    return int(v.get("max_ido_mp_manual", 300))


def start_video_recording(ctrl):
    """
    V toggle: videó felvétel indítása. vid/ mappába timestamp-elt .mp4 (max 5 perc).
    Háttérszál 5 perc után automatikusan leállítja a felvételt.
    """
    if getattr(ctrl, "video_recording", False):
        if hasattr(ctrl, "logger"):
            ctrl.logger.info("[VID] Már fut a felvétel; nyomd meg újra a V-t a leállításhoz.")
        return
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    runtime_dir = os.path.join(project_root, "runtime")
    os.makedirs(runtime_dir, exist_ok=True)
    try:
        from driver.cam import Camera, vid_dir, video_ts_base
        folder = vid_dir()
        base = video_ts_base()
        path_mp4 = os.path.join(folder, base + ".mp4")
        cam = Camera(resolution_preset="kozepes")
        cam.start()
        cam.start_recording_to_file(path_mp4)
        ctrl._video_camera = cam
        ctrl.video_recording = True
        ctrl.video_stop_requested = False
        ctrl._video_start_time = time.time()
        ctrl._video_path_abs = path_mp4
        rel = os.path.relpath(path_mp4, project_root).replace("\\", "/")
        ul = get_unified_logger()
        if ul is not None:
            ul.log_event(CHANNEL_TELEMETRY, "camera", "video_started", {"path": rel}, level="INFO")
        if hasattr(ctrl, "logger"):
            ctrl.logger.info(f"[VID] Felvétel indul: {rel} (max {_video_max_seconds(ctrl)} s)")
        # Háttérszál: max idő után leállítás
        max_sec = _video_max_seconds(ctrl)
        def _timeout_stop():
            time.sleep(max_sec)
            if getattr(ctrl, "video_recording", False) and not getattr(ctrl, "video_stop_requested", True):
                stop_video_recording(ctrl)
        t = threading.Thread(target=_timeout_stop, daemon=True)
        t.start()
        ctrl._video_timeout_thread = t
    except Exception as e:
        import traceback
        if hasattr(ctrl, "logger"):
            ctrl.logger.warn(f"[VID] Indítás hiba: {e}")
            ctrl.logger.warn(traceback.format_exc())
        ctrl.video_recording = False
        ctrl._video_camera = None


def stop_video_recording(ctrl):
    """V toggle: videó felvétel leállítása, videó log (stop + duration) írása."""
    if not getattr(ctrl, "video_recording", False):
        return
    ctrl.video_stop_requested = True
    cam = getattr(ctrl, "_video_camera", None)
    start_time = getattr(ctrl, "_video_start_time", None)
    path_abs = getattr(ctrl, "_video_path_abs", "")
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    runtime_dir = os.path.join(project_root, "runtime")
    try:
        if cam:
            cam.stop_recording_to_file()
            cam.stop()
    except Exception as e:
        if hasattr(ctrl, "logger"):
            ctrl.logger.warn(f"[VID] Leállítás hiba: {e}")
    finally:
        ctrl._video_camera = None
        ctrl.video_recording = False
    duration = (time.time() - start_time) if start_time else 0
    rel = os.path.relpath(path_abs, project_root).replace("\\", "/") if path_abs else ""
    try:
        ul = get_unified_logger()
        if ul is not None:
            ul.log_event(CHANNEL_TELEMETRY, "camera", "video_stopped", {"path": rel, "duration_sec": round(duration, 1)}, level="INFO")
    except Exception:
        pass
    if hasattr(ctrl, "logger"):
        ctrl.logger.info(f"[VID] Felvétel kész: {rel} ({duration:.1f} s)")


def full_calibration(ctrl):
    """A teljes szenzorrendszer újrakalibrálása. Resettel indul (motor 0, EKF, LIDAR yaw, stb.)."""
    strong_reset(ctrl, reason="CALIBRATION_START")
    ctrl.sm.transition_to(RobotState.CALIBRATING)
    ctrl.logger.warn(">>> Rendszer kalibráció folyamatban... <<<")

    kalib_cfg = ctrl.cfg.get("vezerles", {}).get("kalibracio") or {}
    imu = getattr(ctrl, "imu_driver", None)
    if (
        str(getattr(imu, "provider", "") or "").strip().lower() != "bno055"
        or not bool(getattr(imu, "calibration_managed_by_device", False))
    ):
        raise RuntimeError("BNO055 device-managed IMU required")
    info = imu.calibrate(samples=int(kalib_cfg.get("bno055_startup_samples", 60)))
    stationary = imu.measure_stationary_error(
        samples=int(kalib_cfg.get("bno055_stationary_check_samples", 40))
    )
    ctrl.logger.info(
        "BNO055 IMU device-managed kalibráció ellenőrizve: "
        f"calibration={info}, stationary={stationary}"
    )

    # A KIT0085 quadrature encoder nem threshold-kalibrálandó. Karbantartáskor
    # csak a statikus driverállapotot rögzítjük; mozgás kizárólag az audit
    # profil normál command bus útvonalán történhet.
    enc_l = getattr(ctrl, "enc_l", None)
    enc_r = getattr(ctrl, "enc_r", None)
    health_l = str(getattr(enc_l, "health", "MISSING") or "MISSING")
    health_r = str(getattr(enc_r, "health", "MISSING") or "MISSING")
    enc_cal = {
        "ok": health_l in ("OK", "DEGRADED") and health_r in ("OK", "DEGRADED"),
        "mode": "STATIC_QUADRATURE_READINESS",
        "model": str(getattr(enc_l, "model", "")),
        "health_left": health_l,
        "health_right": health_r,
        "pulse_count_left": int(getattr(enc_l, "pulse_count", 0) or 0),
        "pulse_count_right": int(getattr(enc_r, "pulse_count", 0) or 0),
    }
    ctrl.last_encoder_calibration = dict(enc_cal or {})
    # Beragadt PWM elkerülés: kalibráció után stop-only nullázás
    try:
        ctrl.motor_l.stop()
        ctrl.motor_r.stop()
    except Exception:
        pass

    ctrl.logger.success(">>> Kalibráció befejezve. <<<")
    reset_position(ctrl)
    ctrl.sm.transition_to(RobotState.IDLE)

def reset_position(ctrl):
    """Atomically reset every pose/localization owner to the same zero anchor."""
    lock = getattr(ctrl, "pose_reset_lock", None)
    if lock is None:
        lock = threading.RLock()
        ctrl.pose_reset_lock = lock

    with lock:
        previous = dict(getattr(ctrl, "pose_reset_status", {}) or {})
        generation = int(previous.get("generation", 0) or 0) + 1
        started_at = time.time()
        status = {
            "generation": generation,
            "in_progress": True,
            "success": False,
            "state": "RESETTING",
            "started_at": started_at,
            "completed_at": None,
            "anchor": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "steps": [],
            "errors": [],
        }
        ctrl.pose_reset_status = dict(status)

        def step(name, fn, *, required=True):
            try:
                fn()
                status["steps"].append({"name": str(name), "ok": True, "required": bool(required)})
            except Exception as exc:
                row = {"name": str(name), "ok": False, "required": bool(required), "error": str(exc)}
                status["steps"].append(row)
                if required:
                    status["errors"].append(f"{name}:{exc}")

        def zero_motion_intent():
            for attr in ("v_target", "omega_target", "v_cmd"):
                if hasattr(ctrl, attr):
                    setattr(ctrl, attr, 0.0)
            ctrl.requested_motion_intent = {"v": 0.0, "omega": 0.0}
            ctrl.limited_motion_intent = {"v": 0.0, "omega": 0.0}
            ctrl.requested_track_reference = {"left_mps": 0.0, "right_mps": 0.0}
            if getattr(ctrl, "motion_executor", None) is not None:
                ctrl.motion_executor.reset()

        def reset_ekf_pair():
            manager = getattr(ctrl, "ekf_manager", None)
            live = getattr(manager, "ekf_live", None) if manager is not None else None
            if live is None:
                live = getattr(ctrl, "ekf", None)
            if live is None or not hasattr(live, "reset"):
                raise RuntimeError("live_ekf_missing")
            live.reset(px=0.0, py=0.0, theta=0.0, v=0.0)
            ctrl.ekf = live
            if getattr(ctrl, "control_loop", None) is not None:
                ctrl.control_loop.ekf = live
            if manager is not None and hasattr(manager, "resync_shadow"):
                manager.resync_shadow()

        def reset_encoder_estimator():
            service = getattr(ctrl, "encoder_service", None)
            estimator = getattr(service, "estimator", None) if service is not None else None
            if estimator is None:
                return
            if hasattr(estimator, "left") and hasattr(estimator.left, "distance"):
                estimator.left.distance = 0.0
            if hasattr(estimator, "right") and hasattr(estimator.right, "distance"):
                estimator.right.distance = 0.0
            for attr in ("theta_enc", "_ds_l_acc", "_ds_r_acc"):
                if hasattr(estimator, attr):
                    setattr(estimator, attr, 0.0)

        def reset_state_provider_alignment():
            providers = [getattr(ctrl, "state_provider", None)]
            loop = getattr(ctrl, "control_loop", None)
            providers.append(getattr(loop, "state_provider", None) if loop is not None else None)
            seen = set()
            for provider in providers:
                if provider is None or id(provider) in seen:
                    continue
                seen.add(id(provider))
                if hasattr(provider, "reset_encoder_yaw_alignment"):
                    provider.reset_encoder_yaw_alignment()

        def reset_matcher_and_maps():
            lidar_service = getattr(ctrl, "lidar_service", None)
            if lidar_service is not None:
                if not hasattr(lidar_service, "reset_estimator"):
                    raise RuntimeError("lidar_service_reset_missing")
                lidar_service.reset_estimator()
            rolling_map = getattr(ctrl, "rolling_local_map", None)
            if rolling_map is not None:
                rolling_map.reset()
            ctrl.rolling_local_map_status = {}
            ctrl.local_navigation_status = {}

        def reset_lidar_odometry_anchor():
            odom = getattr(ctrl, "lidar_odometry", None)
            if odom is None or not hasattr(odom, "reset"):
                raise RuntimeError("lidar_odometry_reset_missing")
            odom.reset(pose_hint={"x": 0.0, "y": 0.0, "theta": 0.0})

        def reset_loop_delivery_state():
            loop = getattr(ctrl, "control_loop", None)
            if loop is not None:
                loop._lidar_idle_anchor_pose = None
                loop._lidar_last_delivered_odom = None
                loop._lidar_last_delivered_ts = 0.0
                loop._lidar_delivery_missing_grace_until_ts = 0.0
                loop._lidar_ekf_last_applied_ts = 0.0
                loop._lidar_ekf_applied_gap_s = None
                loop._last_lidar_speed_sample = None
            ctrl.localization_gate_runtime = {"degraded_started_ts": 0.0, "last_mode": "RESETTING"}
            ctrl.localization_gate_status = {
                "enabled": True,
                "mode": "RESETTING",
                "trust": 0.0,
                "allow_motion": False,
                "speed_scale": 0.0,
                "hard_stop": True,
                "reasons": ["atomic_pose_reset"],
            }

        step("motion_zero", zero_motion_intent)
        step("ekf_live_shadow", reset_ekf_pair)
        step("encoder_estimator", reset_encoder_estimator)
        step("state_provider_encoder_yaw_alignment", reset_state_provider_alignment)
        lidar_lock = getattr(ctrl, "lidar_lock", None)
        if lidar_lock is None:
            step("matcher_map", reset_matcher_and_maps)
            step("lidar_odom_bootstrap_anchor", reset_lidar_odometry_anchor)
        else:
            with lidar_lock:
                step("matcher_map", reset_matcher_and_maps)
                step("lidar_odom_bootstrap_anchor", reset_lidar_odometry_anchor)
        step("delivery_and_gate", reset_loop_delivery_state)

        status["completed_at"] = time.time()
        status["duration_s"] = max(0.0, float(status["completed_at"]) - float(started_at))
        status["in_progress"] = False
        status["success"] = not bool(status["errors"])
        status["state"] = "WAITING_FOR_LOCALIZATION" if status["success"] else "FAILED"
        ctrl.pose_reset_status = dict(status)
        if status["success"]:
            ctrl.logger.info(f"Pozíció atomikusan nullázva. generation={generation}")
        else:
            ctrl.logger.error(f"Pozíció reset részben sikertelen: {status['errors']}")
        return dict(status)


def reload_config(ctrl):
    """
    Összes config fájl újratöltése futásidőben és alkalmazása a vezérlőre.
    A config_manager összes JSON-ját újraolvassa, majd a ctrl.cfg-ból származó
    változókat, PID-et, EKF-et, motion_executor-t és a service-owned LIDAR
    estimatort frissíti.
    EKF állapot (x, P) megmarad; sebesség/turn szint nem nullázódik.
    """
    global_config.load_all()
    ctrl.cfg = global_config.data

    # Vezérlési változók configból (_init_variables megfelelő része)
    vezerles = ctrl.cfg.get("vezerles", {})
    ctrl.recovery_mobility_mode = bool(vezerles.get("RECOVERY_MOBILITY_MODE", False))
    limits = vezerles.get("sebesseg_kezeles", {})
    ctrl.turn_intensity = float(limits.get("fordulasi_intenzitas", 0.765))
    ctrl.turn_min_level = max(0, min(9, int(limits.get("fordulasi_min_fokozat", 3))))
    ctrl.default_speed_level = int(limits.get("alap_fokozat", 0))
    ctrl.max_pwm = float(limits.get("max_pwm", 0.90))
    ctrl.danger_zone = float(ctrl.cfg.get("hardver", {}).get("lidar", {}).get("biztonsagi_zona_m", 0.30))
    mozgas = vezerles.get("mozgas", {})
    ctrl.turn_mix = float(mozgas.get("turn_mix", 1.0))
    ctrl.joy_adapter_cfg = {
        "joy_max_omega_rad_s": float(mozgas.get("joy_max_omega_rad_s", 1.2)),
    }
    ctrl.dock_cfg = vezerles.get("dokkolas", {})

    # PID, EKF, motion executor és service-owned LIDAR estimator újraépítése
    pid_data = vezerles.get("pid_szabalyzo", {})
    ctrl.drive_pid_cfg = PIDConfig(
        kp=pid_data.get("aranyos_tag_p", 0.7),
        ki=pid_data.get("integralo_tag_i", 0.05),
        integrator_limit=pid_data.get("integralo_limit", 0.25),
        k_ff=pid_data.get("elorecsatolasi_tag_ff", 0.45),
        dz_min=pid_data.get("min_pwm_indulas", 0.20),
        wheel_feedback_trust_min=pid_data.get("wheel_feedback_trust_min", 0.55),
    )
    track_width = float(ctrl.cfg["fizika"]["nyomtav_szelesseg_m"])
    ekf_cfg = vezerles.get("ekf") or {}
    old_ekf = ctrl.ekf
    ctrl.ekf = ExtendedKalmanFilter(wheel_base=track_width, config=ekf_cfg)
    ctrl.ekf.x = old_ekf.x.copy()
    ctrl.ekf.P = old_ekf.P.copy()
    ctrl.ekf._x_static = old_ekf._x_static.copy()
    ctrl.ekf._P_static = old_ekf._P_static.copy()
    # Encoder becslő fizika szinkron (wheel_base, step_distance) – EKF és theta_enc konzisztencia
    fizika = ctrl.cfg.get("fizika", {})
    step_m = float(fizika["lepes_hossz_m"])
    step_scale_l = float(fizika.get("lepes_hossz_bal_szorzo", fizika.get("lepes_hossz_bal_scale", 1.0)))
    step_scale_r = float(fizika.get("lepes_hossz_jobb_szorzo", fizika.get("lepes_hossz_jobb_scale", 1.0)))
    if getattr(ctrl, "encoder_service", None) and getattr(ctrl.encoder_service, "estimator", None):
        ctrl.encoder_service.estimator.update_physics(
            wheel_base=track_width,
            step_distance=step_m,
            left_step_scale=step_scale_l,
            right_step_scale=step_scale_r,
        )
    # Quadrature polarity can be hot-reloaded; GPIO reassignment requires restart.
    enc_cfg = (ctrl.cfg.get("hardver", {}) or {}).get("encoderek", {}) or {}
    for side, enc, invert_key in (
        ("left", getattr(ctrl, "enc_l", None), "invert_bal"),
        ("right", getattr(ctrl, "enc_r", None), "invert_jobb"),
    ):
        if enc is None:
            continue
        try:
            enc.invert = bool(enc_cfg.get(invert_key, getattr(enc, "invert", False)))
            enc.forward_b_level = 1 if int(enc_cfg.get("forward_b_level", enc.forward_b_level)) else 0
        except Exception:
            if hasattr(ctrl, "logger") and ctrl.logger:
                ctrl.logger.warn(f"Encoder {side} quadrature polarity sync hiba.")
    pose_cfg = dict(vezerles.get("lidar_pose") or {})
    if getattr(ctrl, "lidar_service", None) is not None:
        pose_provider = None
        motion_reference_provider = lambda: dict(
            getattr(ctrl, "encoder_pipeline_status", {}) or {}
        )
        if hasattr(ctrl, "ekf") and hasattr(ctrl.ekf, "get_state"):
            pose_provider = lambda: ctrl.ekf.get_state()
        ctrl.lidar_service.replace_estimator(
            LidarEstimator(
                danger_zone=ctrl.danger_zone,
                pose_provider=pose_provider,
                motion_reference_provider=motion_reference_provider,
                scan_match_cfg=pose_cfg,
            )
        )
    motion_execution_cfg = vezerles.get("motion_execution") or {}
    control_mode_path = global_config.path("control_mode.json")
    new_mode = load_control_mode(control_mode_path)
    old_mode = normalize_control_mode(getattr(ctrl, "control_mode", None) or getattr(ctrl.motion_executor, "control_mode", None))
    ctrl.motion_executor = MotionExecutor(
        pid_config=ctrl.drive_pid_cfg,
        max_pwm=float(getattr(getattr(ctrl, "speed_limits", None), "max_pwm_cap", ctrl.max_pwm)),
        speed_map=ctrl.cfg.get("speed_map") or {},
        control_mode=new_mode,
        direction_switch_hold_s=float(motion_execution_cfg.get("direction_switch_hold_s", 0.08)),
        direction_switch_debounce_cycles=int(motion_execution_cfg.get("direction_switch_debounce_cycles", 3)),
    )
    ctrl.control_mode = new_mode
    if new_mode != old_mode:
        if ctrl.sm.get_current_state_name() != "IDLE":
            save_control_mode(control_mode_path, old_mode)
            ctrl.control_mode = old_mode
            ctrl.motion_executor.reset()
            if hasattr(ctrl, "logger") and ctrl.logger:
                ctrl.logger.warn("Control mode váltás tiltva (nem IDLE).")
        else:
            ctrl.motion_executor.reset()
            if getattr(ctrl, "watchdog", None) and hasattr(ctrl.watchdog, "reset"):
                ctrl.watchdog.reset()
            if getattr(ctrl, "safety", None) and hasattr(ctrl.safety, "notify_control_mode_change"):
                ctrl.safety.notify_control_mode_change(old_mode, new_mode, ctrl.sm.get_current_state_name())
            if hasattr(ctrl, "telemetry"):
                ctrl.telemetry.emit_audit(
                    "CONTROL_MODE_CHANGE",
                    "SYSTEM",
                    details={"old_mode": old_mode, "new_mode": new_mode, "state": ctrl.sm.get_current_state_name()},
                )
            if hasattr(ctrl, "logger") and ctrl.logger:
                ctrl.logger.info(f"Control mode váltás: {old_mode} -> {new_mode}")
    ctrl.follower_cfg = dict(vezerles.get("follower") or {})
    ctrl.follow_search_pivot_omega_rad_s = float(
        ctrl.follower_cfg.get("search_pivot_omega_rad_s", getattr(ctrl, "follow_search_pivot_omega_rad_s", 0.08)) or 0.08
    )
    ctrl.follow_search_pivot_omega_status = {
        "omega_rad_s": float(ctrl.follow_search_pivot_omega_rad_s),
        "source": "config_reload",
        "updated_ts": time.time(),
    }
    ctrl.follow_use_pursuit = bool(vezerles.get("follow_use_pursuit", False))
    ctrl.follow_pursuit_look_ahead_scale = float(vezerles.get("follow_pursuit_look_ahead_scale", 1.0))
    if getattr(ctrl, "speed_limits", None):
        ctrl.speed_limits.load_from_config(
            vezerles,
            ctrl.control_mode,
            getattr(ctrl, "speed_level", 0),
            ctrl.max_pwm,
            wheel_speed_range_mps=active_wheel_speed_range(
                ctrl.cfg.get("speed_map") or {},
                require_active=True,
            ),
            track_width_m=track_width,
        )
        ctrl.motion_executor.max_pwm = ctrl.speed_limits.max_pwm_cap
    from controller.pose_controller import create_from_config
    ctrl.pose_controller = create_from_config(vezerles)
    if hasattr(ctrl, "logger") and ctrl.logger:
        ctrl.logger.info("Config újratöltve (futásidőben).")


def full_reset(ctrl, reason=None):
    """
    SPACE / RESET gomb: minden állapot alaphelyzetbe – EKF, PID, logolás, kamera kép.
    Vészleállítás + full log új fájl + stream kép invalidálás; a GUI a logokat újratölti.
    """
    if reason is None:
        reason = EMERGENCY_STOP_REASON_FULL_RESET
    emergency_stop(ctrl, reason=reason)
    # Joy kalibráció alaphelyzet (középpont + tartomány)
    try:
        if hasattr(ctrl, "joy_cal"):
            ctrl.joy_cal = {
                "x_center": 0.0, "y_center": 0.0,
                "x_half_range": 0.5, "y_half_range": 0.5,
            }
        if hasattr(ctrl, "joystick_zero_since"):
            ctrl.joystick_zero_since = 0.0
    except Exception:
        pass
    # Full snapshot kapcsoló megőrzése reset után.
    try:
        if bool(getattr(ctrl, "log_capture_active", False)):
            start_full_log(ctrl)
        else:
            stop_full_log(ctrl)
    except Exception as e:
        if hasattr(ctrl, "logger"):
            ctrl.logger.warn(f"Full reset: full log kapcsoló szinkron hiba: {e}")
    # Kamera stream kép invalidálás – következő írt kép lesz a „új”
    try:
        runtime_dir = os.path.dirname(getattr(ctrl, "status_path", "")) or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runtime")
        stream_frame = os.path.join(runtime_dir, "stream_frame.jpg")
        if os.path.exists(stream_frame):
            try:
                os.remove(stream_frame)
            except Exception:
                pass
    except Exception:
        pass
    # FAILSAFE feloldás: IDLE, hogy a robot újra vezérelhető legyen
    try:
        if hasattr(ctrl, "sm") and ctrl.sm:
            ctrl.sm.transition_to(RobotState.IDLE)
    except Exception:
        pass
    if hasattr(ctrl, "logger"):
        ctrl.logger.info("Full reset kész: EKF, PID, log, kamera kép alaphelyzet.")


def strong_reset(ctrl, reason=None):
    """
    Legerősebb reset: motor 0, minden tevékenység stop, EKF+pozíció nullázás,
    LIDAR yaw reset, kerékszabályozó állapot nullázás, kamera teljes újraindítás,
    full log ciklus, joy kalibráció reset. Reset gomb és R billentyű mindkét helyen ezt hívja.
    """
    if reason is None:
        reason = "STRONG_RESET"
    emergency_stop(ctrl, reason=reason)
    # Célpozíció törlése (zárt hurok mozgás megáll)
    try:
        ctrl.target_pose = None
    except Exception:
        pass
    # Pozíció és EKF nullázás
    reset_position(ctrl)
    # A service-owned LIDAR matcher atomikus resetjét reset_position már
    # elvégezte a pose reset lock alatt és a régi scan-generációt érvénytelenítette.
    # Joy kalibráció alaphelyzet
    try:
        if hasattr(ctrl, "joy_cal"):
            ctrl.joy_cal = {
                "x_center": 0.0, "y_center": 0.0,
                "x_half_range": 0.5, "y_half_range": 0.5,
            }
        if hasattr(ctrl, "joystick_zero_since"):
            ctrl.joystick_zero_since = 0.0
    except Exception:
        pass
    # Full snapshot kapcsoló megőrzése reset után.
    try:
        if bool(getattr(ctrl, "log_capture_active", False)):
            start_full_log(ctrl)
        else:
            stop_full_log(ctrl)
    except Exception as e:
        if hasattr(ctrl, "logger"):
            ctrl.logger.warn(f"Strong reset: full log kapcsoló szinkron hiba: {e}")
    # Stream kép invalidálás
    try:
        runtime_dir = os.path.dirname(getattr(ctrl, "status_path", "")) or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runtime")
        stream_frame = os.path.join(runtime_dir, "stream_frame.jpg")
        if os.path.exists(stream_frame):
            try:
                os.remove(stream_frame)
            except Exception:
                pass
    except Exception:
        pass
    # Kamera: ki, majd rövid várakozás után BE (teljes újraindítás)
    try:
        set_peripheral_enabled("camera", False, status_path=getattr(ctrl, "status_path", None))

        def _delayed_camera_on():
            time.sleep(0.8)
            try:
                set_peripheral_enabled("camera", True, status_path=getattr(ctrl, "status_path", None))
            except Exception:
                pass

        t = threading.Thread(target=_delayed_camera_on, daemon=True)
        t.start()
    except Exception:
        pass
    # Watchdog reset: clear stop_triggered flag so FAILSAFE can be properly exited
    try:
        if getattr(ctrl, "watchdog", None) and hasattr(ctrl.watchdog, "reset"):
            ctrl.watchdog.reset()
    except Exception:
        pass
    # Arbiter: clear active source lock so STATE commands work again
    try:
        if getattr(ctrl, "arbiter", None):
            ctrl.arbiter.active = None
            ctrl.arbiter.last_switch = {"ts": time.monotonic(), "from": "MANUAL", "to": None, "reason": "strong_reset"}
    except Exception:
        pass
    # FAILSAFE feloldás: IDLE
    try:
        if hasattr(ctrl, "sm") and ctrl.sm:
            ctrl.sm.transition_to(RobotState.IDLE)
    except Exception:
        pass
    # Clear emergency stop status and motion source lock
    try:
        ctrl.stop_status = {"active": False, "type": "", "reason": "", "source": "", "ts": 0.0}
        ctrl.last_emergency_reason = ""
        ctrl.motion_command_source = "STATE"
    except Exception:
        pass
    if hasattr(ctrl, "logger"):
        ctrl.logger.info("Strong reset kész: motor 0, EKF, LIDAR yaw, PID, kamera újraindítás, log.")


def start_full_log(ctrl):
    """Kompatibilitási név: nehéz fejlesztői snapshot logolás BE."""
    try:
        ul = get_unified_logger()
        if ul is not None:
            ul.set_capture_enabled(True)
        set_log_switch("full_log", True)
        ctrl.log_capture_active = True
        ctrl.logger.info("Fejlesztői full snapshot log engedélyezve.")
        return True
    except Exception as e:
        ctrl.logger.error(f"Log capture indítás hiba: {e}")
        return False


def stop_full_log(ctrl):
    """Kompatibilitási név: nehéz fejlesztői snapshot logolás KI."""
    try:
        set_log_switch("full_log", False)
        ctrl.log_capture_active = False
        ctrl.logger.info("Fejlesztői full snapshot log letiltva; kompakt strukturált log marad.")
        return True
    except Exception as e:
        ctrl.logger.error(f"Log capture leállítás hiba: {e}")
        return False


def toggle_full_log(ctrl):
    """L: fejlesztői full snapshot log indítás/leállítás váltása."""
    if bool(getattr(ctrl, "log_capture_active", False)):
        return stop_full_log(ctrl)
    return start_full_log(ctrl)


def start_b_sequence(ctrl, source="MANUAL"):
    """
    B gomb rutin: 1 m előre, 0.5 mp várakozás, majd 45° jobbra helyben fordulás.
    """
    if not ctrl.allow_source(source):
        return False

    # Sor ürítése
    if hasattr(ctrl, "core"):
        ctrl.core.queue.clear()
        ctrl.core.executor.is_running = False

    speed_level = 3  # konzervatív, biztonságos fokozat

    ctrl.core.queue.add(RobotTask(
        type=TaskType.MOVE,
        params={
            "distance": 1.0,
            "direction": 1,
            "speed_level": speed_level,
            "dock": False,
            "source": source,
        },
        priority=TaskPriority.HIGH
    ))
    ctrl.core.queue.add(RobotTask(
        type=TaskType.WAIT,
        params={
            "duration": 0.5,
            "source": source,
        },
        priority=TaskPriority.HIGH
    ))
    ctrl.core.queue.add(RobotTask(
        type=TaskType.TURN,
        params={
            "angle": 45.0,  # pozitív = jobbra (a manual turn logika szerint)
            "dock_before": False,
            "source": source,
        },
        priority=TaskPriority.HIGH
    ))

    ctrl.mark_input(source)
    if hasattr(ctrl, "logger"):
        ctrl.logger.info("[B_SEQ] 1m előre → 0.5s stop → 45° jobbra indítva.")
    return True


def start_square(ctrl, side_m=1.0, source="MANUAL"):
    """
    1×1 m négyszög mentén halad (side_m oldalhossz), a végén a kezdőpozícióba érkezik.
    4× egyenes (side_m m) + 4× 90° forduló. Automatikus dokkolás opcionális a sarkokon.
    """
    if not ctrl.allow_source(source):
        return False

    # Sor ürítése
    if hasattr(ctrl, "core"):
        ctrl.core.queue.clear()
        ctrl.core.executor.is_running = False

    # Paraméterek a konfigból
    dock = bool(ctrl.dock_cfg.get("aktiv", True))
    dock_dist = float(ctrl.dock_cfg.get("tav_m", 0.15))
    dock_speed = int(ctrl.dock_cfg.get("sebesseg_fokozat", 1))
    dock_turn = bool(ctrl.dock_cfg.get("turn_dock", True))
    speed_level = 3

    # Feladatok generálása (4 oldal + 4 forduló)
    for _ in range(4):
        # Egyenes szakasz
        ctrl.core.queue.add(RobotTask(
            type=TaskType.MOVE,
            params={
                "distance": side_m,
                "direction": 1,
                "speed_level": speed_level,
                "dock": dock,
                "dock_dist": dock_dist,
                "dock_speed": dock_speed,
                "source": source,
            },
            priority=TaskPriority.HIGH
        ))
        # Forduló
        ctrl.core.queue.add(RobotTask(
            type=TaskType.TURN,
            params={
                "angle": 90.0,
                "dock_before": dock_turn,
                "dock_speed": dock_speed,
                "source": source,
            },
            priority=TaskPriority.HIGH
        ))

    ctrl.mark_input(source)
    ctrl.logger.info("[SQUARE] 1m négyzet szekvencia indítva.")
    return True


def start_circle(ctrl, source="MANUAL"):
    """
    KARIKA: 60cm átmérőjű kör megrajzolása, visszatérés kiindulási pozícióba.
    
    Működés (dejargonizáltan, timestampekkel):
    - T0.000s: Kiindulási pozíció mentése (x0, y0) az EKF-ből
    - T0.000s: CIRCLE állapot aktiválása
    - T0.000s-TX.XXXs: Folyamatos mozgás:
      * Előre haladás: v_target = 0.15 m/s (lassú, egyenletes)
      * Jobbra fordulás: omega_target = 0.5 rad/s (v / r = 0.15 / 0.3)
      * Diff-drive: bal kerék lassabban (v_l = v - omega*L/2), jobb kerék gyorsabban (v_r = v + omega*L/2)
      * Eredmény: körív megrajzolása, sugár = 0.3m (60cm átmérő / 2)
    - TX.XXXs: Visszatérés észlelése: sqrt((x-x0)^2 + (y-y0)^2) < 0.1m
    - TX.XXXs: Leállítás (IDLE), v_target = 0, omega_target = 0
    
    Paraméterek:
    - Kör sugara: 0.3m (60cm átmérő / 2)
    - Lineáris sebesség: 0.15 m/s (lassú, egyenletes)
    - Szögsebesség: 0.5 rad/s (v / r = 0.15 / 0.3)
    - Visszatérés küszöb: 0.1m
    - Kör kerülete: ~1.885m (2π * 0.3m)
    - Becsült időtartam: ~12.6s (1.885m / 0.15 m/s)
    """
    from controller.commands import set_motion_source
    if not set_motion_source(ctrl, "STATE"):
        return False

    if hasattr(ctrl, "core"):
        ctrl.core.queue.clear()
        ctrl.core.executor.is_running = False

    # CIRCLE: STATE forrás (control_loop nem írja felül v/omega)
    ctrl.sm.transition_to(RobotState.CIRCLE)
    
    if hasattr(ctrl, "logger"):
        ctrl.logger.info(
            "[CIRCLE] KARIKA indítva: 60cm átmérőjű kör, "
            "lassú egyenletes jobbra mozgással, visszatérés kiindulási pozícióba."
        )
    
    return True
