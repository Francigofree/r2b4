#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Startup fázisok implementációja.
Minden fázis determinisztikus, timeout-tal védett, logolt.
"""

import os
import sys
import time
import json
import signal
import subprocess
import threading

from driver.imu_factory import (
    bno055_address_from_config,
    build_imu_devices,
    imu_presence_from_i2c,
    imu_probe_targets,
    imu_provider_from_config,
)
from startup.state_machine import StartupState, StartupStateMachine, StartupContext
from startup.supervisor import StartupSupervisor


def _parse_i2cdetect_output(output: str) -> list[str]:
    found: list[str] = []
    for raw in str(output or "").replace("\t", " ").split():
        token = raw.strip().lower()
        if len(token) != 2:
            continue
        if all(ch in "0123456789abcdef" for ch in token):
            found.append(f"0x{token}")
    return sorted(set(found), key=lambda item: int(item, 16))


def _probe_i2c_addr_fallback(
    *,
    bus_num: int,
    addr: int,
    register: int = 0x00,
    timeout_s: float = 0.45,
) -> bool:
    """
    Bounded fallback probe when `i2cdetect` is unavailable.
    Runs in a subprocess so a stuck ioctl cannot freeze startup.
    """
    probe_code = (
        "import smbus2\n"
        f"bus = smbus2.SMBus({int(bus_num)})\n"
        "ok = False\n"
        "try:\n"
        f"    bus.read_byte_data({int(addr)}, {int(register)})\n"
        "    ok = True\n"
        "except Exception:\n"
        "    ok = False\n"
        "finally:\n"
        "    bus.close()\n"
        "print('1' if ok else '0')\n"
    )
    try:
        proc = subprocess.run(
            [sys.executable, "-c", probe_code],
            capture_output=True,
            text=True,
            timeout=max(0.1, float(timeout_s)),
            check=False,
        )
        return proc.returncode == 0 and proc.stdout.strip() == "1"
    except Exception:
        return False


def _i2c_scan(bus_num: int = 1) -> list:
    """
    I2C busz vizsgálata timeout-biztos módon.
    Elsődlegesen `i2cdetect`-et használunk, mert a közvetlen smbus scan egy
    rosszul válaszoló eszköznél címenként több másodpercre blokkolhat.
    """
    found: list[str] = []
    return _i2c_scan_for_imu(bus_num)


def _i2c_scan_for_imu(
    bus_num: int = 1,
    *,
    imu_provider: str = "bno055",
    bno055_addr: int = 0x28,
) -> list:
    found: list[str] = []
    try:
        proc = subprocess.run(
            ["i2cdetect", "-y", str(int(bus_num))],
            capture_output=True,
            text=True,
            timeout=1.5,
            check=False,
        )
        if proc.returncode == 0 and proc.stdout:
            found = _parse_i2cdetect_output(proc.stdout)
            if _imu_present_in_i2c(found, provider=imu_provider, bno055_addr=bno055_addr):
                return found
    except Exception:
        pass

    fallback_probes = tuple(imu_probe_targets(provider=imu_provider, bno055_addr=bno055_addr))
    merged = {str(addr).lower() for addr in found}
    for addr, register in fallback_probes:
        addr_hex = hex(addr).lower()
        if addr_hex in merged:
            continue
        if _probe_i2c_addr_fallback(bus_num=bus_num, addr=addr, register=register):
            merged.add(addr_hex)
    return sorted(merged, key=lambda item: int(item, 16))


def _imu_present_in_i2c(
    addrs: list[str],
    *,
    provider: str = "bno055",
    bno055_addr: int = 0x28,
) -> bool:
    ok, _active, _details = imu_presence_from_i2c(addrs, provider=provider, bno055_addr=bno055_addr)
    return bool(ok)


def _i2c_scan_until_imu_ready(
    bus_num: int = 1,
    *,
    attempts: int = 8,
    delay_s: float = 1.0,
    imu_provider: str = "bno055",
    bno055_addr: int = 0x28,
) -> tuple[list[str], list[list[str]]]:
    """
    Bounded retry for transient I2C discovery misses.
    Safety is unchanged: IMU is still required, this only debounces the probe.
    """
    tries: list[list[str]] = []
    max_attempts = max(1, int(attempts))
    for idx in range(max_attempts):
        found = _i2c_scan_for_imu(bus_num, imu_provider=imu_provider, bno055_addr=bno055_addr)
        tries.append(list(found))
        if _imu_present_in_i2c(found, provider=imu_provider, bno055_addr=bno055_addr):
            return list(found), tries
        if idx < max_attempts - 1:
            time.sleep(max(0.0, float(delay_s)))
    return (list(tries[-1]) if tries else []), tries


def _check_usb_serial(port_pattern: str = "/dev/ttyUSB") -> bool:
    """USB soros port létezik-e (pl. LIDAR)."""
    for i in range(4):
        if os.path.exists(f"{port_pattern}{i}"):
            return True
    return False


def _check_camera() -> bool:
    """Kamera elérhetőség (libcamera/picamera2)."""
    cam = None
    try:
        from driver.cam import camera_lifecycle_lock, safe_stop_close
        from picamera2 import Picamera2
        with camera_lifecycle_lock():
            cam = Picamera2()
            cam.start()
        return True
    except Exception:
        pass
    finally:
        try:
            from driver.cam import safe_stop_close
            safe_stop_close(cam)
        except Exception:
            pass
    return False


def _check_gpio_available() -> bool:
    """GPIO (lgpio) elérhetőség motor driverhez."""
    try:
        import lgpio
        h = lgpio.gpiochip_open(0)
        lgpio.gpiochip_close(h)
        return True
    except Exception:
        return False


def _encoder_stream_ready(
    ctrl,
    *,
    max_snapshot_age_s: float = 0.25,
) -> tuple[bool, dict]:
    """
    KIT0085 encoder stream pre-check.
    Stationary startup is valid: pulses are not required, but both quadrature
    drivers and the immutable service snapshot must be healthy and fresh.
    """
    enc_l = getattr(ctrl, "enc_l", None)
    enc_r = getattr(ctrl, "enc_r", None)
    svc = getattr(ctrl, "encoder_service", None)
    now_perf = time.perf_counter()

    info = {
        "service_running": bool(getattr(svc, "_running", False)) if svc is not None else False,
        "driver_model_l": str(getattr(enc_l, "model", "")) if enc_l is not None else None,
        "driver_model_r": str(getattr(enc_r, "model", "")) if enc_r is not None else None,
        "driver_health_l": str(getattr(enc_l, "health", "MISSING")) if enc_l is not None else None,
        "driver_health_r": str(getattr(enc_r, "health", "MISSING")) if enc_r is not None else None,
        "pin_a_l": int(getattr(enc_l, "pin_a", -1)) if enc_l is not None else None,
        "pin_b_l": int(getattr(enc_l, "pin_b", -1)) if enc_l is not None else None,
        "pin_a_r": int(getattr(enc_r, "pin_a", -1)) if enc_r is not None else None,
        "pin_b_r": int(getattr(enc_r, "pin_b", -1)) if enc_r is not None else None,
        "level_a_l": int(getattr(enc_l, "level_a", -1)) if enc_l is not None else None,
        "level_b_l": int(getattr(enc_l, "level_b", -1)) if enc_l is not None else None,
        "level_a_r": int(getattr(enc_r, "level_a", -1)) if enc_r is not None else None,
        "level_b_r": int(getattr(enc_r, "level_b", -1)) if enc_r is not None else None,
        "pulse_count_l": int(getattr(enc_l, "pulse_count", 0) or 0) if enc_l is not None else None,
        "pulse_count_r": int(getattr(enc_r, "pulse_count", 0) or 0) if enc_r is not None else None,
        "snapshot_health": None,
        "snapshot_age_s": None,
    }

    if enc_l is None or enc_r is None or svc is None:
        return False, info
    if not bool(getattr(svc, "_running", False)):
        return False, info

    snap = None
    try:
        snap = svc.get_snapshot()
    except Exception:
        snap = None
    if snap is None:
        return False, info

    snap_health = str(getattr(snap, "health", "N/A") or "N/A").upper()
    snap_ts = float(getattr(snap, "timestamp", 0.0) or 0.0)
    snap_age = (now_perf - snap_ts) if snap_ts > 0.0 else None
    info["snapshot_health"] = snap_health
    info["snapshot_age_s"] = snap_age

    if info["driver_model_l"] != "DFROBOT_KIT0085_28PA51G":
        return False, info
    if info["driver_model_r"] != "DFROBOT_KIT0085_28PA51G":
        return False, info
    if info["driver_health_l"] not in ("OK", "DEGRADED"):
        return False, info
    if info["driver_health_r"] not in ("OK", "DEGRADED"):
        return False, info
    if any(info[key] not in (0, 1) for key in ("level_a_l", "level_b_l", "level_a_r", "level_b_r")):
        return False, info
    if snap_health != "OK":
        return False, info
    if snap_age is None or snap_age > float(max_snapshot_age_s):
        return False, info
    return True, info


def _wait_encoder_stream_ready(
    ctrl,
    *,
    timeout_s: float,
    poll_s: float = 0.02,
    max_snapshot_age_s: float = 0.25,
) -> tuple[bool, dict]:
    start = time.perf_counter()
    timeout_s = max(0.0, float(timeout_s))
    sleep_s = max(0.005, float(poll_s))
    last_info = {}

    while (time.perf_counter() - start) <= timeout_s:
        ok, info = _encoder_stream_ready(
            ctrl,
            max_snapshot_age_s=max_snapshot_age_s,
        )
        last_info = dict(info or {})
        if ok:
            last_info["wait_time_s"] = round(time.perf_counter() - start, 4)
            return True, last_info
        time.sleep(sleep_s)

    if not last_info:
        _, info = _encoder_stream_ready(
            ctrl,
            max_snapshot_age_s=max_snapshot_age_s,
        )
        last_info = dict(info or {})
    last_info["wait_time_s"] = round(time.perf_counter() - start, 4)
    return False, last_info


def phase_boot(ctrl, sm: StartupStateMachine) -> tuple[StartupState | None, str]:
    """
    BOOT: konfig betöltés, log, runtime könyvtárak, korábbi kalibráció ellenőrzés.
    A ctrl-nek már léteznie kell: logger, cfg (minimálisan).
    """
    sm.ctx.set_state(StartupState.BOOT)
    sm.log(f"BOOT")

    # Runtime könyvtárak
    runtime_dir = os.path.dirname(getattr(ctrl, "status_path", "")) or "runtime"
    if not runtime_dir.startswith("/"):
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        runtime_dir = os.path.join(base, runtime_dir)
    os.makedirs(runtime_dir, exist_ok=True)

    # Korábbi kalibrációs fájl
    sm.log(f"Runtime: {runtime_dir}, IMU provider: BNO055 device-managed")
    return StartupState.HARDWARE_DISCOVERY, ""


def phase_hardware_discovery(ctrl, sm: StartupStateMachine, sup: StartupSupervisor) -> tuple[StartupState | None, str]:
    """HARDWARE_DISCOVERY: I2C scan, USB, LIDAR, IMU, kamera, motor GPIO."""
    sm.ctx.set_state(StartupState.HARDWARE_DISCOVERY)
    sm.log(f"HARDWARE_DISCOVERY")

    discovery = {}

    # I2C busz
    imu_provider = imu_provider_from_config(getattr(ctrl, "cfg", {}) or {})
    bno055_addr = bno055_address_from_config(getattr(ctrl, "cfg", {}) or {})
    i2c_addrs, i2c_attempts = _i2c_scan_until_imu_ready(
        1,
        imu_provider=imu_provider,
        bno055_addr=bno055_addr,
    )
    discovery["i2c_addresses"] = i2c_addrs
    discovery["i2c_scan_attempts"] = i2c_attempts

    # IMU: the configured DFRobot SEN0253/BNO055 is the only accepted path.
    def _has_addr(addr: int) -> bool:
        s = hex(addr).lower()
        return any(a.lower() == s for a in i2c_addrs)
    has_gyro = _has_addr(0x68)
    has_accel = _has_addr(0x53)
    has_mag = _has_addr(0x0C)
    imu_ok, active_imu_provider, imu_details = imu_presence_from_i2c(
        i2c_addrs,
        getattr(ctrl, "cfg", {}) or {},
        provider=imu_provider,
        bno055_addr=bno055_addr,
    )
    discovery["imu"] = {
        "ok": imu_ok,
        "provider": active_imu_provider,
        "gyro": has_gyro,
        "accel": has_accel,
        "mag": has_mag,
        **dict(imu_details),
    }

    # Motor GPIO
    gpio_ok = _check_gpio_available()
    discovery["motor_gpio"] = {"ok": gpio_ok}

    # LIDAR (USB)
    lidar_ok = _check_usb_serial("/dev/ttyUSB")
    discovery["lidar"] = {"ok": lidar_ok}

    # Kamera
    cam_ok = _check_camera()
    discovery["camera"] = {"ok": cam_ok}

    # Enkóder: GPIO-alapú, a motor GPIO-val együtt jár
    discovery["encoder"] = {"ok": gpio_ok}

    sm.ctx.hardware_discovery = discovery
    sm.log(f"IMU: {imu_ok} ({active_imu_provider}), Motor: {gpio_ok}, LIDAR: {lidar_ok}, Kamera: {cam_ok}")

    ok, err = sup.validate_hardware_discovery(discovery)
    if not ok:
        sm.ctx.last_error = err
        return StartupState.FAILSAFE, err

    # DEGRADED ok – hiányzó nem-kritikus periféria
    if not lidar_ok:
        sm.ctx.add_degraded_reason("LIDAR hiányzik (navigation disabled)")
    if not cam_ok:
        sm.ctx.add_degraded_reason("Kamera hiányzik (vision disabled)")

    return StartupState.PERIPHERAL_INIT, ""


def phase_peripheral_init(ctrl, sm: StartupStateMachine) -> tuple[StartupState | None, str]:
    """
    PERIPHERAL_INIT: motor, enkóder, IMU, LIDAR, kamera driver inicializálás.
    A controller/components _init_hardware és _init_sensor_services logikáját foglalja magában.
    """
    sm.ctx.set_state(StartupState.PERIPHERAL_INIT)
    sm.log(f"PERIPHERAL_INIT")

    discovery = sm.ctx.hardware_discovery
    health = {}

    # Motorok – első lépés, induláskor 0
    try:
        from driver.motor import AlbaMotor
        ctrl.motor_l = AlbaMotor("bal_oldal")
        ctrl.motor_r = AlbaMotor("jobb_oldal")
        ctrl.motor_l.set_pwm(0.0)
        ctrl.motor_r.set_pwm(0.0)
        ctrl.motor_l.stop()
        ctrl.motor_r.stop()
        health["motor"] = {"ok": True}
    except Exception as e:
        health["motor"] = {"ok": False, "error": str(e)}
        sm.ctx.last_error = f"Motor init: {e}"
        return StartupState.FAILSAFE, str(e)

    # KIT0085 kétfázisú Hall enkóderek
    try:
        from driver.encoder import DFRobotQuadratureEncoder
        enc_cfg = ctrl.cfg["hardver"]["encoderek"]
        counts_per_revolution = int(enc_cfg["counts_per_revolution"])
        forward_b_level = int(enc_cfg.get("forward_b_level", 1))
        a_debounce_micros = int(enc_cfg.get("a_debounce_micros", 0))
        pull_up = bool(enc_cfg.get("input_pull_up", False))
        ctrl.enc_l = DFRobotQuadratureEncoder(
            pin_a=enc_cfg["bal_a_pin"],
            pin_b=enc_cfg["bal_b_pin"],
            name="ENC_L",
            counts_per_revolution=counts_per_revolution,
            forward_b_level=forward_b_level,
            a_debounce_micros=a_debounce_micros,
            invert=bool(enc_cfg.get("invert_bal", False)),
            pull_up=pull_up,
        )
        ctrl.enc_r = DFRobotQuadratureEncoder(
            pin_a=enc_cfg["jobb_a_pin"],
            pin_b=enc_cfg["jobb_b_pin"],
            name="ENC_R",
            counts_per_revolution=counts_per_revolution,
            forward_b_level=forward_b_level,
            a_debounce_micros=a_debounce_micros,
            invert=bool(enc_cfg.get("invert_jobb", False)),
            pull_up=pull_up,
        )
        ctrl.enc_l.start()
        ctrl.enc_r.start()
        health["encoder"] = {
            "ok": True,
            "model": ctrl.enc_l.model,
            "count_mode": ctrl.enc_l.count_mode,
            "counts_per_revolution": counts_per_revolution,
        }
    except Exception as e:
        health["encoder"] = {"ok": False, "error": str(e)}
        sm.ctx.last_error = f"Encoder init: {e}"
        return StartupState.FAILSAFE, str(e)

    # IMU
    try:
        imu_provider = str(discovery.get("imu", {}).get("provider", "") or imu_provider_from_config(ctrl.cfg))
        imu_devices = build_imu_devices(ctrl.cfg, provider=imu_provider, initialize=True)
        ctrl.imu_provider = str(imu_devices.get("provider", imu_provider))
        ctrl.imu_driver = imu_devices.get("driver")
        health["imu"] = {
            "ok": bool(getattr(ctrl.imu_driver, "initialized", False)),
            "provider": ctrl.imu_provider,
        }
    except Exception as e:
        health["imu"] = {"ok": False, "error": str(e)}
        sm.ctx.last_error = f"IMU init: {e}"
        return StartupState.FAILSAFE, str(e)

    # danger_zone (LIDAR biztonsági zóna) – mindig szükséges
    hw = ctrl.cfg.get("hardver") or {}
    lidar_cfg = (hw.get("lidar") or {}) if isinstance(hw, dict) else {}
    ctrl.danger_zone = float(lidar_cfg.get("biztonsagi_zona_m", 0.30)) if isinstance(lidar_cfg, dict) else 0.30

    # LIDAR (opcionális – discovery alapján)
    if discovery.get("lidar", {}).get("ok", False):
        try:
            from sensors.lidar_service import LidarService
            ctrl.lidar_service = LidarService(danger_zone=ctrl.danger_zone)
            ctrl.lidar_service.start()
            health["lidar"] = {"ok": True}
        except Exception as e:
            health["lidar"] = {"ok": False, "error": str(e)}
            sm.ctx.add_degraded_reason(f"LIDAR init hiba: {e}")
            ctrl.lidar_service = None
            ctrl.danger_zone = getattr(ctrl, "danger_zone", 0.30) or 0.30
    else:
        ctrl.lidar_service = None
        health["lidar"] = {"ok": False, "reason": "nem található"}

    # IMU + encoder service – szükséges a control loop-hoz
    try:
        from sensors.imu_service import IMUService
        from sensors.encoder_service import EncoderService

        ctrl.imu_service = IMUService(ctrl.imu_driver)
        ctrl.encoder_service = EncoderService(ctrl.enc_l, ctrl.enc_r)
        ctrl.encoder_service._update_rate_hz = float(
            max(50, int((ctrl.cfg["hardver"]["encoderek"] or {}).get("snapshot_hz", 400)))
        )
        ctrl.imu_service.start()
        ctrl.encoder_service.start()
        health["imu_service"] = {"ok": True}
        health["encoder_service"] = {"ok": True}
    except Exception as e:
        health["imu_service"] = {"ok": False, "error": str(e)}
        sm.ctx.last_error = f"Szenzor szolgáltatás: {e}"
        return StartupState.FAILSAFE, str(e)

    if not ctrl.lidar_service:
        ctrl.lidar_summary = {"min_dist": 5.0, "blocked_front": False, "blocked_back": False}
        ctrl.lidar_health = "N/A"
        ctrl.lidar_lock = threading.Lock()
        ctrl.lidar_worker_running = True
        ctrl.lidar_last_update = time.monotonic()

    sm.ctx.sensor_health = health
    sm.log("Perifériák inicializálva")
    return StartupState.SENSOR_STABILIZATION, ""


def phase_sensor_stabilization(ctrl, sm: StartupStateMachine) -> tuple[StartupState | None, str]:
    """SENSOR_STABILIZATION: IMU warmup, mintavételezés, zajszint mérés."""
    sm.ctx.set_state(StartupState.SENSOR_STABILIZATION)
    sm.log(f"SENSOR_STABILIZATION")

    cfg = sm.config.get("sensor_stabilization", {})
    warmup = float(cfg.get("imu_warmup_sec", 1.5))
    samples = int(cfg.get("samples", 100))

    time.sleep(warmup)

    # Gyro zajszint mérése
    gx, gy, gz = [], [], []
    try:
        for _ in range(min(samples, 80)):
            sample = ctrl.imu_driver.read_sample(force=True)
            x, y, z = tuple(sample.get("raw_gyro", (0, 0, 0)))
            gx.append(x)
            gy.append(y)
            gz.append(z)
            time.sleep(0.012)
    except Exception as e:
        sm.ctx.last_error = f"Stabilizáció: {e}"
        return StartupState.FAILSAFE, str(e)

    if gx:
        import statistics
        sm.ctx.baseline["gyro_raw_std"] = {
            "x": statistics.stdev(gx),
            "y": statistics.stdev(gy),
            "z": statistics.stdev(gz),
        }

    sm.log("Szenzor stabilizáció kész")
    return StartupState.CALIBRATION, ""


def phase_calibration(ctrl, sm: StartupStateMachine, sup: StartupSupervisor) -> tuple[StartupState | None, str]:
    """CALIBRATION: IMU calibration and stationary KIT0085 encoder validation."""
    sm.ctx.set_state(StartupState.CALIBRATION)
    sm.log(f"CALIBRATION")

    kalib_cfg = ctrl.cfg.get("vezerles", {}).get("kalibracio") or {}
    cal_status = {"gyro": {"ok": False}, "accel": {"ok": False}, "encoder": {"ok": False}}
    imu = getattr(ctrl, "imu_driver", None)
    if (
        str(getattr(imu, "provider", "") or "").strip().lower() != "bno055"
        or not bool(getattr(imu, "calibration_managed_by_device", False))
    ):
        error = "BNO055 device-managed IMU required"
        sm.ctx.last_error = error
        return StartupState.FAILSAFE, error
    try:
        cal_info = imu.calibrate(samples=int(kalib_cfg.get("bno055_startup_samples", 60)))
        err = imu.measure_stationary_error(
            samples=int(kalib_cfg.get("bno055_stationary_check_samples", 40))
        )
        cal_status["gyro"] = {
            "ok": True,
            "provider": "bno055",
            "device_managed": True,
            "details": dict(cal_info or {}),
            "stationary": dict(err or {}),
        }
        cal_status["accel"] = {
            "ok": True,
            "provider": "bno055",
            "device_managed": True,
        }
        sm.log("BNO055 IMU device-managed kalibráció ellenőrizve")
    except Exception as e:
        sm.ctx.last_error = f"IMU kalibráció: {e}"
        return StartupState.FAILSAFE, str(e)

    # A quadrature encoder nem igényel threshold-kalibrációt és startupkor
    # nem mozgatjuk meg a robotot. A mérési skála a datasheet 663 PPR értéke.
    gate_timeout_s = float(kalib_cfg.get("encoder_ready_timeout_sec", 2.5))
    gate_max_snapshot_age_s = float(kalib_cfg.get("encoder_ready_max_snapshot_age_sec", 0.25))
    gate_ok, gate_info = _wait_encoder_stream_ready(
        ctrl,
        timeout_s=gate_timeout_s,
        max_snapshot_age_s=gate_max_snapshot_age_s,
    )
    if not gate_ok:
        cal_status["encoder"]["ok"] = False
        cal_status["encoder"]["details"] = {
            "ok": False,
            "error": "ENCODER_STREAM_NOT_READY",
            **dict(gate_info or {}),
        }
        sm.ctx.calibration_status = cal_status
        sm.ctx.last_error = "KIT0085 encoder stream not ready"
        return StartupState.FAILSAFE, "KIT0085 encoder stream not ready"
    gate_wait_s = float((gate_info or {}).get("wait_time_s", 0.0) or 0.0)
    gate_age_s = float((gate_info or {}).get("snapshot_age_s", 0.0) or 0.0)
    sm.log(
        "Encoder stream ready "
        f"(wait={gate_wait_s:.3f}s, age={gate_age_s * 1000.0:.1f}ms)"
    )

    cal_status["encoder"]["ok"] = True
    cal_status["encoder"]["details"] = {
        "ok": True,
        "mode": "STATIC_QUADRATURE_READINESS",
        "model": str(getattr(ctrl.enc_l, "model", "")),
        "count_mode": str(getattr(ctrl.enc_l, "count_mode", "")),
        "counts_per_revolution": int(getattr(ctrl.enc_l, "counts_per_revolution", 0)),
        **dict(gate_info or {}),
    }

    sm.ctx.calibration_status = cal_status

    ok, err = sup.validate_calibration(cal_status)
    if not ok:
        sm.ctx.last_error = err
        return StartupState.FAILSAFE, err

    sm.log("CALIBRATION_COMPLETE")
    return StartupState.BASELINE_CAPTURE, ""


def phase_baseline_capture(ctrl, sm: StartupStateMachine) -> tuple[StartupState | None, str]:
    """BASELINE_CAPTURE: referencia állapot rögzítése – EKF init alapja."""
    sm.ctx.set_state(StartupState.BASELINE_CAPTURE)
    sm.log(f"BASELINE_CAPTURE")

    baseline = sm.ctx.baseline
    baseline["imu_provider"] = str(getattr(ctrl, "imu_provider", "bno055") or "bno055")
    imu_snapshot = ctrl.imu_service.get_snapshot() if getattr(ctrl, "imu_service", None) else None
    if imu_snapshot is None:
        error = "BNO055 baseline snapshot missing"
        sm.ctx.last_error = error
        return StartupState.FAILSAFE, error
    baseline["imu_calibration"] = dict(getattr(imu_snapshot, "calibration", {}) or {})
    baseline["imu_measurement_timestamp"] = float(getattr(imu_snapshot, "timestamp", 0.0) or 0.0)
    baseline["encoder_zero"] = True

    sm.log("Baseline rögzítve")
    return StartupState.SAFETY_VALIDATION, ""


def phase_safety_validation(ctrl, sm: StartupStateMachine, sup: StartupSupervisor) -> tuple[StartupState | None, str]:
    """SAFETY_VALIDATION: IMU valid, gyro bias, szenzorok, motor safe."""
    sm.ctx.set_state(StartupState.SAFETY_VALIDATION)
    sm.log(f"SAFETY_VALIDATION")

    ok, err = sup.validate_safety_before_armed(sm.ctx)
    if not ok:
        sm.ctx.last_error = err
        return StartupState.FAILSAFE, err

    sm.log("Biztonsági ellenőrzés OK")
    return StartupState.CONTROL_ARMED, ""


def phase_control_armed(ctrl, sm: StartupStateMachine) -> tuple[StartupState | None, str]:
    """
    CONTROL_ARMED: EKF, control loop, szenzor szálak indulnak.
    Motorok maradnak SAFE (0).
    A components.py többi initja itt fut: software systems, state machine, control loop, watchdog.
    """
    sm.ctx.set_state(StartupState.CONTROL_ARMED)
    sm.log(f"CONTROL_ARMED")

    # Import és init a components-ból – a _init_software_systems, _init_state_machine, stb.
    # (_init_variables, _init_security_layers már a pipeline előtt futott)
    from controller.components import (
        _init_software_systems,
        _init_state_machine,
        _init_lidar_summary_worker,
        _init_core_ai,
        _init_control_loop,
    )
    from controller.tables import build_speed_tables

    _init_software_systems(ctrl)
    _init_state_machine(ctrl)

    # Mindig – lidar_lock, lidar_summary stb. szükséges a főhurokhoz
    _init_lidar_summary_worker(ctrl)

    _init_core_ai(ctrl)
    build_speed_tables(ctrl)
    _init_control_loop(ctrl)

    # Stream writer
    try:
        from controller.stream_writer import start_stream_writer
        start_stream_writer(ctrl)
    except Exception as e:
        if ctrl.logger:
            ctrl.logger.warn(f"[stream_writer] Indítás kihagyva: {e}")

    # EKF alaphelyzet – baseline alapján
    ctrl.ekf.reset()
    if getattr(ctrl, "encoder_service", None) and getattr(ctrl.encoder_service, "estimator", None):
        est = ctrl.encoder_service.estimator
        if hasattr(est, "left") and hasattr(est.left, "distance"):
            est.left.distance = 0.0
        if hasattr(est, "right") and hasattr(est.right, "distance"):
            est.right.distance = 0.0
        if hasattr(est, "theta_enc"):
            est.theta_enc = 0.0
        if hasattr(est, "_ds_l_acc"):
            est._ds_l_acc = 0.0
            est._ds_r_acc = 0.0

    sm.log("Control armed – EKF, PID, watchdog aktiv")
    return StartupState.READY, ""


def phase_ready(ctrl, sm: StartupStateMachine) -> tuple[StartupState | None, str]:
    """READY: robot teljesen működőképes."""
    sm.ctx.set_state(StartupState.READY)
    sm.log("READY – rendszer indítás kész")
    return None, ""


def run_startup_pipeline(ctrl) -> StartupState:
    """
    Teljes startup pipeline futtatása. Blokkoló hívás.
    Visszatérés: READY, DEGRADED vagy FAILSAFE.
    """
    import signal
    from config_manager import config as global_config

    # Startup konfig
    startup_cfg = {}
    try:
        cfg_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "conf", "startup.json")
        if os.path.exists(cfg_path):
            with open(cfg_path, "r", encoding="utf-8") as f:
                startup_cfg = json.load(f)
    except Exception:
        pass

    sm = StartupStateMachine(startup_cfg, logger=getattr(ctrl, "logger", None))
    sup = StartupSupervisor(sm, logger=getattr(ctrl, "logger", None))
    sm.ctx.set_state(StartupState.BOOT)  # Időzítő induljon az első fázistól

    phases = [
        (StartupState.BOOT, lambda: phase_boot(ctrl, sm)),
        (StartupState.HARDWARE_DISCOVERY, lambda: phase_hardware_discovery(ctrl, sm, sup)),
        (StartupState.PERIPHERAL_INIT, lambda: phase_peripheral_init(ctrl, sm)),
        (StartupState.SENSOR_STABILIZATION, lambda: phase_sensor_stabilization(ctrl, sm)),
        (StartupState.CALIBRATION, lambda: phase_calibration(ctrl, sm, sup)),
        (StartupState.BASELINE_CAPTURE, lambda: phase_baseline_capture(ctrl, sm)),
        (StartupState.SAFETY_VALIDATION, lambda: phase_safety_validation(ctrl, sm, sup)),
        (StartupState.CONTROL_ARMED, lambda: phase_control_armed(ctrl, sm)),
        (StartupState.READY, lambda: phase_ready(ctrl, sm)),
    ]

    # Ciklus: fázisonként halad, timeout ellenőrzéssel
    for state, run_phase in phases:
        if sm.is_timed_out():
            sm.ctx.last_error = f"Timeout: {sm.ctx.current_state.name}"
            if ctrl.logger:
                ctrl.logger.error(f"[STARTUP] {sm.ctx.last_error}")
            sm.ctx.set_state(StartupState.FAILSAFE)
            ctrl.startup_status = sm.get_state_for_gui()
            return StartupState.FAILSAFE

        next_state, err = run_phase()

        if next_state == StartupState.FAILSAFE:
            sm.ctx.set_state(StartupState.FAILSAFE)
            ctrl.startup_status = sm.get_state_for_gui()
            if ctrl.logger:
                ctrl.logger.error(f"[STARTUP] FAILSAFE: {err}")
            return StartupState.FAILSAFE

        if next_state == StartupState.DEGRADED:
            sm.ctx.set_state(StartupState.DEGRADED)
            ctrl.startup_status = sm.get_state_for_gui()
            if ctrl.logger:
                ctrl.logger.warn(f"[STARTUP] DEGRADED: {sm.ctx.degraded_reasons}")
            return StartupState.DEGRADED

        if next_state is None:
            break

        sm.ctx.set_state(next_state)

    ctrl.startup_status = sm.get_state_for_gui()
    return sm.ctx.current_state
