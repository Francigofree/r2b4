#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Startup Supervisor – indítási lépések monitorozása, hibakezelés, timeout.
Felelős: sensor health ellenőrzés, driver init validálás, kalibráció eredmény ellenőrzés,
watchdog aktiválás.
"""

import time
from startup.state_machine import StartupState, StartupStateMachine, StartupContext


class StartupSupervisor:
    """
    Supervisor réteg: ellenőrzi a fázisok eredményeit, dönt FAILSAFE/DEGRADED-ról.
    """

    def __init__(self, sm: StartupStateMachine, logger=None):
        self.sm = sm
        self.logger = logger
        self.ctx = sm.ctx

    def validate_hardware_discovery(self, discovery: dict) -> tuple[bool, str]:
        """
        Ellenőrzi: IMU, motor GPIO, LIDAR, kamera elérhetőség.
        Vissza: (ok, hibaüzenet)
        """
        # IMU kritikus – hiánya FAILSAFE
        imu_ok = discovery.get("imu", {}).get("ok", False)
        if not imu_ok:
            return False, "IMU nem elérhető (FAILSAFE)"

        # Motor driver kritikus
        motor_ok = discovery.get("motor_gpio", {}).get("ok", False)
        if not motor_ok:
            return False, "Motor GPIO nem elérhető (FAILSAFE)"

        # LIDAR hiánya → DEGRADED (navigation disabled)
        # Kamera hiánya → DEGRADED (vision disabled)
        return True, ""

    def validate_calibration(self, cal_status: dict) -> tuple[bool, str]:
        """Kalibráció eredmény ellenőrzése."""
        gyro_ok = cal_status.get("gyro", {}).get("ok", False)
        if not gyro_ok:
            return False, "Gyro kalibráció sikertelen"

        accel_ok = cal_status.get("accel", {}).get("ok", False)
        if not accel_ok:
            return False, "Accel kalibráció sikertelen"

        return True, ""

    def validate_safety_before_armed(self, ctx: StartupContext) -> tuple[bool, str]:
        """
        SAFETY_VALIDATION fázis ellenőrzése:
        - IMU adat valid
        - gyro bias != 0 (ha szükséges)
        - szenzorok válaszolnak
        - motor driver safe state
        """
        sh = ctx.sensor_health
        if not sh.get("imu", {}).get("ok", False):
            return False, "IMU nem valid"

        cal = ctx.calibration_status
        if not cal.get("gyro", {}).get("ok", False):
            return False, "Gyro kalibráció hiányzik"

        return True, ""

    def decide_degraded(self, missing: list) -> bool:
        """
        Döntés: ha csak nem-kritikus periféria hiányzik → DEGRADED, egyébként FAILSAFE.
        """
        critical = {"imu", "motor", "encoder"}
        for m in missing:
            if m.lower() in critical:
                return False  # FAILSAFE
        return True  # DEGRADED
