#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
from dataclasses import dataclass, replace
from typing import Optional


@dataclass(frozen=True)
class IMUSnapshot:
    """Immutable IMU snapshot with timestamp."""
    timestamp: float
    accel: tuple  # (x, y, z) in g
    gyro: tuple   # (x, y, z) in dps
    mag: Optional[float]  # heading in degrees, or None if not available
    health: str  # "OK", "ERROR"
    source: str = "bno055"
    euler: Optional[dict] = None
    quaternion: Optional[tuple] = None
    calibration: Optional[dict] = None
    linear_accel_mps2: Optional[tuple] = None
    gravity_mps2: Optional[tuple] = None
    published_at: Optional[float] = None
    last_error: str = ""
    consecutive_errors: int = 0


class IMUService:
    """Threaded snapshot service for the single active BNO055 driver."""

    def __init__(self, imu):
        if imu is None:
            raise ValueError("bno055_driver_missing")
        provider = str(getattr(imu, "provider", "") or "").strip().lower()
        if provider != "bno055":
            raise ValueError(f"unsupported_imu_provider:{provider or 'MISSING'}")
        self.imu = imu
        
        self._lock = threading.Lock()
        self._current_snapshot: Optional[IMUSnapshot] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._update_rate_hz = 50.0

    def start(self):
        """Start the IMU service thread."""
        if self._running:
            return True
        
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        return True

    def stop(self, *, join_timeout_s: float = 1.0, close_devices: bool = False):
        """Stop the IMU service thread."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=max(0.05, float(join_timeout_s)))
        if close_devices:
            close = getattr(self.imu, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    pass

    def _worker(self):
        """Worker thread: continuously reads IMU and updates snapshot."""
        dt_target = 1.0 / self._update_rate_hz
        next_time = time.perf_counter()
        
        while self._running:
            try:
                now = time.perf_counter()

                read_sample = getattr(self.imu, "read_sample", None)
                if not callable(read_sample):
                    raise RuntimeError("bno055_read_sample_missing")
                sample = read_sample()
                if not isinstance(sample, dict):
                    raise RuntimeError("bno055_sample_invalid")

                accel_data = tuple(sample.get("accel_g", (0.0, 0.0, 0.0)))
                gyro_data = tuple(sample.get("gyro_dps", (0.0, 0.0, 0.0)))
                mag_heading = sample.get("heading_deg")
                source = str(sample.get("source", "bno055") or "bno055").strip().lower()
                if source != "bno055":
                    raise RuntimeError(f"unexpected_imu_sample_source:{source or 'MISSING'}")
                euler = sample.get("euler") if isinstance(sample.get("euler"), dict) else None
                quaternion = sample.get("quaternion")
                calibration = sample.get("calibration") if isinstance(sample.get("calibration"), dict) else None
                linear_accel_mps2 = sample.get("linear_accel_mps2")
                gravity_mps2 = sample.get("gravity_mps2")
                
                snapshot = IMUSnapshot(
                    timestamp=now,
                    accel=accel_data,
                    gyro=gyro_data,
                    mag=mag_heading,
                    health="OK",
                    source=source,
                    euler=euler,
                    quaternion=quaternion,
                    calibration=calibration,
                    linear_accel_mps2=linear_accel_mps2,
                    gravity_mps2=gravity_mps2,
                    published_at=now,
                    last_error="",
                    consecutive_errors=0,
                )
                
                with self._lock:
                    self._current_snapshot = snapshot
                
                # Timing control
                next_time += dt_target
                pause = next_time - time.perf_counter()
                if pause > 0:
                    time.sleep(pause)
                else:
                    next_time = time.perf_counter()
                    
            except Exception as e:
                now = time.perf_counter()
                self._publish_read_error(now=now, error=e)
                time.sleep(0.1)

    def _publish_read_error(self, *, now: float, error: Exception) -> None:
        """Publish acquisition state without forging a fresh IMU measurement.

        A single I2C read failure must not replace a still-fresh, valid sample
        with fresh zero vectors.  Preserve the last measurement timestamp and
        values as ``DEGRADED``; the safety stale-age contract decides how long
        that measurement remains usable.  Before the first valid sample, the
        service remains a hard ``ERROR``.
        """
        error_text = f"{type(error).__name__}: {error}"
        with self._lock:
            previous = self._current_snapshot
            previous_health = str(getattr(previous, "health", "") or "").upper()
            previous_ts = float(getattr(previous, "timestamp", 0.0) or 0.0)
            previous_errors = int(getattr(previous, "consecutive_errors", 0) or 0)
            if previous is not None and previous_health in ("OK", "DEGRADED") and previous_ts > 0.0:
                snapshot = replace(
                    previous,
                    health="DEGRADED",
                    published_at=float(now),
                    last_error=error_text,
                    consecutive_errors=previous_errors + 1,
                )
            else:
                snapshot = IMUSnapshot(
                    timestamp=float(now),
                    accel=(0.0, 0.0, 0.0),
                    gyro=(0.0, 0.0, 0.0),
                    mag=None,
                    health="ERROR",
                    source=str(getattr(self.imu, "provider", "unknown") or "unknown"),
                    published_at=float(now),
                    last_error=error_text,
                    consecutive_errors=previous_errors + 1,
                )
            self._current_snapshot = snapshot

    def get_snapshot(self) -> Optional[IMUSnapshot]:
        """
        Get the latest immutable snapshot.
        Returns None if no snapshot available yet.
        """
        with self._lock:
            return self._current_snapshot
