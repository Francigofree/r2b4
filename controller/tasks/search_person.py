#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
KERESD AZ EMBERT: H billentyűvel indított 360° forgatás, kis felbontású,
alacsony fps-es kamera keresés. Első ember találatnál STOP + TTS "EMBER".
"""

import math
import time
from typing import Optional, Tuple

from driver.cam import camera_lifecycle_lock, safe_stop_close
from middleware.peripheral_usage import is_peripheral_enabled

# Forgás: elég nagy omega, hogy a motion_executor tiszta forgatás ágon legyen.
SEARCH_OMEGA_RAD_PER_SEC = 0.55
SEARCH_IMAGE_SIZE = (320, 180)


class SearchCamera:
    """
    Keresés-specifikus kamera management (nyers 320x180, 180 fok után is 320x180, alacsony fps).
    Kiváltja a modul-szintű globális változókat.
    """

    def __init__(self, open_cooldown_sec: float = 5.0, capture_interval_sec: float = 0.45):
        self._camera = None
        self._pose_detector = None
        self._last_capture_time: float = 0.0
        self._last_open_fail_time: float = 0.0
        self._open_cooldown_sec = open_cooldown_sec
        self._capture_interval_sec = capture_interval_sec

    def ensure_open(self, ctrl) -> bool:
        if not is_peripheral_enabled("camera", status_path=getattr(ctrl, "status_path", None), default=False):
            return False
        if self._camera is not None:
            return True
        now = time.monotonic()
        if now - self._last_open_fail_time < self._open_cooldown_sec:
            return False
        try:
            from picamera2 import Picamera2
            with camera_lifecycle_lock():
                cam = Picamera2()
                config = cam.create_preview_configuration(main={"size": SEARCH_IMAGE_SIZE})
                cam.configure(config)
                cam.start()
            time.sleep(0.25)
            self._camera = cam
            if hasattr(ctrl, "logger"):
                ctrl.logger.info("[KERESD] Kamera indítva (keresés).")
            return True
        except Exception as e:
            self._last_open_fail_time = now
            safe_stop_close(locals().get("cam"))
            if hasattr(ctrl, "logger"):
                ctrl.logger.warn(f"[KERESD] Kamera hiba: {e}")
            return False

    def release(self, ctrl=None):
        if self._camera is not None:
            safe_stop_close(self._camera)
            self._camera = None
            if ctrl and hasattr(ctrl, "logger"):
                ctrl.logger.info("[KERESD] Kamera leállítva.")

    def _ensure_pose_detector(self) -> bool:
        if self._pose_detector is not None:
            return True
        try:
            import mediapipe as mp
            mp_pose = mp.solutions.pose
            self._pose_detector = mp_pose.Pose(
                static_image_mode=False,
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.4,
            )
            return True
        except ImportError:
            return False

    def detect_person(self, frame_rgb) -> bool:
        if not self._ensure_pose_detector():
            return False
        try:
            results = self._pose_detector.process(frame_rgb)
            return results.pose_landmarks is not None
        except Exception:
            return False

    def capture_frame(self, ctrl) -> Optional[Tuple]:
        if self._camera is None and not self.ensure_open(ctrl):
            return None
        try:
            from driver.cam import _camera_rotation_deg, _rotate_image
            arr = self._camera.capture_array()
            if arr is None or arr.size == 0:
                return None
            rotation_deg = _camera_rotation_deg()
            arr = _rotate_image(arr, rotation_deg)
            h, w = arr.shape[:2]
            if len(arr.shape) == 2:
                import numpy as np
                frame_rgb = np.stack([arr] * 3, axis=-1)
            else:
                frame_rgb = arr
                if arr.shape[2] == 4:
                    frame_rgb = frame_rgb[:, :, :3]
            return (frame_rgb, w, h)
        except Exception:
            self.release()
            return None

    def should_capture(self) -> bool:
        now = time.monotonic()
        if now - self._last_capture_time >= self._capture_interval_sec:
            self._last_capture_time = now
            return True
        return False


def _get_search_camera(ctrl) -> SearchCamera:
    """Lazy init: ctrl.search_camera példány."""
    cam = getattr(ctrl, "search_camera", None)
    if cam is None:
        cam = SearchCamera()
        ctrl.search_camera = cam
    return cam


def start_search_person(ctrl) -> None:
    """H billentyű: KERESD AZ EMBERT – 360° forgatás, kis felbontás, alacsony fps."""
    if getattr(ctrl, "searching_person", False):
        if hasattr(ctrl, "logger"):
            ctrl.logger.info("[KERESD] Már fut a keresés.")
        return
    ctrl.searching_person = True
    ctrl._search_start_yaw = None
    ctrl._search_total_rotated_deg = 0.0
    ctrl._search_last_theta = None
    try:
        from controller.status import append_camera_log
        scam = _get_search_camera(ctrl)
        append_camera_log(ctrl, "search_start", resolution="320x180", fps_hint=1.0 / scam._capture_interval_sec)
    except Exception:
        pass
    if hasattr(ctrl, "logger"):
        ctrl.logger.info("[KERESD] KERESD AZ EMBERT – 360° forgatás indul (kis felbontás, alacsony fps).")


def stop_search_person(ctrl) -> None:
    """Keresés leállítása; kamera felszabadítása."""
    ctrl.searching_person = False
    try:
        from controller.status import append_camera_log
        append_camera_log(ctrl, "search_stop", rotated_deg=getattr(ctrl, "_search_total_rotated_deg", 0))
    except Exception:
        pass
    scam = _get_search_camera(ctrl)
    scam.release(ctrl)


def tick_search_person(ctrl, dt: float) -> Tuple[Optional[float], bool, bool]:
    """
    Egy keresési ciklus. Vissza: (omega_target vagy None, stop_search, person_found).
    Ha stop_search: hívd stop_search_person(ctrl). Ha person_found: TTS "EMBER".
    """
    if not getattr(ctrl, "searching_person", False):
        return (None, False, False)

    scam = _get_search_camera(ctrl)
    ekf_state = ctrl.ekf.get_state()
    theta_deg = ekf_state.get("theta_deg", 0.0)

    if ctrl._search_start_yaw is None:
        ctrl._search_start_yaw = theta_deg
        ctrl._search_last_theta = theta_deg
        ctrl._search_total_rotated_deg = 0.0

    # Delta szög (rövid dt alatt)
    if ctrl._search_last_theta is not None:
        d = (theta_deg - ctrl._search_last_theta + 180) % 360 - 180
        ctrl._search_total_rotated_deg += abs(d)
    ctrl._search_last_theta = theta_deg

    # 360° megvolt → stop, nem találtunk
    if ctrl._search_total_rotated_deg >= 355.0:
        try:
            from controller.status import append_camera_log
            append_camera_log(ctrl, "search_360_done", person_found=False)
        except Exception:
            pass
        if hasattr(ctrl, "logger"):
            ctrl.logger.info("[KERESD] 360° kész – ember nem található.")
        return (0.0, True, False)

    # Throttle: csak időnként kép + detekció (alacsony fps)
    if scam.should_capture():
        cap = scam.capture_frame(ctrl)
        if cap is not None:
            frame_rgb, w, h = cap
            if scam.detect_person(frame_rgb):
                try:
                    from controller.status import append_camera_log
                    append_camera_log(ctrl, "person_found", rotated_deg=ctrl._search_total_rotated_deg)
                except Exception:
                    pass
                if hasattr(ctrl, "logger"):
                    ctrl.logger.info("[KERESD] EMBER találva – megállok.")
                return (0.0, True, True)

    return (SEARCH_OMEGA_RAD_PER_SEC, False, False)


def is_searching(ctrl) -> bool:
    return getattr(ctrl, "searching_person", False)
