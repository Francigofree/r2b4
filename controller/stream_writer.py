#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GUI stream kép író: ha a kamera BE van és nincs követés/keresés,
periodikusan írja a runtime/stream_frame.jpg-t, hogy a GUI MJPEG megjeleníthesse.
A GUI SOHA nem nyitja a kamerát – csak ezt a fájlt olvassa.
"""

import os
import time
import threading

from driver.cam import camera_lifecycle_lock, safe_stop_close
from middleware.peripheral_usage import is_peripheral_enabled

try:
    from config_manager import config as global_config
except Exception:
    global_config = None


def _log_stream_debug(ctrl, message: str) -> None:
    logger = getattr(ctrl, "logger", None)
    if logger is None:
        return
    log_fn = getattr(logger, "debug", None) or getattr(logger, "warn", None) or getattr(logger, "info", None)
    if callable(log_fn):
        try:
            log_fn(str(message))
        except Exception:
            pass


def _is_camera_enabled(ctrl) -> bool:
    """Kamera BE/KI a canonical JSON SSOT-ból."""
    return is_peripheral_enabled("camera", status_path=getattr(ctrl, "status_path", None), default=False)


def _idle_preview_enabled() -> bool:
    """Idle GUI preview opens its own camera only when explicitly enabled."""
    if global_config is None:
        return False
    try:
        cam_cfg = global_config.get("cam") or {}
        preview_cfg = cam_cfg.get("előnézet") or {}
        return bool(preview_cfg.get("enabled", False))
    except Exception:
        return False


def _stream_writer_pause_active(ctrl, now=None) -> bool:
    """True, ha más kamera-tulajdonos kézi átadást kért."""
    if bool(getattr(ctrl, "_stream_writer_release_requested", False)):
        return True
    try:
        pause_until = float(getattr(ctrl, "_stream_writer_pause_until", 0.0) or 0.0)
    except Exception:
        pause_until = 0.0
    return float(now if now is not None else time.monotonic()) < pause_until


def request_stream_writer_camera_release(ctrl, timeout_s: float = 1.0, poll_s: float = 0.02) -> bool:
    """
    Kamera-handover FOLLOW/SEARCH előtt.
    A GUI stream writer csak preview-owner; ha mozgási/percepciós task indul,
    determinisztikusan el kell engednie a Picamera2 példányt.
    """
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    pause_until = deadline + 1.0
    try:
        current_pause = float(getattr(ctrl, "_stream_writer_pause_until", 0.0) or 0.0)
    except Exception:
        current_pause = 0.0
    ctrl._stream_writer_pause_until = max(current_pause, pause_until)
    ctrl._stream_writer_release_requested = True

    while time.monotonic() < deadline:
        if not bool(getattr(ctrl, "_stream_writer_camera_active", False)):
            ctrl._stream_writer_release_requested = False
            return True
        time.sleep(max(0.001, float(poll_s)))

    ctrl._stream_writer_release_requested = False
    return not bool(getattr(ctrl, "_stream_writer_camera_active", False))


def _stream_writer_loop(ctrl):
    """Háttérszál: kamera BE + nincs követés/keresés esetén ír stream_frame.jpg-t."""
    interval = 0.25  # ~4 fps, illeszkedik a GUI local profilhoz (smooth)
    cam = None
    runtime_dir = os.path.dirname(getattr(ctrl, "status_path", ""))
    if not runtime_dir:
        return
    stream_path = os.path.join(runtime_dir, "stream_frame.jpg")
    tmp_path = stream_path + ".stream.tmp"
    last_open_fail_ts = 0.0
    open_fail_cooldown_sec = 5.0

    while getattr(ctrl, "_stream_writer_running", True):
        try:
            time.sleep(interval)
            enabled = _is_camera_enabled(ctrl)
            preview_enabled = _idle_preview_enabled()
            following = getattr(ctrl, "following_active", False)
            searching = getattr(ctrl, "searching_person", False)
            paused = _stream_writer_pause_active(ctrl)

            if not enabled or not preview_enabled or following or searching or paused:
                if cam is not None:
                    safe_stop_close(cam)
                    cam = None
                    ctrl._stream_writer_camera_active = False
                    if hasattr(ctrl, "_stream_writer_first_frame_logged"):
                        del ctrl._stream_writer_first_frame_logged
                elif bool(getattr(ctrl, "_stream_writer_camera_active", False)):
                    ctrl._stream_writer_camera_active = False
                continue

            if cam is None:
                if time.monotonic() - last_open_fail_ts < open_fail_cooldown_sec:
                    continue
                try:
                    from picamera2 import Picamera2
                    with camera_lifecycle_lock():
                        cam = Picamera2()
                        # Native 16:9 IMX708 capture; software rotation corrects physical mounting.
                        cam.configure(cam.create_preview_configuration(main={"size": (640, 360)}))
                        cam.start()
                    ctrl._stream_writer_camera_active = True
                    if _stream_writer_pause_active(ctrl) or getattr(ctrl, "following_active", False) or getattr(ctrl, "searching_person", False):
                        safe_stop_close(cam)
                        cam = None
                        ctrl._stream_writer_camera_active = False
                        continue
                    time.sleep(0.3)
                    if getattr(ctrl, "logger", None):
                        ctrl.logger.info("[stream_writer] Kamera megnyitva, stream_frame.jpg írás indul.")
                    if hasattr(ctrl, "_stream_writer_failed_logged"):
                        del ctrl._stream_writer_failed_logged
                except Exception as e:
                    last_open_fail_ts = time.monotonic()
                    safe_stop_close(cam)
                    ctrl._stream_writer_camera_active = False
                    if not getattr(ctrl, "_stream_writer_failed_logged", False):
                        if getattr(ctrl, "logger", None):
                            ctrl.logger.warn(
                                "[stream_writer] A kamera nem nyitható meg. "
                                "Várok újrapróbálás előtt, hogy ne pörögjön hibába."
                            )
                        ctrl._stream_writer_failed_logged = True
                    cam = None
                    continue

            first_frame_done = getattr(ctrl, "_stream_writer_first_frame_logged", False)
            try:
                saved = False
                arr = cam.capture_array("main")
                if arr is not None:
                    raw_arr = arr
                    try:
                        from PIL import Image
                        from driver.cam import _camera_rotation_deg, _rotate_image
                        arr = _rotate_image(raw_arr, _camera_rotation_deg())
                        # main stream általában RGB888 vagy BGR888; 3 csatorna → kép
                        if len(arr.shape) == 3 and arr.shape[2] == 3:
                            img = Image.fromarray(arr, mode="RGB")
                        else:
                            img = Image.fromarray(arr)
                        img.save(tmp_path, "JPEG", quality=85)
                        os.replace(tmp_path, stream_path)
                        saved = True
                    except Exception:
                        # BGR esetén csatornák cseréje
                        try:
                            import numpy as np
                            from PIL import Image
                            from driver.cam import _camera_rotation_deg, _rotate_image
                            arr = _rotate_image(raw_arr, _camera_rotation_deg())
                            if len(arr.shape) == 3 and arr.shape[2] == 3:
                                arr = arr[:, :, ::-1].copy()
                            img = Image.fromarray(arr)
                            img.save(tmp_path, "JPEG", quality=85)
                            os.replace(tmp_path, stream_path)
                            saved = True
                        except Exception:
                            try:
                                if os.path.exists(tmp_path):
                                    os.unlink(tmp_path)
                            except Exception:
                                pass
                            pass
                if not saved:
                    # PIL hiány vagy konverziós hiba esetén próbáljuk közvetlenül a kamerát.
                    try:
                        cam.capture_file(tmp_path)
                        try:
                            from PIL import Image
                            from driver.cam import _camera_rotation_deg
                            rotation_deg = _camera_rotation_deg()
                            with Image.open(tmp_path) as img:
                                if rotation_deg:
                                    img.rotate(rotation_deg, expand=True).save(tmp_path, "JPEG", quality=85)
                            os.replace(tmp_path, stream_path)
                        except Exception:
                            os.replace(tmp_path, stream_path)
                            pass
                        saved = True
                    except Exception:
                        try:
                            if os.path.exists(tmp_path):
                                os.unlink(tmp_path)
                        except Exception:
                            pass
                        pass
                if saved and not first_frame_done and getattr(ctrl, "logger", None):
                    ctrl.logger.info("[stream_writer] Első kép írva: stream_frame.jpg")
                    ctrl._stream_writer_first_frame_logged = True
            except Exception as e:
                _log_stream_debug(ctrl, f"[stream_writer] Capture: {e}")
                try:
                    if os.path.exists(tmp_path):
                        os.unlink(tmp_path)
                except Exception:
                    pass
                safe_stop_close(cam)
                cam = None
                ctrl._stream_writer_camera_active = False
                if hasattr(ctrl, "_stream_writer_first_frame_logged"):
                    del ctrl._stream_writer_first_frame_logged

        except Exception as e:
            _log_stream_debug(ctrl, f"[stream_writer] {e}")
            safe_stop_close(cam)
            cam = None
            ctrl._stream_writer_camera_active = False

    if cam is not None:
        safe_stop_close(cam)
    ctrl._stream_writer_camera_active = False


def start_stream_writer(ctrl):
    """Stream writer szál indítása (controller indítás után hívandó)."""
    ctrl._stream_writer_running = True
    t = threading.Thread(target=_stream_writer_loop, args=(ctrl,), daemon=True)
    t.start()
    if getattr(ctrl, "logger", None):
        ctrl.logger.info("[stream_writer] GUI stream író szál indítva (Kamera BE → stream_frame.jpg).")
