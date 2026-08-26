"""
cam.py - Kamera / videó / kép: conf/cam.json alapján.
Fotó és videó mentése a Pic/ könyvtárba timestamp-elt névvel.
"""

import os
import time
import logging
import threading
from datetime import datetime

# Libcamera C++ log csöndesítés (ha még nincs beállítva, pl. os.py előtt import)
os.environ.setdefault("LIBCAMERA_LOG_LEVELS", "*:ERROR")

from picamera2 import Picamera2, Preview
# Picamera2 Python log csöndesítés – ne szemeteljen INFO a terminálba
logging.getLogger("picamera2").setLevel(logging.WARNING)
logging.getLogger("picamera2.picamera2").setLevel(logging.WARNING)
from picamera2.encoders import H264Encoder
try:
    from picamera2.outputs import FfmpegOutput  # MP4 kimenethez
except Exception:
    FfmpegOutput = None

try:
    from config_manager import config as global_config
except ImportError:
    global_config = None


_CAMERA_LIFECYCLE_LOCK = threading.RLock()


def camera_lifecycle_lock():
    """Közös lock a Picamera2 példányok nyitásához és lezárásához."""
    return _CAMERA_LIFECYCLE_LOCK


def safe_stop_close(camera) -> None:
    """Picamera2 példány leállítása és lezárása best-effort módban."""
    if camera is None:
        return
    with _CAMERA_LIFECYCLE_LOCK:
        try:
            camera.stop_recording()
        except Exception:
            pass
        try:
            camera.stop()
        except Exception:
            pass
        try:
            camera.stop_preview()
        except Exception:
            pass
        try:
            if hasattr(camera, "close"):
                camera.close()
        except Exception:
            pass


def project_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _cam_cfg():
    """conf/cam.json kamera és kép beállítások. get(section) egy arg = teljes szekció."""
    if global_config:
        return global_config.get("cam") or {}
    return {}


def pic_dir():
    """Mentési mappa: conf/cam.json → kep.mentes_mappa, alapértelmezetten Pic."""
    cfg = _cam_cfg()
    mappa = (cfg.get("kep") or {}).get("mentes_mappa", "Pic")
    d = os.path.join(project_root(), mappa)
    os.makedirs(d, exist_ok=True)
    return d


def vid_dir():
    """Videó mentési mappa: conf/cam.json → video.mentes_mappa, alapértelmezetten vid."""
    cfg = _cam_cfg()
    mappa = (cfg.get("video") or {}).get("mentes_mappa", "vid")
    d = os.path.join(project_root(), mappa)
    os.makedirs(d, exist_ok=True)
    return d


def video_ts_base(prefix=None):
    """Videó fájlnév előtag: video.fajl_prefix vagy prefix."""
    cfg = _cam_cfg()
    p = (cfg.get("video") or {}).get("fajl_prefix", "vid")
    if prefix is not None:
        p = prefix
    fmt = (cfg.get("kep") or {}).get("timestamp_formátum", "%Y-%m-%d_%H-%M-%S")
    return f"{p}{datetime.now().strftime(fmt)}"


def _resolution(preset="kozepes"):
    """Felbontás (width, height) a conf/cam.json felbontas preset alapján."""
    cfg = _cam_cfg()
    fel = (cfg.get("kamera") or {}).get("felbontas") or {}
    preset = (cfg.get("kamera") or {}).get("alap_felbontas", preset)
    res = fel.get(preset, fel.get("kozepes", {"width": 960, "height": 720}))
    return (int(res.get("width", 960)), int(res.get("height", 720)))


def _video_bitrate():
    cfg = _cam_cfg()
    return int((cfg.get("video") or {}).get("bitrate", 2_000_000))


def _camera_device():
    """conf/cam.json → kamera.device (0 vagy 1, két slot esetén)."""
    cfg = _cam_cfg()
    return int((cfg.get("kamera") or {}).get("device", 0))


def _camera_rotation_deg():
    """conf/cam.json → kamera.forgatas_fok (0, 90, 180, 270)."""
    cfg = _cam_cfg()
    return int((cfg.get("kamera") or {}).get("forgatas_fok", 0))


def _rotate_image(img_array, rotation_deg):
    """Kép forgatása numpy array formátumban (0, 90, 180, 270 fok)."""
    if rotation_deg == 0:
        return img_array
    try:
        import numpy as np
        if rotation_deg == 90:
            return np.rot90(img_array, k=1, axes=(0, 1))
        elif rotation_deg == 180:
            return np.rot90(img_array, k=2, axes=(0, 1))
        elif rotation_deg == 270:
            return np.rot90(img_array, k=3, axes=(0, 1))
    except Exception:
        pass
    return img_array


def ts_base(prefix=None):
    """Timestamp-elt fájlnév előtag. conf/cam.json → kep.fajl_prefix."""
    cfg = _cam_cfg()
    p = (cfg.get("kep") or {}).get("fajl_prefix", "pic")
    if prefix is not None:
        p = prefix
    fmt = (cfg.get("kep") or {}).get("timestamp_formátum", "%Y-%m-%d_%H-%M-%S")
    return f"{p}{datetime.now().strftime(fmt)}"


class Camera:
    """Pelda.py mintájára: 640x480, Picamera2(), videó konfig, capture_file()."""
    def __init__(self, width=None, height=None, bitrate=None, resolution_preset="kozepes", camera_num=None):
        # Pelda pontosan: 640, 480, 2_000_000 – config nélkül, hogy kizárjuk a config→unhashable láncot
        width = width if width is not None else 640
        height = height if height is not None else 480
        bitrate = bitrate if bitrate is not None else 2_000_000
        size = (int(width), int(height))
        with _CAMERA_LIFECYCLE_LOCK:
            self.picam2 = Picamera2()
            self.video_config = self.picam2.create_video_configuration(main={"size": size})
            self.picam2.configure(self.video_config)
        self.encoder = H264Encoder(bitrate=bitrate)
        self.rotation_deg = _camera_rotation_deg()

    def start(self):
        with _CAMERA_LIFECYCLE_LOCK:
            self.picam2.start_preview(Preview.NULL)
            self.picam2.start()
        time.sleep(0.3)

    def capture(self, filename):
        folder = os.path.dirname(filename)
        if folder and not os.path.exists(folder):
            os.makedirs(folder, exist_ok=True)
        # Pelda szerint: egyszerűen capture_file (videó konfiggal már fut)
        if self.rotation_deg == 0:
            self.picam2.capture_file(filename)
        else:
            # Forgatás: capture_array → forgatás → PIL save
            arr = self.picam2.capture_array()
            arr_rotated = _rotate_image(arr, self.rotation_deg)
            try:
                from PIL import Image
                img = Image.fromarray(arr_rotated)
                img.save(filename)
            except Exception:
                self.picam2.capture_file(filename)
        print(f"[CAMERA] Készült kép: {filename}")
    
    def capture_array(self):
        """Frame capture numpy array-ként (forgatással)."""
        arr = self.picam2.capture_array()
        return _rotate_image(arr, self.rotation_deg)

    def record(self, seconds, filename_mp4):
        folder = os.path.dirname(filename_mp4)
        if folder and not os.path.exists(folder):
            os.makedirs(folder, exist_ok=True)

        if FfmpegOutput is not None:
            out = FfmpegOutput(filename_mp4)
            print(f"[CAMERA] Felvétel indul (MP4): {filename_mp4} [{seconds} s]")
            self.picam2.start_recording(self.encoder, out, name="main")
            time.sleep(float(seconds))
            self.picam2.stop_recording()
        else:
            # Fallback: nyers H.264
            h264_path = os.path.splitext(filename_mp4)[0] + ".h264"
            print(f"[CAMERA] Felvétel indul (H264): {h264_path} [{seconds} s]")
            self.picam2.start_recording(self.encoder, h264_path, name="main")
            time.sleep(float(seconds))
            self.picam2.stop_recording()
        print("[CAMERA] Felvétel kész.")

    def start_recording_to_file(self, path_mp4):
        """V toggle: felvétel indítása fájlba (start() után hívandó)."""
        folder = os.path.dirname(path_mp4)
        if folder and not os.path.exists(folder):
            os.makedirs(folder, exist_ok=True)
        if FfmpegOutput is not None:
            self._recording_output = FfmpegOutput(path_mp4)
            self.picam2.start_recording(self.encoder, self._recording_output, name="main")
        else:
            h264_path = os.path.splitext(path_mp4)[0] + ".h264"
            self._recording_output = open(h264_path, "wb")
            self.picam2.start_recording(self.encoder, self._recording_output, name="main")
        print(f"[CAMERA] Videó felvétel indul: {path_mp4}")

    def stop_recording_to_file(self):
        """V toggle: felvétel leállítása."""
        try:
            self.picam2.stop_recording()
        except Exception:
            pass
        self._recording_output = getattr(self, "_recording_output", None)
        if self._recording_output and hasattr(self._recording_output, "close"):
            try:
                self._recording_output.close()
            except Exception:
                pass
        print("[CAMERA] Videó felvétel leállítva.")

    def stop(self):
        safe_stop_close(self.picam2)
        time.sleep(0.5)


# Standalone futtatás: fotó + 2 mp videó a Pic mappába
if __name__ == "__main__":
    base = os.path.join(pic_dir(), ts_base("teszt"))
    img_path = base + ".jpg"
    vid_path = base + ".mp4"

    cam = Camera()
    try:
        cam.start()
        cam.capture(img_path)
        cam.record(2, vid_path)
    finally:
        cam.stop()
        print("[CAMERA] Kész, kilépés.")
