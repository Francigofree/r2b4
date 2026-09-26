#!/usr/bin/env python3
"""Apply the R2B4 camera-geometry refactor to main@87cec6c.

The installer is intentionally marker-based and refuses ambiguous edits.  It
preserves user files by backing up every touched existing file before writing.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

EXPECTED_HEAD = "87cec6c224d47b48cf02605eba50f5aff81e3bdd"
UPGRADE_ID = "camera_geometry_v1_20260926"


def _replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source marker, found {count}")
    return text.replace(old, new, 1)


def _git_head(root: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _patch_hardver(text: str) -> str:
    old = '    "continuous_autofocus": true,\n    "max_frame_completion_lag_ns": 500000000,\n'
    new = '''    "continuous_autofocus": true,\n    "geometry": {\n      "factory_profile": "imx708_wide_noir",\n      "mount": {\n        "x_m": 0.05,\n        "y_m": 0.0,\n        "z_m": 0.19,\n        "roll_deg": 0.0,\n        "pitch_deg": 15.0,\n        "yaw_deg": 0.0\n      },\n      "intrinsic": {\n        "source": "factory_nominal",\n        "distortion_model": "unknown",\n        "distortion_coefficients": null\n      }\n    },\n    "max_frame_completion_lag_ns": 500000000,\n'''
    return _replace_once(text, old, new, "conf/hardver.json geometry")


def _patch_config_hardware(text: str) -> str:
    text = _replace_once(
        text,
        'from v3.adapters.bno055_imu import Bno055ImuBackendConfig\n',
        'from v3.adapters.bno055_imu import Bno055ImuBackendConfig\nfrom v3.adapters.camera_geometry import (\n    CameraGeometryConfig,\n    camera_geometry_config_from_mapping,\n)\n',
        "config_hardware geometry import",
    )
    text = _replace_once(
        text,
        '    camera_device: Picamera2CameraConfig | None = None\n    camera_source: NativeCameraConfig | None = None\n',
        '    camera_device: Picamera2CameraConfig | None = None\n    camera_geometry: CameraGeometryConfig | None = None\n    camera_source: NativeCameraConfig | None = None\n',
        "config_hardware camera locals",
    )
    text = _replace_once(
        text,
        '''            camera_device = picamera2_camera_config_from_mapping(camera)\n            camera_source = NativeCameraConfig(\n''',
        '''            geometry_value = camera.get("geometry")\n            if geometry_value is None:\n                raise ValueError("enabled camera requires hardware config camera.geometry")\n            camera_geometry = camera_geometry_config_from_mapping(\n                _mapping(geometry_value, "hardware config camera.geometry")\n            )\n            camera_device = picamera2_camera_config_from_mapping(\n                {key: item for key, item in camera.items() if key != "geometry"}\n            )\n            camera_source = NativeCameraConfig(\n''',
        "config_hardware split acquisition/geometry",
    )
    text = _replace_once(
        text,
        '        camera_device=camera_device,\n        person_detection_backend=person_detection_backend,\n',
        '        camera_device=camera_device,\n        camera_geometry=camera_geometry,\n        person_detection_backend=person_detection_backend,\n',
        "config_hardware return geometry",
    )
    return text


def _patch_native_sensor_inputs(text: str) -> str:
    text = _replace_once(
        text,
        'from v3.adapters.bno055_device import NativeBno055DeviceConfig\n',
        'from v3.adapters.bno055_device import NativeBno055DeviceConfig\nfrom v3.adapters.camera_geometry import CameraGeometryConfig\n',
        "native_sensor_inputs geometry import",
    )
    text = _replace_once(
        text,
        '    camera_device: Picamera2CameraConfig | None = None\n    person_detection_backend: LiteRtPersonDetectorConfig | None = None\n',
        '    camera_device: Picamera2CameraConfig | None = None\n    camera_geometry: CameraGeometryConfig | None = None\n    person_detection_backend: LiteRtPersonDetectorConfig | None = None\n',
        "native_sensor_inputs geometry field",
    )
    old = '''        if self.camera_device is not None and not isinstance(\n            self.camera_device, Picamera2CameraConfig\n        ):\n            raise TypeError("camera_device must be Picamera2CameraConfig or None")\n        if (self.camera_device is None) != (self.inputs.camera_source is None):\n'''
    new = '''        if self.camera_device is not None and not isinstance(\n            self.camera_device, Picamera2CameraConfig\n        ):\n            raise TypeError("camera_device must be Picamera2CameraConfig or None")\n        if self.camera_geometry is not None and not isinstance(\n            self.camera_geometry, CameraGeometryConfig\n        ):\n            raise TypeError("camera_geometry must be CameraGeometryConfig or None")\n        if (self.camera_device is None) != (self.camera_geometry is None):\n            raise ValueError("camera device and geometry configs must be enabled together")\n        if (self.camera_device is None) != (self.inputs.camera_source is None):\n'''
    return _replace_once(text, old, new, "native_sensor_inputs geometry invariant")


def _patch_process_vision(text: str) -> str:
    text = _replace_once(
        text,
        'from .litert_person_detector import LiteRtPersonDetectorConfig, LiteRtSsdPersonDetector\n',
        'from .camera_geometry import CameraGeometryConfig\nfrom .litert_person_detector import LiteRtPersonDetectorConfig, LiteRtSsdPersonDetector\n',
        "process_vision geometry import",
    )
    text = _replace_once(
        text,
        '''def _vision_process_main(\n    camera_config: Picamera2CameraConfig,\n    detector_config: LiteRtPersonDetectorConfig | None,\n''',
        '''def _vision_process_main(\n    camera_config: Picamera2CameraConfig,\n    camera_geometry: CameraGeometryConfig | None,\n    detector_config: LiteRtPersonDetectorConfig | None,\n''',
        "process_vision child signature",
    )
    text = _replace_once(
        text,
        '''        camera = NativePicamera2Camera(\n            camera_config,\n            picamera_factory=default_picamera2_factory,\n''',
        '''        camera = NativePicamera2Camera(\n            camera_config,\n            camera_geometry_config=camera_geometry,\n            picamera_factory=default_picamera2_factory,\n''',
        "process_vision child camera geometry",
    )
    text = _replace_once(
        text,
        '''        detector_config: LiteRtPersonDetectorConfig | None,\n        *,\n        worker_cpu: int | None = None,\n''',
        '''        detector_config: LiteRtPersonDetectorConfig | None,\n        *,\n        camera_geometry: CameraGeometryConfig | None = None,\n        worker_cpu: int | None = None,\n''',
        "process_vision constructor signature",
    )
    text = _replace_once(
        text,
        '''        if detector_config is not None and not isinstance(detector_config, LiteRtPersonDetectorConfig):\n            raise TypeError("detector_config must be LiteRtPersonDetectorConfig or None")\n        if worker_cpu is not None and (\n''',
        '''        if detector_config is not None and not isinstance(detector_config, LiteRtPersonDetectorConfig):\n            raise TypeError("detector_config must be LiteRtPersonDetectorConfig or None")\n        if camera_geometry is not None and not isinstance(camera_geometry, CameraGeometryConfig):\n            raise TypeError("camera_geometry must be CameraGeometryConfig or None")\n        if worker_cpu is not None and (\n''',
        "process_vision geometry validation",
    )
    text = _replace_once(
        text,
        '''            args=(\n                camera_config,\n                detector_config,\n''',
        '''            args=(\n                camera_config,\n                camera_geometry,\n                detector_config,\n''',
        "process_vision child args",
    )
    return text


def _patch_hardware_runtime(text: str) -> str:
    text = _replace_once(
        text,
        '''                    camera = ProcessVisionPort(\n                        config.camera_device,\n                        config.person_detection_backend,\n                        worker_cpu=(affinity.vision_cpu if affinity.enabled else None),\n''',
        '''                    camera = ProcessVisionPort(\n                        config.camera_device,\n                        config.person_detection_backend,\n                        camera_geometry=config.camera_geometry,\n                        worker_cpu=(affinity.vision_cpu if affinity.enabled else None),\n''',
        "hardware_runtime process vision geometry",
    )
    text = _replace_once(
        text,
        '''                    camera = NativePicamera2Camera(\n                        config.camera_device,\n                        picamera_factory=open_camera,\n''',
        '''                    camera = NativePicamera2Camera(\n                        config.camera_device,\n                        camera_geometry_config=config.camera_geometry,\n                        picamera_factory=open_camera,\n''',
        "hardware_runtime direct camera geometry",
    )
    return text


def _patch_picamera(text: str) -> str:
    text = _replace_once(
        text,
        'from typing import Protocol\n\n\nclass Picamera2Request',
        'from typing import Protocol\n\nfrom .camera_geometry import (\n    CameraGeometryConfig,\n    CameraGeometryStatus,\n    SensorCrop,\n    camera_geometry_status_from_properties,\n    sensor_crop_from_value,\n)\n\n\nclass Picamera2Request',
        "picamera geometry imports",
    )
    text = _replace_once(
        text,
        '''    lens_position: float | None\n    image_bytes: bytes\n\n    def __post_init__(self) -> None:\n''',
        '''    lens_position: float | None\n    image_bytes: bytes\n    sensor_crop: SensorCrop | None = None\n\n    def __post_init__(self) -> None:\n''',
        "picamera frame crop field",
    )
    text = _replace_once(
        text,
        '''        if len(self.image_bytes) != self.frame_size_bytes:\n            raise ValueError("camera payload size does not match configured frame size")\n\n    @property\n''',
        '''        if len(self.image_bytes) != self.frame_size_bytes:\n            raise ValueError("camera payload size does not match configured frame size")\n        if self.sensor_crop is not None and not isinstance(self.sensor_crop, SensorCrop):\n            raise TypeError("sensor_crop must be SensorCrop or None")\n\n    @property\n''',
        "picamera frame crop validation",
    )
    text = _replace_once(
        text,
        '''    __slots__ = (\n        "_config",\n        "_controls_factory",\n''',
        '''    __slots__ = (\n        "_config",\n        "_camera_geometry_config",\n        "_camera_geometry_status",\n        "_default_sensor_crop",\n        "_controls_factory",\n''',
        "picamera geometry slots",
    )
    text = _replace_once(
        text,
        '''        config: Picamera2CameraConfig,\n        *,\n        picamera_factory: Picamera2Factory,\n''',
        '''        config: Picamera2CameraConfig,\n        *,\n        camera_geometry_config: CameraGeometryConfig | None = None,\n        picamera_factory: Picamera2Factory,\n''',
        "picamera constructor geometry arg",
    )
    text = _replace_once(
        text,
        '''        if not isinstance(config, Picamera2CameraConfig):\n            raise TypeError("config must be Picamera2CameraConfig")\n        for callback, name in (\n''',
        '''        if not isinstance(config, Picamera2CameraConfig):\n            raise TypeError("config must be Picamera2CameraConfig")\n        if camera_geometry_config is not None and not isinstance(\n            camera_geometry_config, CameraGeometryConfig\n        ):\n            raise TypeError("camera_geometry_config must be CameraGeometryConfig or None")\n        for callback, name in (\n''',
        "picamera constructor geometry validation",
    )
    text = _replace_once(
        text,
        '''        self._config = config\n        self._factory = picamera_factory\n''',
        '''        self._config = config\n        self._camera_geometry_config = camera_geometry_config\n        self._camera_geometry_status: CameraGeometryStatus | None = None\n        self._default_sensor_crop: SensorCrop | None = None\n        self._factory = picamera_factory\n''',
        "picamera geometry initialization",
    )
    text = _replace_once(
        text,
        '''    @property\n    def config(self) -> Picamera2CameraConfig:\n        return self._config\n\n    def start(self) -> bool:\n''',
        '''    @property\n    def config(self) -> Picamera2CameraConfig:\n        return self._config\n\n    @property\n    def camera_geometry_config(self) -> CameraGeometryConfig | None:\n        return self._camera_geometry_config\n\n    def get_camera_geometry_status(self) -> CameraGeometryStatus | None:\n        with self._lock:\n            return self._camera_geometry_status\n\n    def start(self) -> bool:\n''',
        "picamera geometry status surface",
    )
    text = _replace_once(
        text,
        '''            configuration = build_video_configuration(camera, self._config)\n            camera.configure(configuration)\n            geometry = camera_stream_geometry(camera, self._config.stream_name)\n''',
        '''            configuration = build_video_configuration(camera, self._config)\n            camera.configure(configuration)\n            geometry = camera_stream_geometry(camera, self._config.stream_name)\n            camera_geometry_status = None\n            default_sensor_crop = None\n            if self._camera_geometry_config is not None:\n                camera_geometry_status = camera_geometry_status_from_properties(\n                    self._camera_geometry_config, camera.camera_properties\n                )\n                default_sensor_crop = camera_geometry_status.sensor_crop\n''',
        "picamera runtime geometry validation",
    )
    text = _replace_once(
        text,
        '''            self._picamera = camera\n            self._geometry = geometry\n            self._model = model\n''',
        '''            self._picamera = camera\n            self._geometry = geometry\n            self._model = model\n            self._camera_geometry_status = camera_geometry_status\n            self._default_sensor_crop = default_sensor_crop\n''',
        "picamera geometry status publication",
    )
    text = _replace_once(
        text,
        '''        focus_state = str(metadata.get("AfState", "UNKNOWN")) or "UNKNOWN"\n        with self._lock:\n            previous_sensor_ns = self._last_sensor_timestamp_ns\n''',
        '''        focus_state = str(metadata.get("AfState", "UNKNOWN")) or "UNKNOWN"\n        sensor_crop = sensor_crop_from_value(metadata.get("ScalerCrop"))\n        with self._lock:\n            if sensor_crop is None:\n                sensor_crop = self._default_sensor_crop\n            previous_sensor_ns = self._last_sensor_timestamp_ns\n''',
        "picamera frame crop lineage",
    )
    text = _replace_once(
        text,
        '''            focus_state=focus_state,\n            lens_position=lens_position,\n            image_bytes=image_bytes,\n        )\n''',
        '''            focus_state=focus_state,\n            lens_position=lens_position,\n            image_bytes=image_bytes,\n            sensor_crop=sensor_crop,\n        )\n''',
        "picamera frame crop snapshot",
    )
    return text


def _patch_camera_tool(text: str) -> str:
    return _replace_once(
        text,
        '    config = picamera2_camera_config_from_mapping(camera)\n',
        '    config = picamera2_camera_config_from_mapping(\n        {key: value for key, value in camera.items() if key != "geometry"}\n    )\n',
        "camera tool acquisition split",
    )


def _patch_person_tool(text: str) -> str:
    return _replace_once(
        text,
        '    camera = {key: value for key, value in camera.items() if key != "person_detection"}\n',
        '    camera = {\n        key: value\n        for key, value in camera.items()\n        if key not in {"person_detection", "geometry"}\n    }\n',
        "person tool acquisition split",
    )


PATCHERS = {
    "conf/hardver.json": _patch_hardver,
    "v3/config_hardware.py": _patch_config_hardware,
    "v3/composition/native_sensor_inputs.py": _patch_native_sensor_inputs,
    "v3/adapters/process_vision_port.py": _patch_process_vision,
    "v3_hardware_runtime.py": _patch_hardware_runtime,
    "v3/adapters/picamera2_camera.py": _patch_picamera,
    "tools/v3_camera_test.py": _patch_camera_tool,
    "tools/v3_person_detection_test.py": _patch_person_tool,
}

NEW_FILES = (
    "v3/adapters/camera_geometry.py",
    "tests/feature/test_v3_camera_geometry.py",
)


def _prepare(root: Path, payload: Path) -> dict[str, str]:
    updates: dict[str, str] = {}
    for relative, patcher in PATCHERS.items():
        path = root / relative
        if not path.is_file():
            raise RuntimeError(f"missing target file: {relative}")
        updates[relative] = patcher(path.read_text(encoding="utf-8"))
    for relative in NEW_FILES:
        source = payload / relative
        if not source.is_file():
            raise RuntimeError(f"upgrade payload missing: {relative}")
        target = root / relative
        content = source.read_text(encoding="utf-8")
        if target.exists() and target.read_text(encoding="utf-8") != content:
            raise RuntimeError(f"new target already exists with different content: {relative}")
        updates[relative] = content
    # Sanity check the JSON before touching the repository.
    json.loads(updates["conf/hardver.json"])
    return updates


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default=".", help="R2B4 repository root")
    parser.add_argument("--check", action="store_true", help="validate applicability without writing")
    parser.add_argument("--allow-head-mismatch", action="store_true")
    args = parser.parse_args()

    root = Path(args.repo).expanduser().resolve()
    payload = Path(__file__).resolve().parent / "payload"
    head = _git_head(root)
    if head != EXPECTED_HEAD and not args.allow_head_mismatch:
        raise SystemExit(
            f"REFUSED: expected HEAD {EXPECTED_HEAD}, got {head or 'UNKNOWN'}. "
            "Rebase/regenerate the upgrade or use --allow-head-mismatch only after reviewing the diff."
        )
    updates = _prepare(root, payload)
    print(f"CHECK=PASS upgrade={UPGRADE_ID} head={head}")
    print("FILES=" + ",".join(sorted(updates)))
    if args.check:
        return 0

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"{UPGRADE_ID}_{stamp}"
    backup.mkdir(parents=True, exist_ok=False)
    for relative in PATCHERS:
        src = root / relative
        dst = backup / relative
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)

    try:
        for relative, content in updates.items():
            target = root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(content, encoding="utf-8")
        subprocess.run(
            [
                sys.executable,
                "-m",
                "py_compile",
                str(root / "v3/adapters/camera_geometry.py"),
                str(root / "v3/adapters/picamera2_camera.py"),
                str(root / "v3/adapters/process_vision_port.py"),
                str(root / "v3/config_hardware.py"),
                str(root / "v3/composition/native_sensor_inputs.py"),
                str(root / "v3_hardware_runtime.py"),
            ],
            check=True,
        )
    except BaseException:
        for relative in PATCHERS:
            shutil.copy2(backup / relative, root / relative)
        for relative in NEW_FILES:
            target = root / relative
            if target.exists():
                target.unlink()
        raise

    print(f"APPLY=PASS backup={backup}")
    print("NEXT=./r test")
    print("NEXT=python3 -m pytest -q tests/feature/test_v3_camera_geometry.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
