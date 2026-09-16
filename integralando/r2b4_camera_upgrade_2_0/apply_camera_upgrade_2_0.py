#!/usr/bin/env python3
"""Apply R2B4 native Camera Module 3 integration upgrade 2.0.

Inspected baseline: Francigofree/r2b4 main
  affec338d60b141a6fcbd1eef2d58a110d656cb6

The patcher prepares every transformed source in memory before writing.  It does
not start the camera, motors, LiDAR or IMU.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
OVERLAY = PACKAGE_ROOT / "overlay"
EXPECTED_BASE = "affec338d60b141a6fcbd1eef2d58a110d656cb6"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def add_import_after(text: str, anchor: str, addition: str, label: str) -> str:
    if addition.strip() in text:
        return text
    return replace_once(text, anchor, anchor + addition, label)


def patch_native_sensor_inputs(text: str) -> str:
    text = add_import_after(
        text,
        "from v3.adapters.live_lidar import NativeLidarConfig, NativeLidarSource\n",
        "from v3.adapters.live_camera import CameraFramePort, NativeCameraConfig, NativeCameraSource\n"
        "from v3.adapters.live_inputs import LiveDeviceSource\n"
        "from v3.adapters.picamera2_camera import Picamera2CameraConfig\n",
        "native sensor camera imports",
    )
    text = replace_once(
        text,
        "    lidar_backend: LatestLidarBackendConfig\n    lidar_source: NativeLidarConfig\n",
        "    lidar_backend: LatestLidarBackendConfig\n    lidar_source: NativeLidarConfig\n"
        "    camera_source: NativeCameraConfig | None = None\n",
        "native sensor config camera field",
    )
    old = '''        device_ids = (\n            self.encoder_source.device_id,\n            self.imu_source.device_id,\n            self.lidar_source.device_id,\n        )\n        if len(set(device_ids)) != 3:\n            raise ValueError("native sensor source device IDs must be unique")\n'''
    new = '''        if self.camera_source is not None and not isinstance(\n            self.camera_source, NativeCameraConfig\n        ):\n            raise TypeError("camera_source must be NativeCameraConfig or None")\n        device_ids = (\n            self.encoder_source.device_id,\n            self.imu_source.device_id,\n            self.lidar_source.device_id,\n        )\n        if self.camera_source is not None:\n            device_ids += (self.camera_source.device_id,)\n        if len(set(device_ids)) != len(device_ids):\n            raise ValueError("native sensor source device IDs must be unique")\n'''
    text = replace_once(text, old, new, "native sensor unique IDs")
    text = replace_once(
        text,
        "    lidar_danger_zone_m: float\n\n    def __post_init__(self) -> None:\n",
        "    lidar_danger_zone_m: float\n"
        "    camera_device: Picamera2CameraConfig | None = None\n\n"
        "    def __post_init__(self) -> None:\n",
        "native hardware camera field",
    )
    anchor = '''        if not isinstance(self.inputs, NativeSensorInputConfig):\n            raise TypeError("inputs must be NativeSensorInputConfig")\n'''
    addition = '''        if self.camera_device is not None and not isinstance(\n            self.camera_device, Picamera2CameraConfig\n        ):\n            raise TypeError("camera_device must be Picamera2CameraConfig or None")\n        if (self.camera_device is None) != (self.inputs.camera_source is None):\n            raise ValueError("camera device and source configs must be enabled together")\n'''
    text = replace_once(text, anchor, anchor + addition, "hardware camera validation")
    text = replace_once(
        text,
        '''        "_closed",\n        "_encoder_source",\n''',
        '''        "_camera_port",\n        "_camera_source",\n        "_closed",\n        "_encoder_source",\n''',
        "owner camera slots",
    )
    text = replace_once(
        text,
        '''        lidar_port: LatestMatcherResultPort,\n        config: NativeSensorInputConfig,\n    ) -> None:\n''',
        '''        lidar_port: LatestMatcherResultPort,\n        config: NativeSensorInputConfig,\n        *,\n        camera_port: CameraFramePort | None = None,\n    ) -> None:\n''',
        "owner camera argument",
    )
    anchor = '''        if not isinstance(config, NativeSensorInputConfig):\n            raise TypeError("config must be NativeSensorInputConfig")\n\n'''
    addition = '''        if (config.camera_source is None) != (camera_port is None):\n            raise ValueError("camera port and source config must be enabled together")\n        if camera_port is not None and not callable(getattr(camera_port, "stop", None)):\n            raise TypeError("camera port owner must provide stop")\n\n'''
    text = replace_once(text, anchor, anchor + addition, "owner camera pairing")
    text = replace_once(
        text,
        '''            imu_source = NativeImuSource(imu_backend, config.imu_source)\n            lidar_source = NativeLidarSource(lidar_backend, config.lidar_source)\n''',
        '''            imu_source = NativeImuSource(imu_backend, config.imu_source)\n            lidar_source = NativeLidarSource(lidar_backend, config.lidar_source)\n            camera_source = (\n                NativeCameraSource(camera_port, config.camera_source)\n                if camera_port is not None and config.camera_source is not None\n                else None\n            )\n''',
        "owner construct camera source",
    )
    text = replace_once(
        text,
        '''            for close in (\n                getattr(lidar_port, "stop", lambda: None),\n''',
        '''            for close in (\n                getattr(camera_port, "stop", lambda: None),\n                getattr(lidar_port, "stop", lambda: None),\n''',
        "owner exception camera close",
    )
    text = replace_once(
        text,
        '''        self._encoder_source = encoder_source\n        self._imu_source = imu_source\n        self._lidar_source = lidar_source\n''',
        '''        self._encoder_source = encoder_source\n        self._imu_source = imu_source\n        self._lidar_source = lidar_source\n        self._camera_source = camera_source\n        self._camera_port = camera_port\n''',
        "owner camera attributes",
    )
    property_anchor = '''    @property\n    def closed(self) -> bool:\n        return self._closed\n\n'''
    property_addition = '''    @property\n    def camera_source(self) -> NativeCameraSource | None:\n        return self._camera_source\n\n    @property\n    def camera_frame_port(self) -> CameraFramePort | None:\n        return self._camera_port\n\n    @property\n    def auxiliary_sources(self) -> tuple[LiveDeviceSource, ...]:\n        return (self._camera_source,) if self._camera_source is not None else ()\n\n'''
    text = replace_once(text, property_anchor, property_anchor + property_addition, "owner camera properties")
    text = replace_once(
        text,
        '''        for close in (\n            self._lidar_port.stop,\n''',
        '''        for close in (\n            getattr(self._camera_port, "stop", lambda: None),\n            self._lidar_port.stop,\n''',
        "owner normal camera close",
    )
    text = text.replace("Single owner for the bounded native encoder, IMU and lidar sources.", "Single owner for native core sources plus optional auxiliary camera input.")
    text = text.replace("Own all three source lifetimes without owning a clock or runtime loop.", "Own core source lifetimes and optional auxiliary camera without a runtime loop.")
    return text


def patch_live_composition(text: str) -> str:
    text = replace_once(
        text,
        "from v3.adapters.live_inputs import NativeLiveInputReader\n",
        "from v3.adapters.live_inputs import LiveDeviceSource, NativeLiveInputReader\n",
        "live composition source protocol import",
    )
    text = replace_once(
        text,
        '''        lidar_source: NativeLidarSource,\n        config: LiveInputCompositionConfig = LiveInputCompositionConfig(),\n    ) -> None:\n''',
        '''        lidar_source: NativeLidarSource,\n        config: LiveInputCompositionConfig = LiveInputCompositionConfig(),\n        *,\n        auxiliary_sources: tuple[LiveDeviceSource, ...] = (),\n    ) -> None:\n''',
        "live composition auxiliary argument",
    )
    validation_anchor = '''        if not isinstance(config, LiveInputCompositionConfig):\n            raise TypeError("config must be LiveInputCompositionConfig")\n\n'''
    text = replace_once(
        text,
        validation_anchor,
        validation_anchor + '''        if not isinstance(auxiliary_sources, tuple):\n            raise TypeError("auxiliary_sources must be tuple[LiveDeviceSource, ...]")\n\n''',
        "live composition auxiliary validation",
    )
    text = replace_once(
        text,
        '''        self._reader = NativeLiveInputReader(\n            (encoder_source, imu_source, lidar_source)\n        )\n''',
        '''        self._reader = NativeLiveInputReader(\n            (encoder_source, imu_source, lidar_source, *auxiliary_sources)\n        )\n''',
        "live composition reader",
    )
    text = text.replace("Poll the three native sources once", "Poll native core and auxiliary sources once")
    return text


def patch_live_control(text: str, resident: bool) -> str:
    text = replace_once(
        text,
        "from v3.adapters.live_inputs import NativeLiveInputReader\n",
        "from v3.adapters.live_inputs import LiveDeviceSource, NativeLiveInputReader\n",
        "live control source protocol import",
    )
    if resident:
        old = '''        motor_writer: object,\n        config: ResidentLiveControlConfig,\n    ) -> None:\n'''
        new = '''        motor_writer: object,\n        config: ResidentLiveControlConfig,\n        *,\n        auxiliary_sources: tuple[LiveDeviceSource, ...] = (),\n    ) -> None:\n'''
        validation = '''        if not isinstance(config, ResidentLiveControlConfig):\n            raise TypeError("config must be ResidentLiveControlConfig")\n\n'''
    else:
        old = '''        motor_writer: object,\n        config: BoundedLiveControlConfig,\n    ) -> None:\n'''
        new = '''        motor_writer: object,\n        config: BoundedLiveControlConfig,\n        *,\n        auxiliary_sources: tuple[LiveDeviceSource, ...] = (),\n    ) -> None:\n'''
        validation = '''        if not isinstance(config, BoundedLiveControlConfig):\n            raise TypeError("config must be BoundedLiveControlConfig")\n\n'''
    text = replace_once(text, old, new, "live control auxiliary argument")
    text = replace_once(
        text,
        validation,
        validation + '''        if not isinstance(auxiliary_sources, tuple):\n            raise TypeError("auxiliary_sources must be tuple[LiveDeviceSource, ...]")\n\n''',
        "live control auxiliary validation",
    )
    text = replace_once(
        text,
        '''        self._reader = NativeLiveInputReader(\n            (encoder_source, imu_source, lidar_source)\n        )\n''',
        '''        self._reader = NativeLiveInputReader(\n            (encoder_source, imu_source, lidar_source, *auxiliary_sources)\n        )\n''',
        "live control reader",
    )
    return text


def patch_physical_control(text: str, resident: bool) -> str:
    text = add_import_after(
        text,
        "from v3.adapters.live_lidar import NativeLidarSource\n",
        "from v3.adapters.live_inputs import LiveDeviceSource\n",
        "physical auxiliary source import",
    )
    if resident:
        old = '''        gpio_backend: PwmGpioBackend,\n        config: ResidentPhysicalControlConfig,\n    ) -> None:\n'''
        new = '''        gpio_backend: PwmGpioBackend,\n        config: ResidentPhysicalControlConfig,\n        *,\n        auxiliary_sources: tuple[LiveDeviceSource, ...] = (),\n    ) -> None:\n'''
        call = '''                motor_output,\n                config.live_control,\n            )\n'''
        call_new = '''                motor_output,\n                config.live_control,\n                auxiliary_sources=auxiliary_sources,\n            )\n'''
    else:
        old = '''        gpio_backend: PwmGpioBackend,\n        config: BoundedPhysicalControlConfig,\n    ) -> None:\n'''
        new = '''        gpio_backend: PwmGpioBackend,\n        config: BoundedPhysicalControlConfig,\n        *,\n        auxiliary_sources: tuple[LiveDeviceSource, ...] = (),\n    ) -> None:\n'''
        call = '''                motor_output,\n                config.live_control,\n            )\n'''
        call_new = '''                motor_output,\n                config.live_control,\n                auxiliary_sources=auxiliary_sources,\n            )\n'''
    text = replace_once(text, old, new, "physical control auxiliary argument")
    text = replace_once(text, call, call_new, "physical control auxiliary pass-through")
    return text


def patch_bounded_runtime(text: str) -> str:
    text = add_import_after(
        text,
        "from v3.adapters.live_lidar import NativeLidarSource\n",
        "from v3.adapters.live_inputs import LiveDeviceSource\n",
        "bounded runtime auxiliary import",
    )
    text = replace_once(
        text,
        '''    config: BoundedPhysicalRuntimeConfig,\n    *,\n    stop_requested: Callable[[], bool],\n''',
        '''    config: BoundedPhysicalRuntimeConfig,\n    *,\n    auxiliary_sources: tuple[LiveDeviceSource, ...] = (),\n    stop_requested: Callable[[], bool],\n''',
        "bounded runtime auxiliary argument",
    )
    text = replace_once(
        text,
        '''        gpio_backend,\n        config.composition,\n    )\n''',
        '''        gpio_backend,\n        config.composition,\n        auxiliary_sources=auxiliary_sources,\n    )\n''',
        "bounded runtime physical pass-through",
    )
    owned_call = '''            gpio_backend,\n            config,\n            stop_requested=stop_requested,\n'''
    text = replace_once(
        text,
        owned_call,
        '''            gpio_backend,\n            config,\n            auxiliary_sources=sensor_inputs.auxiliary_sources,\n            stop_requested=stop_requested,\n''',
        "bounded owned auxiliary pass-through",
    )
    return text


def patch_resident_runtime(text: str) -> str:
    text = add_import_after(
        text,
        "from v3.adapters.live_lidar import NativeLidarSource\n",
        "from v3.adapters.live_inputs import LiveDeviceSource\n",
        "resident runtime auxiliary import",
    )
    text = replace_once(
        text,
        '''    config: ResidentPhysicalRuntimeConfig,\n    *,\n    stop_requested: Callable[[], bool],\n''',
        '''    config: ResidentPhysicalRuntimeConfig,\n    *,\n    auxiliary_sources: tuple[LiveDeviceSource, ...] = (),\n    stop_requested: Callable[[], bool],\n''',
        "resident runtime auxiliary argument",
    )
    text = replace_once(
        text,
        '''        gpio_backend,\n        config.composition,\n    )\n''',
        '''        gpio_backend,\n        config.composition,\n        auxiliary_sources=auxiliary_sources,\n    )\n''',
        "resident physical auxiliary pass-through",
    )
    text = replace_once(
        text,
        '''            gpio_backend,\n            config,\n            stop_requested=stop_requested,\n''',
        '''            gpio_backend,\n            config,\n            auxiliary_sources=sensor_inputs.auxiliary_sources,\n            stop_requested=stop_requested,\n''',
        "resident owned auxiliary pass-through",
    )
    return text


def patch_hardware_runtime(text: str) -> str:
    text = add_import_after(
        text,
        "from v3.adapters.native_lidar_port import TimedPoseReference\n",
        "from v3.adapters.picamera2_camera import (\n"
        "    NativePicamera2Camera,\n"
        "    Picamera2Factory,\n"
        "    default_picamera2_factory,\n"
        "    raspberry_pi_sensor_timestamp_to_monotonic_ns,\n"
        ")\n",
        "hardware runtime camera imports",
    )
    text = add_import_after(
        text,
        "from v3.ports import CommandGateway\n",
        "from v3.device_health_policy import (\n"
        "    PRODUCTION_CRITICAL_DEVICE_IDS,\n"
        "    critical_devices_ready,\n"
        ")\n",
        "hardware runtime critical policy import",
    )
    old_health = '''                and all(\n                    item.state is DeviceHealthState.OK\n                    for item in acquisition.io_health\n                )\n'''
    text = replace_once(
        text,
        old_health,
        '''                and critical_devices_ready(\n                    acquisition.io_health,\n                    PRODUCTION_CRITICAL_DEVICE_IDS,\n                )\n''',
        "sensor measurement critical health semantics",
    )
    text = replace_once(
        text,
        '''        config: NativeSensorHardwareConfig,\n        *,\n        monotonic_ns: Callable[[], int] = time.monotonic_ns,\n''',
        '''        config: NativeSensorHardwareConfig,\n        *,\n        open_camera: Picamera2Factory = default_picamera2_factory,\n        monotonic_ns: Callable[[], int] = time.monotonic_ns,\n''',
        "hardware owner camera factory argument",
    )
    text = replace_once(
        text,
        '''            (open_imu_bus, "open_imu_bus"),\n            (open_lidar_port, "open_lidar_port"),\n            (monotonic_ns, "monotonic_ns"),\n            (sleep, "sleep"),\n''',
        '''            (open_imu_bus, "open_imu_bus"),\n            (open_lidar_port, "open_lidar_port"),\n            (open_camera, "open_camera"),\n            (monotonic_ns, "monotonic_ns"),\n            (sleep, "sleep"),\n''',
        "hardware owner camera factory validation",
    )
    text = replace_once(
        text,
        '''        lidar: LatestMatcherResultPort | None = None\n        inputs: NativeSensorInputOwner | None = None\n''',
        '''        lidar: LatestMatcherResultPort | None = None\n        camera: NativePicamera2Camera | None = None\n        inputs: NativeSensorInputOwner | None = None\n''',
        "hardware owner camera local",
    )
    text = replace_once(
        text,
        '''            lidar = open_lidar_port(pose_feedback)\n            inputs = NativeSensorInputOwner(\n                counter_gpio_backend,\n                imu,\n                lidar,\n                config.inputs,\n            )\n''',
        '''            lidar = open_lidar_port(pose_feedback)\n            if config.camera_device is not None:\n                camera = NativePicamera2Camera(\n                    config.camera_device,\n                    picamera_factory=open_camera,\n                    sensor_timestamp_mapper=(\n                        raspberry_pi_sensor_timestamp_to_monotonic_ns\n                    ),\n                    monotonic_ns=monotonic_ns,\n                )\n                # Camera is explicitly non-critical. A start failure remains\n                # visible as CAMERA_FRONT FAILED but must not abort core input\n                # ownership or the motor-control runtime.\n                camera.start()\n            inputs = NativeSensorInputOwner(\n                counter_gpio_backend,\n                imu,\n                lidar,\n                config.inputs,\n                camera_port=camera,\n            )\n''',
        "hardware owner camera creation",
    )
    text = replace_once(
        text,
        '''            else:\n                if lidar is not None:\n''',
        '''            else:\n                if camera is not None:\n                    try:\n                        camera.stop()\n                    except Exception:\n                        pass\n                if lidar is not None:\n''',
        "hardware owner camera exception close",
    )
    text = replace_once(
        text,
        '''        runtime = LiveInputComposition(*owner.inputs.sources, config.live_inputs)\n''',
        '''        runtime = LiveInputComposition(\n            *owner.inputs.sources,\n            config.live_inputs,\n            auxiliary_sources=owner.inputs.auxiliary_sources,\n        )\n''',
        "measurement auxiliary input wiring",
    )
    text = text.replace("Acquire and release the bus, matcher port and three typed V3 sources.", "Acquire/release core sensors plus the optional native camera capability.")
    return text


def patch_bounded_config(text: str) -> str:
    text = add_import_after(
        text,
        "from v3.adapters.live_lidar import NativeLidarConfig\n",
        "from v3.adapters.live_camera import NativeCameraConfig\n"
        "from v3.adapters.picamera2_camera import (\n"
        "    Picamera2CameraConfig,\n"
        "    picamera2_camera_config_from_mapping,\n"
        ")\n",
        "config camera imports",
    )
    text = replace_once(
        text,
        '''    encoder_maximum_estimation_window_ns: int = 160_000_000\n''',
        '''    encoder_maximum_estimation_window_ns: int = 160_000_000\n    camera_maximum_frame_age_ns: int = 250_000_000\n''',
        "camera policy freshness field",
    )
    validate_anchor = '''        NativeLidarConfig(\n            "validation-lidar",\n            self.lidar_minimum_confidence,\n            self.lidar_maximum_measurement_age_ns,\n            POSE_FRAME_ID,\n        )\n'''
    text = replace_once(
        text,
        validate_anchor,
        validate_anchor + '''        NativeCameraConfig(\n            "CAMERA_FRONT",\n            self.camera_maximum_frame_age_ns,\n        )\n''',
        "camera policy validation",
    )
    inputs_anchor = '''    inputs = NativeSensorInputConfig(\n'''
    camera_parse = '''    camera_device: Picamera2CameraConfig | None = None\n    camera_source: NativeCameraConfig | None = None\n    camera_value = hardware.get("camera")\n    if camera_value is not None:\n        camera = _mapping(camera_value, "hardware config camera")\n        enabled = camera.get("enabled", False)\n        if type(enabled) is not bool:\n            raise ValueError("hardware config camera.enabled must be bool")\n        if enabled:\n            if camera.get("provider", "picamera2") != "picamera2":\n                raise ValueError("hardware config camera.provider must be picamera2")\n            camera_device = picamera2_camera_config_from_mapping(camera)\n            camera_source = NativeCameraConfig(\n                "CAMERA_FRONT",\n                policy.camera_maximum_frame_age_ns,\n            )\n\n'''
    text = replace_once(text, inputs_anchor, camera_parse + inputs_anchor, "camera hardware parsing")
    text = replace_once(
        text,
        '''        lidar_source=NativeLidarConfig(\n            "RPLIDAR_C1",\n''',
        '''        camera_source=camera_source,\n        lidar_source=NativeLidarConfig(\n            "RPLIDAR_C1",\n''',
        "camera source config wiring",
    )
    old_return = '''    return NativeSensorHardwareConfig(\n        imu_device,\n        inputs,\n        _positive_float(\n            lidar.get("biztonsagi_zona_m"),\n            "hardware config lidar.biztonsagi_zona_m",\n        ),\n    )\n'''
    new_return = '''    return NativeSensorHardwareConfig(\n        imu_device=imu_device,\n        inputs=inputs,\n        lidar_danger_zone_m=_positive_float(\n            lidar.get("biztonsagi_zona_m"),\n            "hardware config lidar.biztonsagi_zona_m",\n        ),\n        camera_device=camera_device,\n    )\n'''
    text = replace_once(text, old_return, new_return, "camera hardware config return")
    return text


def patch_hardware_json(text: str) -> str:
    data = json.loads(text)
    if not isinstance(data, dict):
        raise RuntimeError("conf/hardver.json must contain an object")
    if "camera" not in data:
        data["camera"] = {
            "enabled": True,
            "provider": "picamera2",
            "camera_index": 0,
            "expected_model": "imx708",
            "stream_name": "lores",
            "width": 640,
            "height": 360,
            "pixel_format": "RGB888",
            "main_width": 1280,
            "main_height": 720,
            "main_pixel_format": "YUV420",
            "fps": 20.0,
            "buffer_count": 6,
            "queue": False,
            "continuous_autofocus": True,
            "max_frame_completion_lag_ns": 500000000,
            "stop_join_timeout_s": 2.0,
        }
    return json.dumps(data, ensure_ascii=False, indent=2) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default=".")
    parser.add_argument("--check", action="store_true", help="verify all structural anchors without writing")
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    if not (repo / ".git").exists() or not (repo / "v3").is_dir():
        raise SystemExit(f"not an R2B4 checkout: {repo}")

    try:
        head = subprocess.check_output(
            ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True
        ).strip()
    except Exception:
        head = "UNKNOWN"
    if head != EXPECTED_BASE:
        print(
            f"NOTE: HEAD is {head[:12]}, inspected baseline was {EXPECTED_BASE[:12]}; "
            "structural anchors will decide compatibility."
        )

    patchers = {
        "v3/composition/native_sensor_inputs.py": patch_native_sensor_inputs,
        "v3/composition/live_inputs.py": patch_live_composition,
        "v3/composition/bounded_live_control.py": lambda t: patch_live_control(t, False),
        "v3/composition/resident_live_control.py": lambda t: patch_live_control(t, True),
        "v3/composition/bounded_physical_control.py": lambda t: patch_physical_control(t, False),
        "v3/composition/resident_physical_control.py": lambda t: patch_physical_control(t, True),
        "v3_bounded_runtime.py": patch_bounded_runtime,
        "v3_runtime.py": patch_resident_runtime,
        "v3_hardware_runtime.py": patch_hardware_runtime,
        "v3_bounded_config.py": patch_bounded_config,
        "conf/hardver.json": patch_hardware_json,
    }

    prepared: dict[Path, str] = {}
    for relative, patcher in patchers.items():
        path = repo / relative
        if not path.is_file():
            raise RuntimeError(f"required source missing: {relative}")
        prepared[path] = patcher(path.read_text(encoding="utf-8"))

    overlays = {
        "v3/adapters/picamera2_camera.py": "v3/adapters/picamera2_camera.py",
        "v3/adapters/live_camera.py": "v3/adapters/live_camera.py",
        "v3/adapters/camera_media.py": "v3/adapters/camera_media.py",
        "tools/v3_camera_test.py": "tools/v3_camera_test.py",
        "tests/test_v3_camera_foundation.py": "tests/test_v3_camera_foundation.py",
    }
    for source_rel, dest_rel in overlays.items():
        source = OVERLAY / source_rel
        if not source.is_file():
            raise RuntimeError(f"package overlay missing: {source_rel}")
        prepared[repo / dest_rel] = source.read_text(encoding="utf-8")

    if args.check:
        print("camera upgrade 2.0 structural compatibility: PASS")
        print(f"prepared {len(prepared)} files; nothing written")
        return 0

    for path, content in prepared.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    try:
        (repo / "tools/v3_camera_test.py").chmod(0o755)
    except OSError:
        pass

    print("R2B4 native camera integration upgrade 2.0 applied")
    print("No camera, motor, LiDAR or IMU was started.")
    print("Run validation_camera_upgrade_2_0.sh from this package.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
