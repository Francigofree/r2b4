"""Single owner for native core sources plus optional auxiliary camera input."""

from __future__ import annotations

from dataclasses import dataclass
import math

from v3.adapters.bno055_imu import (
    Bno055ImuBackendConfig,
    Bno055SamplePort,
    NativeBno055ImuBackend,
)
from v3.adapters.bno055_device import NativeBno055DeviceConfig
from v3.adapters.counter_encoder import CounterEncoderBackendConfig
from v3.adapters.gpio_counter import GpioCounterBackend, GpioCounterPairConfig
from v3.adapters.gpio_encoder import NativeGpioEncoderSource
from v3.adapters.latest_lidar import (
    LatestLidarBackendConfig,
    LatestMatcherResultPort,
    NativeLatestLidarBackend,
)
from v3.adapters.live_encoder import NativeEncoderConfig, NativeEncoderSource
from v3.adapters.live_imu import NativeImuConfig, NativeImuSource
from v3.adapters.live_lidar import NativeLidarConfig, NativeLidarSource
from v3.adapters.live_camera import CameraFramePort, NativeCameraConfig, NativeCameraSource
from v3.adapters.live_inputs import LiveDeviceSource
from v3.adapters.live_person_detection import (
    NativePersonDetectionConfig,
    NativePersonDetectionSource,
)
from v3.adapters.litert_person_detector import LiteRtPersonDetectorConfig
from v3.adapters.person_detection import PersonDetectionPort
from v3.adapters.person_photo_evidence import PersonPhotoEvidenceConfig
from v3.adapters.picamera2_camera import Picamera2CameraConfig


@dataclass(frozen=True, slots=True)
class NativeSensorInputConfig:
    """All explicit policy needed to bind the three native source ports."""

    encoder_counter: GpioCounterPairConfig
    encoder_backend: CounterEncoderBackendConfig
    encoder_source: NativeEncoderConfig
    imu_backend: Bno055ImuBackendConfig
    imu_source: NativeImuConfig
    lidar_backend: LatestLidarBackendConfig
    lidar_source: NativeLidarConfig
    camera_source: NativeCameraConfig | None = None
    person_detection_source: NativePersonDetectionConfig | None = None

    def __post_init__(self) -> None:
        expected_types = (
            (self.encoder_counter, GpioCounterPairConfig, "encoder_counter"),
            (self.encoder_backend, CounterEncoderBackendConfig, "encoder_backend"),
            (self.encoder_source, NativeEncoderConfig, "encoder_source"),
            (self.imu_backend, Bno055ImuBackendConfig, "imu_backend"),
            (self.imu_source, NativeImuConfig, "imu_source"),
            (self.lidar_backend, LatestLidarBackendConfig, "lidar_backend"),
            (self.lidar_source, NativeLidarConfig, "lidar_source"),
        )
        for value, expected_type, name in expected_types:
            if not isinstance(value, expected_type):
                raise TypeError(f"{name} must be {expected_type.__name__}")
        if self.camera_source is not None and not isinstance(
            self.camera_source, NativeCameraConfig
        ):
            raise TypeError("camera_source must be NativeCameraConfig or None")
        if self.person_detection_source is not None and not isinstance(
            self.person_detection_source, NativePersonDetectionConfig
        ):
            raise TypeError(
                "person_detection_source must be NativePersonDetectionConfig or None"
            )
        device_ids = (
            self.encoder_source.device_id,
            self.imu_source.device_id,
            self.lidar_source.device_id,
        )
        if self.camera_source is not None:
            device_ids += (self.camera_source.device_id,)
        if self.person_detection_source is not None:
            device_ids += (self.person_detection_source.device_id,)
        if len(set(device_ids)) != len(device_ids):
            raise ValueError("native sensor source device IDs must be unique")
        if self.lidar_source.pose_frame_id != self.lidar_backend.pose_frame_id:
            raise ValueError("lidar source and backend pose frame IDs must match")
        if (
            self.imu_source.allow_rate_only
            and self.lidar_source.minimum_confidence <= 0.0
        ):
            raise ValueError(
                "rate-only IMU health requires positive LIDAR_FIRST confidence"
            )


@dataclass(frozen=True, slots=True)
class NativeSensorHardwareConfig:
    """Closed native device and typed-source configuration for one owner."""

    imu_device: NativeBno055DeviceConfig
    inputs: NativeSensorInputConfig
    lidar_danger_zone_m: float
    camera_device: Picamera2CameraConfig | None = None
    person_detection_backend: LiteRtPersonDetectorConfig | None = None
    person_photo_evidence: PersonPhotoEvidenceConfig | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.imu_device, NativeBno055DeviceConfig):
            raise TypeError("imu_device must be NativeBno055DeviceConfig")
        if not isinstance(self.inputs, NativeSensorInputConfig):
            raise TypeError("inputs must be NativeSensorInputConfig")
        if self.camera_device is not None and not isinstance(
            self.camera_device, Picamera2CameraConfig
        ):
            raise TypeError("camera_device must be Picamera2CameraConfig or None")
        if (self.camera_device is None) != (self.inputs.camera_source is None):
            raise ValueError("camera device and source configs must be enabled together")
        if self.person_detection_backend is not None and not isinstance(
            self.person_detection_backend, LiteRtPersonDetectorConfig
        ):
            raise TypeError(
                "person_detection_backend must be LiteRtPersonDetectorConfig or None"
            )
        if self.person_photo_evidence is not None and not isinstance(
            self.person_photo_evidence, PersonPhotoEvidenceConfig
        ):
            raise TypeError(
                "person_photo_evidence must be PersonPhotoEvidenceConfig or None"
            )
        if (self.person_detection_backend is None) != (
            self.inputs.person_detection_source is None
        ):
            raise ValueError(
                "person detector backend and source configs must be enabled together"
            )
        if self.person_detection_backend is not None and self.camera_device is None:
            raise ValueError("person detection requires the native camera capability")
        if self.person_photo_evidence is not None and self.person_detection_backend is None:
            raise ValueError("person photo evidence requires person detection")
        if (
            isinstance(self.lidar_danger_zone_m, bool)
            or not isinstance(self.lidar_danger_zone_m, (int, float))
            or not math.isfinite(self.lidar_danger_zone_m)
            or self.lidar_danger_zone_m <= 0.0
        ):
            raise ValueError("lidar_danger_zone_m must be finite and positive")


class NativeSensorInputOwner:
    """Own core source lifetimes and optional auxiliary camera without a runtime loop."""

    __slots__ = (
        "_camera_port",
        "_camera_source",
        "_person_detection_port",
        "_person_detection_source",
        "_closed",
        "_encoder_source",
        "_imu_device",
        "_imu_source",
        "_lidar_port",
        "_lidar_source",
    )

    def __init__(
        self,
        counter_gpio_backend: GpioCounterBackend,
        imu_device: Bno055SamplePort,
        lidar_port: LatestMatcherResultPort,
        config: NativeSensorInputConfig,
        *,
        camera_port: CameraFramePort | None = None,
        person_detection_port: PersonDetectionPort | None = None,
    ) -> None:
        if not isinstance(config, NativeSensorInputConfig):
            raise TypeError("config must be NativeSensorInputConfig")

        if (config.camera_source is None) != (camera_port is None):
            raise ValueError("camera port and source config must be enabled together")
        if camera_port is not None and not callable(getattr(camera_port, "stop", None)):
            raise TypeError("camera port owner must provide stop")
        if (config.person_detection_source is None) != (person_detection_port is None):
            raise ValueError(
                "person detection port and source config must be enabled together"
            )
        if person_detection_port is not None and not callable(
            getattr(person_detection_port, "stop", None)
        ):
            raise TypeError("person detection port owner must provide stop")

        try:
            imu_backend = NativeBno055ImuBackend(imu_device, config.imu_backend)
            lidar_backend = NativeLatestLidarBackend(lidar_port, config.lidar_backend)
            encoder_source = NativeGpioEncoderSource(
                counter_gpio_backend,
                config.encoder_counter,
                config.encoder_backend,
                config.encoder_source,
            )
            imu_source = NativeImuSource(imu_backend, config.imu_source)
            lidar_source = NativeLidarSource(lidar_backend, config.lidar_source)
            camera_source = (
                NativeCameraSource(camera_port, config.camera_source)
                if camera_port is not None and config.camera_source is not None
                else None
            )
            person_detection_source = (
                NativePersonDetectionSource(
                    person_detection_port, config.person_detection_source
                )
                if person_detection_port is not None
                and config.person_detection_source is not None
                else None
            )
        except Exception:
            for close in (
                getattr(person_detection_port, "stop", lambda: None),
                getattr(camera_port, "stop", lambda: None),
                getattr(lidar_port, "stop", lambda: None),
                getattr(imu_device, "close", lambda: None),
                getattr(locals().get("encoder_source"), "close", lambda: None),
            ):
                try:
                    close()
                except Exception:
                    pass
            raise

        self._encoder_source = encoder_source
        self._imu_source = imu_source
        self._lidar_source = lidar_source
        self._camera_source = camera_source
        self._camera_port = camera_port
        self._person_detection_source = person_detection_source
        self._person_detection_port = person_detection_port
        self._imu_device = imu_device
        self._lidar_port = lidar_port
        self._closed = False

    @property
    def encoder_source(self) -> NativeEncoderSource:
        return self._encoder_source

    @property
    def imu_source(self) -> NativeImuSource:
        return self._imu_source

    @property
    def lidar_source(self) -> NativeLidarSource:
        return self._lidar_source

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def camera_source(self) -> NativeCameraSource | None:
        return self._camera_source

    @property
    def camera_frame_port(self) -> CameraFramePort | None:
        return self._camera_port

    @property
    def person_detection_source(self) -> NativePersonDetectionSource | None:
        return self._person_detection_source

    @property
    def person_detection_port(self) -> PersonDetectionPort | None:
        return self._person_detection_port

    @property
    def auxiliary_sources(self) -> tuple[LiveDeviceSource, ...]:
        return tuple(
            source
            for source in (self._camera_source, self._person_detection_source)
            if source is not None
        )

    @property
    def sources(
        self,
    ) -> tuple[NativeEncoderSource, NativeImuSource, NativeLidarSource]:
        return (self.encoder_source, self.imu_source, self.lidar_source)

    def raw_lidar_snapshot(self) -> object | None:
        """Return the latest immutable raw revision for passive capture only."""

        if self._closed:
            return None
        getter = getattr(self._lidar_port, "get_raw_scan_snapshot", None)
        return getter() if callable(getter) else None

    def close(self) -> None:
        """Release every transferred source capability exactly once."""

        if self._closed:
            return
        self._closed = True
        first_error: Exception | None = None
        for close in (
            getattr(self._person_detection_port, "stop", lambda: None),
            getattr(self._camera_port, "stop", lambda: None),
            self._lidar_port.stop,
            self._imu_device.close,
            self._encoder_source.close,
        ):
            try:
                close()
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error


__all__ = [
    "NativeSensorHardwareConfig",
    "NativeSensorInputConfig",
    "NativeSensorInputOwner",
]
