"""Single owner for the bounded native encoder, IMU and lidar sources."""

from __future__ import annotations

from dataclasses import dataclass

from v3.adapters.bno055_imu import (
    Bno055ImuBackendConfig,
    Bno055SamplePort,
    NativeBno055ImuBackend,
)
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
        device_ids = (
            self.encoder_source.device_id,
            self.imu_source.device_id,
            self.lidar_source.device_id,
        )
        if len(set(device_ids)) != 3:
            raise ValueError("native sensor source device IDs must be unique")
        if self.lidar_source.pose_frame_id != self.lidar_backend.pose_frame_id:
            raise ValueError("lidar source and backend pose frame IDs must match")


class NativeSensorInputOwner:
    """Own all three source lifetimes without owning a clock or runtime loop."""

    __slots__ = (
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
    ) -> None:
        if not isinstance(config, NativeSensorInputConfig):
            raise TypeError("config must be NativeSensorInputConfig")

        imu_backend = NativeBno055ImuBackend(imu_device, config.imu_backend)
        lidar_backend = NativeLatestLidarBackend(lidar_port, config.lidar_backend)
        encoder_source = NativeGpioEncoderSource(
            counter_gpio_backend,
            config.encoder_counter,
            config.encoder_backend,
            config.encoder_source,
        )
        try:
            imu_source = NativeImuSource(imu_backend, config.imu_source)
            lidar_source = NativeLidarSource(lidar_backend, config.lidar_source)
        except Exception:
            for close in (
                lidar_port.stop,
                imu_device.close,
                encoder_source.close,
            ):
                try:
                    close()
                except Exception:
                    pass
            raise

        self._encoder_source = encoder_source
        self._imu_source = imu_source
        self._lidar_source = lidar_source
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
    def sources(
        self,
    ) -> tuple[NativeEncoderSource, NativeImuSource, NativeLidarSource]:
        return (self.encoder_source, self.imu_source, self.lidar_source)

    def close(self) -> None:
        """Release every transferred source capability exactly once."""

        if self._closed:
            return
        self._closed = True
        first_error: Exception | None = None
        for close in (
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


__all__ = ["NativeSensorInputConfig", "NativeSensorInputOwner"]
