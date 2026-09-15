"""Native V3 L0 camera-health source over an immutable latest camera frame.

The raw image never enters DeviceSample. This first integration slice exposes
only bounded frame metadata/health; semantic detections are intentionally left
for the next camera-perception slice.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from v3.adapters.live_inputs import LiveDeviceSnapshot
from v3.adapters.picamera2_camera import CameraFrameSnapshot, CameraRuntimeStatus
from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    TickContext,
)


class CameraFramePort(Protocol):
    def get_latest_frame(self) -> CameraFrameSnapshot | None: ...

    def get_runtime_status(self) -> CameraRuntimeStatus: ...


@dataclass(frozen=True, slots=True)
class NativeCameraConfig:
    device_id: str = "CAMERA_FRONT"
    maximum_frame_age_ns: int = 250_000_000

    def __post_init__(self) -> None:
        if not isinstance(self.device_id, str) or not self.device_id.strip():
            raise ValueError("device_id must be a non-empty string")
        if (
            not isinstance(self.maximum_frame_age_ns, int)
            or isinstance(self.maximum_frame_age_ns, bool)
            or self.maximum_frame_age_ns <= 0
        ):
            raise ValueError("maximum_frame_age_ns must be a positive integer")


class NativeCameraSource:
    """Close latest camera metadata into one bounded L0 source snapshot."""

    __slots__ = ("_config", "_port")

    def __init__(self, port: CameraFramePort, config: NativeCameraConfig) -> None:
        if not callable(getattr(port, "get_latest_frame", None)):
            raise TypeError("port must provide get_latest_frame")
        if not callable(getattr(port, "get_runtime_status", None)):
            raise TypeError("port must provide get_runtime_status")
        if not isinstance(config, NativeCameraConfig):
            raise TypeError("config must be NativeCameraConfig")
        self._port = port
        self._config = config

    @property
    def device_id(self) -> str:
        return self._config.device_id

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        status = self._port.get_runtime_status()
        if not isinstance(status, CameraRuntimeStatus):
            raise TypeError("camera runtime status must be CameraRuntimeStatus")
        frame = self._port.get_latest_frame()

        if not status.running:
            reason = "CAMERA_RUNTIME_ERROR" if status.last_error else "CAMERA_NOT_RUNNING"
            return LiveDeviceSnapshot(
                context,
                DeviceHealth(self.device_id, DeviceHealthState.FAILED, reason),
            )
        if status.last_error:
            return LiveDeviceSnapshot(
                context,
                DeviceHealth(self.device_id, DeviceHealthState.FAILED, "CAMERA_RUNTIME_ERROR"),
            )
        if frame is None:
            return LiveDeviceSnapshot(
                context,
                DeviceHealth(self.device_id, DeviceHealthState.UNKNOWN, "CAMERA_NO_FRAME"),
            )
        if not isinstance(frame, CameraFrameSnapshot):
            raise TypeError("camera port must return CameraFrameSnapshot or None")
        if frame.measurement_monotonic_ns > context.monotonic_ns:
            return LiveDeviceSnapshot(
                context,
                DeviceHealth(self.device_id, DeviceHealthState.FAILED, "CAMERA_TIME_INVALID"),
            )

        age_ns = context.monotonic_ns - frame.measurement_monotonic_ns
        state = DeviceHealthState.OK
        reason = None
        if age_ns > self._config.maximum_frame_age_ns:
            state = DeviceHealthState.DEGRADED
            reason = "CAMERA_FRAME_STALE"

        sample = DeviceSample(
            device_id=self.device_id,
            kind="camera_frame_health",
            sequence=frame.sequence,
            captured_monotonic_ns=frame.measurement_monotonic_ns,
            values=(
                DataField("age_ns", age_ns),
                DataField("width", frame.width),
                DataField("height", frame.height),
                DataField("pixel_format", frame.pixel_format),
                DataField("exposure_time_ns", frame.exposure_time_ns),
                DataField("frame_duration_ns", frame.frame_duration_ns),
                DataField("focus_state", frame.focus_state),
                DataField("payload_bytes", len(frame.image_bytes)),
            ),
        )
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, state, reason),
            (sample,),
        )


__all__ = ["CameraFramePort", "NativeCameraConfig", "NativeCameraSource"]
