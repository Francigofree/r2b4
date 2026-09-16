"""Native V3 camera-health source over a coherent latest camera edge snapshot.

The raw frame remains on the camera frame port and never enters DeviceSample.
This source contributes only bounded metadata, lineage and health to L0/L1/L2.
Camera failures are deliberately isolated into CAMERA_FRONT health because the
camera is a non-critical capability in the current production safety policy.
"""

from __future__ import annotations

from dataclasses import dataclass
from v3.adapters.live_inputs import LiveDeviceSnapshot
from v3.adapters.picamera2_camera import CameraEdgeSnapshot, CameraFramePort
from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    TickContext,
)


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
    """Close camera health/metadata into one bounded non-critical L0 source."""

    __slots__ = ("_config", "_port")

    def __init__(self, port: CameraFramePort, config: NativeCameraConfig) -> None:
        if not callable(getattr(port, "get_edge_snapshot", None)):
            raise TypeError("port must provide get_edge_snapshot")
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
        try:
            edge = self._port.get_edge_snapshot()
        except Exception:
            # A non-critical camera adapter must never turn a camera/driver
            # exception into a whole-robot L0 fault.  The capability itself is
            # marked failed and missions that require it can reject it later.
            return self._failed(context, "CAMERA_PORT_ERROR")
        if not isinstance(edge, CameraEdgeSnapshot):
            return self._failed(context, "CAMERA_PORT_INVALID")

        status = edge.status
        frame = edge.frame
        if not status.running:
            return self._failed(
                context,
                "CAMERA_RUNTIME_ERROR" if status.last_error else "CAMERA_NOT_RUNNING",
            )
        if status.last_error:
            return self._failed(context, "CAMERA_RUNTIME_ERROR")
        if frame is None:
            return LiveDeviceSnapshot(
                context,
                DeviceHealth(self.device_id, DeviceHealthState.UNKNOWN, "CAMERA_NO_FRAME"),
            )
        if status.frame_sequence != frame.sequence:
            return self._failed(context, "CAMERA_LINEAGE_INVALID")
        if frame.measurement_monotonic_ns > context.monotonic_ns:
            return self._failed(context, "CAMERA_TIME_INVALID")

        age_ns = context.monotonic_ns - frame.measurement_monotonic_ns
        stale = age_ns > self._config.maximum_frame_age_ns
        state = DeviceHealthState.DEGRADED if stale else DeviceHealthState.OK
        reason = "CAMERA_FRAME_STALE" if stale else None
        sample = DeviceSample(
            device_id=self.device_id,
            kind="camera_frame_health",
            sequence=frame.sequence,
            captured_monotonic_ns=frame.measurement_monotonic_ns,
            values=(
                DataField("age_ns", age_ns),
                DataField("measurement_timing_valid", True),
                DataField("measurement_stale", stale),
                DataField("completion_lag_ns", frame.completion_lag_ns),
                DataField("width", frame.width),
                DataField("height", frame.height),
                DataField("pixel_format", frame.pixel_format),
                DataField("exposure_time_ns", frame.exposure_time_ns),
                DataField("frame_duration_ns", frame.frame_duration_ns),
                DataField("focus_state", frame.focus_state),
                DataField("lens_position", frame.lens_position),
                DataField("payload_bytes", len(frame.image_bytes)),
                DataField("camera_model", status.camera_model),
            ),
        )
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, state, reason),
            (sample,),
        )

    def _failed(self, context: TickContext, reason: str) -> LiveDeviceSnapshot:
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, DeviceHealthState.FAILED, reason),
        )


__all__ = ["CameraFramePort", "NativeCameraConfig", "NativeCameraSource"]
