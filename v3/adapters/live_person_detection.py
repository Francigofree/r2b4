"""Non-critical L0 source for the latest person-detection semantic result."""

from __future__ import annotations

from dataclasses import dataclass
from v3.async_capability import CapabilitySnapshot, CapabilityState, TransportSemantics, latest_state_snapshot

from v3.adapters.live_inputs import LiveDeviceSnapshot
from v3.adapters.person_detection import PersonDetectionPort, PersonDetectionSnapshot
from v3.contracts import DataField, DeviceHealth, DeviceHealthState, DeviceSample, TickContext


@dataclass(frozen=True, slots=True)
class NativePersonDetectionConfig:
    device_id: str = "PERSON_DETECTOR_FRONT"
    maximum_result_age_ns: int = 500_000_000
    maximum_detections: int = 5

    def __post_init__(self) -> None:
        if not isinstance(self.device_id, str) or not self.device_id.strip():
            raise ValueError("device_id must be a non-empty string")
        if (
            not isinstance(self.maximum_result_age_ns, int)
            or isinstance(self.maximum_result_age_ns, bool)
            or self.maximum_result_age_ns <= 0
        ):
            raise ValueError("maximum_result_age_ns must be a positive integer")
        if (
            not isinstance(self.maximum_detections, int)
            or isinstance(self.maximum_detections, bool)
            or self.maximum_detections <= 0
        ):
            raise ValueError("maximum_detections must be a positive integer")


class NativePersonDetectionSource:
    """Expose detector health plus bounded person boxes as one L0 observation."""

    transport_semantics = TransportSemantics.LATEST_STATE

    __slots__ = ("_config", "_port", "_last_capability")

    def __init__(self, port: PersonDetectionPort, config: NativePersonDetectionConfig) -> None:
        if not callable(getattr(port, "get_detection_snapshot", None)):
            raise TypeError("port must provide get_detection_snapshot")
        if not callable(getattr(port, "get_detection_status", None)):
            raise TypeError("port must provide get_detection_status")
        if not isinstance(config, NativePersonDetectionConfig):
            raise TypeError("config must be NativePersonDetectionConfig")
        self._port = port
        self._config = config
        self._last_capability = None

    def capability_snapshot(self, observed_monotonic_ns: int) -> CapabilitySnapshot:
        previous = self._last_capability
        return latest_state_snapshot(
            name="vision.person_detection", observed_monotonic_ns=observed_monotonic_ns,
            source_sequence=None if previous is None else previous.source_sequence,
            source_monotonic_ns=None if previous is None else previous.source_monotonic_ns,
            stale_after_ns=self._config.maximum_result_age_ns, running=True,
            error=None if previous is None else previous.error,
        )

    @property
    def device_id(self) -> str:
        return self._config.device_id

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        try:
            status = self._port.get_detection_status()
            result = self._port.get_detection_snapshot()
        except Exception:
            return self._failed(context, "PERSON_DETECTOR_PORT_ERROR")
        if status.last_error:
            return self._failed(context, "PERSON_DETECTOR_RUNTIME_ERROR")
        if not status.running:
            return self._failed(context, "PERSON_DETECTOR_NOT_RUNNING")
        if result is None:
            return LiveDeviceSnapshot(
                context,
                DeviceHealth(self.device_id, DeviceHealthState.UNKNOWN, "PERSON_DETECTOR_NO_RESULT"),
            )
        if not isinstance(result, PersonDetectionSnapshot):
            return self._failed(context, "PERSON_DETECTOR_RESULT_INVALID")
        if result.measurement_monotonic_ns > context.monotonic_ns:
            return self._failed(context, "PERSON_DETECTOR_TIME_INVALID")

        age_ns = context.monotonic_ns - result.measurement_monotonic_ns
        self._last_capability = latest_state_snapshot(
            name="vision.person_detection", observed_monotonic_ns=context.monotonic_ns,
            source_sequence=result.sequence,
            source_monotonic_ns=result.measurement_monotonic_ns,
            stale_after_ns=self._config.maximum_result_age_ns, running=True,
        )
        stale = self._last_capability.state is CapabilityState.STALE
        state = DeviceHealthState.DEGRADED if stale else DeviceHealthState.OK
        reason = "PERSON_DETECTOR_RESULT_STALE" if stale else None
        detections = result.detections[: self._config.maximum_detections]
        primary = detections[0] if detections else None
        values = [
            DataField("age_ns", age_ns),
            DataField("measurement_timing_valid", True),
            DataField("measurement_stale", stale),
            DataField("source_frame_sequence", result.source_frame_sequence),
            DataField("inference_duration_ns", result.inference_duration_ns),
            DataField("person_count", len(result.detections)),
            DataField("emitted_person_count", len(detections)),
            DataField("person_detected", primary is not None),
        ]
        if primary is not None:
            values.extend(
                (
                    DataField("primary_confidence", primary.confidence),
                    DataField("primary_xmin", primary.box.xmin),
                    DataField("primary_ymin", primary.box.ymin),
                    DataField("primary_xmax", primary.box.xmax),
                    DataField("primary_ymax", primary.box.ymax),
                    DataField("primary_center_x", primary.box.center_x),
                    DataField("primary_center_y", primary.box.center_y),
                    DataField("primary_area", primary.box.area),
                )
            )
        for index, detection in enumerate(detections):
            prefix = f"person_{index:03d}"
            box = detection.box
            values.extend(
                (
                    DataField(f"{prefix}_confidence", detection.confidence),
                    DataField(f"{prefix}_xmin", box.xmin),
                    DataField(f"{prefix}_ymin", box.ymin),
                    DataField(f"{prefix}_xmax", box.xmax),
                    DataField(f"{prefix}_ymax", box.ymax),
                    DataField(f"{prefix}_center_x", box.center_x),
                    DataField(f"{prefix}_center_y", box.center_y),
                    DataField(f"{prefix}_area", box.area),
                )
            )
        sample = DeviceSample(
            device_id=self.device_id,
            kind="person_detection",
            sequence=result.sequence,
            captured_monotonic_ns=result.measurement_monotonic_ns,
            values=tuple(values),
        )
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, state, reason),
            (sample,),
        )

    def _failed(self, context: TickContext, reason: str) -> LiveDeviceSnapshot:
        self._last_capability = latest_state_snapshot(
            name="vision.person_detection", observed_monotonic_ns=context.monotonic_ns,
            source_sequence=None, source_monotonic_ns=None,
            stale_after_ns=self._config.maximum_result_age_ns, running=False, error=reason,
        )
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, DeviceHealthState.FAILED, reason),
        )


__all__ = ["NativePersonDetectionConfig", "NativePersonDetectionSource"]
