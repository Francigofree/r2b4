"""L2 admission for the fake-only STOP vertical slice."""

from __future__ import annotations

from v3.contracts import AcquisitionFrame, AdmittedFrame, DeviceHealthState, Observation


def admit(frame: AcquisitionFrame) -> AdmittedFrame:
    """Admit immutable fake samples without introducing history or authority."""

    accepted = tuple(
        Observation(
            kind=sample.kind,
            source_device_id=sample.device_id,
            source_sequence=sample.sequence,
            captured_monotonic_ns=sample.captured_monotonic_ns,
            values=sample.values,
        )
        for sample in frame.samples
    )
    degraded = tuple(
        health.device_id
        for health in frame.io_health
        if health.state is not DeviceHealthState.OK
    )
    return AdmittedFrame(frame.context, accepted, (), degraded)


__all__ = ["admit"]
