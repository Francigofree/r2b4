"""L2 freshness, ordering and source-health admission."""

from __future__ import annotations

from dataclasses import dataclass

from v3.contracts import (
    AcquisitionFrame,
    AdmittedFrame,
    DeviceHealthState,
    Observation,
    RejectedObservation,
    RejectionReason,
)


@dataclass(frozen=True, slots=True)
class AdmissionConfig:
    """Immutable time-alignment limits injected into the L2 state owner."""

    max_sample_age_ns: int
    max_future_skew_ns: int = 0

    def __post_init__(self) -> None:
        for name in ("max_sample_age_ns", "max_future_skew_ns"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")


class InputAdmission:
    """Own per-source sequence history and close L2 admission deterministically."""

    __slots__ = ("_config", "_last_sequences")

    def __init__(self, config: AdmissionConfig) -> None:
        self._config = config
        self._last_sequences: dict[tuple[str, str], int] = {}

    def __call__(self, frame: AcquisitionFrame) -> AdmittedFrame:
        health_by_device = {item.device_id: item.state for item in frame.io_health}
        degraded = {
            item.device_id
            for item in frame.io_health
            if item.state is not DeviceHealthState.OK
        }
        accepted: list[Observation] = []
        rejected: list[RejectedObservation] = []

        for sample in frame.samples:
            key = (sample.device_id, sample.kind)
            previous_sequence = self._last_sequences.get(key)
            reason: RejectionReason | None = None
            age_ns = max(0, frame.context.monotonic_ns - sample.captured_monotonic_ns)

            if previous_sequence is not None and sample.sequence == previous_sequence:
                reason = RejectionReason.DUPLICATE
            elif previous_sequence is not None and sample.sequence < previous_sequence:
                reason = RejectionReason.OUT_OF_ORDER
            elif (
                sample.captured_monotonic_ns
                > frame.context.monotonic_ns + self._config.max_future_skew_ns
            ):
                reason = RejectionReason.TIME_ALIGNMENT_FAILED
            elif age_ns > self._config.max_sample_age_ns:
                reason = RejectionReason.STALE
            elif health_by_device.get(sample.device_id) in {
                None,
                DeviceHealthState.FAILED,
                DeviceHealthState.UNKNOWN,
            }:
                degraded.add(sample.device_id)
                reason = RejectionReason.UNTRUSTED

            if previous_sequence is None or sample.sequence > previous_sequence:
                self._last_sequences[key] = sample.sequence

            if reason is not None:
                rejected.append(
                    RejectedObservation(
                        source_device_id=sample.device_id,
                        source_sequence=sample.sequence,
                        reason=reason,
                        age_ns=age_ns,
                    )
                )
                continue
            accepted.append(
                Observation(
                    kind=sample.kind,
                    source_device_id=sample.device_id,
                    source_sequence=sample.sequence,
                    captured_monotonic_ns=sample.captured_monotonic_ns,
                    values=sample.values,
                )
            )

        return AdmittedFrame(
            frame.context,
            tuple(accepted),
            tuple(rejected),
            tuple(sorted(degraded)),
        )


def admit(frame: AcquisitionFrame) -> AdmittedFrame:
    """Preserve the stateless fake-only compatibility path."""

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


__all__ = ["AdmissionConfig", "InputAdmission", "admit"]
