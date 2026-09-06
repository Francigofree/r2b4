"""L1 acquisition for the fake-only STOP vertical slice."""

from __future__ import annotations

from v3.contracts import AcquisitionFrame, RawDeviceBatch


def acquire(raw: RawDeviceBatch) -> AcquisitionFrame:
    """Copy the already closed L0 snapshot into the L1 boundary value."""

    return AcquisitionFrame(raw.context, raw.samples, raw.device_health)


__all__ = ["acquire"]
