"""L1 acquisition from one already closed immutable L0 snapshot."""

from __future__ import annotations

from v3.contracts import AcquisitionFrame, RawDeviceBatch


def acquire(raw: RawDeviceBatch) -> AcquisitionFrame:
    """Copy the closed L0 snapshot without performing hidden I/O."""

    return AcquisitionFrame(raw.context, raw.samples, raw.device_health)


__all__ = ["acquire"]
