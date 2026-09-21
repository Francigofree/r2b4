"""Bounded temporal histories owned by L4.

These helpers are deliberately free of I/O and wall-clock access. They only
consume immutable V3 values and monotonically stamped measurements supplied by
L4, so checkpoint/restore and replay stay deterministic.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import DataField, RobotEstimate


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True, slots=True)
class PoseSample:
    frame_id: str
    monotonic_ns: int
    x_m: float
    y_m: float
    yaw_rad: float
    position_variance_x: float = 0.0
    position_variance_y: float = 0.0
    yaw_variance: float = 0.0


@dataclass(frozen=True, slots=True)
class PoseHistoryCheckpoint:
    samples: tuple[PoseSample, ...]


class PoseHistory:
    __slots__ = ("_max_age_ns", "_max_samples", "_lookup_max_skew_ns", "_samples")

    def __init__(self, max_age_ns: int, max_samples: int, lookup_max_skew_ns: int) -> None:
        if max_age_ns <= 0 or max_samples <= 0 or lookup_max_skew_ns < 0:
            raise ValueError("invalid pose history bounds")
        self._max_age_ns = max_age_ns
        self._max_samples = max_samples
        self._lookup_max_skew_ns = lookup_max_skew_ns
        self._samples: list[PoseSample] = []

    @property
    def frame_id(self) -> str | None:
        return self._samples[-1].frame_id if self._samples else None

    def clear(self) -> None:
        self._samples.clear()

    def add(self, estimate: RobotEstimate) -> bool:
        sample = PoseSample(
            estimate.frame_id,
            estimate.context.monotonic_ns,
            estimate.x_m,
            estimate.y_m,
            estimate.yaw_rad,
            estimate.covariance_5x5[0],
            estimate.covariance_5x5[6],
            estimate.covariance_5x5[12],
        )
        frame_changed = bool(self._samples and self._samples[-1].frame_id != sample.frame_id)
        if frame_changed:
            self._samples.clear()
        if self._samples:
            previous = self._samples[-1]
            if sample.monotonic_ns < previous.monotonic_ns:
                raise ValueError("L4 pose history time must not move backwards")
            if sample.monotonic_ns == previous.monotonic_ns:
                if sample != previous:
                    raise ValueError("L4 pose history timestamp was rewritten")
                return frame_changed
        self._samples.append(sample)
        newest_ns = sample.monotonic_ns
        cutoff_ns = newest_ns - self._max_age_ns
        first_keep = 0
        while first_keep < len(self._samples) and self._samples[first_keep].monotonic_ns < cutoff_ns:
            first_keep += 1
        if first_keep:
            del self._samples[:first_keep]
        if len(self._samples) > self._max_samples:
            del self._samples[: len(self._samples) - self._max_samples]
        return frame_changed

    def lookup(self, monotonic_ns: int, frame_id: str) -> PoseSample | None:
        if not self._samples:
            return None
        samples = self._samples
        if samples[-1].frame_id != frame_id:
            return None
        if monotonic_ns <= samples[0].monotonic_ns:
            sample = samples[0]
            return sample if sample.monotonic_ns - monotonic_ns <= self._lookup_max_skew_ns else None
        if monotonic_ns >= samples[-1].monotonic_ns:
            sample = samples[-1]
            return sample if monotonic_ns - sample.monotonic_ns <= self._lookup_max_skew_ns else None
        for index in range(1, len(samples)):
            after = samples[index]
            if monotonic_ns > after.monotonic_ns:
                continue
            before = samples[index - 1]
            if monotonic_ns == after.monotonic_ns:
                return after
            span_ns = after.monotonic_ns - before.monotonic_ns
            if span_ns <= 0:
                return before
            ratio = (monotonic_ns - before.monotonic_ns) / span_ns
            yaw_delta = _wrap_angle(after.yaw_rad - before.yaw_rad)
            return PoseSample(
                frame_id,
                monotonic_ns,
                before.x_m + ratio * (after.x_m - before.x_m),
                before.y_m + ratio * (after.y_m - before.y_m),
                _wrap_angle(before.yaw_rad + ratio * yaw_delta),
                max(before.position_variance_x, after.position_variance_x),
                max(before.position_variance_y, after.position_variance_y),
                max(before.yaw_variance, after.yaw_variance),
            )
        return None

    def checkpoint(self) -> PoseHistoryCheckpoint:
        return PoseHistoryCheckpoint(tuple(self._samples))

    def restore(self, checkpoint: PoseHistoryCheckpoint) -> None:
        if not isinstance(checkpoint, PoseHistoryCheckpoint):
            raise TypeError("checkpoint must be PoseHistoryCheckpoint")
        self._samples = list(checkpoint.samples)


@dataclass(frozen=True, slots=True)
class LidarScanSnapshot:
    source_sequence: int
    captured_monotonic_ns: int
    frame_id: str
    values: tuple[DataField, ...]
    pose: PoseSample


@dataclass(frozen=True, slots=True)
class ScanHistoryCheckpoint:
    scans: tuple[LidarScanSnapshot, ...]


class ScanHistory:
    __slots__ = ("_max_age_ns", "_max_scans", "_scans")

    def __init__(self, max_age_ns: int, max_scans: int) -> None:
        if max_age_ns <= 0 or max_scans <= 0:
            raise ValueError("invalid scan history bounds")
        self._max_age_ns = max_age_ns
        self._max_scans = max_scans
        self._scans: list[LidarScanSnapshot] = []

    def clear(self) -> None:
        self._scans.clear()

    def add(self, scan: LidarScanSnapshot) -> None:
        if self._scans:
            previous = self._scans[-1]
            if scan.source_sequence < previous.source_sequence:
                raise ValueError("L4 scan history sequence must not move backwards")
            if scan.captured_monotonic_ns < previous.captured_monotonic_ns:
                raise ValueError("L4 scan history time must not move backwards")
            if scan.source_sequence == previous.source_sequence:
                if scan != previous:
                    raise ValueError("L4 scan history sequence was rewritten")
                return
        self._scans.append(scan)
        cutoff_ns = scan.captured_monotonic_ns - self._max_age_ns
        first_keep = 0
        while first_keep < len(self._scans) and self._scans[first_keep].captured_monotonic_ns < cutoff_ns:
            first_keep += 1
        if first_keep:
            del self._scans[:first_keep]
        if len(self._scans) > self._max_scans:
            del self._scans[: len(self._scans) - self._max_scans]

    def nearest(self, monotonic_ns: int, max_skew_ns: int, frame_id: str) -> LidarScanSnapshot | None:
        candidates = tuple(scan for scan in self._scans if scan.frame_id == frame_id)
        if not candidates:
            return None
        best = min(
            candidates,
            key=lambda scan: (
                abs(scan.captured_monotonic_ns - monotonic_ns),
                scan.captured_monotonic_ns,
                scan.source_sequence,
            ),
        )
        if abs(best.captured_monotonic_ns - monotonic_ns) > max_skew_ns:
            return None
        return best

    def checkpoint(self) -> ScanHistoryCheckpoint:
        return ScanHistoryCheckpoint(tuple(self._scans))

    def restore(self, checkpoint: ScanHistoryCheckpoint) -> None:
        if not isinstance(checkpoint, ScanHistoryCheckpoint):
            raise TypeError("checkpoint must be ScanHistoryCheckpoint")
        self._scans = list(checkpoint.scans)


__all__ = [
    "LidarScanSnapshot",
    "PoseHistory",
    "PoseHistoryCheckpoint",
    "PoseSample",
    "ScanHistory",
    "ScanHistoryCheckpoint",
]
