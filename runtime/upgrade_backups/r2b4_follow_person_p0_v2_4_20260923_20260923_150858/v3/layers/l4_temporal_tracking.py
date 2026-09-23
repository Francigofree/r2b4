"""Bounded deterministic temporal object tracking for L4."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import ObstacleTrack


@dataclass(frozen=True, slots=True)
class PersonMeasurement:
    confidence: float
    x_m: float
    y_m: float


@dataclass(frozen=True, slots=True)
class TemporalTrackState:
    track: ObstacleTrack
    captured_ns: int
    updates: int


@dataclass(frozen=True, slots=True)
class TemporalTrackCheckpoint:
    states: tuple[TemporalTrackState, ...]
    # Monotonic allocator state is replay authority: a person UID must never be
    # recycled after expiry/clear inside one runtime lineage.
    next_person_track_sequence: int = 1

    def __post_init__(self) -> None:
        value = self.next_person_track_sequence
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            raise ValueError("next_person_track_sequence must be a positive integer")


class TemporalTrackStore:
    __slots__ = (
        "_alpha",
        "_beta",
        "_prediction_max_age_ns",
        "_max_speed_mps",
        "_next_person_track_sequence",
        "_states",
    )

    def __init__(
        self,
        *,
        alpha: float,
        beta: float,
        prediction_max_age_ns: int,
        max_speed_mps: float,
    ) -> None:
        if not 0.0 < alpha <= 1.0 or not 0.0 < beta <= 1.0:
            raise ValueError("track alpha/beta must be in (0, 1]")
        if prediction_max_age_ns < 0 or max_speed_mps <= 0.0:
            raise ValueError("invalid track prediction bounds")
        self._alpha = alpha
        self._beta = beta
        self._prediction_max_age_ns = prediction_max_age_ns
        self._max_speed_mps = max_speed_mps
        self._next_person_track_sequence = 1
        self._states: dict[str, TemporalTrackState] = {}

    def clear(self) -> bool:
        changed = bool(self._states)
        self._states.clear()
        return changed

    def upsert_external(self, track: ObstacleTrack, captured_ns: int) -> bool:
        previous = self._states.get(track.track_id)
        if previous is not None and captured_ns < previous.captured_ns:
            raise ValueError("L4 obstacle track time must not move backwards")
        updates = 1 if previous is None else previous.updates + 1
        state = TemporalTrackState(track, captured_ns, updates)
        changed = state != previous
        self._states[track.track_id] = state
        self._observe_person_track_id(track.track_id)
        return changed

    def associate_people(
        self,
        measurements: tuple[PersonMeasurement, ...],
        *,
        captured_ns: int,
        radius_m: float,
        max_association_distance_m: float,
    ) -> bool:
        available = {
            track_id: state
            for track_id, state in self._states.items()
            if track_id.startswith("person-")
        }
        used_track_ids: set[str] = set()
        changed = False
        for measurement in measurements:
            best: tuple[float, str, TemporalTrackState] | None = None
            for track_id in sorted(available):
                if track_id in used_track_ids:
                    continue
                state = available[track_id]
                dt_ns = captured_ns - state.captured_ns
                if dt_ns <= 0:
                    continue
                dt_s = dt_ns / 1e9
                previous = state.track
                predicted_x = previous.x_m + previous.vx_mps * dt_s
                predicted_y = previous.y_m + previous.vy_mps * dt_s
                distance_m = math.hypot(
                    measurement.x_m - predicted_x,
                    measurement.y_m - predicted_y,
                )
                raw_speed_mps = math.hypot(
                    measurement.x_m - previous.x_m,
                    measurement.y_m - previous.y_m,
                ) / dt_s
                if distance_m > max_association_distance_m or raw_speed_mps > self._max_speed_mps:
                    continue
                candidate = (distance_m, track_id, state)
                if best is None or candidate[:2] < best[:2]:
                    best = candidate

            if best is None:
                track_id = self._allocate_person_track_id()
                track = ObstacleTrack(
                    track_id=track_id,
                    x_m=measurement.x_m,
                    y_m=measurement.y_m,
                    radius_m=radius_m,
                    vx_mps=0.0,
                    vy_mps=0.0,
                    confidence=measurement.confidence,
                )
                self._states[track_id] = TemporalTrackState(track, captured_ns, 1)
                changed = True
                continue

            _, track_id, state = best
            used_track_ids.add(track_id)
            previous = state.track
            dt_s = (captured_ns - state.captured_ns) / 1e9
            if state.updates == 1:
                # Preserve the established V3 behaviour on the first velocity
                # estimate; filtering begins once a real motion history exists.
                x_m = measurement.x_m
                y_m = measurement.y_m
                vx_mps = (measurement.x_m - previous.x_m) / dt_s
                vy_mps = (measurement.y_m - previous.y_m) / dt_s
            else:
                predicted_x = previous.x_m + previous.vx_mps * dt_s
                predicted_y = previous.y_m + previous.vy_mps * dt_s
                residual_x = measurement.x_m - predicted_x
                residual_y = measurement.y_m - predicted_y
                x_m = predicted_x + self._alpha * residual_x
                y_m = predicted_y + self._alpha * residual_y
                vx_mps = previous.vx_mps + self._beta * residual_x / dt_s
                vy_mps = previous.vy_mps + self._beta * residual_y / dt_s
                speed = math.hypot(vx_mps, vy_mps)
                if speed > self._max_speed_mps:
                    scale = self._max_speed_mps / speed
                    vx_mps *= scale
                    vy_mps *= scale
            track = ObstacleTrack(
                track_id=track_id,
                x_m=x_m,
                y_m=y_m,
                radius_m=radius_m,
                vx_mps=vx_mps,
                vy_mps=vy_mps,
                confidence=measurement.confidence,
            )
            self._states[track_id] = TemporalTrackState(track, captured_ns, state.updates + 1)
            changed = True
        return changed

    def expire(self, now_ns: int, *, person_max_age_ns: int, other_max_age_ns: int) -> bool:
        expired = tuple(
            track_id
            for track_id, state in self._states.items()
            if now_ns - state.captured_ns
            > (person_max_age_ns if track_id.startswith("person-") else other_max_age_ns)
        )
        for track_id in expired:
            del self._states[track_id]
        return bool(expired)

    def projected_tracks(self, now_ns: int) -> tuple[ObstacleTrack, ...]:
        result: list[ObstacleTrack] = []
        for track_id in sorted(self._states):
            state = self._states[track_id]
            track = state.track
            age_ns = max(0, now_ns - state.captured_ns)
            projected_ns = min(age_ns, self._prediction_max_age_ns)
            dt_s = projected_ns / 1e9
            result.append(
                ObstacleTrack(
                    track_id=track.track_id,
                    x_m=track.x_m + track.vx_mps * dt_s,
                    y_m=track.y_m + track.vy_mps * dt_s,
                    radius_m=track.radius_m,
                    vx_mps=track.vx_mps,
                    vy_mps=track.vy_mps,
                    confidence=track.confidence,
                )
            )
        return tuple(result)

    def checkpoint(self) -> TemporalTrackCheckpoint:
        return TemporalTrackCheckpoint(
            tuple(self._states[key] for key in sorted(self._states)),
            self._next_person_track_sequence,
        )

    def restore(self, checkpoint: TemporalTrackCheckpoint) -> None:
        if not isinstance(checkpoint, TemporalTrackCheckpoint):
            raise TypeError("checkpoint must be TemporalTrackCheckpoint")
        self._states = {state.track.track_id: state for state in checkpoint.states}
        self._next_person_track_sequence = checkpoint.next_person_track_sequence
        # Accept older/externally-built checkpoints defensively: the allocator
        # must always advance beyond every numeric person UID already present.
        for track_id in self._states:
            self._observe_person_track_id(track_id)

    def _observe_person_track_id(self, track_id: str) -> None:
        if not track_id.startswith("person-"):
            return
        suffix = track_id[len("person-") :]
        if suffix.isdigit():
            self._next_person_track_sequence = max(
                self._next_person_track_sequence,
                int(suffix) + 1,
            )

    def _allocate_person_track_id(self) -> str:
        candidate = self._next_person_track_sequence
        while f"person-{candidate}" in self._states:
            candidate += 1
        self._next_person_track_sequence = candidate + 1
        return f"person-{candidate}"


__all__ = [
    "PersonMeasurement",
    "TemporalTrackCheckpoint",
    "TemporalTrackState",
    "TemporalTrackStore",
]
