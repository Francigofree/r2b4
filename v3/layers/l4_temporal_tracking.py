# R2B4_FOLLOW_PERSON_P0_V2_20260923
"""Bounded deterministic temporal object tracking for L4."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import ObstacleTrack, TrackEstimateStatus


@dataclass(frozen=True, slots=True)
class PersonImageRegion:
    """Measurement-time image extent; horizontal angles are in the map frame."""

    bearing_rad: float
    width_rad: float
    ymin: float
    ymax: float

    def overlap(self, other: PersonImageRegion) -> float:
        delta = math.atan2(
            math.sin(other.bearing_rad - self.bearing_rad),
            math.cos(other.bearing_rad - self.bearing_rad),
        )
        width = max(0.0, min(self.width_rad / 2, delta + other.width_rad / 2)
                    - max(-self.width_rad / 2, delta - other.width_rad / 2))
        height = max(0.0, min(self.ymax, other.ymax) - max(self.ymin, other.ymin))
        intersection = width * height
        union = (self.width_rad * (self.ymax - self.ymin)
                 + other.width_rad * (other.ymax - other.ymin) - intersection)
        return intersection / union if union > 0.0 else 0.0


@dataclass(frozen=True, slots=True)
class PersonMeasurement:
    confidence: float
    x_m: float
    y_m: float
    image_region: PersonImageRegion | None = None


@dataclass(frozen=True, slots=True)
class TemporalTrackState:
    track: ObstacleTrack
    captured_ns: int
    updates: int
    image_region: PersonImageRegion | None = None
    observed_monotonic_ns: int | None = None



@dataclass(frozen=True, slots=True)
class TemporalTrackCheckpoint:
    states: tuple[TemporalTrackState, ...]
    next_person_track_sequence: int = 1
    # Dormant identities are replay authority but are never public obstacle tracks.
    dormant_states: tuple[TemporalTrackState, ...] = ()

    def __post_init__(self) -> None:
        value = self.next_person_track_sequence
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            raise ValueError("next_person_track_sequence must be a positive integer")
        active = {state.track.track_id for state in self.states}
        dormant = {state.track.track_id for state in self.dormant_states}
        if active & dormant:
            raise ValueError("active and dormant track identities must be disjoint")




class TemporalTrackStore:
    """Bounded active tracking plus non-published short-lived person identity memory."""

    __slots__ = (
        "_alpha",
        "_beta",
        "_prediction_max_age_ns",
        "_max_speed_mps",
        "_person_reacquire_max_age_ns",
        "_next_person_track_sequence",
        "_states",
        "_dormant_states",
    )

    def __init__(
        self,
        *,
        alpha: float,
        beta: float,
        prediction_max_age_ns: int,
        max_speed_mps: float,
        person_reacquire_max_age_ns: int = 2_500_000_000,
    ) -> None:
        if not 0.0 < alpha <= 1.0 or not 0.0 < beta <= 1.0:
            raise ValueError("track alpha/beta must be in (0, 1]")
        if prediction_max_age_ns < 0 or max_speed_mps <= 0.0:
            raise ValueError("invalid track prediction bounds")
        if (
            not isinstance(person_reacquire_max_age_ns, int)
            or isinstance(person_reacquire_max_age_ns, bool)
            or person_reacquire_max_age_ns <= 0
        ):
            raise ValueError("person_reacquire_max_age_ns must be positive integer")
        self._alpha = alpha
        self._beta = beta
        self._prediction_max_age_ns = prediction_max_age_ns
        self._max_speed_mps = max_speed_mps
        self._person_reacquire_max_age_ns = person_reacquire_max_age_ns
        self._next_person_track_sequence = 1
        self._states: dict[str, TemporalTrackState] = {}
        self._dormant_states: dict[str, TemporalTrackState] = {}

    def clear(self) -> bool:
        changed = bool(self._states or self._dormant_states)
        self._states.clear()
        self._dormant_states.clear()
        # The monotonic person allocator intentionally survives clear().
        return changed

    def upsert_external(
        self, track: ObstacleTrack, captured_ns: int, *, observed_ns: int | None = None,
    ) -> bool:
        previous = self._states.get(track.track_id)
        if previous is None:
            previous = self._dormant_states.get(track.track_id)
        if previous is not None and captured_ns < previous.captured_ns:
            raise ValueError("L4 obstacle track time must not move backwards")
        updates = 1 if previous is None else previous.updates + 1
        state = TemporalTrackState(track, captured_ns, updates, observed_monotonic_ns=observed_ns)
        changed = state != previous or track.track_id in self._dormant_states
        self._dormant_states.pop(track.track_id, None)
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
        observed_ns: int | None = None,
    ) -> bool:
        active = {
            track_id: state
            for track_id, state in self._states.items()
            if track_id.startswith("person-")
        }
        dormant = {
            track_id: state
            for track_id, state in self._dormant_states.items()
            if track_id.startswith("person-")
            and 0 < captured_ns - state.captured_ns <= self._person_reacquire_max_age_ns
        }
        used_track_ids: set[str] = set()
        changed = False
        for measurement in measurements:
            best = self._best_person_match(
                active, used_track_ids, measurement, captured_ns, max_association_distance_m
            )
            reactivated = False
            if best is None:
                best = self._best_person_match(
                    dormant, used_track_ids, measurement, captured_ns, max_association_distance_m
                )
                reactivated = best is not None

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
                self._states[track_id] = TemporalTrackState(
                    track, captured_ns, 1, measurement.image_region, observed_ns
                )
                used_track_ids.add(track_id)
                changed = True
                continue

            _, _, track_id, state = best
            used_track_ids.add(track_id)
            if reactivated:
                self._dormant_states.pop(track_id, None)
            self._states[track_id] = self._updated_person_state(
                track_id, state, measurement, captured_ns, radius_m,
                reactivated=reactivated, observed_ns=observed_ns,
            )
            changed = True
        return changed

    def _best_person_match(
        self,
        pool: dict[str, TemporalTrackState],
        used_track_ids: set[str],
        measurement: PersonMeasurement,
        captured_ns: int,
        max_association_distance_m: float,
    ) -> tuple[float, float, str, TemporalTrackState] | None:
        best: tuple[float, float, str, TemporalTrackState] | None = None
        for track_id in sorted(pool):
            if track_id in used_track_ids:
                continue
            state = pool[track_id]
            dt_ns = captured_ns - state.captured_ns
            if dt_ns <= 0:
                continue
            dt_s = dt_ns / 1e9
            previous = state.track
            projected_s = min(dt_ns, self._prediction_max_age_ns) / 1e9
            predicted_x = previous.x_m + previous.vx_mps * projected_s
            predicted_y = previous.y_m + previous.vy_mps * projected_s
            distance_m = math.hypot(measurement.x_m - predicted_x, measurement.y_m - predicted_y)
            raw_speed_mps = math.hypot(
                measurement.x_m - previous.x_m, measurement.y_m - previous.y_m
            ) / dt_s
            if distance_m > max_association_distance_m or raw_speed_mps > self._max_speed_mps:
                continue
            # A nearby LiDAR return can move between a person's legs/background.
            # Retain the visual identity when both observations carry its extent;
            # image overlap never bypasses the spatial/speed admission above.
            overlap = 0.0
            if state.image_region is not None and measurement.image_region is not None:
                overlap = state.image_region.overlap(measurement.image_region)
            candidate = (-overlap, distance_m, track_id, state)
            if best is None or candidate[:3] < best[:3]:
                best = candidate
        return best

    def _updated_person_state(
        self,
        track_id: str,
        state: TemporalTrackState,
        measurement: PersonMeasurement,
        captured_ns: int,
        radius_m: float,
        *,
        reactivated: bool,
        observed_ns: int | None = None,
    ) -> TemporalTrackState:
        previous = state.track
        dt_s = (captured_ns - state.captured_ns) / 1e9
        if reactivated or state.updates == 1:
            # Reactivate exactly at the fresh measurement; stale velocity is not
            # extrapolated blindly through an occlusion.
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
        return TemporalTrackState(
            ObstacleTrack(
                track_id=track_id,
                x_m=x_m,
                y_m=y_m,
                radius_m=radius_m,
                vx_mps=vx_mps,
                vy_mps=vy_mps,
                confidence=measurement.confidence,
            ),
            captured_ns,
            state.updates + 1,
            measurement.image_region,
            observed_ns,
        )

    def expire(self, now_ns: int, *, person_max_age_ns: int, other_max_age_ns: int) -> bool:
        expired_active = tuple(
            track_id
            for track_id, state in self._states.items()
            if now_ns - state.captured_ns
            > (person_max_age_ns if track_id.startswith("person-") else other_max_age_ns)
        )
        for track_id in expired_active:
            state = self._states.pop(track_id)
            if track_id.startswith("person-"):
                self._dormant_states[track_id] = state

        expired_dormant = tuple(
            track_id
            for track_id, state in self._dormant_states.items()
            if now_ns - state.captured_ns > self._person_reacquire_max_age_ns
        )
        for track_id in expired_dormant:
            del self._dormant_states[track_id]
        # Only active-set changes affect the public world snapshot/map revision.
        return bool(expired_active)

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
                    estimate_status=(
                        TrackEstimateStatus.DEGRADED
                        if age_ns > self._prediction_max_age_ns
                        else TrackEstimateStatus.OBSERVED
                        if now_ns == (state.observed_monotonic_ns
                                      if state.observed_monotonic_ns is not None else state.captured_ns)
                        else TrackEstimateStatus.PREDICTED
                    ),
                    measurement_monotonic_ns=state.captured_ns,
                    prediction_valid_until_ns=state.captured_ns + self._prediction_max_age_ns,
                )
            )
        return tuple(result)

    def checkpoint(self) -> TemporalTrackCheckpoint:
        return TemporalTrackCheckpoint(
            tuple(self._states[key] for key in sorted(self._states)),
            self._next_person_track_sequence,
            tuple(self._dormant_states[key] for key in sorted(self._dormant_states)),
        )

    def restore(self, checkpoint: TemporalTrackCheckpoint) -> None:
        if not isinstance(checkpoint, TemporalTrackCheckpoint):
            raise TypeError("checkpoint must be TemporalTrackCheckpoint")
        self._states = {state.track.track_id: state for state in checkpoint.states}
        self._dormant_states = {state.track.track_id: state for state in checkpoint.dormant_states}
        if self._states.keys() & self._dormant_states.keys():
            raise ValueError("active and dormant track identities must be disjoint")
        self._next_person_track_sequence = checkpoint.next_person_track_sequence
        for track_id in (*self._states, *self._dormant_states):
            self._observe_person_track_id(track_id)

    def _observe_person_track_id(self, track_id: str) -> None:
        if not track_id.startswith("person-"):
            return
        suffix = track_id[len("person-") :]
        if suffix.isdigit():
            self._next_person_track_sequence = max(self._next_person_track_sequence, int(suffix) + 1)

    def _allocate_person_track_id(self) -> str:
        candidate = self._next_person_track_sequence
        while f"person-{candidate}" in self._states or f"person-{candidate}" in self._dormant_states:
            candidate += 1
        self._next_person_track_sequence = candidate + 1
        return f"person-{candidate}"



__all__ = [
    "PersonImageRegion",
    "PersonMeasurement",
    "TemporalTrackCheckpoint",
    "TemporalTrackState",
    "TemporalTrackStore",
]
