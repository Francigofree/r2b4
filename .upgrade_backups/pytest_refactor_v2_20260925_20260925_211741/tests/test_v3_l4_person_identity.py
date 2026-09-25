import math

import pytest

from v3.layers.l4_temporal_tracking import (
    PersonImageRegion,
    PersonMeasurement,
    TemporalTrackStore,
)


def _store():
    return TemporalTrackStore(
        alpha=0.85, beta=0.35, prediction_max_age_ns=350_000_000,
        max_speed_mps=6.0,
    )


def _observe(store, time_ns, *measurements):
    store.associate_people(
        measurements, captured_ns=time_ns, radius_m=0.30,
        max_association_distance_m=0.75,
    )


def test_visual_continuity_prevents_nearby_track_from_taking_target_update():
    # Full-height target and a small neighboring detection, as in the live run.
    # A changed LiDAR return is spatially nearer the neighboring track.
    target = PersonImageRegion(3.0, 0.30, 0.0, 0.95)
    neighbor = PersonImageRegion(3.2, 0.10, 0.5, 0.90)
    store = _store()
    _observe(store, 1_000_000_000,
             PersonMeasurement(0.69, -2.01, 0.09, target),
             PersonMeasurement(0.50, -1.95, -0.23, neighbor))
    checkpoint = store.checkpoint()
    restored = _store()
    restored.restore(checkpoint)
    for tracker in (store, restored):
        _observe(tracker, 1_100_000_000,
                 PersonMeasurement(0.71, -1.95, -0.08, target))
        states = {s.track.track_id: s for s in tracker.checkpoint().states}
        assert states['person-1'].captured_ns == 1_100_000_000
        assert states['person-2'].captured_ns == 1_000_000_000
        assert states['person-1'].track.confidence == 0.71
        tracker.expire(1_550_000_000, person_max_age_ns=500_000_000, other_max_age_ns=1)
        assert [t.track_id for t in tracker.projected_tracks(1_550_000_000)] == ['person-1']
    assert store.checkpoint() == restored.checkpoint()


@pytest.mark.parametrize('distance,elapsed_ns', [(1.0, 1_000_000_000), (0.5, 10_000_000)])
def test_image_overlap_never_bypasses_distance_or_speed_gate(distance, elapsed_ns):
    region = PersonImageRegion(0.0, 0.30, 0.0, 1.0)
    store = _store()
    _observe(store, 1_000_000_000, PersonMeasurement(0.9, 2.0, 0.0, region))
    _observe(store, 1_000_000_000 + elapsed_ns,
             PersonMeasurement(0.9, 2.0 + distance, 0.0, region))
    assert [s.updates for s in store.checkpoint().states] == [1, 1]


def test_image_overlap_wraps_at_pi_and_distinguishes_vertical_extent():
    region = PersonImageRegion(math.pi - 0.01, 0.30, 0.0, 1.0)
    moved = PersonImageRegion(-math.pi + 0.01, 0.30, 0.0, 1.0)
    partial = PersonImageRegion(math.pi - 0.01, 0.30, 0.5, 1.0)
    assert region.overlap(moved) == pytest.approx(0.28 / 0.32)
    assert region.overlap(partial) == pytest.approx(0.5)


def test_observations_without_image_extent_keep_spatial_association():
    store = _store()
    _observe(store, 1_000_000_000, PersonMeasurement(0.9, 2.0, 0.0),
             PersonMeasurement(0.8, 2.0, 0.5))
    _observe(store, 1_100_000_000, PersonMeasurement(0.7, 2.0, 0.4))
    assert [s.updates for s in store.checkpoint().states] == [1, 2]
