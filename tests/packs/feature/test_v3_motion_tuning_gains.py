from __future__ import annotations

from dataclasses import replace

import pytest

from rig import resolved_config
from v3.contracts import (
    MotionObjective, MotionObjectiveKind, RobotEstimate, TickContext,
    VelocityTarget, WorldSnapshot,
)
from v3.layers.l8_motion_realization import MotionRealizer


@pytest.mark.parametrize("field", ["heading_gain", "cross_track_gain", "angular_velocity_gain",
                                  "max_tracking_correction_rad_s"])
@pytest.mark.parametrize("invalid", [-.1, float("nan"), float("inf")])
def test_feedback_tuning_rejects_negative_or_nonfinite_values(field, invalid):
    config = resolved_config().runtime.composition.live_control.control.motion_realization
    with pytest.raises(ValueError, match=field):
        replace(config, **{field: invalid})


@pytest.mark.parametrize("field,enabled,errors,other_gains", [
    ("heading_gain", 1., {"yaw_rad": .1}, {"cross_track_gain": 0., "angular_velocity_gain": 0.}),
    ("cross_track_gain", 3., {"y_m": .05}, {"heading_gain": 1., "angular_velocity_gain": 0.}),
    ("angular_velocity_gain", .25, {"omega_rad_s": .4}, {"heading_gain": 0., "cross_track_gain": 0.}),
    ("max_tracking_correction_rad_s", .6, {"yaw_rad": .1}, {"heading_gain": 1., "cross_track_gain": 0., "angular_velocity_gain": 0.}),
])
def test_zero_disables_selected_feedback_term_and_keeps_feedforward(field, enabled, errors, other_gains):
    control = resolved_config().runtime.composition.live_control.control

    def realize(value):
        config = replace(control.motion_realization, **{**other_gains, field: value})
        realizer = MotionRealizer(config)
        for tick in (0, 1):
            context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
            estimate = RobotEstimate(context, "odom", 0., 0., 0., .3, 0., (0.,) * 25)
            if tick:
                estimate = replace(estimate, **errors)
            objective = MotionObjective(context, "tuning", MotionObjectiveKind.VELOCITY,
                0, tick, "TUNING", None, VelocityTarget(.3, 0.), control.mission.default_constraints)
            motion = realizer.evaluate(objective, estimate, WorldSnapshot(context, "odom", tick, (), 0))
        assert motion.stop_reason is None
        assert motion.requested_v_mps > 0.
        assert motion.requested_v_mps <= objective.constraints.max_v_mps
        assert abs(motion.requested_omega_rad_s) <= objective.constraints.max_omega_rad_s
        # The same configured controller still revokes motion on stale world.
        stopped = realizer.evaluate(objective, estimate, WorldSnapshot(context, "odom", tick, (), config.max_world_freshness_ns + 1))
        assert stopped.stop_reason == "WORLD_STALE"
        assert stopped.requested_v_mps == stopped.requested_omega_rad_s == 0.
        return motion

    assert realize(enabled).requested_omega_rad_s < 0.
    assert realize(0.).requested_omega_rad_s == 0.
