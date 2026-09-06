import math

import pytest

from v3.adapters.bounded_command import (
    BoundedTeleopCommandGateway,
    BoundedTeleopProfile,
)
from v3.contracts import (
    CommandMode,
    DataField,
    MissionLifecycle,
    TickContext,
)
from v3.layers.l5_command_mission import MissionManager


def _profile() -> BoundedTeleopProfile:
    return BoundedTeleopProfile(
        command_id="phase12-bounded-teleop",
        start_tick_id=3,
        active_tick_count=2,
        v_mps=0.08,
        omega_rad_s=-0.10,
        max_v_mps=0.10,
        max_omega_rad_s=0.20,
    )


def _context(tick_id: int) -> TickContext:
    return TickContext(tick_id, 1_000_000 + tick_id * 20_000_000)


def test_gateway_is_stop_outside_and_teleop_only_inside_absolute_tick_window():
    gateway = BoundedTeleopCommandGateway(_profile())

    before = gateway.snapshot(_context(2))
    first = gateway.snapshot(_context(3))
    last = gateway.snapshot(_context(4))
    after = gateway.snapshot(_context(5))
    much_later = gateway.snapshot(_context(100))

    assert before.mode is CommandMode.STOP
    assert before.goal == ()
    assert before.expiry_tick == 2
    assert first.mode is CommandMode.TELEOP
    assert last.mode is CommandMode.TELEOP
    assert first.command_id == last.command_id == "phase12-bounded-teleop"
    assert first.goal == last.goal == (
        DataField("v_mps", 0.08),
        DataField("omega_rad_s", -0.10),
        DataField("max_v_mps", 0.10),
        DataField("max_omega_rad_s", 0.20),
    )
    assert first.expiry_tick == last.expiry_tick == 4
    assert after.mode is CommandMode.STOP
    assert much_later.mode is CommandMode.STOP


def test_active_command_is_accepted_by_existing_l5_then_stop_cancels_it():
    gateway = BoundedTeleopCommandGateway(_profile())
    mission = MissionManager()

    active = mission.evaluate(gateway.snapshot(_context(3)))
    stopped = mission.evaluate(gateway.snapshot(_context(5)))

    assert active.lifecycle is MissionLifecycle.ACTIVE
    assert active.velocity_target is not None
    assert active.velocity_target.v_mps == 0.08
    assert active.velocity_target.omega_rad_s == -0.10
    assert active.constraints.max_v_mps == 0.10
    assert active.constraints.max_omega_rad_s == 0.20
    assert stopped.lifecycle is MissionLifecycle.CANCELLED
    assert stopped.mode is CommandMode.STOP
    assert stopped.stop_reason == "COMMAND_STOP"


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"command_id": "bad command"}, "command_id"),
        ({"start_tick_id": -1}, "start_tick_id"),
        ({"active_tick_count": 0}, "active_tick_count"),
        ({"v_mps": math.inf}, "v_mps"),
        ({"max_v_mps": 0.0}, "motion limits"),
        ({"v_mps": 0.11}, "max_v_mps"),
        ({"omega_rad_s": -0.21}, "max_omega_rad_s"),
    ],
)
def test_profile_rejects_unbounded_or_internally_inconsistent_values(
    changes: dict[str, object],
    message: str,
):
    values = {
        "command_id": "phase12-bounded-teleop",
        "start_tick_id": 3,
        "active_tick_count": 2,
        "v_mps": 0.08,
        "omega_rad_s": -0.10,
        "max_v_mps": 0.10,
        "max_omega_rad_s": 0.20,
    }
    values.update(changes)

    with pytest.raises(ValueError, match=message):
        BoundedTeleopProfile(**values)  # type: ignore[arg-type]


def test_gateway_rejects_invalid_context_without_external_state_or_activation():
    gateway = BoundedTeleopCommandGateway(_profile())

    with pytest.raises(TypeError, match="context must be TickContext"):
        gateway.snapshot(object())  # type: ignore[arg-type]

    assert not hasattr(gateway, "activate")
    assert not hasattr(gateway, "clock")
