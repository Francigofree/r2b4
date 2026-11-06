from __future__ import annotations

from pathlib import Path

from v3.config import ConfigResolver


ROOT = next(
    (
        path
        for path in Path(__file__).resolve().parents
        if (path / "conf" / "hardver.json").is_file() and (path / "v3").is_dir()
    ),
    Path.cwd(),
)


def test_live_config_preserves_robot_motion_feasibility():
    resolved = ConfigResolver.for_project(ROOT).resolve()
    room = resolved.roomcruise
    assert room is not None

    control = resolved.runtime.composition.live_control.control
    wheels = control.navigation.wheel_limits
    ingress = resolved.edges.command_ingress

    effective_omega = min(
        room.max_omega_rad_s,
        control.mission.default_constraints.max_omega_rad_s,
        control.operational_constraints.max_omega_rad_s,
    )
    effective_v = min(
        room.max_v_mps,
        control.mission.default_constraints.max_v_mps,
        control.operational_constraints.max_v_mps,
        wheels.maximum_mps,
    )

    # Relationships are contracts; concrete tuning values are intentionally free.
    assert wheels.minimum_mps <= wheels.target_floor_mps <= wheels.maximum_mps
    assert control.navigation.localization_recovery_omega_rad_s == wheels.target_center_spin_rad_s
    assert effective_omega >= wheels.target_center_spin_rad_s
    assert effective_v >= wheels.target_floor_mps
    assert room.max_v_mps <= ingress.maximum_linear_speed_mps
    assert room.max_omega_rad_s <= ingress.maximum_angular_speed_rad_s
    assert ingress.reader_poll_s * 1_000_000_000 < ingress.maximum_ttl_ns
