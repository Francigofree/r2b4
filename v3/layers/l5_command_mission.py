"""L5 command validation and mission-lifecycle ownership."""

from __future__ import annotations

from dataclasses import dataclass

from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    MissionConstraints,
    MissionIntent,
    MissionLifecycle,
    VelocityTarget,
    Waypoint,
)


@dataclass(frozen=True, slots=True)
class MissionConfig:
    default_constraints: MissionConstraints = MissionConstraints(
        max_v_mps=0.35,
        max_omega_rad_s=1.2,
        corridor_radius_m=0.30,
        goal_tolerance_m=0.08,
        yaw_tolerance_rad=0.10,
    )


class MissionManager:
    """Validate gateway values and own the active command identity."""

    __slots__ = ("_active_command", "_config")

    def __init__(self, config: MissionConfig = MissionConfig()) -> None:
        self._config = config
        self._active_command: tuple[str, CommandMode, tuple[DataField, ...]] | None = None

    def evaluate(self, command: CommandRequest) -> MissionIntent:
        if command.mode is CommandMode.STOP:
            lifecycle = (
                MissionLifecycle.CANCELLED
                if self._active_command is not None
                else MissionLifecycle.IDLE
            )
            self._active_command = None
            return self._stopped(command, "COMMAND_STOP", lifecycle)
        if command.mode is CommandMode.SERVICE:
            self._active_command = None
            return self._stopped(command, "SERVICE_UNSUPPORTED", MissionLifecycle.FAILED)

        signature = (command.command_id, command.mode, command.goal)
        if self._active_command is not None:
            active_id, _, _ = self._active_command
            if active_id == command.command_id and self._active_command != signature:
                self._active_command = None
                return self._stopped(command, "COMMAND_ID_REUSED", MissionLifecycle.FAILED)

        try:
            values = _field_values(command.goal)
            constraints = _constraints(values, self._config.default_constraints)
            if command.mode is CommandMode.NAVIGATE:
                _require_keys(
                    values,
                    required=frozenset({"x_m", "y_m"}),
                    optional=frozenset(
                        {
                            "yaw_rad",
                            "max_v_mps",
                            "max_omega_rad_s",
                            "corridor_radius_m",
                            "goal_tolerance_m",
                            "yaw_tolerance_rad",
                        }
                    ),
                )
                target_pose = Waypoint(
                    _number(values, "x_m"),
                    _number(values, "y_m"),
                    _optional_number(values, "yaw_rad"),
                )
                velocity_target = None
            else:
                _require_keys(
                    values,
                    required=frozenset({"v_mps", "omega_rad_s"}),
                    optional=frozenset({"max_v_mps", "max_omega_rad_s"}),
                )
                target_pose = None
                velocity_target = VelocityTarget(
                    _number(values, "v_mps"),
                    _number(values, "omega_rad_s"),
                )
        except (KeyError, TypeError, ValueError):
            self._active_command = None
            return self._stopped(command, "INVALID_COMMAND", MissionLifecycle.FAILED)

        self._active_command = signature
        return MissionIntent(
            context=command.context,
            mission_id=f"mission-{command.command_id}",
            mode=command.mode,
            target_pose=target_pose,
            velocity_target=velocity_target,
            constraints=constraints,
            lifecycle=MissionLifecycle.ACTIVE,
        )

    def _stopped(
        self,
        command: CommandRequest,
        reason: str,
        lifecycle: MissionLifecycle,
    ) -> MissionIntent:
        return MissionIntent(
            context=command.context,
            mission_id=f"mission-{command.command_id}",
            mode=CommandMode.STOP,
            target_pose=None,
            velocity_target=None,
            constraints=self._config.default_constraints,
            lifecycle=lifecycle,
            stop_reason=reason,
        )


def force_stop_mission(command: CommandRequest) -> MissionIntent:
    """Preserve the deliberately inert behavior of existing STOP-only roots."""

    return MissionIntent(
        context=command.context,
        mission_id=f"stop-{command.command_id}",
        mode=CommandMode.STOP,
        target_pose=None,
        velocity_target=None,
        constraints=MissionConfig().default_constraints,
        lifecycle=MissionLifecycle.IDLE,
        stop_reason="STOP_ONLY_SLICE",
    )


def _field_values(fields: tuple[DataField, ...]) -> dict[str, object]:
    return {field.key: field.value for field in fields}


def _require_keys(
    values: dict[str, object],
    *,
    required: frozenset[str],
    optional: frozenset[str],
) -> None:
    keys = frozenset(values)
    if not required <= keys or not keys <= required | optional:
        raise ValueError("command fields do not match the selected mode")


def _number(values: dict[str, object], key: str) -> float:
    value = values[key]
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise TypeError(f"{key} must be numeric")
    return float(value)


def _optional_number(values: dict[str, object], key: str) -> float | None:
    if key not in values:
        return None
    return _number(values, key)


def _constraints(
    values: dict[str, object],
    defaults: MissionConstraints,
) -> MissionConstraints:
    requested_v = _optional_number(values, "max_v_mps")
    requested_omega = _optional_number(values, "max_omega_rad_s")
    return MissionConstraints(
        max_v_mps=min(
            defaults.max_v_mps if requested_v is None else requested_v,
            defaults.max_v_mps,
        ),
        max_omega_rad_s=min(
            defaults.max_omega_rad_s if requested_omega is None else requested_omega,
            defaults.max_omega_rad_s,
        ),
        corridor_radius_m=(
            _number(values, "corridor_radius_m")
            if "corridor_radius_m" in values
            else defaults.corridor_radius_m
        ),
        goal_tolerance_m=(
            _number(values, "goal_tolerance_m")
            if "goal_tolerance_m" in values
            else defaults.goal_tolerance_m
        ),
        yaw_tolerance_rad=(
            _number(values, "yaw_tolerance_rad")
            if "yaw_tolerance_rad" in values
            else defaults.yaw_tolerance_rad
        ),
    )


__all__ = ["MissionConfig", "MissionManager", "force_stop_mission"]
