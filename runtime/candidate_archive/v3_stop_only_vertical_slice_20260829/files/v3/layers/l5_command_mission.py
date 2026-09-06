"""L5 STOP mission projection for the fake-only vertical slice."""

from __future__ import annotations

from v3.contracts import CommandMode, CommandRequest, MissionIntent, MissionLifecycle


def force_stop_mission(command: CommandRequest) -> MissionIntent:
    """Convert every valid gateway command into an explicit inert mission."""

    return MissionIntent(
        context=command.context,
        mission_id=f"stop-{command.command_id}",
        mode=CommandMode.STOP,
        goal=(),
        constraints=(),
        lifecycle=MissionLifecycle.IDLE,
        stop_reason="STOP_ONLY_SLICE",
    )


__all__ = ["force_stop_mission"]
