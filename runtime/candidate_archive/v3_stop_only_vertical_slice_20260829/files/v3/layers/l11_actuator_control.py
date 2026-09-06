"""L11 zero actuator request for the fake-only STOP vertical slice."""

from __future__ import annotations

from v3.contracts import ActuatorRequest, AdmittedFrame, WheelVelocitySetpoint


def zero_actuator_request(
    wheels: WheelVelocitySetpoint,
    frame: AdmittedFrame,
) -> ActuatorRequest:
    return ActuatorRequest(wheels.context, left_normalized=0.0, right_normalized=0.0)


__all__ = ["zero_actuator_request"]
