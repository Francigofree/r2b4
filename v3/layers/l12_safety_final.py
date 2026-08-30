"""Fail-closed L12 safety decision and the sole final motor-write capability."""

from __future__ import annotations

from v3.contracts import (
    ActuatorRequest,
    DeviceHealth,
    DeviceHealthState,
    FinalActuation,
    LifecycleState,
    SafetyDecision,
    TickContext,
)
from v3.ports import MotorWriter


class MotorWriteError(RuntimeError):
    """Raised after the single atomic writer call failed."""


class FinalSafetyGate:
    """Own the fault latch, final decision, and only normal motor writer."""

    def __init__(self, writer: MotorWriter) -> None:
        self._writer = writer
        self._fault_latched = False

    @property
    def fault_latched(self) -> bool:
        return self._fault_latched

    def finalize(
        self,
        context: TickContext,
        request: ActuatorRequest | None,
        critical_health: tuple[DeviceHealth, ...],
        lifecycle: LifecycleState,
        upstream_fault: str | None,
    ) -> FinalActuation:
        """Make one fail-closed decision and perform one atomic writer call."""

        failed_device = next(
            (item for item in critical_health if item.state is DeviceHealthState.FAILED),
            None,
        )
        unknown_device = next(
            (item for item in critical_health if item.state is DeviceHealthState.UNKNOWN),
            None,
        )
        degraded_device = next(
            (item for item in critical_health if item.state is DeviceHealthState.DEGRADED),
            None,
        )

        if upstream_fault is not None:
            self._fault_latched = True
            command = self._stop(context, SafetyDecision.FAULT, upstream_fault)
        elif failed_device is not None:
            self._fault_latched = True
            command = self._stop(context, SafetyDecision.FAULT, "CRITICAL_DEVICE_FAILED")
        elif self._fault_latched:
            command = self._stop(context, SafetyDecision.FAULT, "FAULT_LATCHED")
        elif unknown_device is not None:
            command = self._stop(context, SafetyDecision.STOP, "CRITICAL_DEVICE_UNKNOWN")
        elif degraded_device is not None:
            command = self._stop(context, SafetyDecision.STOP, "CRITICAL_DEVICE_DEGRADED")
        elif lifecycle is not LifecycleState.ACTIVE:
            command = self._stop(context, SafetyDecision.STOP, "NOT_ACTIVE")
        elif request is None:
            self._fault_latched = True
            command = self._stop(context, SafetyDecision.FAULT, "MISSING_ACTUATOR_REQUEST")
        elif request.context != context:
            self._fault_latched = True
            command = self._stop(context, SafetyDecision.FAULT, "REQUEST_CONTEXT_MISMATCH")
        else:
            command = FinalActuation(
                context=context,
                left_output=request.left_normalized,
                right_output=request.right_normalized,
                enabled=True,
                safety_decision=SafetyDecision.ALLOW,
                latch_state="CLEAR",
            )

        try:
            self._writer.write(command)
        except Exception as exc:
            self._fault_latched = True
            raise MotorWriteError("the single final motor write failed") from exc
        return command

    @staticmethod
    def _stop(
        context: TickContext,
        decision: SafetyDecision,
        reason: str,
    ) -> FinalActuation:
        return FinalActuation(
            context=context,
            left_output=0.0,
            right_output=0.0,
            enabled=False,
            safety_decision=decision,
            latch_state="FAULT" if decision is SafetyDecision.FAULT else "STOPPED",
            reason=reason,
        )


__all__ = ["FinalSafetyGate", "MotorWriteError"]
