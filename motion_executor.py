#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""V2.1 L9 wheel-only actuator controller."""

from __future__ import annotations

import copy
import math
from typing import Any, Mapping

from controller.motion_platform_contract import (
    MOTION_PLATFORM_CONTRACT_ID,
    CandidateMotorOutput,
    CycleContext,
    WheelFeedback,
    WheelVelocitySetpoint,
)
from core.control_strategies import (
    WHEEL_STARTUP_RELEASE_DWELL_S,
    WHEEL_STARTUP_RELEASE_SPEED_MPS,
    WheelSpeedPILoop,
    normalize_control_mode,
    wheel_feedback_timing_error,
)
from middleware.ffp import AlbaDriveController, PIDConfig


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _sign(value: float) -> int:
    return 1 if value > 1e-9 else -1 if value < -1e-9 else 0


class MotionExecutor:
    """Own speed-map, feed-forward, wheel PI and actuator-only compensation."""

    def __init__(
        self,
        *,
        pid_config: PIDConfig,
        max_pwm: float,
        speed_map: Mapping[str, Any],
        control_mode: str = "UNIFIED",
        direction_switch_hold_s: float = 0.08,
        direction_switch_debounce_cycles: int = 3,
    ):
        self.drive_pid_cfg = copy.deepcopy(pid_config)
        self.control_mode = normalize_control_mode(control_mode)
        self.max_pwm = max(0.0, min(1.0, float(max_pwm)))
        self.direction_switch_hold_s = max(0.0, float(direction_switch_hold_s))
        self.direction_switch_debounce_cycles = max(
            1,
            int(direction_switch_debounce_cycles),
        )
        self.drive_ctrl = AlbaDriveController(
            self.drive_pid_cfg,
            speed_map=copy.deepcopy(dict(speed_map or {})),
        )
        self.wheel_pi = WheelSpeedPILoop(
            self.drive_pid_cfg,
            max_pwm=self.max_pwm,
            dead_zone=0.0,
            overspeed_holdoff_enabled=False,
        )
        self._startup_active = {"left": False, "right": False}
        self._startup_sign = {"left": 0, "right": 0}
        self._startup_release_dwell_s = {"left": 0.0, "right": 0.0}
        self._last_sign = {"left": 0, "right": 0}
        self._pending_sign = {"left": 0, "right": 0}
        self._pending_count = {"left": 0, "right": 0}
        self._direction_switch_hold_until = 0.0
        self._last_output: CandidateMotorOutput | None = None
        self._last_pid_diag: dict[str, Any] = {}
        self._replayer_reset_generation = 0

    def get_control_mode(self) -> str:
        return self.control_mode

    def reset(self) -> None:
        self.wheel_pi.reset()
        self.drive_ctrl.reset()
        self._startup_active = {"left": False, "right": False}
        self._startup_sign = {"left": 0, "right": 0}
        self._startup_release_dwell_s = {"left": 0.0, "right": 0.0}
        self._last_sign = {"left": 0, "right": 0}
        self._pending_sign = {"left": 0, "right": 0}
        self._pending_count = {"left": 0, "right": 0}
        self._direction_switch_hold_until = 0.0
        self._replayer_reset_generation += 1

    @staticmethod
    def _output(
        setpoint: WheelVelocitySetpoint,
        *,
        left_pwm: float,
        right_pwm: float,
        reason: str,
        diagnostics: Mapping[str, Any] | None = None,
    ) -> CandidateMotorOutput:
        return CandidateMotorOutput(
            contract_id=MOTION_PLATFORM_CONTRACT_ID,
            candidate_output_id=f"candidate:{setpoint.wheel_setpoint_id}",
            wheel_setpoint_id=setpoint.wheel_setpoint_id,
            physical_command_id=setpoint.physical_command_id,
            resolved_id=setpoint.resolved_id,
            cycle_id=setpoint.cycle_id,
            left_pwm=max(-1.0, min(1.0, float(left_pwm))),
            right_pwm=max(-1.0, min(1.0, float(right_pwm))),
            output_reason=str(reason),
            wheel_control_diagnostics=dict(diagnostics or {}),
        )

    def _remember(self, output: CandidateMotorOutput) -> CandidateMotorOutput:
        self._last_output = output
        self._last_pid_diag = {
            **dict(output.wheel_control_diagnostics),
            "output_reason": output.output_reason,
            "v_l_ref": (
                output.wheel_control_diagnostics.get("left_reference_mps", 0.0)
            ),
            "v_r_ref": (
                output.wheel_control_diagnostics.get("right_reference_mps", 0.0)
            ),
            "pwm_executor_l": float(output.left_pwm),
            "pwm_executor_r": float(output.right_pwm),
        }
        return output

    def _zero(
        self,
        setpoint: WheelVelocitySetpoint,
        *,
        reason: str,
        diagnostics: Mapping[str, Any] | None = None,
        reset_control: bool = True,
    ) -> CandidateMotorOutput:
        if reset_control:
            self.wheel_pi.reset()
        return self._remember(
            self._output(
                setpoint,
                left_pwm=0.0,
                right_pwm=0.0,
                reason=reason,
                diagnostics=diagnostics,
            )
        )

    def _boundary_error(
        self,
        cycle_context: CycleContext,
        setpoint: WheelVelocitySetpoint,
        feedback: WheelFeedback,
    ) -> str:
        if setpoint.contract_id != MOTION_PLATFORM_CONTRACT_ID:
            return "CONTRACT_ID_INVALID"
        if str(setpoint.cycle_id) != str(cycle_context.cycle_id):
            return "CYCLE_ID_MISMATCH"
        if not setpoint.wheel_setpoint_id or not setpoint.physical_command_id:
            return "LINEAGE_MISSING"
        if not setpoint.feasible:
            return "SETPOINT_INFEASIBLE"
        if not all(
            _finite(value)
            for value in (
                cycle_context.monotonic_time,
                cycle_context.dt_observed_s,
                cycle_context.dt_control_s,
                setpoint.left_target_mps,
                setpoint.right_target_mps,
            )
        ):
            return "INPUT_NONFINITE"
        if not cycle_context.timing_valid or cycle_context.dt_control_s <= 0.0:
            return "CYCLE_TIMING_INVALID"
        if abs(setpoint.left_target_mps) <= 1e-12 and abs(setpoint.right_target_mps) <= 1e-12:
            return ""
        if not feedback.measurement_id or feedback.measurement_id == "MISSING":
            return "WHEEL_MEASUREMENT_ID_MISSING"
        if not all(
            _finite(value)
            for value in (
                feedback.source_timestamp,
                feedback.left_mps,
                feedback.right_mps,
                feedback.combined_trust,
            )
        ):
            return "WHEEL_FEEDBACK_NONFINITE"
        feedback_timing_error = wheel_feedback_timing_error(
            timing_valid=feedback.timing_valid,
            stale=feedback.stale,
            timing_reason=feedback.timing_reason,
        )
        if feedback_timing_error:
            return feedback_timing_error
        if feedback.combined_trust < self.wheel_pi.encoder_feedback_trust_min:
            return "WHEEL_FEEDBACK_UNTRUSTED"
        return ""

    def _direction_switch_reason(
        self,
        *,
        cycle_context: CycleContext,
        left_target: float,
        right_target: float,
    ) -> str:
        targets = {"left": float(left_target), "right": float(right_target)}
        if all(_sign(value) == 0 for value in targets.values()):
            self._last_sign = {"left": 0, "right": 0}
            self._pending_sign = {"left": 0, "right": 0}
            self._pending_count = {"left": 0, "right": 0}
            return ""
        switch_pending = False
        confirmed = False
        for side, target in targets.items():
            new_sign = _sign(target)
            old_sign = self._last_sign[side]
            if new_sign == 0:
                self._pending_sign[side] = 0
                self._pending_count[side] = 0
                continue
            if old_sign == 0 or new_sign == old_sign:
                self._last_sign[side] = new_sign
                self._pending_sign[side] = 0
                self._pending_count[side] = 0
                continue
            switch_pending = True
            if self._pending_sign[side] != new_sign:
                self._pending_sign[side] = new_sign
                self._pending_count[side] = 1
            else:
                self._pending_count[side] += 1
            if self._pending_count[side] >= self.direction_switch_debounce_cycles:
                self._last_sign[side] = new_sign
                self._pending_sign[side] = 0
                self._pending_count[side] = 0
                confirmed = True
        if confirmed:
            self._direction_switch_hold_until = (
                float(cycle_context.monotonic_time) + self.direction_switch_hold_s
            )
            self.wheel_pi.reset()
            return "DIRECTION_SWITCH_HOLD"
        if switch_pending:
            return "DIRECTION_SWITCH_DEBOUNCE"
        if float(cycle_context.monotonic_time) < self._direction_switch_hold_until:
            return "DIRECTION_SWITCH_HOLD"
        return ""

    def _startup_floor(
        self,
        *,
        side: str,
        reference_mps: float,
        measured_mps: float,
        maintenance_pwm: float,
        feedforward_diagnostics: Mapping[str, Any],
        dt_s: float,
    ) -> tuple[float, dict[str, Any]]:
        reference = float(reference_mps)
        if abs(reference) <= 1e-12:
            self._startup_active[side] = False
            self._startup_sign[side] = 0
            self._startup_release_dwell_s[side] = 0.0
            return 0.0, {
                "startup_floor_active": False,
                "startup_floor_applied": False,
            }
        sign = _sign(reference)
        if self._startup_sign[side] != sign:
            self._startup_sign[side] = sign
            self._startup_active[side] = True
            self._startup_release_dwell_s[side] = 0.0
        elif self._startup_active[side]:
            aligned_speed = sign * float(measured_mps)
            if aligned_speed >= WHEEL_STARTUP_RELEASE_SPEED_MPS:
                self._startup_release_dwell_s[side] += max(0.0, float(dt_s))
                if self._startup_release_dwell_s[side] >= WHEEL_STARTUP_RELEASE_DWELL_S:
                    self._startup_active[side] = False
            else:
                self._startup_release_dwell_s[side] = 0.0
        startup_pwm = min(
            self.max_pwm,
            abs(
                float(
                    feedforward_diagnostics.get(
                        "startup_pwm",
                        abs(float(maintenance_pwm)),
                    )
                    or abs(float(maintenance_pwm))
                )
            ),
        )
        applied = bool(
            self._startup_active[side]
            and startup_pwm > abs(float(maintenance_pwm)) + 1e-12
        )
        effective = (
            math.copysign(max(abs(float(maintenance_pwm)), startup_pwm), reference)
            if self._startup_active[side]
            else float(maintenance_pwm)
        )
        return float(effective), {
            "startup_floor_active": bool(self._startup_active[side]),
            "startup_floor_applied": applied,
            "startup_pwm": float(startup_pwm),
            "maintenance_pwm": float(maintenance_pwm),
        }

    def compute(
        self,
        cycle_context: CycleContext,
        wheel_setpoint: WheelVelocitySetpoint,
        wheel_feedback: WheelFeedback,
    ) -> CandidateMotorOutput:
        """Compute exactly one candidate motor output from wheel-only inputs."""

        boundary_error = self._boundary_error(
            cycle_context,
            wheel_setpoint,
            wheel_feedback,
        )
        targets_zero = bool(
            abs(float(wheel_setpoint.left_target_mps)) <= 1e-12
            and abs(float(wheel_setpoint.right_target_mps)) <= 1e-12
        )
        if targets_zero and not boundary_error:
            self._last_sign = {"left": 0, "right": 0}
            self._startup_active = {"left": False, "right": False}
            return self._zero(wheel_setpoint, reason="ZERO_TARGET")
        if boundary_error:
            return self._zero(wheel_setpoint, reason=boundary_error)

        switch_reason = self._direction_switch_reason(
            cycle_context=cycle_context,
            left_target=wheel_setpoint.left_target_mps,
            right_target=wheel_setpoint.right_target_mps,
        )
        if switch_reason:
            return self._zero(
                wheel_setpoint,
                reason=switch_reason,
                reset_control=False,
            )

        left_ff, left_ff_diag = self.drive_ctrl.get_wheel_feedforward(
            "left",
            wheel_setpoint.left_target_mps,
        )
        right_ff, right_ff_diag = self.drive_ctrl.get_wheel_feedforward(
            "right",
            wheel_setpoint.right_target_mps,
        )
        if not bool(left_ff_diag.get("valid") and right_ff_diag.get("valid")):
            return self._zero(
                wheel_setpoint,
                reason="WHEEL_SPEED_MAP_UNAVAILABLE",
                diagnostics={
                    "left_feedforward": left_ff_diag,
                    "right_feedforward": right_ff_diag,
                },
            )

        left_effective, left_startup = self._startup_floor(
            side="left",
            reference_mps=wheel_setpoint.left_target_mps,
            measured_mps=wheel_feedback.left_mps,
            maintenance_pwm=left_ff,
            feedforward_diagnostics=left_ff_diag,
            dt_s=cycle_context.dt_control_s,
        )
        right_effective, right_startup = self._startup_floor(
            side="right",
            reference_mps=wheel_setpoint.right_target_mps,
            measured_mps=wheel_feedback.right_mps,
            maintenance_pwm=right_ff,
            feedforward_diagnostics=right_ff_diag,
            dt_s=cycle_context.dt_control_s,
        )
        left_maintenance = abs(float(left_ff_diag.get("maintenance_pwm", left_ff) or left_ff))
        right_maintenance = abs(float(right_ff_diag.get("maintenance_pwm", right_ff) or right_ff))
        left_pwm, right_pwm, pi_diag = self.wheel_pi.compute(
            left_reference_mps=wheel_setpoint.left_target_mps,
            right_reference_mps=wheel_setpoint.right_target_mps,
            left_measured_mps=wheel_feedback.left_mps,
            right_measured_mps=wheel_feedback.right_mps,
            dt_s=cycle_context.dt_control_s,
            feedforward_pwm_l=left_effective,
            feedforward_pwm_r=right_effective,
            maintenance_floor_pwm_l=left_maintenance,
            maintenance_floor_pwm_r=right_maintenance,
        )
        diagnostics = {
            **dict(pi_diag),
            "measurement_id": str(wheel_feedback.measurement_id),
            "measurement_source_timestamp": float(wheel_feedback.source_timestamp),
            "measurement_trust": float(wheel_feedback.combined_trust),
            "left_feedforward": left_ff_diag,
            "right_feedforward": right_ff_diag,
            "left_startup": left_startup,
            "right_startup": right_startup,
            "feedforward_map_applied": True,
            "speed_map_lookup_count": 2,
            "wheel_pi_update_count": 2,
        }
        return self._remember(
            self._output(
                wheel_setpoint,
                left_pwm=max(-self.max_pwm, min(self.max_pwm, left_pwm)),
                right_pwm=max(-self.max_pwm, min(self.max_pwm, right_pwm)),
                reason="WHEEL_SPEED_LOOP",
                diagnostics=diagnostics,
            )
        )

    def get_last_pid_diagnostics(self) -> dict[str, Any]:
        return copy.deepcopy(self._last_pid_diag)

    def get_last_control_monitor(self) -> dict[str, Any]:
        return self.get_last_pid_diagnostics()
