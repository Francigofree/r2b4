"""Thin adapters around production runtime components; no parallel control logic."""

from __future__ import annotations

import dataclasses
import math
import platform
import sys
from pathlib import Path
from typing import Any, Dict, List, Protocol

from controller.localization_gate import (
    apply_localization_gate_to_command,
    evaluate_localization_gate,
)
from controller.heading_turn_controller import HeadingTurnController
from controller.motion_controller import MotionController, MotionControllerConfig
from controller.motion_guidance import MotionGuidance
from controller.motion_guidance_contract import (
    MotionGuidanceInput,
    PoseSnapshot,
    ResolvedMotionIntent,
    WorldModelSnapshot,
)
from controller.motion_platform_adapter import ServiceActuationAdapter, contract_dict
from controller.motion_platform_contract import (
    CycleContext,
    DriveCapabilities,
    MotionEnvelope,
    PhysicalMotionCommand,
    ServiceActuationRequest,
    WheelFeedback,
    WheelVelocitySetpoint,
)
from controller.motion_resolver import (
    build_resolved_motion_intent,
    limit_motion_proposals,
    resolve_motion_proposals,
)
from controller.motion_semantics_engine import MotionSemanticsEngine
from controller.motion_tick_context import MotionTickContext, Pose2D, Velocity, new_motion_tick_cache
from core.motion.speed_limits import SpeedLimitsRuntime
from middleware.ffp import PIDConfig
from motion_executor import MotionExecutor

from replayer.contracts import (
    ADAPTER_ID,
    LAYER_BOUNDARIES_SCHEMA_V21,
    LAYER_BOUNDARY_ORDER_V21,
    LAYER_L6_INTENT_RESOLVER,
    LAYER_L7A_MOTION_GUIDANCE,
    LAYER_L10B_SAFETY_GATE,
    LAYER_L8_MOTION_CONTROLLER,
    LAYER_L9_MOTION_EXECUTOR,
    LAYER_SERVICE_ACTUATION,
    PIPELINE_ADAPTER_ID,
    PIPELINE_ADAPTER_ID_V21,
    PIPELINE_FRAME_SCHEMA_V2,
    PIPELINE_STAGE_ORDER,
    SOURCE_MANIFEST_SCHEMA,
    ReplayerError,
    seal_payload,
    sha256_file,
)
from replayer.storage import read_json


CONFIG_FILES = (
    "conf/control_mode.json",
    "conf/fizika.json",
    "conf/speed_map.json",
    "conf/vezerles.json",
)

SOURCE_FILES = (
    ("config_manager.py", "PRODUCTION_DEPENDENCY"),
    ("cont.py", "RUNTIME_TAP_INTEGRATION"),
    ("motion_executor.py", "PRODUCTION_COMPONENT"),
    ("core/control_strategies.py", "PRODUCTION_DEPENDENCY"),
    ("controller/motion_kinematics.py", "PRODUCTION_DEPENDENCY"),
    ("controller/localization_gate.py", "PRODUCTION_PIPELINE_COMPONENT"),
    ("controller/motion_controller.py", "PRODUCTION_PIPELINE_COMPONENT"),
    ("controller/motion_guidance.py", "PRODUCTION_PIPELINE_COMPONENT"),
    ("controller/motion_guidance_contract.py", "PRODUCTION_PIPELINE_CONTRACT"),
    ("controller/heading_turn_controller.py", "PRODUCTION_PIPELINE_COMPONENT"),
    ("controller/motion_semantics_engine.py", "PRODUCTION_PIPELINE_COMPONENT"),
    ("controller/motion_resolver.py", "PRODUCTION_PIPELINE_COMPONENT"),
    ("controller/motion_schema.py", "PRODUCTION_DEPENDENCY"),
    ("controller/motion_tick_context.py", "PRODUCTION_PIPELINE_DEPENDENCY"),
    ("core/motion/speed_limits.py", "PRODUCTION_PIPELINE_DEPENDENCY"),
    ("middleware/ffp.py", "PRODUCTION_DEPENDENCY"),
    ("middleware/lidar_estim.py", "MATCHER_EVIDENCE_PRODUCER"),
    ("middleware/scan_matcher_contract.py", "MATCHER_CONTRACT"),
    ("middleware/scan_matching.py", "MATCHER_PRODUCTION_COMPONENT"),
    ("replayer/adapters.py", "REPLAY_ADAPTER"),
    ("replayer/capture.py", "CAPTURE_CONTRACT"),
    ("replayer/contracts.py", "EVIDENCE_CONTRACT"),
    ("replayer/replay.py", "REPLAY_ENGINE"),
    ("replayer/runtime_capture.py", "RUNTIME_TAP"),
    ("replayer/matcher_adapter.py", "MATCHER_REPLAY_ADAPTER"),
    ("replayer/storage.py", "INTEGRITY_STORAGE"),
)


def build_source_manifest(
    project_root: Path,
    *,
    adapter_id: str = ADAPTER_ID,
    component: str = "motion_executor.MotionExecutor",
) -> Dict[str, Any]:
    root = Path(project_root).resolve()
    files: List[Dict[str, Any]] = []
    missing: List[str] = []
    for rel, role in SOURCE_FILES:
        path = root / rel
        if not path.is_file():
            missing.append(rel)
            continue
        files.append(
            {
                "path": rel,
                "role": role,
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        )
    payload = {
        "schema": SOURCE_MANIFEST_SCHEMA,
        "component": str(component),
        "adapter_id": str(adapter_id),
        "python": platform.python_version(),
        "implementation": platform.python_implementation(),
        "platform": sys.platform,
        "files": files,
        "missing": missing,
    }
    return seal_payload(payload, "source_manifest_sha256")


def compare_source_manifests(captured: Dict[str, Any], current: Dict[str, Any]) -> Dict[str, Any]:
    captured_files = {
        str(row.get("path")): str(row.get("sha256"))
        for row in list(captured.get("files") or [])
        if isinstance(row, dict)
    }
    current_files = {
        str(row.get("path")): str(row.get("sha256"))
        for row in list(current.get("files") or [])
        if isinstance(row, dict)
    }
    changed = [
        path
        for path in sorted(set(captured_files) | set(current_files))
        if captured_files.get(path) != current_files.get(path)
    ]
    environment_fields = ("python", "implementation", "platform", "adapter_id")
    environment_changed = [
        field
        for field in environment_fields
        if str(captured.get(field, "")) != str(current.get(field, ""))
    ]
    return {
        "match": (
            not changed
            and not environment_changed
            and not captured.get("missing")
            and not current.get("missing")
        ),
        "changed_paths": changed,
        "changed_environment_fields": environment_changed,
        "captured_manifest_sha256": captured.get("source_manifest_sha256"),
        "current_manifest_sha256": current.get("source_manifest_sha256"),
    }


def executor_contract_from_instance(executor: MotionExecutor) -> Dict[str, Any]:
    if not isinstance(executor, MotionExecutor):
        raise ReplayerError("production_motion_executor_required")
    pid_cfg = getattr(executor, "drive_pid_cfg", None)
    if not dataclasses.is_dataclass(pid_cfg):
        raise ReplayerError("pid_config_dataclass_required")
    drive_ctrl = getattr(executor, "drive_ctrl", None)
    runtime_speed_map = dict(getattr(drive_ctrl, "speed_map", {}) or {})
    if not runtime_speed_map:
        raise ReplayerError("runtime_speed_map_missing")
    return {
        "adapter_id": ADAPTER_ID,
        "component": "motion_executor.MotionExecutor",
        "control_mode": str(executor.get_control_mode()),
        "constructor": {
            "pid_config": dataclasses.asdict(pid_cfg),
            "max_pwm": float(executor.max_pwm),
            "direction_switch_hold_s": float(executor.direction_switch_hold_s),
            "direction_switch_debounce_cycles": int(executor.direction_switch_debounce_cycles),
        },
        "runtime_bound_config": {"speed_map": runtime_speed_map},
    }


def motion_pipeline_contract_from_controller(controller: Any) -> Dict[str, Any]:
    """Build the legacy V2 semantic-stage contract for capture compatibility."""
    motion_controller = getattr(controller, "motion_controller", None)
    speed_limits = getattr(controller, "speed_limits", None)
    if not isinstance(motion_controller, MotionController):
        raise ReplayerError("production_motion_controller_required")
    if not isinstance(speed_limits, SpeedLimitsRuntime):
        raise ReplayerError("production_speed_limits_runtime_required")
    constructor = dataclasses.asdict(motion_controller.config)
    return {
        "adapter_id": PIPELINE_ADAPTER_ID,
        "components": [
            "controller.motion_resolver.limit_motion_proposals",
            "controller.motion_resolver.resolve_motion_proposals",
            "controller.motion_guidance.MotionGuidance",
            "controller.motion_semantics_engine.MotionSemanticsEngine",
            "controller.localization_gate.evaluate_localization_gate",
            "controller.localization_gate.apply_localization_gate_to_command",
            "controller.motion_controller.MotionController",
            "core.motion.speed_limits.SpeedLimitsRuntime",
            "motion_executor.MotionExecutor",
        ],
        "stage_order": list(PIPELINE_STAGE_ORDER),
        "motion_controller_constructor": constructor,
        "plant_adapter": {
            "adapter_id": "NONE",
            "available": False,
            "boundary": "PWM_TO_PHYSICAL_OBSERVATION",
        },
    }


def motion_layer_contract_from_controller(controller: Any) -> Dict[str, Any]:
    """Bind V2.1 replay to the sealed physical layer boundaries."""
    legacy = motion_pipeline_contract_from_controller(controller)
    return {
        **legacy,
        "adapter_id": PIPELINE_ADAPTER_ID_V21,
        "contract_version": "2.1",
        "components": [
            "controller.motion_resolver.resolve_motion_proposals",
            "controller.motion_guidance.MotionGuidance",
            "controller.motion_controller.MotionController",
            "motion_executor.MotionExecutor",
            "controller.motion_platform_adapter.ServiceActuationAdapter",
        ],
        "layer_order": list(LAYER_BOUNDARY_ORDER_V21),
        "replayable_layers": [
            LAYER_L6_INTENT_RESOLVER,
            LAYER_L7A_MOTION_GUIDANCE,
            LAYER_L8_MOTION_CONTROLLER,
            LAYER_L9_MOTION_EXECUTOR,
            LAYER_SERVICE_ACTUATION,
        ],
    }


def motion_executor_output_projection(
    executor_call: Dict[str, Any],
    output: Dict[str, Any],
) -> Dict[str, Any]:
    """Return the closed L9/service output fields available in V2 captures."""
    call = dict(executor_call or {})
    result = dict(output or {})
    method = str(call.get("method", "") or "")
    if method == "compute":
        setpoint = dict(call.get("wheel_setpoint") or {})
        return {
            "schema": "R2B4_CANDIDATE_MOTOR_OUTPUT_REPLAY_PROJECTION_V1",
            "contract_id": str(setpoint.get("contract_id", "") or ""),
            "candidate_output_id": f"candidate:{setpoint.get('wheel_setpoint_id', '')}",
            "wheel_setpoint_id": str(setpoint.get("wheel_setpoint_id", "") or ""),
            "physical_command_id": str(setpoint.get("physical_command_id", "") or ""),
            "resolved_id": str(setpoint.get("resolved_id", "") or ""),
            "cycle_id": str(setpoint.get("cycle_id", "") or ""),
            "left_pwm": float(result.get("left_pwm", result.get("pwm_l", 0.0)) or 0.0),
            "right_pwm": float(result.get("right_pwm", result.get("pwm_r", 0.0)) or 0.0),
            "output_reason": str(result.get("output_reason", "NONE") or "NONE"),
        }
    if method == "service_compute":
        reason = str(result.get("reason", result.get("output_reason", "NONE")) or "NONE")
        return {
            "schema": "R2B4_SERVICE_ACTUATION_OUTPUT_REPLAY_PROJECTION_V1",
            "left_pwm": float(result.get("left_pwm", result.get("pwm_l", 0.0)) or 0.0),
            "right_pwm": float(result.get("right_pwm", result.get("pwm_r", 0.0)) or 0.0),
            "accepted": bool(result.get("accepted", reason == "SERVICE_REQUEST_ACCEPTED")),
            "reason": reason,
        }
    raise ReplayerError(f"unsupported_executor_method:{method or 'MISSING'}")


def layer_boundaries_from_frame(frame: Dict[str, Any]) -> Dict[str, Any]:
    """Project the existing passive tap into explicit V2.1 layer contracts."""
    pipeline = dict(frame.get("pipeline") or {})
    stages = dict(pipeline.get("stages") or {})
    requested = dict(stages.get("requested_motion") or {})
    resolver = dict(stages.get("resolver") or {})
    guidance = dict(stages.get("guidance") or {})
    reference = dict(stages.get("reference") or {})
    executor = dict(stages.get("motion_executor") or {})
    l8_input = dict(reference.get("input") or {})
    l8_output = dict(reference.get("recorded_output") or {})
    executor_call = dict(frame.get("executor_call") or {})
    executor_output = dict(frame.get("recorded_executor_output") or {})
    l6_input = {
        "requested_motion": dict(requested.get("input") or {}),
        "resolver": dict(resolver.get("input") or {}),
        "cycle_context": dict(
            (guidance.get("input") or {}).get("cycle_context") or {}
        ),
    }
    l6_output = dict(resolver.get("recorded_output") or {})
    l7_input = dict(guidance.get("input") or {})
    l7_output = dict(guidance.get("recorded_output") or {})
    if not l6_input["requested_motion"] or not l6_input["resolver"] or not l6_input["cycle_context"]:
        raise ReplayerError("v21_l6_boundary_missing")
    if not l6_output or not l7_input or not l7_output:
        raise ReplayerError("v21_l7a_boundary_missing")
    if dict(l6_output.get("resolved_intent") or {}) != dict(
        l7_input.get("resolved_intent") or {}
    ):
        raise ReplayerError("v21_l6_l7a_lineage_mismatch")
    if dict(l7_output.get("physical_command") or {}) != dict(
        l8_input.get("physical_command") or {}
    ):
        raise ReplayerError("v21_l7a_l8_lineage_mismatch")
    if not l8_input or not l8_output:
        raise ReplayerError("v21_l8_boundary_missing")
    if dict(executor.get("input") or {}) != executor_call:
        raise ReplayerError("v21_l9_input_lineage_mismatch")
    if dict(executor.get("recorded_output") or {}) != executor_output:
        raise ReplayerError("v21_l9_output_lineage_mismatch")
    method = str(executor_call.get("method", "") or "")
    projection = motion_executor_output_projection(executor_call, executor_output)
    l9_available = method == "compute"
    service_available = method == "service_compute"
    candidate = projection if l9_available else {}
    service = projection if service_available else {}
    return {
        "schema": LAYER_BOUNDARIES_SCHEMA_V21,
        "layer_order": list(LAYER_BOUNDARY_ORDER_V21),
        "layers": {
            LAYER_L6_INTENT_RESOLVER: {
                "available": True,
                "replayable": True,
                "input": l6_input,
                "recorded_output": l6_output,
            },
            LAYER_L7A_MOTION_GUIDANCE: {
                "available": True,
                "replayable": True,
                "input": l7_input,
                "recorded_output": l7_output,
            },
            LAYER_L8_MOTION_CONTROLLER: {
                "available": True,
                "replayable": True,
                "input": l8_input,
                "recorded_output": l8_output,
            },
            LAYER_L9_MOTION_EXECUTOR: {
                "available": l9_available,
                "replayable": True,
                "unavailable_reason": "" if l9_available else "service_path_active",
                "input": executor_call if l9_available else {},
                "recorded_output": candidate,
            },
            LAYER_SERVICE_ACTUATION: {
                "available": service_available,
                "replayable": True,
                "unavailable_reason": "" if service_available else "normal_motion_path_active",
                "input": executor_call if service_available else {},
                "recorded_output": service,
            },
            LAYER_L10B_SAFETY_GATE: {
                "available": True,
                "replayable": False,
                "unavailable_reason": "raw_safety_snapshot_not_captured",
                "input": {
                    "candidate_output": projection,
                    "safety_lineage": dict(frame.get("safety_lineage") or {}),
                },
                "recorded_output": dict(frame.get("final_output") or {}),
            },
        },
    }


def motion_tick_context_capture_payload(context: MotionTickContext) -> Dict[str, Any]:
    if not isinstance(context, MotionTickContext):
        raise ReplayerError("production_motion_tick_context_required")

    def finite_or_none(value: Any) -> float | None:
        number = float(value)
        return float(number) if math.isfinite(number) else None

    return {
        "pose": {
            "x": float(context.pose.x),
            "y": float(context.pose.y),
            "theta_rad": float(context.pose.theta_rad),
        },
        "velocity": {
            "v_mps": float(context.velocity.v_mps),
            "omega_rad_s": float(context.velocity.omega_rad_s),
            "left_mps": float(context.velocity.left_mps),
            "right_mps": float(context.velocity.right_mps),
        },
        "front_clearance_m": finite_or_none(context.front_clearance_m),
        "left_clearance_m": finite_or_none(context.left_clearance_m),
        "right_clearance_m": finite_or_none(context.right_clearance_m),
        "emergency": bool(context.emergency),
        "target_visible": bool(context.target_visible),
        "target_distance_m": finite_or_none(context.target_distance_m),
        "target_bearing_rad": finite_or_none(context.target_bearing_rad),
        "lidar_seq": int(context.lidar_seq),
    }


def motion_guidance_input_from_capture_payload(
    payload: Dict[str, Any],
) -> MotionGuidanceInput:
    """Rehydrate only the immutable L7A input captured at the boundary."""

    source = dict(payload or {})
    try:
        return MotionGuidanceInput(
            resolved_intent=ResolvedMotionIntent(
                **dict(source["resolved_intent"])
            ),
            pose=PoseSnapshot(**dict(source["pose"])),
            world=WorldModelSnapshot(**dict(source["world"])),
            cycle_context=CycleContext(**dict(source["cycle_context"])),
            drive_capabilities=DriveCapabilities(
                **dict(source["drive_capabilities"])
            ),
            executed_left_mps=source.get("executed_left_mps"),
            executed_right_mps=source.get("executed_right_mps"),
            actual_linear_mps=source.get("actual_linear_mps"),
            actual_angular_dps=source.get("actual_angular_dps"),
            measured_left_mps=source.get("measured_left_mps"),
            measured_right_mps=source.get("measured_right_mps"),
            gyro_z_rad_s=source.get("gyro_z_rad_s"),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ReplayerError("guidance_input_contract_invalid") from exc


class PlantAdapter(Protocol):
    """Narrow future seam; V2 deliberately ships without a physical model."""

    adapter_id: str

    def step(
        self,
        *,
        pwm_output: Dict[str, float],
        dt_s: float,
        observation: Dict[str, Any],
    ) -> Dict[str, Any]: ...


class ProductionMotionExecutorAdapter:
    """Replays frames through the imported production MotionExecutor class."""

    def __init__(self, *, contract: Dict[str, Any], speed_map: Dict[str, Any]):
        if str(contract.get("adapter_id", "")) != ADAPTER_ID:
            raise ReplayerError("unsupported_adapter_contract")
        if str(contract.get("component", "")) != "motion_executor.MotionExecutor":
            raise ReplayerError("unsupported_production_component")
        constructor = dict(contract.get("constructor") or {})
        runtime_bound = dict(contract.get("runtime_bound_config") or {})
        runtime_speed_map = dict(runtime_bound.get("speed_map") or {})
        if not runtime_speed_map or runtime_speed_map != dict(speed_map):
            raise ReplayerError("captured_speed_map_contract_mismatch")
        pid_raw = dict(constructor.get("pid_config") or {})
        allowed_pid = {field.name for field in dataclasses.fields(PIDConfig)}
        unknown_pid = sorted(set(pid_raw) - allowed_pid)
        missing_pid = sorted(allowed_pid - set(pid_raw))
        if unknown_pid or missing_pid:
            raise ReplayerError(
                "pid_contract_mismatch:unknown=" + ",".join(unknown_pid) + ":missing=" + ",".join(missing_pid)
            )
        self.executor = MotionExecutor(
            pid_config=PIDConfig(**pid_raw),
            max_pwm=float(constructor["max_pwm"]),
            speed_map=runtime_speed_map,
            control_mode=str(contract["control_mode"]),
            direction_switch_hold_s=float(constructor["direction_switch_hold_s"]),
            direction_switch_debounce_cycles=int(constructor["direction_switch_debounce_cycles"]),
        )
        self._last_executor_reset_generation: int | None = None

    @classmethod
    def from_capture(cls, capture_path: Path, manifest: Dict[str, Any]) -> "ProductionMotionExecutorAdapter":
        speed_map = read_json(capture_path / "config" / "speed_map.json")
        return cls(contract=dict(manifest.get("executor_contract") or {}), speed_map=speed_map)

    def replay_frame(self, frame: Dict[str, Any]) -> Dict[str, Any]:
        if "executor_reset_generation" in frame:
            raw_generation = frame["executor_reset_generation"]
            if isinstance(raw_generation, bool):
                raise ReplayerError("executor_reset_generation_invalid")
            try:
                reset_generation = int(raw_generation)
            except (TypeError, ValueError) as exc:
                raise ReplayerError("executor_reset_generation_invalid") from exc
            if reset_generation < 0:
                raise ReplayerError("executor_reset_generation_invalid")
            previous_generation = self._last_executor_reset_generation
            if previous_generation is not None and reset_generation < previous_generation:
                raise ReplayerError("executor_reset_generation_decreased")
            if previous_generation is not None and reset_generation != previous_generation:
                self.executor.reset()
            self._last_executor_reset_generation = reset_generation
        call = dict(frame.get("executor_call") or {})
        method = str(call.get("method", "") or "")
        if method == "compute":
            output = self.executor.compute(
                CycleContext(**dict(call.get("cycle_context") or {})),
                WheelVelocitySetpoint(**dict(call.get("wheel_setpoint") or {})),
                WheelFeedback(**dict(call.get("wheel_feedback") or {})),
            )
            pwm_l = output.left_pwm
            pwm_r = output.right_pwm
            reason = output.output_reason
        elif method == "service_compute":
            output = ServiceActuationAdapter.compute(
                ServiceActuationRequest(**dict(call.get("request") or {})),
                monotonic_time=float(call.get("monotonic_time", 0.0)),
            )
            pwm_l = output.left_pwm
            pwm_r = output.right_pwm
            reason = output.reason
        else:
            raise ReplayerError(f"unsupported_executor_method:{method or 'MISSING'}")
        return {
            "pwm_l": float(pwm_l),
            "pwm_r": float(pwm_r),
            "output_reason": str(reason or "NONE"),
        }


def _optional_float(value: Any) -> float:
    if value is None:
        return math.nan
    return float(value)


def _motion_context_from_payload(payload: Dict[str, Any]) -> MotionTickContext:
    src = dict(payload or {})
    pose = dict(src.get("pose") or {})
    velocity = dict(src.get("velocity") or {})
    return MotionTickContext(
        pose=Pose2D(
            x=float(pose.get("x", 0.0) or 0.0),
            y=float(pose.get("y", 0.0) or 0.0),
            theta_rad=float(pose.get("theta_rad", 0.0) or 0.0),
        ),
        velocity=Velocity(
            v_mps=float(velocity.get("v_mps", 0.0) or 0.0),
            omega_rad_s=float(velocity.get("omega_rad_s", 0.0) or 0.0),
            left_mps=float(velocity.get("left_mps", 0.0) or 0.0),
            right_mps=float(velocity.get("right_mps", 0.0) or 0.0),
        ),
        front_clearance_m=_optional_float(src.get("front_clearance_m")),
        left_clearance_m=_optional_float(src.get("left_clearance_m")),
        right_clearance_m=_optional_float(src.get("right_clearance_m")),
        emergency=bool(src.get("emergency", False)),
        target_visible=bool(src.get("target_visible", False)),
        target_distance_m=_optional_float(src.get("target_distance_m")),
        target_bearing_rad=_optional_float(src.get("target_bearing_rad")),
        lidar_seq=int(src.get("lidar_seq", 0) or 0),
    )


class ProductionMotionPipelineAdapter:
    """Replay V2 stages through the imported production components."""

    def __init__(
        self,
        *,
        pipeline_contract: Dict[str, Any],
        executor_contract: Dict[str, Any],
        speed_map: Dict[str, Any],
        vezerles_config: Dict[str, Any],
        fizika_config: Dict[str, Any],
        plant_adapter: PlantAdapter | None = None,
    ) -> None:
        contract = dict(pipeline_contract or {})
        if str(contract.get("adapter_id", "")) not in {
            PIPELINE_ADAPTER_ID,
            PIPELINE_ADAPTER_ID_V21,
        }:
            raise ReplayerError("unsupported_pipeline_adapter_contract")
        if list(contract.get("stage_order") or []) != list(PIPELINE_STAGE_ORDER):
            raise ReplayerError("pipeline_stage_order_contract_mismatch")
        if (
            str(contract.get("adapter_id", "")) == PIPELINE_ADAPTER_ID_V21
            and list(contract.get("layer_order") or []) != list(LAYER_BOUNDARY_ORDER_V21)
        ):
            raise ReplayerError("layer_boundary_order_contract_mismatch")
        constructor = dict(contract.get("motion_controller_constructor") or {})
        required = {field.name for field in dataclasses.fields(MotionControllerConfig)}
        if set(constructor) != required:
            raise ReplayerError("motion_controller_constructor_contract_mismatch")
        self.motion_controller = MotionController(
            config=MotionControllerConfig(**constructor)
        )
        self.vezerles_config = dict(vezerles_config or {})
        self.fizika_config = dict(fizika_config or {})
        readiness_cfg = dict(
            self.vezerles_config.get("motion_readiness") or {}
        )
        self.motion_guidance = MotionGuidance(
            semantics=MotionSemanticsEngine(
                dict(readiness_cfg.get("motion_semantics") or {})
            ),
            heading_controller=HeadingTurnController(
                float(self.fizika_config.get("nyomtav_szelesseg_m", 0.175)),
                dict(readiness_cfg.get("heading_turn") or {}),
            ),
            policy_config=dict(
                self.vezerles_config.get("global_motion_policy") or {}
            ),
        )
        self.executor_adapter = ProductionMotionExecutorAdapter(
            contract=executor_contract,
            speed_map=speed_map,
        )
        self.plant_adapter = plant_adapter

    @staticmethod
    def _resolve_stage(
        *,
        requested_input: Dict[str, Any],
        resolver_input: Dict[str, Any],
    ) -> tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
        limited, limit_status = limit_motion_proposals(
            list(requested_input.get("proposals") or []),
            active_source=str(requested_input.get("active_source", "") or ""),
            category_caps=dict(requested_input.get("category_caps") or {}),
            max_total=int(requested_input.get("max_total", 8) or 8),
        )
        context = _motion_context_from_payload(
            dict(resolver_input.get("motion_tick_context") or {})
        )
        resolved, resolution_status = resolve_motion_proposals(
            limited,
            active_source=str(requested_input.get("active_source", "") or ""),
            context=context,
            cache=new_motion_tick_cache(context),
            proposal_limit_status=limit_status,
            now_monotonic=float(resolver_input["now_monotonic_s"]),
            now_wall=float(resolver_input["now_wall_s"]),
        )
        return (
            {
                "limited_motion_proposals": limited,
                "proposal_limit_status": limit_status,
            },
            {
                "resolved_motion": resolved,
                "resolution_status": resolution_status,
            },
            resolved,
        )

    def replay_resolver(
        self,
        *,
        requested_input: Dict[str, Any],
        resolver_input: Dict[str, Any],
        cycle_context: CycleContext,
    ) -> tuple[Dict[str, Any], Dict[str, Any]]:
        requested_output, resolver_output, resolved = self._resolve_stage(
            requested_input=dict(requested_input or {}),
            resolver_input=dict(resolver_input or {}),
        )
        resolver_output["resolved_intent"] = contract_dict(
            build_resolved_motion_intent(
                resolved,
                cycle_context=cycle_context,
            )
        )
        return requested_output, resolver_output

    def replay_guidance(
        self,
        guidance_input: Dict[str, Any],
    ) -> Dict[str, Any]:
        typed_input = motion_guidance_input_from_capture_payload(guidance_input)
        physical = self.motion_guidance.compute(typed_input)
        diagnostics = self.motion_guidance.diagnostics()
        if diagnostics is None:
            raise ReplayerError("guidance_diagnostics_missing")
        return {
            "physical_command": contract_dict(physical),
            "diagnostics": contract_dict(diagnostics),
        }

    @classmethod
    def from_capture(
        cls,
        capture_path: Path,
        manifest: Dict[str, Any],
        *,
        plant_adapter: PlantAdapter | None = None,
    ) -> "ProductionMotionPipelineAdapter":
        return cls(
            pipeline_contract=dict(manifest.get("pipeline_contract") or {}),
            executor_contract=dict(manifest.get("executor_contract") or {}),
            speed_map=read_json(capture_path / "config" / "speed_map.json"),
            vezerles_config=read_json(capture_path / "config" / "vezerles.json"),
            fizika_config=read_json(capture_path / "config" / "fizika.json"),
            plant_adapter=plant_adapter,
        )

    def _replay_reference(
        self,
        reference_input: Dict[str, Any],
        *,
        physical_command: Dict[str, Any],
        gate_status: Dict[str, Any],
        gate_apply: Dict[str, Any],
    ) -> Dict[str, Any]:
        payload = dict(reference_input or {})
        captured_physical = dict(payload.get("physical_command") or {})
        if captured_physical != dict(physical_command or {}):
            raise ReplayerError("guidance_l8_physical_lineage_mismatch")
        output = self.motion_controller.compute(
            CycleContext(**dict(payload.get("cycle_context") or {})),
            PhysicalMotionCommand(**dict(physical_command or {})),
            MotionEnvelope(**dict(payload.get("motion_envelope") or {})),
            DriveCapabilities(**dict(payload.get("drive_capabilities") or {})),
        )
        return contract_dict(output)

    def replay_frame(self, frame: Dict[str, Any]) -> Dict[str, Any]:
        pipeline = dict(frame.get("pipeline") or {})
        if str(pipeline.get("schema", "")) != PIPELINE_FRAME_SCHEMA_V2:
            raise ReplayerError("pipeline_frame_schema_invalid")
        if list(pipeline.get("stage_order") or []) != list(PIPELINE_STAGE_ORDER):
            raise ReplayerError("pipeline_frame_stage_order_invalid")
        stages = dict(pipeline.get("stages") or {})

        requested_input = dict(
            (stages.get("requested_motion") or {}).get("input") or {}
        )
        resolver_input = dict((stages.get("resolver") or {}).get("input") or {})
        guidance_input = dict((stages.get("guidance") or {}).get("input") or {})
        typed_guidance = motion_guidance_input_from_capture_payload(guidance_input)
        requested_output, resolver_output = self.replay_resolver(
            requested_input=requested_input,
            resolver_input=resolver_input,
            cycle_context=typed_guidance.cycle_context,
        )
        captured_intent = contract_dict(typed_guidance.resolved_intent)
        if dict(resolver_output["resolved_intent"]) != captured_intent:
            raise ReplayerError("resolver_guidance_intent_lineage_mismatch")
        guidance_output = self.replay_guidance(guidance_input)

        gate_input = dict((stages.get("localization_gate") or {}).get("input") or {})
        captured_runtime = dict(gate_input.get("runtime_state") or {})
        # The runtime can replace gate state between ticks (notably during an
        # atomic pose reset).  The captured pre-stage state is therefore the
        # production boundary for this frame; carrying only the previous replay
        # output would silently discard those external transitions.
        runtime_state = captured_runtime
        gate_status = evaluate_localization_gate(
            lidar_odom_status=dict(gate_input.get("lidar_odom_status") or {}),
            now_s=float(gate_input["now_s"]),
            moving_command=bool(gate_input.get("moving_command", False)),
            runtime_state=runtime_state,
            cfg=dict(gate_input.get("cfg") or {}),
        )
        gate_apply = apply_localization_gate_to_command(
            v_target=float(gate_input.get("v_target", 0.0) or 0.0),
            omega_target=float(gate_input.get("omega_target", 0.0) or 0.0),
            execution_mode=str(gate_input.get("execution_mode", "") or ""),
            requested_track_reference=dict(gate_input.get("requested_track_reference") or {}),
            gate_status=gate_status,
            track_width_m=float(gate_input.get("track_width_m", 0.175) or 0.175),
        )
        gate_output = {"gate_status": gate_status, "gate_apply": gate_apply}

        reference_output = self._replay_reference(
            dict((stages.get("reference") or {}).get("input") or {}),
            physical_command=dict(guidance_output["physical_command"]),
            gate_status=gate_status,
            gate_apply=gate_apply,
        )
        call = dict(frame.get("executor_call") or {})
        if str(call.get("method", "") or "") == "compute":
            call["wheel_setpoint"] = dict(reference_output)
        executor_frame = dict(frame)
        executor_frame["executor_call"] = call
        executor_output = self.executor_adapter.replay_frame(executor_frame)
        stage_outputs = {
            "requested_motion": requested_output,
            "resolver": resolver_output,
            "guidance": guidance_output,
            "localization_gate": gate_output,
            "reference": reference_output,
            "motion_executor": executor_output,
            "pwm": {
                "pwm_l": float(executor_output["pwm_l"]),
                "pwm_r": float(executor_output["pwm_r"]),
            },
        }
        plant_result: Dict[str, Any] = {
            "adapter_id": "NONE",
            "available": False,
            "reason": "physical_plant_model_not_installed",
        }
        if self.plant_adapter is not None:
            plant_result = dict(
                self.plant_adapter.step(
                    pwm_output=dict(stage_outputs["pwm"]),
                    dt_s=float(frame["dt_s"]),
                    observation=dict(pipeline.get("plant_observation") or {}),
                )
                or {}
            )
            plant_result["adapter_id"] = str(self.plant_adapter.adapter_id)
            plant_result["available"] = True
        return {
            "stage_outputs": stage_outputs,
            "executor_output": executor_output,
            "plant": plant_result,
        }


class ProductionMotionLayerAdapter:
    """Replay only the sealed V2.1 boundary selected by the caller."""

    def __init__(
        self,
        *,
        pipeline_contract: Dict[str, Any],
        executor_contract: Dict[str, Any],
        speed_map: Dict[str, Any],
        vezerles_config: Dict[str, Any],
        fizika_config: Dict[str, Any],
    ) -> None:
        contract = dict(pipeline_contract or {})
        if str(contract.get("adapter_id", "")) != PIPELINE_ADAPTER_ID_V21:
            raise ReplayerError("unsupported_v21_layer_adapter_contract")
        if list(contract.get("layer_order") or []) != list(LAYER_BOUNDARY_ORDER_V21):
            raise ReplayerError("v21_layer_order_contract_mismatch")
        self._pipeline = ProductionMotionPipelineAdapter(
            pipeline_contract=contract,
            executor_contract=executor_contract,
            speed_map=speed_map,
            vezerles_config=vezerles_config,
            fizika_config=fizika_config,
        )

    @classmethod
    def from_capture(
        cls,
        capture_path: Path,
        manifest: Dict[str, Any],
    ) -> "ProductionMotionLayerAdapter":
        return cls(
            pipeline_contract=dict(manifest.get("pipeline_contract") or {}),
            executor_contract=dict(manifest.get("executor_contract") or {}),
            speed_map=read_json(capture_path / "config" / "speed_map.json"),
            vezerles_config=read_json(capture_path / "config" / "vezerles.json"),
            fizika_config=read_json(capture_path / "config" / "fizika.json"),
        )

    @staticmethod
    def _boundary(frame: Dict[str, Any], layer: str) -> Dict[str, Any]:
        boundaries = dict(frame.get("layer_boundaries") or {})
        if str(boundaries.get("schema", "")) != LAYER_BOUNDARIES_SCHEMA_V21:
            raise ReplayerError("v21_layer_boundaries_schema_invalid")
        boundary = dict((boundaries.get("layers") or {}).get(layer) or {})
        if not boundary:
            raise ReplayerError(f"v21_layer_boundary_missing:{layer}")
        return boundary

    def replay_layer(self, frame: Dict[str, Any], layer: str) -> Dict[str, Any]:
        boundary = self._boundary(frame, layer)
        if not bool(boundary.get("available", False)):
            return {
                "available": False,
                "output": {},
                "reason": str(boundary.get("unavailable_reason", "") or "not_available"),
            }
        if layer == LAYER_L6_INTENT_RESOLVER:
            payload = dict(boundary.get("input") or {})
            _, output = self._pipeline.replay_resolver(
                requested_input=dict(payload.get("requested_motion") or {}),
                resolver_input=dict(payload.get("resolver") or {}),
                cycle_context=CycleContext(
                    **dict(payload.get("cycle_context") or {})
                ),
            )
            return {"available": True, "output": output, "reason": ""}
        if layer == LAYER_L7A_MOTION_GUIDANCE:
            return {
                "available": True,
                "output": self._pipeline.replay_guidance(
                    dict(boundary.get("input") or {})
                ),
                "reason": "",
            }
        if layer == LAYER_L8_MOTION_CONTROLLER:
            payload = dict(boundary.get("input") or {})
            output = self._pipeline.motion_controller.compute(
                CycleContext(**dict(payload.get("cycle_context") or {})),
                PhysicalMotionCommand(**dict(payload.get("physical_command") or {})),
                MotionEnvelope(**dict(payload.get("motion_envelope") or {})),
                DriveCapabilities(**dict(payload.get("drive_capabilities") or {})),
            )
            return {"available": True, "output": contract_dict(output), "reason": ""}
        if layer in {LAYER_L9_MOTION_EXECUTOR, LAYER_SERVICE_ACTUATION}:
            replay_frame = dict(frame)
            replay_frame["executor_call"] = dict(boundary.get("input") or {})
            output = self._pipeline.executor_adapter.replay_frame(replay_frame)
            return {
                "available": True,
                "output": motion_executor_output_projection(
                    replay_frame["executor_call"],
                    output,
                ),
                "reason": "",
            }
        raise ReplayerError(f"v21_layer_not_replayable:{layer}")

    def relevant_state(self, layer: str) -> Dict[str, Any]:
        if layer == LAYER_L6_INTENT_RESOLVER:
            return {"stateful": False}
        if layer == LAYER_L7A_MOTION_GUIDANCE:
            diagnostics = self._pipeline.motion_guidance.diagnostics()
            return {
                "semantics": self._pipeline.motion_guidance.semantics.status(),
                "guidance_reason": "" if diagnostics is None else diagnostics.reason,
            }
        if layer == LAYER_L8_MOTION_CONTROLLER:
            controller = self._pipeline.motion_controller
            return {
                "left_slew_mps": float(controller._left_slew_mps),
                "right_slew_mps": float(controller._right_slew_mps),
            }
        if layer == LAYER_L9_MOTION_EXECUTOR:
            adapter = self._pipeline.executor_adapter
            executor = adapter.executor
            return {
                "reset_generation": adapter._last_executor_reset_generation,
                "last_sign": dict(executor._last_sign),
                "pending_sign": dict(executor._pending_sign),
                "pending_count": dict(executor._pending_count),
                "direction_switch_hold_until": float(executor._direction_switch_hold_until),
                "startup_active": dict(executor._startup_active),
                "startup_release_dwell_s": dict(executor._startup_release_dwell_s),
                "last_control": executor.get_last_pid_diagnostics(),
            }
        if layer == LAYER_SERVICE_ACTUATION:
            return {"stateful": False}
        return {}


def current_config_comparison(project_root: Path, capture_path: Path) -> Dict[str, Any]:
    rows: List[Dict[str, Any]] = []
    for rel in CONFIG_FILES:
        name = Path(rel).name
        current = Path(project_root) / rel
        captured = capture_path / "config" / name
        current_hash = sha256_file(current) if current.is_file() else ""
        captured_hash = sha256_file(captured) if captured.is_file() else ""
        rows.append(
            {
                "path": rel,
                "captured_sha256": captured_hash,
                "current_sha256": current_hash,
                "match": bool(current_hash and current_hash == captured_hash),
            }
        )
    return {
        "match": all(bool(row["match"]) for row in rows),
        "changed_paths": [row["path"] for row in rows if not row["match"]],
        "files": rows,
        "replay_uses": "CAPTURED_IMMUTABLE_CONFIG",
    }
