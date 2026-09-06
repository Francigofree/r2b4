from dataclasses import FrozenInstanceError
import math

import pytest

from v3.adapters.live_encoder import (
    EncoderVelocityReading,
    NativeEncoderConfig,
    NativeEncoderSource,
)
from v3.adapters.live_imu import ImuHeadingReading, NativeImuConfig, NativeImuSource
from v3.adapters.live_lidar import (
    LidarHealthReading,
    LidarPoseReading,
    NativeLidarConfig,
    NativeLidarSource,
)
from v3.composition.live_inputs import (
    LiveInputComposition,
    LiveInputCompositionConfig,
)
from v3.contracts import RejectionReason, SafetyDecision, TickContext


class _Backend:
    def __init__(self, name: str, result: object, order: list[str]) -> None:
        self._name = name
        self._result = result
        self._order = order
        self.calls: list[TickContext] = []

    def read(self, context: TickContext):
        self.calls.append(context)
        self._order.append(self._name)
        if isinstance(self._result, Exception):
            raise self._result
        if callable(self._result):
            return self._result(context)
        return self._result


def _sources(
    context: TickContext,
    *,
    encoder_result: EncoderVelocityReading | Exception | None = None,
    lidar_revisions: dict[int, int] | None = None,
):
    order: list[str] = []
    encoder = _Backend(
        "encoder",
        encoder_result
        if encoder_result is not None
        else lambda read_context: EncoderVelocityReading(
            sequence=read_context.tick_id,
            captured_monotonic_ns=read_context.monotonic_ns,
            left_mps=0.1,
            right_mps=0.12,
            trust=0.9,
            stale=False,
            timing_valid=True,
        ),
        order,
    )
    imu = _Backend(
        "imu",
        lambda read_context: ImuHeadingReading(
            sequence=read_context.tick_id,
            captured_monotonic_ns=read_context.monotonic_ns,
            yaw_rad=math.pi / 4.0,
            omega_rad_s=0.1,
            confidence=0.9,
            calibration=3,
            stale=False,
            timing_valid=True,
        ),
        order,
    )
    lidar = _Backend(
        "lidar",
        lambda read_context: LidarHealthReading(
            revision=(lidar_revisions or {}).get(
                read_context.tick_id,
                read_context.tick_id,
            ),
            captured_monotonic_ns=read_context.monotonic_ns,
            measurement_age_ns=20,
            confidence=0.8,
            stale=False,
            timing_valid=True,
            pose=LidarPoseReading(
                x_m=0.25,
                y_m=0.0,
                yaw_rad=math.pi / 4.0,
                r_scale=1.0,
            ),
        ),
        order,
    )
    return (
        NativeEncoderSource(
            encoder,
            NativeEncoderConfig("KIT0085_ENCODER", minimum_trust=0.3),
        ),
        NativeImuSource(
            imu,
            NativeImuConfig(
                "BNO055_IMU",
                minimum_confidence=0.4,
                minimum_calibration=2,
            ),
        ),
        NativeLidarSource(
            lidar,
            NativeLidarConfig(
                "LIDAR_LOCALIZATION",
                minimum_confidence=0.3,
                maximum_measurement_age_ns=250_000_000,
            ),
        ),
        (encoder, imu, lidar),
        order,
    )


def _layer(result, name: str):
    return next(record.output for record in result.trace.layers if record.layer == name)


def test_live_input_composition_polls_three_sources_once_and_commits_one_stop():
    context = TickContext(0, 1_000)
    encoder, imu, lidar, backends, order = _sources(context)
    runtime = LiveInputComposition(encoder, imu, lidar)

    result = runtime.tick(context)

    assert order == ["encoder", "imu", "lidar"]
    assert tuple(tuple(backend.calls) for backend in backends) == (
        (context,),
        (context,),
        (context,),
    )
    assert tuple(record.layer for record in result.trace.layers) == tuple(
        f"L{number}" for number in range(1, 13)
    )
    assert result.trace.fault_layer is None
    assert _layer(result, "L3").yaw_rad == pytest.approx(math.pi / 4.0)
    assert _layer(result, "L3").x_m > 0.0
    assert _layer(result, "L4").map_revision == 1
    assert _layer(result, "L4").freshness_ns == 20
    assert result.final_actuation.safety_decision is SafetyDecision.STOP
    assert result.final_actuation.enabled is False
    assert result.final_actuation.left_output == 0.0
    assert result.final_actuation.right_output == 0.0
    assert runtime.zero_commits == (result.final_actuation,)
    assert not hasattr(runtime, "activate")


def test_live_input_lidar_pose_revision_is_never_reapplied():
    first_context = TickContext(0, 1_000)
    encoder, imu, lidar, _, _ = _sources(
        first_context,
        lidar_revisions={0: 7, 1: 7, 2: 6},
    )
    runtime = LiveInputComposition(encoder, imu, lidar)

    first = runtime.tick(first_context)
    duplicate = runtime.tick(TickContext(1, 20_001_000))
    out_of_order = runtime.tick(TickContext(2, 40_001_000))

    assert _layer(first, "L3").x_m > 0.0
    duplicate_l2 = _layer(duplicate, "L2")
    out_of_order_l2 = _layer(out_of_order, "L2")
    assert tuple(item.reason for item in duplicate_l2.rejected) == (
        RejectionReason.DUPLICATE,
    )
    assert tuple(item.reason for item in out_of_order_l2.rejected) == (
        RejectionReason.OUT_OF_ORDER,
    )
    assert "lidar_pose" not in {item.kind for item in duplicate_l2.accepted}
    assert "lidar_pose" not in {item.kind for item in out_of_order_l2.accepted}
    assert duplicate.trace.fault_layer is None
    assert out_of_order.trace.fault_layer is None
    assert all(
        result.final_actuation.safety_decision is SafetyDecision.STOP
        for result in (first, duplicate, out_of_order)
    )


def test_live_input_source_failure_closes_one_l12_fault_without_retry():
    context = TickContext(0, 1_000)
    encoder, imu, lidar, backends, order = _sources(
        context,
        encoder_result=OSError("injected encoder failure"),
    )
    runtime = LiveInputComposition(encoder, imu, lidar)

    result = runtime.tick(context)

    assert order == ["encoder"]
    assert tuple(tuple(backend.calls) for backend in backends) == ((context,), (), ())
    assert tuple(record.layer for record in result.trace.layers) == ("L12",)
    assert result.trace.fault_layer == "L0"
    assert result.final_actuation.safety_decision is SafetyDecision.FAULT
    assert result.final_actuation.reason == "L0_ERROR"
    assert result.final_actuation.enabled is False
    assert result.final_actuation.left_output == 0.0
    assert result.final_actuation.right_output == 0.0
    assert runtime.zero_commits == (result.final_actuation,)


def test_live_input_composition_config_is_immutable_and_source_roles_are_typed():
    context = TickContext(0, 1_000)
    encoder, imu, lidar, _, _ = _sources(context)
    config = LiveInputCompositionConfig()

    with pytest.raises(FrozenInstanceError):
        config.world_model = config.world_model
    with pytest.raises(TypeError, match="encoder_source"):
        LiveInputComposition(imu, imu, lidar)
    with pytest.raises(TypeError, match="context must be TickContext"):
        LiveInputComposition(encoder, imu, lidar).tick(1)
