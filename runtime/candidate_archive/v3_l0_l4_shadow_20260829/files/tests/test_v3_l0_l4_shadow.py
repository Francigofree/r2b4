from dataclasses import replace
import math
from pathlib import Path

import pytest

from v3.adapters.capture_edges import (
    ENCODER_DEVICE_ID,
    ESTIMATE_DEVICE_ID,
    LIDAR_DEVICE_ID,
    ReplayerV1InputAdapter,
)
from v3.composition import InputShadowComposition
from v3.contracts import (
    AcquisitionFrame,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    RejectionReason,
    SafetyDecision,
    TickContext,
)
from v3.layers.l2_admission import AdmissionConfig, InputAdmission
from v3.layers.l3_state_estimation import StateEstimatorConfig
from v3.replay import first_divergence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CAPTURE_EXCERPT = PROJECT_ROOT / "tests" / "fixtures" / "v3_l0_l4_capture_excerpt.jsonl"


def _layer(trace, name):
    return next(record.output for record in trace.layers if record.layer == name)


def test_l0_closes_canonical_postpromotion_capture_excerpt_into_typed_batches():
    # Byte-preserved frames 227-231 from capture_m1_postpromote_20260822T1938Z.
    batches = ReplayerV1InputAdapter().load(CAPTURE_EXCERPT)

    assert tuple(batch.context.tick_id for batch in batches) == (227, 228, 229, 230, 231)
    assert len(batches) == 5
    assert tuple(sample.kind for sample in batches[0].samples) == (
        "wheel_velocity",
        "ekf_heading",
        "lidar_health",
    )
    assert tuple(item.device_id for item in batches[0].device_health) == (
        ENCODER_DEVICE_ID,
        ESTIMATE_DEVICE_ID,
        LIDAR_DEVICE_ID,
    )
    assert all(item.state is DeviceHealthState.OK for item in batches[0].device_health)
    assert batches[0].samples[0].values == (
        DataField("left_mps", 0.012786),
        DataField("right_mps", 0.019179),
        DataField("trust", 0.831),
    )
    assert tuple(batch.samples[2].sequence for batch in batches) == (0, 0, 0, 0, 0)


def test_l2_owns_sequence_freshness_alignment_and_health_history():
    admission = InputAdmission(
        AdmissionConfig(max_sample_age_ns=100, max_future_skew_ns=10)
    )
    health = (DeviceHealth("encoder", DeviceHealthState.OK),)

    def frame(tick_id, now_ns, sequence, captured_ns, io_health=health):
        context = TickContext(tick_id, now_ns)
        return AcquisitionFrame(
            context,
            (
                DeviceSample(
                    "encoder",
                    "wheel_velocity",
                    sequence,
                    captured_ns,
                    (DataField("left_mps", 0.0),),
                ),
            ),
            io_health,
        )

    accepted = admission(frame(0, 1_000, 5, 950))
    duplicate = admission(frame(1, 1_100, 5, 1_050))
    out_of_order = admission(frame(2, 1_200, 4, 1_150))
    stale = admission(frame(3, 1_300, 6, 1_000))
    future = admission(frame(4, 1_400, 7, 1_500))
    untrusted = admission(frame(5, 1_500, 8, 1_500, ()))

    assert len(accepted.accepted) == 1
    assert tuple(item.reason for item in duplicate.rejected) == (RejectionReason.DUPLICATE,)
    assert tuple(item.reason for item in out_of_order.rejected) == (
        RejectionReason.OUT_OF_ORDER,
    )
    assert tuple(item.reason for item in stale.rejected) == (RejectionReason.STALE,)
    assert tuple(item.reason for item in future.rejected) == (
        RejectionReason.TIME_ALIGNMENT_FAILED,
    )
    assert tuple(item.reason for item in untrusted.rejected) == (
        RejectionReason.UNTRUSTED,
    )
    assert untrusted.degraded_sources == ("encoder",)


def test_real_capture_replay_is_deterministic_and_commits_only_zero_shadow_output():
    batches = ReplayerV1InputAdapter().load(CAPTURE_EXCERPT)
    first_runtime = InputShadowComposition()
    second_runtime = InputShadowComposition()

    first = first_runtime.replay(batches)
    second = second_runtime.replay(batches)

    assert first == second
    assert first_divergence(first, second) is None
    assert len(first) == len(first_runtime.zero_commits) == 5
    assert all(
        commit.enabled is False
        and commit.left_output == 0.0
        and commit.right_output == 0.0
        and commit.safety_decision is SafetyDecision.STOP
        for commit in first_runtime.zero_commits
    )
    assert all(tuple(record.layer for record in trace.layers) == tuple(
        f"L{index}" for index in range(1, 13)
    ) for trace in first)

    final_estimate = _layer(first[-1], "L3")
    final_world = _layer(first[-1], "L4")
    assert final_estimate.frame_id == "R2B4_BOOT_ROBOT_MAP"
    assert final_estimate.x_m > 0.0
    assert final_estimate.y_m < 0.0
    assert final_estimate.yaw_rad == pytest.approx(math.radians(-0.11405439333417759))
    assert final_estimate.v_mps == pytest.approx(0.081005)
    assert final_world.map_revision == 1
    assert final_world.obstacle_tracks == ()
    assert final_world.freshness_ns > 80_000_000


def test_real_capture_config_change_reports_l3_as_first_diverging_layer():
    batches = ReplayerV1InputAdapter().load(CAPTURE_EXCERPT)
    expected = InputShadowComposition().replay(batches)
    actual = InputShadowComposition(
        estimator_config=StateEstimatorConfig(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            track_width_m=0.3557,
            initial_position_variance=0.08,
        )
    ).replay(batches)

    divergence = first_divergence(expected, actual)

    assert divergence is not None
    assert divergence.tick_id == 227
    assert divergence.layer == "L3"


def test_missing_captured_wheel_input_fails_closed_at_l3_with_one_zero_commit():
    first_batch = ReplayerV1InputAdapter().load(CAPTURE_EXCERPT)[0]
    missing_wheel = replace(
        first_batch,
        samples=tuple(sample for sample in first_batch.samples if sample.kind != "wheel_velocity"),
    )
    runtime = InputShadowComposition()

    traces = runtime.replay((missing_wheel,))

    assert len(traces) == len(runtime.zero_commits) == 1
    assert traces[0].fault_layer == "L3"
    assert tuple(record.layer for record in traces[0].layers) == ("L1", "L2", "L12")
    final = runtime.zero_commits[0]
    assert final.safety_decision is SafetyDecision.FAULT
    assert final.reason == "L3_ERROR"
    assert final.left_output == final.right_output == 0.0


def test_l0_rejects_noncontiguous_capture_before_any_layer_runs(tmp_path):
    lines = CAPTURE_EXCERPT.read_text(encoding="utf-8").splitlines()
    broken = tmp_path / "broken.jsonl"
    broken.write_text("\n".join((lines[0], lines[2])) + "\n", encoding="utf-8")

    with pytest.raises(ValueError, match="contiguous"):
        ReplayerV1InputAdapter().load(broken)


def test_missing_lidar_snapshot_is_typed_unknown_then_l4_recovers_next_tick(tmp_path):
    lines = CAPTURE_EXCERPT.read_text(encoding="utf-8").splitlines()
    lines[0] = lines[0].replace(
        '"lidar_latest_age_s":0.00678288299968699',
        '"lidar_latest_age_s":null',
    )
    capture = tmp_path / "missing-lidar.jsonl"
    capture.write_text("\n".join(lines[:2]) + "\n", encoding="utf-8")

    batches = ReplayerV1InputAdapter().load(capture)
    first_lidar_health = next(
        item for item in batches[0].device_health if item.device_id == LIDAR_DEVICE_ID
    )
    runtime = InputShadowComposition()
    traces = runtime.replay(batches)

    assert first_lidar_health.state is DeviceHealthState.UNKNOWN
    assert all(sample.kind != "lidar_health" for sample in batches[0].samples)
    assert traces[0].fault_layer == "L4"
    assert traces[1].fault_layer is None
    assert tuple(record.layer for record in traces[1].layers) == tuple(
        f"L{index}" for index in range(1, 13)
    )
    assert all(commit.left_output == commit.right_output == 0.0 for commit in runtime.zero_commits)
