"""Optional motorless edge evidence; ROS integration is an explicit CLI run."""
from copy import deepcopy
from pathlib import Path

import pytest

from tools.navigation_simulator.scenario import World,builtin,validate
from tools.navigation_simulator.runtime import Plant,VirtualMotorSink,parameter_overlay
from tools.navigation_simulator.experiments import compare,read_suite
from tools.tuners.r2b4_motion_tuner import resolve_config,PROJECT_ROOT,run_experiment


@pytest.mark.parametrize("translation,heading,continuous,discontinuity", [
    ("GOOD", "GOOD", True, False), ("DEGRADED", "GOOD", True, False),
    ("LOST", "GOOD", True, False), ("GOOD", "LOST", True, False),
    ("GOOD", "GOOD", False, False), ("GOOD", "GOOD", True, True),
])
def test_simulator_slam_odometry_matches_production_export(translation, heading, continuous, discontinuity):
    from collections import deque
    from v3.adapters.process_lidar_port import ProcessLidarPort
    from v3.contracts import (GLOBAL_FRAME_ID, LOCAL_FRAME_ID, LocalizationQuality,
        Pose2D, QualityState, RobotEstimate, TickContext)
    from tools.navigation_simulator.runtime import slam_odometry
    quality = LocalizationQuality(local_translation=QualityState(translation),
        heading=QualityState(heading), local_pose_continuous=continuous,
        pose_discontinuity=discontinuity, generation=4)
    estimate = RobotEstimate(TickContext(8, 2_040_000_000), GLOBAL_FRAME_ID,
        100., 200., 1., 0., 0., (0.,)*25,
        local_pose=Pose2D(LOCAL_FRAME_ID, 1., 2., .3),
        global_pose=Pose2D(GLOBAL_FRAME_ID, 100., 200., 1.),
        map_to_odom=Pose2D(GLOBAL_FRAME_ID, 99., 198., .7), localization_quality=quality)
    port = ProcessLidarPort.__new__(ProcessLidarPort)
    port._stopped, port._slam_active, port._slam_enabled = False, True, True
    port._pending_slam_odometry = deque(maxlen=1)
    port.publish_slam_odometry(estimate)
    packet = slam_odometry(estimate, 1, 2)
    assert (packet.tick_id, packet.monotonic_ns, packet.generation,
        packet.x_m, packet.y_m, packet.yaw_rad,
        packet.local_translation_valid, packet.heading_valid) == port._pending_slam_odometry[-1]


def test_world_uses_offset_rectangular_body_and_obstacle_motion():
    scene=builtin("moving_obstacle")
    scene["walls"]=[[1.5,-.5,1.6,.5]]
    world=World(validate(scene),(-.1,.26,-.18,.18))
    assert world.ray(0,0,0,0)==pytest.approx(1.5)
    assert world.clearance(1.3,0,0,0)==0
    assert world.clearance(0,0,0,0)>0
    assert world.circles(10)[0]["y_m"]==pytest.approx(0)
    # Rotating the off-center rectangle changes its wall clearance.
    assert world.clearance(0,1.7,0,0)>world.clearance(0,1.7,1.5707963267948966,0)


def test_unknown_interface_and_unsafe_ros_overrides_are_rejected():
    scene=builtin()
    with pytest.raises(ValueError):
        validate({**scene,"invented_sensor_contract":True})
    with pytest.raises(ValueError):
        validate({**scene,"faults":[{"kind":"process_exit","target":"robot_runtime","start_s":0.1}]})
    with pytest.raises(ValueError):
        parameter_overlay({"robot_radius":.3},{"robot_radius":.05})
    with pytest.raises(ValueError):
        parameter_overlay({"plugin":"real"},{"plugin":"fake"})
    assert parameter_overlay({"weight":2.},{"weight":3.})=={"weight":3.}


def test_real_v3_sensor_edge_preserves_delayed_lineage_without_truth_fix(tmp_path):
    from v3.adapters.slam_toolbox_transport import SlamLocalizationReceiver
    from v3.composition.native_control import NativeControlComposition
    from types import SimpleNamespace
    resolved=resolve_config(PROJECT_ROOT)
    config=resolved.runtime.composition.live_control.control
    sink=VirtualMotorSink()
    composition=NativeControlComposition(sink,config)
    records=[]
    scene=builtin()
    sender=SimpleNamespace(receive=lambda:())
    receiver=SlamLocalizationReceiver(1,500_000_000)
    plant=Plant(scene,config,composition,sink,tmp_path,1_000_000_000,lambda record,scan:records.append(record),lambda *args:None,
        world=World(scene,resolved.lidar.safety_body_bounds_m),resolved=resolved,sender=sender,receiver=receiver,
        local_perception_config=resolved.runtime.sensor_inputs.inputs.lidar_source)
    try:
        plant.step()
        first={row.kind:row for row in records[-1].inputs.raw_devices.samples}
        assert "lidar_pose" not in first and "lidar_relative_motion" not in first
        safety={field.key:field.value for field in first["lidar_safety_clearance"].values}
        assert safety["clearance_basis"]==config.lidar_safety.clearance_basis
        measured=first["lidar_local_points"].captured_monotonic_ns
        sequence=first["lidar_local_points"].sequence
        plant.faults={"lidar_drop"}
        for _ in range(20):
            plant.step()
        latest={row.kind:row for row in records[-1].inputs.raw_devices.samples}
        assert latest["lidar_local_points"].captured_monotonic_ns==measured
        assert latest["lidar_local_points"].sequence==sequence
        assert sink.last.left_output==sink.last.right_output==0
        # The bounded sink retains one command rather than a growing history.
        assert sink.write_count==len(records)
    finally:
        composition.close()
    # Persist a real sampled capture and prove that native replay refuses it.
    from v3_process_runtime import McapCaptureSession
    from v3.mcap_capture import McapCaptureConfig
    from tools.navigation_simulator.evidence import replay
    session=McapCaptureSession("sampled",tmp_path/"sampled.mcap",configuration={"production_control":config},
        config=McapCaptureConfig(mode="append_only",tick_sample_hz=5))
    session.start()
    for record in records:
        session.observe(record)
    capture=session.finalize(SimpleNamespace(status=0))
    blocked=replay(capture,PROJECT_ROOT,tmp_path/"sampled_replay.json")
    assert blocked["status"]=="BLOCKED" and "CAPTURE_INCOMPLETE" in blocked["reason"]


def test_direct_mcap_analysis_replay_and_corruption_fail_closed(tmp_path):
    from tools.navigation_simulator.evidence import analyze,replay,scenario_from_mcap
    resolved=resolve_config(PROJECT_ROOT)
    trial=run_experiment(resolved,"straight",tmp_path/"source",scenario={
        "kind":"differential_drive","behavior":"motion","duration_s":.12,"goal":{"x_m":.2,"y_m":0.}},replay=True)
    report=tmp_path/"report"
    report.mkdir()
    result=analyze(trial["capture"],PROJECT_ROOT,report,compile_reports=False)
    assert result["status"]=="PASS" and result["replay_eligible"]
    assert result["measurements"]["sample_count"]>0
    assert replay(trial["capture"],PROJECT_ROOT,report/"replay.json")["status"]=="MATCH"
    with pytest.raises(ValueError,match="SCENARIO_BLOCKED"):
        scenario_from_mcap(trial["capture"])
    capture=Path(trial["capture"])
    broken=tmp_path/"broken.mcap"
    payload=bytearray(capture.read_bytes())
    payload[len(payload)//2]^=1
    broken.write_bytes(payload)
    rejected=tmp_path/"rejected"
    rejected.mkdir()
    try:
        result=analyze(broken,PROJECT_ROOT,rejected,compile_reports=False)
    except RuntimeError:
        pass  # Native reader can reject malformed structure before inspection.
    else:
        assert result["status"]=="BLOCKED"


def _result():
    return {"schema":"R2B4_LOCAL_NAVIGATION_RESULT_V1","backend":"production_v3_slam_nav2",
            "scenario_sha256":"same","source_sha256":"old","status":"PASS","physical_output":False,
            "final_stop":True,"replay":{"status":"MATCH"},"scenario":builtin(),
            "metrics":{"goal_error_m":.01,"active_duration_s":10.,"collision_count":0,"path_length_m":1.}}


def test_comparison_never_rewards_a_short_failed_or_unpaired_run():
    baseline=_result()
    current=deepcopy(baseline)
    current["status"]="FAIL"
    current["metrics"]["active_duration_s"]=.1
    assert compare(baseline,current)["status"]=="REGRESSED"
    current=deepcopy(baseline)
    current["scenario_sha256"]="different"
    assert compare(baseline,current)["status"]=="INCONCLUSIVE"
    current=deepcopy(baseline)
    current["source_sha256"]="new"
    assert compare(baseline,current)["status"]=="UNCHANGED"
    current["replay"]["status"]="BLOCKED"
    assert compare(baseline,current)["status"]=="INCONCLUSIVE"


def test_default_search_has_independent_geometry():
    suite=read_suite(None)
    assert suite["train"][0]["room"]!=suite["holdout"][0]["room"]


def _complete_result():
    result=_result()
    result.update(virtual_body_stopped=True,failures=[],termination="GOAL_COMPLETE")
    result["metrics"].update(active_samples=100,goal_reached=True,longest_stationary_s=.1,unique_coverage_cells=4)
    return result


def test_tuning_rejects_truncated_evidence_and_does_not_reward_loops():
    from tools.navigation_simulator.experiments import safe_evidence,score
    result=_complete_result()
    assert safe_evidence(result)
    result.update(termination="DURATION_REACHED")
    result["metrics"].update(active_duration_s=.02,active_samples=1)
    assert not safe_evidence(result)
    first=_complete_result()
    first["scenario"]["behavior"]="roomcruise"
    second=deepcopy(first)
    second["metrics"]["path_length_m"]*=100
    assert score(first)==score(second)


def test_autotune_freezes_train_winner_before_holdout_and_never_reselects(tmp_path,monkeypatch):
    from tools.navigation_simulator import runtime
    from tools.navigation_simulator.experiments import search
    calls=[]
    def measure(root,directory,scene,*,domain,overlay=None):
        result=_complete_result()
        result["scenario"]=scene
        result["scenario_sha256"]="train" if "train" in directory.parts else "holdout"
        variant=overlay.get("vezerles",{}).get("layers",{}).get("motion_realization",{}).get("heading_gain") if overlay else None
        calls.append(("holdout" if "holdout" in directory.parts else "train",variant))
        if variant is not None:
            result["metrics"]["active_duration_s"]=8 if variant==1.1 else 6
        if "holdout" in directory.parts and variant is not None:
            result.update(status="FAIL",termination="DURATION_REACHED",failures=["TRUTH_GOAL_NOT_REACHED"])
            result["metrics"].update(active_duration_s=scene["duration_s"],goal_reached=False)
        return result
    monkeypatch.setattr(runtime,"run",measure)
    patches=[{"vezerles":{"layers":{"motion_realization":{"heading_gain":value}}}} for value in (1.1,1.2)]
    result=search(PROJECT_ROOT,tmp_path,read_suite(None),patches)
    assert result["frozen_train_winner"]==1 and result["best_index"] is None
    assert calls==[("train",None),("train",1.1),("train",1.2),("holdout",None),("holdout",1.2)]
    assert not (tmp_path/"best_override.json").exists()
