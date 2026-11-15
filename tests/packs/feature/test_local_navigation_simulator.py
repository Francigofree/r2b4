"""Optional motorless edge evidence; ROS integration is an explicit CLI run."""
from copy import deepcopy
from pathlib import Path

import pytest

from tools.navigation_simulator.scenario import World,builtin,validate
from tools.navigation_simulator.runtime import Plant,VirtualMotorSink,parameter_overlay
from tools.navigation_simulator.experiments import compare,read_suite
from tools.tuners.r2b4_motion_tuner import resolve_config,PROJECT_ROOT,run_experiment


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
