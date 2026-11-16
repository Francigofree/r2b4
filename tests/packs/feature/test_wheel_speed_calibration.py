import pytest

from tools.wheel_speed_calibration import _run_step, fit_rows, sweep_plan
from rig import resolved_config


def measurements(*, bad_direction=False, drift=False, settles=False, noisy=False):
    now = 1_000_000_000
    distance = {"left": 0., "right": 0.}
    for sign in (1, -1):
        for speed in (.15, .3):
            for tick in range(150):
                now += 20_000_000
                for side in ("left", "right"):
                    actual = speed * (1 + tick / 200 if drift else
                                      1.4 if settles and tick < 85 else 1.)
                    distance[side] += sign * actual * .02
                    yield {"side": side, "valid": True, "time_ns": now,
                           "measured_ns": now, "sequence": now, "target": sign * speed,
                           "output": sign * speed * 1.3, "distance": distance[side],
                           "quadrature_rejections": now if noisy and side == "left" else 0,
                           "edge_ns": now - 1_000_000,
                           "edge_direction": -sign if bad_direction else sign}


def test_four_physical_curves_use_displacement_and_final_duty():
    baseline = resolved_config().runtime.composition.live_control.control.speed_map
    speed_map, report = fit_rows(measurements(), baseline=baseline, required_maximum_mps=.29)
    assert report["status"] == "PASS"
    assert report["settle_s"] == 1.25
    assert len(speed_map["curves"]) == 4
    for curve in speed_map["curves"].values():
        assert curve["points"][0]["speed_mps"] == pytest.approx(.15)
        assert curve["points"][0]["pwm"] == pytest.approx(.195)
        assert curve["points"][-1]["speed_mps"] == pytest.approx(.3)
    assert report["common_maximum_mps"] == pytest.approx(.3)
    for original in baseline.curves:
        assert speed_map["curves"][original.name]["maintenance_pwm"] == original.maintenance_output
        assert speed_map["curves"][original.name]["startup_pwm"] == original.startup_output


@pytest.mark.parametrize("options", [{"bad_direction": True}, {"drift": True}])
def test_bad_direction_and_unsettled_motion_never_calibrate(options):
    baseline = resolved_config().runtime.composition.live_control.control.speed_map
    speed_map, report = fit_rows(measurements(**options), baseline=baseline, required_maximum_mps=.29)
    assert speed_map is None
    assert report["status"] == "INSUFFICIENT_EVIDENCE"


def test_missing_range_is_not_extrapolated_to_requested_top_speed():
    baseline = resolved_config().runtime.composition.live_control.control.speed_map
    speed_map, report = fit_rows(measurements(), baseline=baseline, required_maximum_mps=1.5)
    assert speed_map is None
    assert len(report["problems"]) == 4


def test_quadrature_noise_cannot_produce_an_activatable_map():
    baseline = resolved_config().runtime.composition.live_control.control.speed_map
    speed_map, report = fit_rows(measurements(noisy=True), baseline=baseline, required_maximum_mps=.29)
    assert speed_map is None
    assert not report["plateaus"]["left_forward"]
    assert not report["plateaus"]["left_reverse"]
    assert report["plateaus"]["right_forward"]
    assert report["rejected_measurements"]["left: QUADRATURE_REJECTION"] > 0


def test_late_settling_uses_proven_suffix_instead_of_transient_average():
    baseline = resolved_config().runtime.composition.live_control.control.speed_map
    speed_map, report = fit_rows(measurements(settles=True), baseline=baseline, required_maximum_mps=.29)
    assert report["status"] == "PASS"
    for plateaus in report["plateaus"].values():
        assert all(point["extra_settle_s"] > 0 and point["duration_s"] >= 1 for point in plateaus)
        assert plateaus[0]["speed_mps"] == pytest.approx(.15)


def test_sweep_preserves_canonical_commands_and_rejects_short_track():
    resolved = resolved_config()
    commands = sweep_plan([.15, .19], 3., 2., resolved)
    assert [command[1] for command in commands] == ["forward", "backward"] * 2
    assert all(command[-3:] == ["c", "50", "--json"] for command in commands)
    with pytest.raises(ValueError, match="free track"):
        sweep_plan([.3], 3., .5, resolved)
    with pytest.raises(ValueError, match="calibrated range"):
        sweep_plan([1.5], 3., 20., resolved)
    with pytest.raises(ValueError, match="free track"):
        sweep_plan([.15, .26, .35], 3.5, 2., resolved, direction="forward")
    with pytest.raises(ValueError, match="caster settling"):
        sweep_plan([.19], 2., 2., resolved)
    with pytest.raises(ValueError, match="settling time"):
        sweep_plan([.19], 3., 2., resolved, settle_s=float("nan"))


@pytest.mark.parametrize("shutdown_fails", (False, True))
def test_sweep_aborts_at_first_safety_stop_without_retry(monkeypatch, shutdown_fails):
    class Process:
        returncode = None
        terminated = False
        def poll(self): return self.returncode
        def terminate(self):
            self.terminated = True
            self.returncode = -15
        def wait(self, timeout): return self.returncode
    class Interface:
        stops = 0
        def read(self, resource):
            return {"runtime_running": True, "status": {"mission": {"mode": "TELEOP"},
                "safety_decision": "STOP", "safety_reason": "LIDAR_CLEARANCE_LOW", "tick_id": 1034}}
        def execute(self, action):
            assert action == "operator.runtime.stop"
            self.stops += 1
            if shutdown_fails:
                raise RuntimeError("runtime already stopped")
    process, interface = Process(), Interface()
    calls = []
    def start(*args, **kwargs):
        calls.append(args)
        return process
    monkeypatch.setattr("tools.wheel_speed_calibration.subprocess.Popen", start)
    with pytest.raises(RuntimeError, match="1034: LIDAR_CLEARANCE_LOW; no retry"):
        _run_step(["canonical-command"], 3.5, interface, None)
    assert len(calls) == 1 and process.terminated and interface.stops > 0
