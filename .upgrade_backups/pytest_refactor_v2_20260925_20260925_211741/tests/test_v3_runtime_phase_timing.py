from v3.runtime_performance import RuntimeTimingAccumulator


def test_phase_timing_snapshot_is_bounded_structured_and_noncausal():
    timing = RuntimeTimingAccumulator(20_000_000)
    for value in (1_000_000, 6_000_000, 11_000_000, 21_000_000):
        timing.observe_control_phase("L4", value)
    timing.observe_control_phase("PIPELINE_TOTAL", 23_000_000)

    evidence = timing.snapshot()
    phases = {item.name: item for item in evidence.control_phases}

    assert phases["L4"].count == 4
    assert phases["L4"].mean_ns == 9_750_000
    assert phases["L4"].max_ns == 21_000_000
    assert phases["L4"].over_5ms_count == 3
    assert phases["L4"].over_10ms_count == 2
    assert phases["L4"].over_20ms_count == 1
    assert phases["PIPELINE_TOTAL"].count == 1

    payload = evidence.as_dict()["control_phase_timing"]
    assert payload["schema"] == "R2B4_RUNTIME_PHASE_TIMING_V1"
    assert payload["causal_claim"] is False
    assert payload["pipeline_total_overlaps_layers"] is True
    assert payload["scope"] == "NORMAL_TICKS_ONLY"
    assert payload["phases"]["L4"]["count"] == 4


def test_unknown_phase_is_rejected_before_runtime_use():
    timing = RuntimeTimingAccumulator(20_000_000)
    try:
        timing.observe_control_phase("NOT_A_REAL_PHASE", 1)
    except ValueError as exc:
        assert "unknown control timing phase" in str(exc)
    else:
        raise AssertionError("unknown timing phase must be rejected")


def test_runtime_phase_timing_integration_surfaces_are_present():
    import inspect
    from v3.engine import TickEngine
    from v3.composition.native_control import NativeControlComposition
    from v3.composition.resident_live_control import ResidentLiveControlComposition
    from v3.composition.resident_physical_control import ResidentPhysicalControlComposition
    import v3_runtime

    assert hasattr(TickEngine, "set_timing_observer")
    assert hasattr(NativeControlComposition, "set_timing_observer")
    assert hasattr(ResidentLiveControlComposition, "set_timing_observer")
    assert hasattr(ResidentPhysicalControlComposition, "set_timing_observer")

    live_source = inspect.getsource(ResidentLiveControlComposition.tick_execution)
    for label in ("L0_READ", "COMMAND_SNAPSHOT", "PIPELINE_TOTAL", "POST_CONTROL"):
        assert label in live_source
    assert "PIPELINE_TOTAL" not in inspect.getsource(ResidentLiveControlComposition.shutdown_execution)

    runtime_source = inspect.getsource(v3_runtime.run_resident_physical_control)
    assert "timing.observe_control_phase" in runtime_source
    assert "runtime.set_timing_observer(None)" in runtime_source
