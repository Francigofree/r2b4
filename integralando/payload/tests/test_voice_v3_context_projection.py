import v3_process_runtime as process
from v3.adapters.fake_edges import FakeCommandGateway, FakeHal
from v3.composition.stop_only import StopOnlyComposition


def test_tick_status_exposes_read_only_l4_l5_l6_summary():
    hal = FakeHal()
    runtime = StopOnlyComposition(hal, FakeCommandGateway(), hal)
    runtime.enter_idle()
    payload = process._tick_status(runtime.tick(1_000_000_000))
    assert "world" in payload
    assert "mission" in payload
    assert "navigation" in payload
    if payload["world"] is not None:
        assert "person_tracks" in payload["world"]
