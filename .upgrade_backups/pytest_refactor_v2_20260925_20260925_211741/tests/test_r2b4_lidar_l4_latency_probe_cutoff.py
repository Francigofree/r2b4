import r2b4_lidar_l4_latency_probe as probe


def _event(kind, event_ns, revision=7, **extra):
    value = {
        "schema": probe.SCHEMA,
        "tool_version": probe.TOOL_VERSION,
        "kind": kind,
        "event_monotonic_ns": event_ns,
        "revision": revision,
    }
    value.update(extra)
    return value


def test_publication_pairing_uses_tick_cutoff_not_l2_execution_time():
    events = [
        _event(
            "lidar_snapshot_update",
            100,
            measurement_monotonic_ns=50,
            scan_end_monotonic_ns=90,
        ),
        _event(
            "multirate_publication",
            120,
            publication_monotonic_ns=120,
            measurement_monotonic_ns=50,
            scan_end_monotonic_ns=90,
        ),
        # Same revision is republished after the tick cutoff but before L2 code
        # happens to execute. It cannot have fed this tick.
        _event(
            "multirate_publication",
            180,
            publication_monotonic_ns=180,
            measurement_monotonic_ns=50,
            scan_end_monotonic_ns=90,
        ),
        _event(
            "l2_admitted",
            200,
            event_override_monotonic_ns=200,
            tick_id=9,
            tick_monotonic_ns=150,
            measurement_monotonic_ns=50,
            scan_end_monotonic_ns=90,
        ),
        _event(
            "l4_accepted",
            210,
            event_override_monotonic_ns=210,
            tick_id=9,
            tick_monotonic_ns=150,
            measurement_monotonic_ns=50,
            scan_end_monotonic_ns=90,
        ),
    ]

    row = probe._build_rows(events)[0]
    assert row["publication_count_for_revision"] == 2
    assert row["publication_monotonic_ns"] == 120
    assert row["C_snapshot_to_publication_ns"] == 20
    assert row["D_publication_to_l2_ns"] == 80
