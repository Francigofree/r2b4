import os
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from controller.slow_tick_diagnostics import (
    AsyncMotionGcWorker,
    MotionGcContract,
    SlowTickDiagnostics,
)
from controller.status import (
    _LatestOnlyStatusPublisher,
    _STATUS_JSON_READER,
    _full_lidar_scan_mode,
    write_loop_phase,
)


class SlowTickDiagnosticsTests(unittest.TestCase):
    class _FakeGc:
        def __init__(self):
            self.enabled = True
            self.callback = None
            self.collect_calls = []

        def isenabled(self):
            return bool(self.enabled)

        def disable(self):
            self.enabled = False

        def enable(self):
            self.enabled = True

        def collect(self, generation=2):
            self.collect_calls.append(int(generation))
            if self.callback is not None:
                self.callback("start", {"generation": int(generation)})
                self.callback(
                    "stop",
                    {"generation": int(generation), "collected": 3, "uncollectable": 0},
                )
            return 3

    @staticmethod
    def _gc_context(*, state="IDLE", pwm_zero=True, intent=False, task=False, motion=False):
        return {
            "state": str(state),
            "pwm_zero": bool(pwm_zero),
            "intent_active": bool(intent),
            "task_active": bool(task),
            "service_motion_active": False,
            "motion_active": bool(motion),
        }

    def test_motion_safe_gc_collects_once_after_startup_and_disables_automatic_gc(self):
        fake = self._FakeGc()
        contract = MotionGcContract(
            policy="motion_safe",
            idle_collect_interval_s=30.0,
            gc_module=fake,
        )
        fake.callback = contract.callback

        status = contract.initialize_after_startup(self._gc_context())

        self.assertFalse(fake.enabled)
        self.assertEqual(fake.collect_calls, [2])
        self.assertTrue(status["startup_full_collect_done"])
        self.assertEqual(status["authorized_collection_count"], 1)
        self.assertEqual(status["unowned_collection_count"], 0)
        self.assertEqual(status["contract_violation_count"], 0)

    def test_motion_safe_gc_defers_startup_collect_until_safe_idle_context(self):
        fake = self._FakeGc()
        now = [100.0]
        contract = MotionGcContract(
            policy="motion_safe",
            idle_collect_interval_s=30.0,
            gc_module=fake,
            clock=lambda: now[0],
        )
        fake.callback = contract.callback

        status = contract.initialize_after_startup(
            self._gc_context(state="STARTING", pwm_zero=True)
        )

        self.assertFalse(fake.enabled)
        self.assertEqual(fake.collect_calls, [])
        self.assertTrue(status["initialized"])
        self.assertFalse(status["startup_full_collect_done"])
        self.assertTrue(status["startup_full_collect_deferred"])
        self.assertEqual(status["contract_violation_count"], 0)

        now[0] = 101.0
        self.assertTrue(
            contract.maybe_collect_idle(
                self._gc_context(),
                now_mono_s=now[0],
                allow_interval_due=False,
            )
        )
        status = contract.status()
        self.assertEqual(fake.collect_calls, [2])
        self.assertTrue(status["startup_full_collect_done"])
        self.assertFalse(status["startup_full_collect_deferred"])
        self.assertEqual(status["authorized_collection_count"], 1)

    def test_motion_safe_gc_only_collects_after_strict_idle_contract(self):
        fake = self._FakeGc()
        now = [100.0]
        contract = MotionGcContract(
            policy="motion_safe",
            idle_collect_interval_s=30.0,
            gc_module=fake,
            clock=lambda: now[0],
        )
        fake.callback = contract.callback
        contract.initialize_after_startup(self._gc_context())
        contract.update_motion_context(
            self._gc_context(state="FORWARD", pwm_zero=False, intent=True, motion=True)
        )
        now[0] = 101.0

        self.assertFalse(
            contract.maybe_collect_idle(
                self._gc_context(state="IDLE", pwm_zero=True, intent=True, motion=True),
                now_mono_s=now[0],
            )
        )
        self.assertTrue(
            contract.maybe_collect_idle(self._gc_context(), now_mono_s=now[0])
        )
        self.assertEqual(fake.collect_calls, [2, 2])
        self.assertEqual(contract.status()["authorized_collection_count"], 2)

    def test_motion_safe_gc_interval_can_be_suppressed_for_async_window(self):
        fake = self._FakeGc()
        now = [100.0]
        contract = MotionGcContract(
            policy="motion_safe",
            idle_collect_interval_s=30.0,
            gc_module=fake,
            clock=lambda: now[0],
        )
        fake.callback = contract.callback
        contract.initialize_after_startup(self._gc_context())
        now[0] = 140.0

        self.assertFalse(
            contract.maybe_collect_idle(
                self._gc_context(),
                now_mono_s=now[0],
                allow_interval_due=False,
            )
        )
        self.assertEqual(fake.collect_calls, [2])

        contract.update_motion_context(
            self._gc_context(state="FORWARD", pwm_zero=False, intent=True, motion=True)
        )
        self.assertTrue(
            contract.maybe_collect_idle(
                self._gc_context(),
                now_mono_s=now[0],
                allow_interval_due=False,
            )
        )
        self.assertEqual(fake.collect_calls, [2, 2])

    def test_motion_safe_idle_maintenance_generation_is_configurable(self):
        fake = self._FakeGc()
        now = [100.0]
        contract = MotionGcContract(
            policy="motion_safe",
            idle_collect_interval_s=30.0,
            idle_maintenance_generation=0,
            gc_module=fake,
            clock=lambda: now[0],
        )
        fake.callback = contract.callback
        contract.initialize_after_startup(self._gc_context())
        contract.update_motion_context(
            self._gc_context(state="FORWARD", pwm_zero=False, intent=True, motion=True)
        )
        now[0] = 101.0

        self.assertTrue(
            contract.maybe_collect_idle(
                self._gc_context(),
                now_mono_s=now[0],
                allow_interval_due=False,
            )
        )

        status = contract.status()
        self.assertEqual(fake.collect_calls, [2, 0])
        self.assertEqual(status["idle_maintenance_generation"], 0)
        self.assertEqual(status["last_collection"]["generation"], 0)

    def test_async_motion_gc_worker_collects_latest_idle_without_fifo_backlog(self):
        fake = self._FakeGc()
        contract = MotionGcContract(
            policy="motion_safe",
            idle_collect_interval_s=30.0,
            gc_module=fake,
        )
        fake.callback = contract.callback
        contract.initialize_after_startup(self._gc_context())
        contract.update_motion_context(
            self._gc_context(state="FORWARD", pwm_zero=False, intent=True, motion=True)
        )
        worker = AsyncMotionGcWorker(
            contract,
            min_idle_s=0.0,
            max_idle_age_s=1.0,
            wake_interval_s=0.01,
        )
        try:
            worker.start()
            self.assertTrue(
                worker.submit_context(
                    self._gc_context(state="FORWARD", pwm_zero=False, intent=True, motion=True)
                )
            )
            for _ in range(20):
                worker.submit_context(self._gc_context())

            deadline = time.time() + 1.0
            while time.time() < deadline and len(fake.collect_calls) < 2:
                time.sleep(0.01)
        finally:
            worker.stop(timeout_s=1.0)

        status = worker.status()
        self.assertEqual(fake.collect_calls, [2, 2])
        self.assertEqual(contract.status()["authorized_collection_count"], 2)
        self.assertEqual(contract.status()["unowned_collection_count"], 0)
        self.assertEqual(status["async_worker"]["queue_capacity"], 1)
        self.assertTrue(status["async_worker"]["latest_only"])
        self.assertGreater(status["async_worker"]["submitted_count"], 1)
        self.assertGreaterEqual(status["async_worker"]["collected_count"], 1)

    def test_async_status_publisher_builds_latest_without_fifo_backlog(self):
        processed = []
        first_processing = threading.Event()

        def fake_publish(ctrl, now, curr, l_sum, pwm_l, pwm_r, v_l_raw=None, v_r_raw=None, **kwargs):
            processed.append(float(now))
            if float(now) == 1.0:
                first_processing.set()
                time.sleep(0.05)

        publisher = _LatestOnlyStatusPublisher(publish_fn=fake_publish)
        ctrl = SimpleNamespace(status_async_publisher_status={})
        try:
            self.assertTrue(publisher.submit(ctrl, 1.0, {}, {}, 0.0, 0.0))
            self.assertTrue(first_processing.wait(timeout=1.0))
            for idx in range(2, 20):
                self.assertTrue(publisher.submit(ctrl, float(idx), {}, {}, 0.0, 0.0))

            deadline = time.time() + 1.0
            while time.time() < deadline and (not processed or processed[-1] != 19.0):
                time.sleep(0.01)
        finally:
            publisher.stop(timeout_s=1.0)

        status = publisher.status()
        self.assertEqual(processed[0], 1.0)
        self.assertEqual(processed[-1], 19.0)
        self.assertLess(len(processed), 19)
        self.assertEqual(status["queue_capacity"], 1)
        self.assertTrue(status["latest_only"])
        self.assertGreater(status["dropped_superseded"], 0)

    def test_async_status_publisher_submit_does_not_block_on_busy_lock(self):
        publisher = _LatestOnlyStatusPublisher(publish_fn=lambda *_args, **_kwargs: None)
        ctrl = SimpleNamespace(status_async_publisher_status={})

        publisher._condition.acquire()
        try:
            self.assertFalse(publisher.submit(ctrl, 1.0, {}, {}, 0.0, 0.0))
        finally:
            publisher._condition.release()
            publisher.stop(timeout_s=0.1)

        status = publisher.status()
        self.assertEqual(status["submit_lock_miss"], 1)

    def test_motion_collection_is_counted_and_production_policy_fails_closed(self):
        fake = self._FakeGc()
        contract = MotionGcContract(policy="motion_safe", gc_module=fake)
        fake.callback = contract.callback
        contract.initialize_after_startup(self._gc_context())
        moving = self._gc_context(
            state="FORWARD", pwm_zero=False, intent=True, motion=True
        )
        contract.update_motion_context(moving)

        contract.callback("start", {"generation": 0})
        contract.callback("stop", {"generation": 0, "collected": 0, "uncollectable": 0})
        status = contract.status()

        self.assertEqual(status["motion_collection_count"], 1)
        self.assertEqual(status["last_violation"]["code"], "GC_FORBIDDEN_WHILE_MOTION_ACTIVE")
        self.assertTrue(status["fail_closed_active"])

    def test_motion_safe_policy_reverts_external_gc_enable_and_latches_stop(self):
        fake = self._FakeGc()
        contract = MotionGcContract(policy="motion_safe", gc_module=fake)
        fake.callback = contract.callback
        contract.initialize_after_startup(self._gc_context())
        fake.enable()

        contract.update_motion_context(
            self._gc_context(state="FORWARD", pwm_zero=False, intent=True, motion=True)
        )
        status = contract.status()

        self.assertFalse(fake.enabled)
        self.assertEqual(status["automatic_reenabled_count"], 1)
        self.assertEqual(status["last_violation"]["code"], "GC_AUTOMATIC_REENABLED")
        self.assertTrue(status["fail_closed_active"])

    def test_automatic_policy_keeps_ab_diagnostic_running_but_cannot_pass_m1_gate(self):
        fake = self._FakeGc()
        contract = MotionGcContract(policy="automatic", gc_module=fake)
        fake.callback = contract.callback
        contract.initialize_after_startup(self._gc_context())
        contract.update_motion_context(
            self._gc_context(state="FORWARD", pwm_zero=False, intent=True, motion=True)
        )

        fake.collect(0)
        status = contract.status()

        self.assertTrue(status["automatic_enabled"])
        self.assertEqual(status["motion_collection_count"], 1)
        self.assertTrue(status["motion_violation_latched"])
        self.assertFalse(status["fail_closed_active"])

    def test_slow_tick_record_keeps_correlation_fields(self):
        diag = SlowTickDiagnostics(target_hz=50.0)

        status = diag.observe(
            {
                "tick_id": 42,
                "tick_total_us": 25_000,
                "processing_total_us": 21_000,
                "lidar_processing_us": 7_000,
                "rolling_map_us": 1_500,
                "context_build_us": 400,
                "proposal_build_us": 800,
                "resolver_us": 6_500,
                "status_enqueue_us": 100,
                "logger_enqueue_us": 80,
                "gc_delta": {"gen0_collections": 1, "collections": 1, "pause_us": 750},
                "run_queue": {"load1": 0.5, "load5": 0.4, "load15": 0.3, "runnable": 2, "threads": 120},
                "sd_write_latency": 11.0,
                "sd_write_event_fresh": True,
                "sd_write_source": "logger_flush",
                "proposal_count": 6,
                "proposal_count_by_source": {"local_planner": 3, "safety": 1},
                "rejected_count": 2,
                "fallback_count": 1,
                "resolver_iterations": 4,
                "lidar_seq": 77,
            }
        )

        self.assertIsNotNone(status)
        self.assertEqual(status["slow_tick_count"], 1)
        self.assertEqual(status["slow_lidar_spike_count"], 1)
        self.assertEqual(status["slow_resolver_spike_count"], 1)
        self.assertEqual(status["slow_lidar_and_resolver_spike_count"], 1)
        self.assertEqual(status["slow_io_event_count"], 1)
        self.assertEqual(status["slow_gc_count"], 1)
        self.assertEqual(status["observed_tick_count"], 1)
        self.assertEqual(status["slow_multi_label_count"], 1)
        self.assertEqual(
            status["counter_semantics"]["primary_timing_class_counts"],
            "exclusive_one_per_slow_tick",
        )
        last = status["last_record"]
        self.assertEqual(last["proposal_count"], 6)
        self.assertEqual(last["proposal_count_by_source"]["local_planner"], 3)
        self.assertEqual(last["rejected_count"], 2)
        self.assertEqual(last["fallback_count"], 1)
        self.assertEqual(last["resolver_iterations"], 4)
        self.assertEqual(last["lidar_seq"], 77)
        self.assertEqual(last["sd_write_source"], "logger_flush")
        self.assertIn("io_event", last["coobserved_categories"])
        self.assertIn("gc_pause", last["coobserved_categories"])

    def test_stale_io_latency_and_short_gc_are_not_claimed_as_causes(self):
        diag = SlowTickDiagnostics(target_hz=50.0)

        status = diag.observe(
            {
                "tick_id": 7,
                "tick_total_us": 25_000,
                "processing_total_us": 21_000,
                "gc_delta": {"gen0_collections": 1, "collections": 1, "pause_us": 100},
                "sd_write_latency": 15.0,
                "sd_write_event_fresh": False,
            }
        )

        self.assertIsNotNone(status)
        self.assertEqual(status["slow_io_event_count"], 0)
        self.assertEqual(status["slow_gc_count"], 0)
        self.assertEqual(status["slow_unattributed_spike_count"], 1)

    def test_slow_period_with_processing_under_budget_is_scheduler_delay(self):
        diag = SlowTickDiagnostics(target_hz=50.0)

        status = diag.observe(
            {
                "tick_id": 8,
                "tick_total_us": 33_000,
                "processing_total_us": 15_000,
                "control_loop_us": 3_000,
            }
        )

        self.assertIsNotNone(status)
        self.assertEqual(status["slow_scheduler_delay_count"], 1)
        self.assertEqual(status["slow_unattributed_spike_count"], 0)
        self.assertTrue(bool(status["last_record"]["categories"]["scheduler_delay"]))
        self.assertEqual(
            status["last_record"]["primary_timing_class"],
            "SCHEDULER_DELAY_OBSERVED",
        )

    def test_slow_tick_missing_run_queue_does_not_read_proc_from_control_thread(self):
        diag = SlowTickDiagnostics(target_hz=50.0)

        with patch("builtins.open", side_effect=AssertionError("control thread file read")):
            status = diag.observe(
                {
                    "tick_id": 81,
                    "tick_total_us": 25_000,
                    "processing_total_us": 21_000,
                    "phase_durations_us": {"control_loop": 21_000},
                }
            )

        self.assertIsNotNone(status)
        self.assertEqual(
            status["last_record"]["run_queue"]["source"],
            "not_sampled_in_control_thread",
        )

    def test_write_loop_phase_queues_without_json_payload_on_caller(self):
        ctrl = SimpleNamespace(status_path="/tmp/status.json")

        with (
            patch("controller.status._LOOP_PHASE_PUBLISHER.submit", return_value=True) as submit,
            patch("controller.status._enqueue_status_json", side_effect=AssertionError("direct json enqueue")),
        ):
            self.assertTrue(write_loop_phase(ctrl, "cycle_start", cycle_id=7, now=12.0))

        submit.assert_called_once()

    def test_inner_timing_records_are_retained_only_above_20ms(self):
        diag = SlowTickDiagnostics(target_hz=50.0)

        self.assertIsNone(
            diag.observe(
                {
                    "tick_id": 11,
                    "tick_total_us": 20_000,
                    "processing_total_us": 4_000,
                    "_inner_timing_segments": [
                        ("control_loop.lidar_odometry", 1_200, 900),
                    ],
                }
            )
        )

        status = diag.observe(
            {
                "tick_id": 12,
                "tick_total_us": 20_001,
                "processing_total_us": 4_000,
                "_inner_timing_segments": [
                    ("control_loop.lidar_odometry", 1_200, 900),
                    ("motion_policy.apply", 2_100, 2_000),
                ],
            }
        )

        self.assertIsNotNone(status)
        self.assertEqual(status["retained_tick_threshold_us"], 20_000)
        inner = status["last_record"]["inner_timing"]
        self.assertEqual(inner["wall_clock"], "perf_counter_ns")
        self.assertEqual(inner["cpu_clock"], "thread_time_ns")
        self.assertEqual(inner["segment_count"], 2)
        self.assertEqual(inner["dominant_wall_segment"]["name"], "motion_policy.apply")
        self.assertEqual(status["inner_wall_max_us"]["motion_policy.apply"], 2_100)
        self.assertEqual(len(status["recent_records"]), 1)

    def test_motion_encoder_gap_keeps_exact_correlated_slow_tick(self):
        diag = SlowTickDiagnostics(target_hz=50.0)
        gap = {
            "motion_active": True,
            "gap_s": 0.084272,
            "dt_control_window_s": 0.084272,
            "dt_snapshot_s": 0.006676,
            "measurement_timestamp_s": 12068.783719,
        }

        status = diag.observe(
            {
                "tick_id": 81,
                "tick_total_us": 84_272,
                "processing_total_us": 9_000,
                "encoder_motion_timing_gap": gap,
            }
        )

        correlated = status["last_motion_timing_gap_record"]
        self.assertEqual(correlated["tick_id"], 81)
        self.assertEqual(correlated["primary_timing_class"], "SCHEDULER_DELAY_OBSERVED")
        self.assertEqual(
            correlated["encoder_gap_timing_class"],
            "PRECEDING_TICK_CONTEXT_UNAVAILABLE",
        )
        self.assertEqual(correlated["encoder_motion_timing_gap"], gap)

    def test_encoder_gap_does_not_attribute_current_tick_phase_to_existing_gap(self):
        diag = SlowTickDiagnostics(target_hz=50.0)
        status = diag.observe(
            {
                "tick_id": 82,
                "tick_total_us": 59_743,
                "processing_total_us": 37_574,
                "phase_durations_us": {
                    "control_loop": 11_938,
                    "proposal_resolution": 15_589,
                    "other": 10_047,
                },
                "preceding_tick_timing": {
                    "tick_id": 81,
                    "processing_total_us": 10_000,
                    "phase_durations_us": {"control_loop": 10_000},
                },
                "encoder_motion_timing_gap": {
                    "motion_active": True,
                    "gap_s": 0.059743,
                },
            }
        )

        correlated = status["last_motion_timing_gap_record"]
        self.assertEqual(
            correlated["primary_timing_class"],
            "PROCESSING_PHASE:proposal_resolution",
        )
        self.assertEqual(
            correlated["encoder_gap_timing_class"],
            "RESIDUAL_PERIOD_DELAY",
        )
        attribution = correlated["period_timing_attribution"]
        self.assertFalse(attribution["current_tick_phases_causal_for_observed_start_gap"])
        self.assertEqual(attribution["preceding_processing_contribution_us"], 0)
        self.assertEqual(attribution["residual_period_delay_us"], 39_743)

    def test_encoder_gap_keeps_preceding_inner_timing_breakdown(self):
        diag = SlowTickDiagnostics(target_hz=50.0)

        status = diag.observe(
            {
                "tick_id": 3707,
                "tick_total_us": 44_623,
                "processing_total_us": 23_222,
                "phase_durations_us": {
                    "control_loop": 5_216,
                    "motion_policy": 3_056,
                    "safety_supervisor": 7_793,
                },
                "preceding_tick_timing": {
                    "tick_id": 3706,
                    "processing_total_us": 41_789,
                    "phase_durations_us": {
                        "control_loop": 17_909,
                        "motion_policy": 13_422,
                        "safety_supervisor": 3_875,
                    },
                    "inner_timing": [
                        ("control_loop.lidar_odometry", 12_400, 11_800),
                        ("motion_policy.apply", 13_100, 12_700),
                    ],
                },
                "encoder_motion_timing_gap": {
                    "motion_active": True,
                    "gap_s": 0.044802,
                },
            }
        )

        correlated = status["last_motion_timing_gap_record"]
        preceding_inner = correlated["period_timing_attribution"]["preceding_inner_timing"]
        self.assertEqual(preceding_inner["segment_count"], 2)
        self.assertEqual(
            preceding_inner["dominant_wall_segment"]["name"],
            "motion_policy.apply",
        )
        self.assertEqual(preceding_inner["dominant_cpu_segment"]["cpu_us"], 12_700)

    def test_encoder_gap_partitions_preceding_overrun_and_residual_delay(self):
        diag = SlowTickDiagnostics(target_hz=50.0)
        status = diag.observe(
            {
                "tick_id": 4207,
                "tick_total_us": 59_743,
                "processing_total_us": 8_000,
                "phase_durations_us": {"control_loop": 8_000},
                "preceding_tick_timing": {
                    "tick_id": 4206,
                    "processing_total_us": 37_574,
                    "phase_durations_us": {
                        "control_loop": 11_938,
                        "proposal_resolution": 15_589,
                        "other": 10_047,
                    },
                    "phase_gc_pause_us": {
                        "control_loop": 0,
                        "proposal_resolution": 0,
                        "other": 0,
                    },
                    "gc_delta": {"collections": 0, "pause_us": 0},
                    "io_event": True,
                    "sd_write_event_fresh": True,
                    "sd_write_latency": 24.4486,
                    "sd_write_source": "status_json_writer",
                },
                "encoder_motion_timing_gap": {
                    "motion_active": True,
                    "gap_s": 0.059251,
                },
            }
        )

        correlated = status["last_motion_timing_gap_record"]
        self.assertEqual(
            correlated["encoder_gap_timing_class"],
            "MIXED_PRECEDING_PROCESSING_AND_RESIDUAL_PERIOD_DELAY",
        )
        attribution = correlated["period_timing_attribution"]
        self.assertEqual(attribution["preceding_tick_id"], 4206)
        self.assertEqual(attribution["period_over_target_us"], 39_743)
        self.assertEqual(
            attribution["preceding_processing_contribution_us"], 17_574
        )
        self.assertEqual(attribution["residual_period_delay_us"], 22_169)
        self.assertEqual(
            attribution["preceding_processing_contribution_us"]
            + attribution["residual_period_delay_us"],
            attribution["period_over_target_us"],
        )
        self.assertEqual(
            attribution["preceding_dominant_processing_phase"],
            "proposal_resolution",
        )
        self.assertTrue(attribution["preceding_io_event"])

    def test_partitioned_phases_produce_one_primary_class_and_nonexclusive_labels(self):
        diag = SlowTickDiagnostics(target_hz=50.0)

        status = diag.observe(
            {
                "tick_id": 9,
                "tick_total_us": 31_000,
                "processing_total_us": 25_000,
                "phase_durations_us": {
                    "control_loop": 4_000,
                    "lidar_context": 3_000,
                    "safety_executor_dispatch": 15_000,
                    "control_logging": 3_000,
                },
                "gc_delta": {"collections": 1, "pause_us": 800},
                "io_event": True,
            }
        )

        self.assertIsNotNone(status)
        last = status["last_record"]
        self.assertEqual(last["unattributed_processing_us"], 0)
        self.assertEqual(last["phase_coverage_ratio"], 1.0)
        self.assertEqual(
            last["primary_timing_class"],
            "PROCESSING_PHASE:safety_executor_dispatch",
        )
        self.assertEqual(last["dominant_processing_phase"], "safety_executor_dispatch")
        self.assertEqual(
            status["primary_timing_class_counts"],
            {"PROCESSING_PHASE:safety_executor_dispatch": 1},
        )
        self.assertEqual(status["slow_multi_label_count"], 1)
        self.assertEqual(status["coobserved_category_counts"]["processing_overrun"], 1)
        self.assertEqual(status["coobserved_category_counts"]["gc_pause"], 1)
        self.assertEqual(status["coobserved_category_counts"]["io_event"], 1)

    def test_dominant_gc_pause_is_primary_without_hiding_its_phase(self):
        diag = SlowTickDiagnostics(target_hz=50.0)

        status = diag.observe(
            {
                "tick_id": 10,
                "tick_total_us": 61_000,
                "processing_total_us": 41_000,
                "phase_durations_us": {
                    "control_loop": 3_000,
                    "executor_compute": 35_000,
                    "status_pose_publish": 3_000,
                },
                "phase_gc_pause_us": {
                    "control_loop": 0,
                    "executor_compute": 30_000,
                    "status_pose_publish": 0,
                },
                "gc_delta": {"collections": 1, "pause_us": 30_000},
            }
        )

        self.assertIsNotNone(status)
        last = status["last_record"]
        self.assertEqual(last["primary_timing_class"], "GC_PAUSE")
        self.assertEqual(last["dominant_processing_phase"], "executor_compute")
        self.assertEqual(last["phase_gc_pause_us"]["executor_compute"], 30_000)
        self.assertEqual(status["phase_gc_pause_max_us"]["executor_compute"], 30_000)

    def test_full_raw_lidar_scan_is_explicit_or_incident_only(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            ctrl = SimpleNamespace(
                status_path=os.path.join(tmpdir, "status.json"),
                log_capture_active=True,
                stop_status={},
                last_emergency_ts=0.0,
            )
            with patch.dict(os.environ, {"R2B4_LIDAR_FULL_SCAN": "0"}):
                self.assertEqual(_full_lidar_scan_mode(ctrl), "")
                ctrl.last_emergency_ts = time.time()
                self.assertEqual(_full_lidar_scan_mode(ctrl), "incident")

    def test_full_raw_lidar_scan_subscriber_uses_cached_async_runtime_read(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            ctrl = SimpleNamespace(
                status_path=os.path.join(tmpdir, "status.json"),
                stop_status={},
                last_emergency_ts=0.0,
            )
            request_path = os.path.join(tmpdir, "lidar_scan_subscriber.json")
            with _STATUS_JSON_READER._condition:
                _STATUS_JSON_READER._cache.pop(request_path, None)
                _STATUS_JSON_READER._pending.pop(request_path, None)
                _STATUS_JSON_READER._last_request_mono.pop(request_path, None)

            with patch.dict(os.environ, {"R2B4_LIDAR_FULL_SCAN": "0"}):
                with patch("builtins.open", side_effect=AssertionError("sync_open_forbidden")):
                    self.assertEqual(_full_lidar_scan_mode(ctrl), "")

                with _STATUS_JSON_READER._condition:
                    _STATUS_JSON_READER._cache[request_path] = {
                        "full_scan": True,
                        "expires_ts": time.time() + 10.0,
                    }
                self.assertEqual(_full_lidar_scan_mode(ctrl), "explicit_subscriber")


if __name__ == "__main__":
    unittest.main()
