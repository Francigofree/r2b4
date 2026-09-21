#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('/home/alba/project_r2b4')


def rewrite(relpath: str, replacements: list[tuple[str, str]]) -> None:
    path = ROOT / relpath
    text = path.read_text(encoding='utf-8')
    for old, new in replacements:
        text = text.replace(old, new)
    path.write_text(text, encoding='utf-8')
    print(f'UPDATED {relpath}')


rewrite(
    'v3/composition/native_control.py',
    [
        (
            'from v3.contracts.planner import PlannerInput\n',
            'from v3.contracts.planner import PlannerInput, TrajectoryRolloutRequest\n',
        ),
        (
            '''        self._engine.restore(checkpoint.engine_last_context)\n\n    def close_inputs(self, inputs: TickInputs) -> TickInputs:\n''',
            '''        self._engine.restore(checkpoint.engine_last_context)\n\n    def _sync_planner_transport(\n        self,\n        request: TrajectoryRolloutRequest | None,\n        started_ns: int,\n    ) -> bool:\n        \"\"\"Synchronize the authority-free worker transport with L6-owned request state.\"\"\"\n        if request == self._transport_request:\n            return False\n        backend = self._rollout_backend\n        if self._transport_id is not None and backend is not None:\n            backend.abandon(self._transport_id)\n        self._transport_request = request\n        self._transport_id = None\n        self._transport_error = None\n        self._transport_started_ns = None\n        if request is None:\n            return False\n        try:\n            if backend is None:\n                raise RuntimeError(\"ASYNC_L6_BACKEND_MISSING\")\n            self._transport_id = backend.submit(request)\n            self._transport_started_ns = started_ns\n        except Exception as exc:\n            self._transport_error = f\"{type(exc).__name__}:{exc}\"[:256]\n        return True\n\n    def dispatch_pending_planner_request(self, monotonic_ns: int) -> bool:\n        \"\"\"Dispatch a request created by the completed tick without exposing a completion.\n\n        L6 remains the sole navigation-state owner. This method only advances the\n        runtime transport edge after L1-L12 has completed; worker output can still\n        become visible only through a later ``close_inputs`` call.\n        \"\"\"\n        if not self._closed_planner_mode:\n            return False\n        if (\n            not isinstance(monotonic_ns, int)\n            or isinstance(monotonic_ns, bool)\n            or monotonic_ns < 0\n        ):\n            raise ValueError(\"planner dispatch monotonic_ns must be non-negative int\")\n        request = self._navigation.pending_rollout_request\n        if request is not None and monotonic_ns < request.context.monotonic_ns:\n            raise ValueError(\"planner dispatch cannot precede request source time\")\n        return self._sync_planner_transport(request, monotonic_ns)\n\n    def close_inputs(self, inputs: TickInputs) -> TickInputs:\n''',
        ),
        (
            '''        event = PlannerInput(inputs.context)\n        request = self._navigation.pending_rollout_request\n        backend = self._rollout_backend\n        if request != self._transport_request:\n            if self._transport_id is not None and backend is not None:\n                backend.abandon(self._transport_id)\n            self._transport_request = request\n            self._transport_id = None\n            self._transport_error = None\n            self._transport_started_ns = None\n            if request is not None:\n                try:\n                    if backend is None:\n                        raise RuntimeError(\"ASYNC_L6_BACKEND_MISSING\")\n                    self._transport_id = backend.submit(request)\n                    # This tick is the first point at which the request actually\n                    # exists outside L6. Do not charge earlier scheduler/closure\n                    # latency against the worker's bounded completion budget.\n                    self._transport_started_ns = inputs.context.monotonic_ns\n                except Exception as exc:\n                    self._transport_error = f\"{type(exc).__name__}:{exc}\"[:256]\n        if request is not None:\n''',
            '''        event = PlannerInput(inputs.context)\n        request = self._navigation.pending_rollout_request\n        # Live production normally dispatches immediately after the tick that\n        # created this immutable request. Keep this fallback for replay/tests and\n        # any caller that does not own an explicit post-tick runtime edge.\n        self._sync_planner_transport(request, inputs.context.monotonic_ns)\n        backend = self._rollout_backend\n        if request is not None:\n''',
        ),
    ],
)

rewrite(
    'v3/composition/resident_live_control.py',
    [
        (
            '''    def close(self) -> None:\n        self._control.close()\n\n    def _preflight_is_fresh_for(self, context: TickContext) -> bool:\n''',
            '''    def close(self) -> None:\n        self._control.close()\n\n    def dispatch_pending_planner_request(self, monotonic_ns: int) -> bool:\n        \"\"\"Forward authority-free post-tick planner dispatch to the runtime edge.\"\"\"\n        return self._control.dispatch_pending_planner_request(monotonic_ns)\n\n    def _preflight_is_fresh_for(self, context: TickContext) -> bool:\n''',
        ),
    ],
)

rewrite(
    'v3/composition/resident_physical_control.py',
    [
        (
            '''    def checkpoint(self) -> NativeControlStateCheckpoint:\n        return self._live_control.checkpoint()\n\n    def set_timing_observer(\n''',
            '''    def checkpoint(self) -> NativeControlStateCheckpoint:\n        return self._live_control.checkpoint()\n\n    def dispatch_pending_planner_request(self, monotonic_ns: int) -> bool:\n        \"\"\"Dispatch a post-tick L6 request without changing layer authority.\"\"\"\n        return self._live_control.dispatch_pending_planner_request(monotonic_ns)\n\n    def set_timing_observer(\n''',
        ),
    ],
)

rewrite(
    'v3/runtime_performance.py',
    [
        (
            '''    \"POST_CONTROL\",\n)\n''',
            '''    \"POST_CONTROL\",\n    \"ASYNC_L6_DISPATCH\",\n)\n''',
        ),
    ],
)

rewrite(
    'v3_runtime.py',
    [
        (
            '''            control_completed_ns = time.perf_counter_ns()\n            if timing is not None:\n                timing.observe_control(control_completed_ns - control_started_ns)\n            observer_started_ns = control_completed_ns\n            if record_observer is not None:\n''',
            '''            control_completed_ns = time.perf_counter_ns()\n            if timing is not None:\n                timing.observe_control(control_completed_ns - control_started_ns)\n\n            # L6 may have created a new immutable rollout request during this\n            # completed tick. Dispatch it now, before observation callbacks and\n            # before the scheduler sleep. The worker still has no navigation\n            # authority and its result remains invisible until a later input\n            # closure freezes it into PlannerInput.\n            if (\n                last_result.trace.fault_layer is None\n                and last_result.final_actuation.safety_decision is not SafetyDecision.FAULT\n            ):\n                dispatch_ns = _read_monotonic_ns(monotonic_ns, previous_clock_ns)\n                previous_clock_ns = dispatch_ns\n                dispatch_started_ns = time.perf_counter_ns()\n                dispatched = runtime.dispatch_pending_planner_request(dispatch_ns)\n                dispatch_completed_ns = time.perf_counter_ns()\n                if timing is not None and dispatched:\n                    timing.observe_control_phase(\n                        \"ASYNC_L6_DISPATCH\",\n                        dispatch_completed_ns - dispatch_started_ns,\n                    )\n\n            observer_started_ns = time.perf_counter_ns()\n            if record_observer is not None:\n''',
        ),
    ],
)

rewrite(
    'tests/test_v3_async_completion_boundary_fix.py',
    [
        (
            '''        self.delay_calls = delay_calls\n        self.calls = 0\n\n    def submit(self, request):\n        self.calls = 0\n        return self.inner.submit(request)\n''',
            '''        self.delay_calls = delay_calls\n        self.calls = 0\n        self.submit_count = 0\n\n    def submit(self, request):\n        self.calls = 0\n        self.submit_count += 1\n        return self.inner.submit(request)\n''',
        ),
        (
            '''    assert result.final_actuation.safety_decision.value != \"FAULT\"\n    production.close()\n\n\ndef test_worker_timeout_is_closed_as_typed_planner_input_after_actual_submit():\n''',
            '''    assert result.final_actuation.safety_decision.value != \"FAULT\"\n    production.close()\n\n\ndef test_post_tick_dispatch_submits_pending_request_before_next_input_closure():\n    config = control_config()\n    backend = DelayedBackend(config.navigation, delay_calls=1)\n    production = NativeControlComposition(\n        RecordingMotorSink(),\n        config,\n        trajectory_rollout_backend=backend,\n    )\n    values = _explore_values()\n\n    production.run_tick(production.close_inputs(values[0]))\n    active = production.close_inputs(values[1])\n    result = production.run_tick(active)\n    assert result.trace.fault_layer is None\n    assert backend.submit_count == 0\n\n    source_ns = values[1].context.monotonic_ns\n    assert production.dispatch_pending_planner_request(source_ns + 1_000_000)\n    assert backend.submit_count == 1\n\n    next_tick = _retime(values[2], source_ns + 20_000_000)\n    closed = production.close_inputs(next_tick)\n    assert backend.submit_count == 1\n    assert closed.planner_input is not None\n    assert closed.planner_input.error is None\n    production.close()\n\n\ndef test_worker_timeout_is_closed_as_typed_planner_input_after_actual_submit():\n''',
        ),
    ],
)

print('R2B4 ASYNC L6 DISPATCH P0 UPGRADE APPLIED')
print('Installer intentionally ran no tests, gates, preflight checks, validation, or rollback.')
print('Optional manual full test: cd /home/alba/project_r2b4 && python3 -m pytest -q')
