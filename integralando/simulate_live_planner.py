#!/usr/bin/env python3
"""Source-first timing simulation for the R2B4 async L6 request lifecycle."""
from dataclasses import dataclass

MAX_AGE_MS = 350.0
TIMEOUT_MS = 300.0
REPLAN_MS = 100.0

@dataclass
class Result:
    fault: str | None
    fault_ms: float | None
    accepts: int
    max_accepted_source_age_ms: float


def run(periods_ms, worker_latency_ms, *, post_tick_dispatch, dispatch_delay_ms=5.5, ticks=250):
    now = 0.0
    plan_source = None
    pending_source = None
    submit = None
    done = None
    accepts = 0
    max_age = 0.0
    for tick in range(ticks):
        if tick:
            now += periods_ms[(tick - 1) % len(periods_ms)]

        if not post_tick_dispatch and pending_source is not None and submit is None:
            submit = now
            done = submit + worker_latency_ms

        has_result = False
        if pending_source is not None and submit is not None:
            if now - submit > TIMEOUT_MS:
                return Result('DEADLINE_MISSED', now, accepts, max_age)
            has_result = done is not None and now >= done

        if pending_source is not None:
            if has_result:
                age = now - pending_source
                max_age = max(max_age, age)
                if age > MAX_AGE_MS:
                    return Result('RESULT_STALE', now, accepts, max_age)
                plan_source = pending_source
                accepts += 1
                pending_source = None
                submit = None
                done = None
            elif plan_source is not None and now - plan_source > MAX_AGE_MS:
                return Result('CACHED_PLAN_STALE', now, accepts, max_age)
        elif plan_source is not None and now - plan_source > MAX_AGE_MS:
            return Result('CACHED_PLAN_STALE', now, accepts, max_age)

        if pending_source is None and (
            plan_source is None or now - plan_source >= REPLAN_MS
        ):
            pending_source = now

        if post_tick_dispatch and pending_source is not None and submit is None:
            submit = now + dispatch_delay_ms
            done = submit + worker_latency_ms

    return Result(None, None, accepts, max_age)


def show(name, periods, latency, delay=5.5):
    old = run(periods, latency, post_tick_dispatch=False)
    new = run(periods, latency, post_tick_dispatch=True, dispatch_delay_ms=delay)
    print(f'{name}: worker={latency:.1f} ms')
    print(f'  old: fault={old.fault} at={old.fault_ms} accepts={old.accepts}')
    print(f'  new: fault={new.fault} at={new.fault_ms} accepts={new.accepts}')


if __name__ == '__main__':
    # Newest 21:23 FULL-capture timing envelope observed in evidence:
    # p50 20.0076 ms, p95 39.3837 ms, p99 59.9923 ms, max 100.2963 ms.
    newest_full_like = [20.0076, 20.0, 39.3837, 20.0, 59.9923, 20.0, 20.0, 100.2963, 20.0, 20.0, 20.0, 20.0]
    # Earlier 20:37 FULL-capture envelope, retained as a second independent live profile.
    earlier_full_like = [20.0067, 20.0, 28.8896, 20.0, 44.9306, 20.0, 20.0, 45.8615, 20.0, 20.0]
    # 20:42 capture-OFF timing envelope: p99 30.18 ms, rare max 92.326 ms.
    off_typical = [20.0, 20.0, 24.4779, 20.0, 30.1800, 20.0, 20.0, 20.0]
    off_with_rare_max = [20.0, 20.0, 92.3262, 20.0, 20.0, 20.0, 20.0, 20.0]

    show('NEWEST FULL-like / 140 ms planner', newest_full_like, 140.0, 3.64)
    show('NEWEST FULL-like / 150 ms planner', newest_full_like, 150.0, 3.64)
    show('NEWEST FULL-like / 160 ms planner', newest_full_like, 160.0, 3.64)
    show('EARLIER FULL-like / 150 ms planner', earlier_full_like, 150.0)
    show('EARLIER FULL-like / 160 ms planner', earlier_full_like, 160.0)
    show('OFF-typical / 165 ms planner', off_typical, 165.0, 3.7)
    show('OFF rare-max / 150 ms planner', off_with_rare_max, 150.0, 3.7)
    show('continuity boundary / 180 ms planner', [20.0], 180.0, 3.7)
