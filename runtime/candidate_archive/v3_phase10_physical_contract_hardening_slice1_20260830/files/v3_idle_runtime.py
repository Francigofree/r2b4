#!/usr/bin/env python3
"""Explicit headless entrypoint for the zero-only V3 live cutover candidate."""

from __future__ import annotations

import json
import signal
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from v3.adapters.live_idle import (
    GpioBackend,
    GpioZeroWriterConfig,
    MotorChannelPhysicalConfig,
    PwmDecayMode,
)
from v3.composition.live_idle import LiveIdleComposition, LiveIdleConfig


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_HARDWARE_CONFIG = PROJECT_ROOT / "conf" / "hardver.json"


@dataclass(frozen=True, slots=True)
class LiveIdleRuntimeConfig:
    """Closed startup configuration for the headless IDLE owner loop."""

    composition: LiveIdleConfig
    tick_period_ns: int = 20_000_000

    def __post_init__(self) -> None:
        if (
            not isinstance(self.tick_period_ns, int)
            or isinstance(self.tick_period_ns, bool)
            or self.tick_period_ns <= 0
        ):
            raise ValueError("tick_period_ns must be a positive integer")


def _motor_channel_config(
    motors: object,
    side: str,
) -> MotorChannelPhysicalConfig:
    if not isinstance(motors, dict):
        raise ValueError("hardware config motorok must be an object")
    value = motors.get(side)
    if not isinstance(value, dict):
        raise ValueError(f"hardware config is missing motorok.{side}")
    invert = value.get("invert", False)
    if type(invert) is not bool:
        raise ValueError(f"hardware config motorok.{side}.invert must be bool")
    raw_decay_mode = value.get(
        "pwm_decay_mode",
        motors.get("pwm_decay_mode", PwmDecayMode.COAST.value),
    )
    try:
        decay_mode = PwmDecayMode(str(raw_decay_mode).strip().lower())
    except ValueError as exc:
        raise ValueError(
            f"hardware config motorok.{side}.pwm_decay_mode is invalid"
        ) from exc
    return MotorChannelPhysicalConfig(
        in1=value.get("gpio_in1"),
        in2=value.get("gpio_in2"),
        invert=invert,
        pwm_decay_mode=decay_mode,
    )


def load_live_idle_runtime_config(
    path: Path = DEFAULT_HARDWARE_CONFIG,
) -> LiveIdleRuntimeConfig:
    """Load hardware pins once and convert them to immutable V3 config."""

    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("hardware config root must be an object")
    motors = payload.get("motorok")
    writer = GpioZeroWriterConfig(
        left=_motor_channel_config(motors, "bal_oldal"),
        right=_motor_channel_config(motors, "jobb_oldal"),
    )
    return LiveIdleRuntimeConfig(LiveIdleConfig(writer))


def run_live_idle(
    backend: GpioBackend,
    config: LiveIdleRuntimeConfig,
    *,
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    max_ticks: int | None = None,
) -> int:
    """Run one sequential headless owner loop without an ACTIVE path."""

    if max_ticks is not None and (
        not isinstance(max_ticks, int) or isinstance(max_ticks, bool) or max_ticks < 0
    ):
        raise ValueError("max_ticks must be a non-negative integer or None")

    runtime = LiveIdleComposition(backend, config.composition)
    last_monotonic_ns = 0
    try:
        last_monotonic_ns = monotonic_ns()
        runtime.enter_idle()
        next_deadline_ns = last_monotonic_ns
        completed_ticks = 0
        while not stop_requested() and (max_ticks is None or completed_ticks < max_ticks):
            now_ns = monotonic_ns()
            remaining_ns = next_deadline_ns - now_ns
            if remaining_ns > 0:
                sleep(remaining_ns / 1_000_000_000.0)
                now_ns = monotonic_ns()
            if now_ns < last_monotonic_ns:
                raise RuntimeError("monotonic clock moved backwards")
            runtime.tick(now_ns)
            last_monotonic_ns = now_ns
            completed_ticks += 1
            next_deadline_ns = max(
                next_deadline_ns + config.tick_period_ns,
                now_ns + 1,
            )
    finally:
        try:
            final_monotonic_ns = monotonic_ns()
        except Exception:
            final_monotonic_ns = last_monotonic_ns + 1
        shutdown_ns = max(final_monotonic_ns, last_monotonic_ns + 1)
        runtime.close(shutdown_ns)
    return 0


def main() -> int:
    """Claim GPIO only after explicit execution of this entrypoint."""

    import lgpio

    stop = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    config = load_live_idle_runtime_config()
    return run_live_idle(lgpio, config, stop_requested=lambda: stop)


if __name__ == "__main__":
    raise SystemExit(main())
