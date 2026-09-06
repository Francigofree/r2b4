"""Owned native GPIO encoder source for the bounded V3 input edge."""

from __future__ import annotations

import math

from v3.contracts import TickContext

from .counter_encoder import (
    CounterEncoderBackendConfig,
    NativeCounterEncoderBackend,
)
from .gpio_counter import (
    GpioCounterBackend,
    GpioCounterPairConfig,
    NativeGpioSignedCounterPair,
)
from .live_encoder import NativeEncoderConfig, NativeEncoderSource
from .live_inputs import LiveDeviceSnapshot


class NativeGpioEncoderSource(NativeEncoderSource):
    """Own one counter pair behind the existing native encoder source port.

    All immutable configs are validated before the GPIO owner is opened. The
    source owns no clock, loop or concrete GPIO module; each read remains bound
    exclusively to its caller-provided ``TickContext``.
    """

    __slots__ = ("_counter_pair", "_source_closed")

    def __init__(
        self,
        gpio_backend: GpioCounterBackend,
        counter_config: GpioCounterPairConfig,
        backend_config: CounterEncoderBackendConfig,
        source_config: NativeEncoderConfig,
    ) -> None:
        if not isinstance(counter_config, GpioCounterPairConfig):
            raise TypeError("counter_config must be GpioCounterPairConfig")
        if not isinstance(backend_config, CounterEncoderBackendConfig):
            raise TypeError("backend_config must be CounterEncoderBackendConfig")
        if not isinstance(source_config, NativeEncoderConfig):
            raise TypeError("source_config must be NativeEncoderConfig")
        required_edge_capacity = math.ceil(
            backend_config.maximum_abs_velocity_mps
            * (backend_config.maximum_estimation_window_ns / 1_000_000_000.0)
            / min(
                backend_config.left_step_distance_m,
                backend_config.right_step_distance_m,
            )
        ) + 1
        if counter_config.edge_history_capacity < required_edge_capacity:
            raise ValueError(
                "edge_history_capacity cannot cover the configured velocity window"
            )

        counter_pair = NativeGpioSignedCounterPair(gpio_backend, counter_config)
        try:
            velocity_backend = NativeCounterEncoderBackend(
                counter_pair.left_counter,
                counter_pair.right_counter,
                backend_config,
            )
            super().__init__(velocity_backend, source_config)
        except Exception:
            counter_pair.close()
            raise

        self._counter_pair = counter_pair
        self._source_closed = False

    @property
    def closed(self) -> bool:
        return self._source_closed

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        if self._source_closed:
            raise RuntimeError("native GPIO encoder source is closed")
        return super().read(context)

    def close(self) -> None:
        """Stop counter mutation and release the sole GPIO owner once."""

        if self._source_closed:
            return
        self._source_closed = True
        self._counter_pair.close()


__all__ = ["NativeGpioEncoderSource"]
