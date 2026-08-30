"""Runtime-free owner for the native V3 final motor-output capability."""

from __future__ import annotations

from v3.adapters.gpio_motor import (
    GpioMotorFrameSink,
    GpioMotorFrameSinkConfig,
    PwmGpioBackend,
)
from v3.adapters.motor_writer import NativeMotorWriter
from v3.contracts import FinalActuation


class NativeMotorOutputComposition:
    """Close one immutable physical config over one writer and one GPIO sink.

    This small composition is the sole lifecycle owner of the sink handle. It
    deliberately contains no layer pipeline, lifecycle transition or runtime
    loop and does not expose its internal frame-sink capability.
    """

    __slots__ = ("_sink", "_writer")

    def __init__(
        self,
        backend: PwmGpioBackend,
        config: GpioMotorFrameSinkConfig,
    ) -> None:
        if not isinstance(config, GpioMotorFrameSinkConfig):
            raise TypeError("config must be GpioMotorFrameSinkConfig")
        sink = GpioMotorFrameSink(backend, config)
        try:
            writer = NativeMotorWriter(config.left, config.right, sink)
        except Exception:
            sink.close()
            raise
        self._sink = sink
        self._writer = writer

    @property
    def closed(self) -> bool:
        return self._sink.closed

    @property
    def failed(self) -> bool:
        return self._sink.failed

    def write(self, command: FinalActuation) -> None:
        """Apply one final L12 command through the single owned sink."""

        if self.closed:
            raise OSError("the native motor output composition is closed")
        self._writer.write(command)

    def close(self) -> None:
        """Zero the physical output and release the owned handle once."""

        self._sink.close()


__all__ = ["NativeMotorOutputComposition"]
