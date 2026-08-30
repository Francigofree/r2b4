"""Manual-tick native live-input composition fixed to IDLE and zero output."""

from __future__ import annotations

from dataclasses import dataclass

from v3.adapters.live_encoder import NativeEncoderSource
from v3.adapters.live_imu import NativeImuSource
from v3.adapters.live_inputs import NativeLiveInputReader
from v3.adapters.live_lidar import NativeLidarSource
from v3.contracts import FinalActuation, TickContext
from v3.engine import TickResult
from v3.layers.l2_admission import AdmissionConfig
from v3.layers.l3_state_estimation import (
    NativeStateEstimator,
    NativeStateEstimatorConfig,
)
from v3.layers.l4_world_model import WorldModelConfig

from .input_shadow import InputShadowComposition


@dataclass(frozen=True, slots=True)
class LiveInputCompositionConfig:
    """Immutable L2-L4 configuration closed before source polling begins."""

    admission: AdmissionConfig = AdmissionConfig(max_sample_age_ns=250_000_000)
    estimation: NativeStateEstimatorConfig = NativeStateEstimatorConfig(
        frame_id="R2B4_BOOT_ROBOT_MAP",
        track_width_m=0.3557,
    )
    world_model: WorldModelConfig = WorldModelConfig()


class LiveInputComposition:
    """Poll the three native sources once and run one IDLE/STOP V3 tick.

    This composition owns no clock or owner loop: its caller supplies the closed
    ``TickContext``.  It has no activation API and its internal final sink rejects
    every enabled or non-zero actuation.
    """

    __slots__ = ("_reader", "_runtime")

    def __init__(
        self,
        encoder_source: NativeEncoderSource,
        imu_source: NativeImuSource,
        lidar_source: NativeLidarSource,
        config: LiveInputCompositionConfig = LiveInputCompositionConfig(),
    ) -> None:
        if not isinstance(encoder_source, NativeEncoderSource):
            raise TypeError("encoder_source must be NativeEncoderSource")
        if not isinstance(imu_source, NativeImuSource):
            raise TypeError("imu_source must be NativeImuSource")
        if not isinstance(lidar_source, NativeLidarSource):
            raise TypeError("lidar_source must be NativeLidarSource")
        if not isinstance(config, LiveInputCompositionConfig):
            raise TypeError("config must be LiveInputCompositionConfig")

        self._reader = NativeLiveInputReader(
            (encoder_source, imu_source, lidar_source)
        )
        self._runtime = InputShadowComposition(
            admission_config=config.admission,
            world_config=config.world_model,
            state_estimator=NativeStateEstimator(config.estimation),
        )

    @property
    def zero_commits(self) -> tuple[FinalActuation, ...]:
        """Return diagnostic values without exposing the writer capability."""

        return self._runtime.zero_commits

    def tick(self, context: TickContext) -> TickResult:
        """Close one source snapshot and exactly one fail-closed V3 decision."""

        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        try:
            batch = self._reader.read(context)
        except Exception:
            return self._runtime.run_fault_tick(context, "L0_ERROR", "L0")
        return self._runtime.run_batch(batch)


__all__ = ["LiveInputComposition", "LiveInputCompositionConfig"]
