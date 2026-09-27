"""Read-only/observation execution through the canonical RobotInterface.

P0 intentionally does not perform image understanding here. It proves that the
canonical calibrated camera can be requested while V3 is off without creating a
second camera owner or robot-control path.
"""
from __future__ import annotations

import os
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


class ObservationInterface(Protocol):
    def execute(self, action: str, **parameters: object) -> object: ...


@dataclass(frozen=True, slots=True)
class ObservationExecution:
    status: str
    observation_name: str
    completed: bool
    spoken_text: str
    artifact: str | None = None
    detail: str | None = None


class ObservationExecutor:
    """Execute supported observation capabilities without V3 authority."""

    def __init__(
        self,
        interface: ObservationInterface,
        *,
        temp_dir: str | Path = "/tmp",
        monotonic_ns=time.monotonic_ns,
    ) -> None:
        if not callable(getattr(interface, "execute", None)):
            raise TypeError("interface must provide execute()")
        if not callable(monotonic_ns):
            raise TypeError("monotonic_ns must be callable")
        self._interface = interface
        self._temp_dir = Path(temp_dir)
        self._monotonic_ns = monotonic_ns

    def execute(self, observation_name: str) -> ObservationExecution:
        if observation_name != "camera.photo":
            return ObservationExecution(
                "REJECTED:OBSERVATION_NOT_SUPPORTED",
                str(observation_name),
                False,
                "Ezt a megfigyelést most nem tudom elvégezni.",
            )
        self._temp_dir.mkdir(parents=True, exist_ok=True)
        output = self._temp_dir / (
            f"r2b4_voice_observation_{os.getpid()}_{self._monotonic_ns()}.jpg"
        )
        try:
            raw = self._interface.execute("camera.photo", output=str(output))
        except Exception as exc:
            return ObservationExecution(
                "OBSERVATION_FAILED",
                observation_name,
                False,
                "A kameraképet most nem tudom elkészíteni.",
                detail=f"{type(exc).__name__}: {exc}",
            )
        if not output.is_file() or output.stat().st_size <= 0:
            return ObservationExecution(
                "OBSERVATION_FAILED",
                observation_name,
                False,
                "A kameraképet most nem tudom elkészíteni.",
                detail="camera.photo returned without a non-empty artifact",
            )
        detail = None
        if isinstance(raw, Mapping):
            detail = str(raw.get("status") or raw.get("camera_model") or "") or None
        return ObservationExecution(
            "OBSERVATION_COMPLETED",
            observation_name,
            True,
            "Kameraképet készítettem.",
            artifact=str(output),
            detail=detail,
        )


__all__ = ["ObservationExecution", "ObservationExecutor"]
