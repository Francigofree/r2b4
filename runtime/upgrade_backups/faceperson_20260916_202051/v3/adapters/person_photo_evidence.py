"""Passive roomcruise person-photo evidence from completed V3 tick values.

This module owns no command, mission, navigation, lifecycle, safety or motor
state.  It observes one already-completed L2 admitted frame together with the
corresponding L5 mission and may enqueue one JPEG request on the existing camera
owner.  Detector results therefore enter the evidence trigger only after the
canonical L0 -> L1 -> L2 admission path has accepted them.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Protocol

from v3.contracts import (
    AdmittedFrame,
    CommandMode,
    MissionIntent,
    MissionLifecycle,
    Observation,
)


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _relative_directory(value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("directory must be a non-empty relative path")
    path = Path(value.strip())
    if path.is_absolute() or ".." in path.parts:
        raise ValueError("directory must stay below the R2B4 project root")
    return path.as_posix()


@dataclass(frozen=True, slots=True)
class PersonPhotoEvidenceConfig:
    """Bounded live-only evidence policy for EXPLORE/roomcruise acceptance."""

    enabled: bool = False
    directory: str = "pic"
    stream_name: str = "main"
    confirm_results: int = 3
    rearm_misses: int = 5
    minimum_interval_ns: int = 5_000_000_000

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool:
            raise TypeError("enabled must be bool")
        _relative_directory(self.directory)
        if self.stream_name not in {"main", "lores"}:
            raise ValueError("stream_name must be 'main' or 'lores'")
        _positive_int(self.confirm_results, "confirm_results")
        _positive_int(self.rearm_misses, "rearm_misses")
        _positive_int(self.minimum_interval_ns, "minimum_interval_ns")


def person_photo_evidence_config_from_mapping(
    value: object,
) -> PersonPhotoEvidenceConfig:
    if not isinstance(value, dict):
        raise TypeError("person_detection.photo_evidence must be an object")
    allowed = {
        "enabled",
        "directory",
        "stream_name",
        "confirm_results",
        "rearm_misses",
        "minimum_interval_ns",
    }
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ValueError(
            "unknown person_detection.photo_evidence keys: " + ", ".join(unknown)
        )
    enabled = value.get("enabled", False)
    if type(enabled) is not bool:
        raise ValueError("person_detection.photo_evidence.enabled must be bool")
    return PersonPhotoEvidenceConfig(
        enabled=enabled,
        directory=_relative_directory(value.get("directory", "pic")),
        stream_name=str(value.get("stream_name", "main")),
        confirm_results=_positive_int(
            value.get("confirm_results", 3),
            "person_detection.photo_evidence.confirm_results",
        ),
        rearm_misses=_positive_int(
            value.get("rearm_misses", 5),
            "person_detection.photo_evidence.rearm_misses",
        ),
        minimum_interval_ns=_positive_int(
            value.get("minimum_interval_ns", 5_000_000_000),
            "person_detection.photo_evidence.minimum_interval_ns",
        ),
    )


class CameraPhotoRequestPort(Protocol):
    def request_jpeg(
        self,
        output: str | Path,
        *,
        stream_name: str = "lores",
    ) -> bool: ...


@dataclass(frozen=True, slots=True)
class PersonPhotoEvidenceStatus:
    enabled: bool
    armed: bool
    photo_request_count: int
    last_output: str | None
    last_error: str | None


def _values(observation: Observation) -> dict[str, object]:
    return {field.key: field.value for field in observation.values}


class PersonPhotoEvidenceRecorder:
    """Request one JPEG per stable person-presence episode during EXPLORE.

    Only *accepted L2 observations* count.  A detector running slower than the
    50 Hz control loop therefore does not create fake misses or duplicate hits:
    repeated detector revisions are rejected by L2 and simply do not appear in
    ``AdmittedFrame.accepted``.  A person must be present in ``confirm_results``
    consecutive new accepted detector results before a photo is requested.
    After one accepted request the recorder rearms only after ``rearm_misses``
    accepted person-free detector results.
    """

    __slots__ = (
        "_armed",
        "_config",
        "_directory",
        "_hit_streak",
        "_last_detection_sequence",
        "_last_error",
        "_last_output",
        "_last_photo_ns",
        "_miss_streak",
        "_photo_port",
        "_photo_request_count",
        "_source_device_id",
    )

    def __init__(
        self,
        photo_port: CameraPhotoRequestPort,
        config: PersonPhotoEvidenceConfig,
        *,
        source_device_id: str = "PERSON_DETECTOR_FRONT",
        project_root: str | Path | None = None,
    ) -> None:
        if not callable(getattr(photo_port, "request_jpeg", None)):
            raise TypeError("photo_port must provide request_jpeg")
        if not isinstance(config, PersonPhotoEvidenceConfig):
            raise TypeError("config must be PersonPhotoEvidenceConfig")
        if not isinstance(source_device_id, str) or not source_device_id.strip():
            raise ValueError("source_device_id must be a non-empty string")

        root = (
            Path(project_root).resolve()
            if project_root is not None
            else Path(__file__).resolve().parents[2]
        )
        relative = Path(_relative_directory(config.directory))
        directory = (root / relative).resolve(strict=False)
        try:
            directory.relative_to(root)
        except ValueError as exc:
            raise ValueError("photo evidence directory escapes project root") from exc

        self._photo_port = photo_port
        self._config = config
        self._source_device_id = source_device_id.strip()
        self._directory: Path | None = directory
        self._last_detection_sequence = 0
        self._hit_streak = 0
        self._miss_streak = 0
        self._armed = True
        self._last_photo_ns: int | None = None
        self._photo_request_count = 0
        self._last_output: str | None = None
        self._last_error: str | None = None

        if config.enabled:
            try:
                directory.mkdir(parents=True, exist_ok=True)
                if directory.is_symlink() or not directory.is_dir():
                    raise OSError("photo evidence directory is not a regular directory")
            except OSError as exc:
                # Evidence is non-critical.  Preserve motor/runtime availability
                # and surface the local evidence failure through status only.
                self._directory = None
                self._last_error = f"{type(exc).__name__}:{exc}"

    @property
    def status(self) -> PersonPhotoEvidenceStatus:
        return PersonPhotoEvidenceStatus(
            enabled=bool(self._config.enabled and self._directory is not None),
            armed=self._armed,
            photo_request_count=self._photo_request_count,
            last_output=self._last_output,
            last_error=self._last_error,
        )

    def observe(self, admitted: AdmittedFrame, mission: MissionIntent) -> bool:
        """Observe one completed V3 tick and maybe enqueue one JPEG request."""

        if not isinstance(admitted, AdmittedFrame):
            raise TypeError("admitted must be AdmittedFrame")
        if not isinstance(mission, MissionIntent):
            raise TypeError("mission must be MissionIntent")
        if admitted.context != mission.context:
            raise ValueError("L2 and L5 evidence contexts must match")

        if (
            not self._config.enabled
            or self._directory is None
            or mission.mode is not CommandMode.EXPLORE
            or mission.lifecycle is not MissionLifecycle.ACTIVE
        ):
            self._reset_episode()
            return False

        observations = tuple(
            item
            for item in admitted.accepted
            if item.kind == "person_detection"
            and item.source_device_id == self._source_device_id
        )
        if not observations:
            # Detector results are intentionally slower than the 50 Hz runtime.
            # No new accepted semantic result means neither hit nor miss.
            return False
        if len(observations) != 1:
            self._last_error = "PERSON_EVIDENCE_MULTIPLE_RESULTS"
            return False
        observation = observations[0]
        if observation.source_sequence <= self._last_detection_sequence:
            self._last_error = "PERSON_EVIDENCE_SEQUENCE_NOT_INCREASING"
            return False
        self._last_detection_sequence = observation.source_sequence

        values = _values(observation)
        person_detected = values.get("person_detected")
        source_frame_sequence = values.get("source_frame_sequence")
        if type(person_detected) is not bool:
            self._last_error = "PERSON_EVIDENCE_DETECTED_FLAG_INVALID"
            return False
        if (
            not isinstance(source_frame_sequence, int)
            or isinstance(source_frame_sequence, bool)
            or source_frame_sequence <= 0
        ):
            self._last_error = "PERSON_EVIDENCE_FRAME_SEQUENCE_INVALID"
            return False

        if not person_detected:
            self._hit_streak = 0
            self._miss_streak += 1
            if self._miss_streak >= self._config.rearm_misses:
                self._armed = True
            self._last_error = None
            return False

        self._miss_streak = 0
        self._hit_streak += 1
        if self._hit_streak < self._config.confirm_results or not self._armed:
            self._last_error = None
            return False
        now_ns = mission.context.monotonic_ns
        if (
            self._last_photo_ns is not None
            and now_ns - self._last_photo_ns < self._config.minimum_interval_ns
        ):
            return False

        now = datetime.now().astimezone()
        stamp = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        output = self._directory / (
            f"person_{stamp}_det{observation.source_sequence:06d}_"
            f"frame{source_frame_sequence:06d}.jpg"
        )
        try:
            accepted = self._photo_port.request_jpeg(
                output,
                stream_name=self._config.stream_name,
            )
        except Exception as exc:
            self._last_error = f"{type(exc).__name__}:{exc}"
            return False
        if not accepted:
            # Camera already has one bounded photo request in flight.  Stay
            # armed so the next accepted detector result can retry.
            return False

        self._armed = False
        self._last_photo_ns = now_ns
        self._photo_request_count += 1
        self._last_output = str(output)
        self._last_error = None
        return True

    def _reset_episode(self) -> None:
        self._hit_streak = 0
        self._miss_streak = 0
        self._armed = True


__all__ = [
    "CameraPhotoRequestPort",
    "PersonPhotoEvidenceConfig",
    "PersonPhotoEvidenceRecorder",
    "PersonPhotoEvidenceStatus",
    "person_photo_evidence_config_from_mapping",
]
