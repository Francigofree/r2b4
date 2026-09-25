from __future__ import annotations

import inspect

from v3.adapters.async_person_photo_evidence import AsyncPersonPhotoEvidenceRecorder
from v3_hardware_runtime import NativeHardwareSensorOwner


def test_person_photo_downstream_work_is_not_in_control_observer_method():
    observe_source = inspect.getsource(AsyncPersonPhotoEvidenceRecorder.observe)
    worker_source = inspect.getsource(AsyncPersonPhotoEvidenceRecorder._run)
    assert "put_nowait" in observe_source
    assert "_recorder.observe" not in observe_source
    assert "_recorder.observe" in worker_source


def test_hardware_tick_observer_only_hands_person_evidence_to_async_edge():
    source = inspect.getsource(NativeHardwareSensorOwner.publish_tick_result)
    assert "_person_evidence.observe" in source
    assert "request_jpeg" not in source
    assert "datetime" not in source
