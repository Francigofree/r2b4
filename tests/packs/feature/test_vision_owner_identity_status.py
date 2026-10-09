"""Camera activation time qualifies host teaching without entering control input."""
from types import SimpleNamespace

import pytest

from v3.adapters.vision_owner import CameraVisionOwner


@pytest.mark.parametrize("start_ok", [True, False])
def test_generation_start_is_captured_before_frames_can_publish_and_cleared_on_close(monkeypatch, start_ok):
    monkeypatch.setattr("v3.adapters.vision_owner.time.monotonic_ns", lambda: 100)
    proof_at_camera_start = []

    class Camera:
        owner_generation = "test-camera-generation"
        running = False

        def start(self):
            proof_at_camera_start.append(owner.status()["owner_generation_started_ns"])
            self.running = start_ok
            return start_ok

        def stop(self):
            self.running = False

        def get_runtime_status(self):
            return SimpleNamespace(running=self.running, last_error=None)

    owner = CameraVisionOwner(Camera, idle_grace_s=30)
    try:
        assert owner.status()["owner_generation_started_ns"] is None
        if start_ok:
            owner.set_manual_demand(True)
            assert owner.status()["owner_generation_started_ns"] == 100
            assert owner.status()["owner_generation"] == "test-camera-generation"
        else:
            with pytest.raises(RuntimeError, match="VISION_CAMERA_FAILED"):
                owner.set_manual_demand(True)
            assert owner.status()["owner_generation_started_ns"] is None
        assert proof_at_camera_start == [100]
    finally:
        owner.close()
    assert owner.status()["owner_generation_started_ns"] is None
    assert owner.status()["owner_generation"] == ""
