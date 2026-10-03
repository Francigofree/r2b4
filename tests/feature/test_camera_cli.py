from __future__ import annotations

from contextlib import contextmanager
import json
import shutil
import subprocess
from types import SimpleNamespace

import pytest


def test_camera_cli_without_v3_and_photo_exact_lineage(monkeypatch, tmp_path, capsys):
    from v3 import interface_cli, launcher_cli, launcher_extras, runtime_performance
    from v3.adapters import camera_media, vision_media_socket
    from v3.adapters.camera import CameraInterfaceAdapter

    calls = []
    metadata = {
        "source_sequence": 17, "measurement_monotonic_ns": 1000,
        "completed_monotonic_ns": 2000, "owner_generation": "camera-a",
        "width": 320, "height": 240, "calibration_state": "CALIBRATED",
        "calibration_id": "empirical-a", "rectified_K": [[200, 0, 160], [0, 200, 120], [0, 0, 1]],
    }
    image = SimpleNamespace(image_bytes=b"\xff\xd8calibrated-frame-17\xff\xd9",
                            metadata=SimpleNamespace(to_jsonable=lambda: dict(metadata)))

    class Client:
        manual = False

        def __init__(self, **kwargs):
            assert kwargs["root"] == tmp_path

        def status(self):
            calls.append("status")
            return {"camera_state": "ON", "manual_demand": self.manual, "consumers": 1 + int(self.manual)}

        def set_manual_demand(self, active):
            calls.append(("manual", active))
            Client.manual = active
            return self.status()

        def observe(self):
            calls.append("observe")
            return image

    monkeypatch.setattr(vision_media_socket, "VisionClient", Client)
    monkeypatch.setattr(camera_media, "VisionClient", Client)
    monkeypatch.setattr(launcher_cli, "project_root", lambda: tmp_path)
    monkeypatch.setattr(runtime_performance, "apply_host_affinity", lambda *a: None)
    monkeypatch.setattr(interface_cli, "main", lambda *a, **k: pytest.fail("camera started the V3 interface"))
    monkeypatch.setattr(launcher_cli, "_auto_prompt", lambda *a: pytest.fail("camera invoked the LLM"))

    assert launcher_cli.main(["cam", "status", "--json"]) == 0
    assert calls == ["status"]
    capsys.readouterr()
    assert launcher_cli.main(["cam", "on"]) == 0
    capsys.readouterr()
    assert launcher_cli.main(["camera", "off"]) == 0
    assert json.loads(capsys.readouterr().out)["consumers"] == 1
    output = tmp_path / "frame.jpg"
    assert launcher_cli.main(["cam", "photo", str(output)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert output.read_bytes() == image.image_bytes
    assert all(result[key] == value for key, value in metadata.items())
    assert "image_bytes" not in result
    assert launcher_cli.main(["cam", "photo", str(output)]) == 1
    assert output.read_bytes() == image.image_bytes
    assert launcher_cli.main(["help", "cam"]) == 0
    hint, candidates = launcher_extras.completion(2, ["cam", ""], tmp_path)
    assert {"status", "on", "off", "photo", "video"} == set(candidates)
    assert "r cam" in hint

    adapter = CameraInterfaceAdapter(SimpleNamespace(root=tmp_path))
    assert adapter.capabilities()["camera.photo"]["available"] is True
    assert adapter.execute("camera.off")["consumers"] == 1
    assert adapter.read("camera.status")["manual_demand"] is False


def test_camera_video_session_and_fail_closed_generation(monkeypatch, tmp_path):
    from v3.adapters import camera_media

    encoder = shutil.which("ffmpeg")
    if encoder is None:
        with pytest.raises(RuntimeError, match="requires ffmpeg"):
            camera_media.capture_h264_video(tmp_path / "clip.mp4", 0.1)
        return
    jpeg = subprocess.run(
        [encoder, "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
         "color=c=black:s=32x32", "-frames:v", "1", "-threads", "1", "-c:v", "mjpeg",
         "-f", "image2pipe", "pipe:1"], check=True, capture_output=True,
    ).stdout
    releases = []

    class Session:
        sequence = 0
        change_generation = False

        def observe(self):
            self.sequence += 1
            metadata = {
                "source_sequence": self.sequence,
                "owner_generation": "b" if self.change_generation and self.sequence > 1 else "a",
                "width": 32, "height": 32, "calibration_id": "empirical",
                "calibration_state": "CALIBRATED",
                "measurement_monotonic_ns": self.sequence * 1000,
                "completed_monotonic_ns": self.sequence * 1000 + 10,
            }
            return SimpleNamespace(image_bytes=jpeg, metadata=SimpleNamespace(to_jsonable=lambda: metadata))

    current = Session()

    class Client:
        def __init__(self, **kwargs): pass

        @contextmanager
        def session(self):
            try:
                yield current
            finally:
                releases.append(current)

    monkeypatch.setattr(camera_media, "VisionClient", Client)
    output = tmp_path / "clip.mp4"
    result = camera_media.capture_h264_video(output, 0.22, fps=10)
    assert output.stat().st_size > 0 and result["frames"] >= 2
    assert result["first_frame"]["source_sequence"] == 1
    assert result["last_frame"]["source_sequence"] == result["frames"]
    assert releases == [current]

    current = Session()
    current.change_generation = True
    failed = tmp_path / "failed.mp4"
    with pytest.raises(RuntimeError, match="generation changed"):
        camera_media.capture_h264_video(failed, 0.22, fps=10)
    assert not failed.exists()
    assert releases[-1] is current
    assert not list(tmp_path.glob(".r2b4-camera-*"))
