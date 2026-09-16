#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

EXPECTED_BASE = "c3eed9753268700d32348f8ff34b7508ade2cf7e"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def patch_picamera_driver(text: str) -> str:
    text = replace_once(text,
        '    def configure(self, configuration: object) -> None: ...\n\n    def set_controls(self, controls: Mapping[str, object]) -> None: ...\n',
        '    def configure(self, configuration: object) -> None: ...\n\n    def stream_configuration(self, stream_name: str) -> Mapping[str, object]: ...\n\n    def set_controls(self, controls: Mapping[str, object]) -> None: ...\n',
        "picamera stream_configuration protocol")

    text = replace_once(text,
        '\n\n@dataclass(frozen=True, slots=True)\nclass CameraFrameSnapshot:\n',
        '''\n\n@dataclass(frozen=True, slots=True)
class CameraStreamGeometry:
    """Actual Picamera2 stream geometry after libcamera configuration."""

    stream_name: str
    width: int
    height: int
    pixel_format: str
    stride_bytes: int
    frame_size_bytes: int

    def __post_init__(self) -> None:
        _nonempty_string(self.stream_name, "stream_name")
        _positive_int(self.width, "width")
        _positive_int(self.height, "height")
        _nonempty_string(self.pixel_format, "pixel_format")
        _positive_int(self.stride_bytes, "stride_bytes")
        _positive_int(self.frame_size_bytes, "frame_size_bytes")
        if self.stride_bytes < self.width:
            raise ValueError("camera stride cannot be smaller than image width")
        if self.frame_size_bytes < self.stride_bytes * self.height:
            raise ValueError("camera frame size is smaller than one full stride plane")


@dataclass(frozen=True, slots=True)
class CameraFrameSnapshot:
''', "camera actual stream geometry contract")

    text = replace_once(text,
        '    width: int\n    height: int\n    pixel_format: str\n    focus_state: str\n',
        '    width: int\n    height: int\n    pixel_format: str\n    stride_bytes: int\n    frame_size_bytes: int\n    focus_state: str\n',
        "camera frame geometry fields")

    text = replace_once(text,
        '        _positive_int(self.width, "width")\n        _positive_int(self.height, "height")\n        _nonempty_string(self.pixel_format, "pixel_format")\n        _nonempty_string(self.focus_state, "focus_state")\n',
        '''        _positive_int(self.width, "width")
        _positive_int(self.height, "height")
        _nonempty_string(self.pixel_format, "pixel_format")
        _positive_int(self.stride_bytes, "stride_bytes")
        _positive_int(self.frame_size_bytes, "frame_size_bytes")
        if self.stride_bytes < self.width:
            raise ValueError("camera stride cannot be smaller than image width")
        if self.frame_size_bytes < self.stride_bytes * self.height:
            raise ValueError("camera frame size is smaller than one full stride plane")
        _nonempty_string(self.focus_state, "focus_state")
''', "camera frame geometry validation")

    text = replace_once(text,
        '        if not self.image_bytes:\n            raise ValueError("image_bytes must not be empty")\n\n    @property\n',
        '        if not self.image_bytes:\n            raise ValueError("image_bytes must not be empty")\n        if len(self.image_bytes) != self.frame_size_bytes:\n            raise ValueError("camera payload size does not match configured frame size")\n\n    @property\n',
        "camera payload exact size validation")

    text = replace_once(text,
        '        "_frame_condition",\n        "_last_error",\n',
        '        "_frame_condition",\n        "_geometry",\n        "_last_error",\n',
        "camera geometry slot")

    text = replace_once(text,
        '        self._picamera: Picamera2Device | None = None\n        self._latest: CameraFrameSnapshot | None = None\n',
        '        self._picamera: Picamera2Device | None = None\n        self._geometry: CameraStreamGeometry | None = None\n        self._latest: CameraFrameSnapshot | None = None\n',
        "camera geometry initialization")

    text = replace_once(text,
        '            self._latest = None\n            self._last_sensor_timestamp_ns = None\n            self._last_error = None\n',
        '            self._latest = None\n            self._geometry = None\n            self._last_sensor_timestamp_ns = None\n            self._last_error = None\n',
        "camera restart geometry reset")

    text = replace_once(text,
        '            configuration = build_video_configuration(camera, self._config)\n            camera.configure(configuration)\n            controls = (\n',
        '            configuration = build_video_configuration(camera, self._config)\n            camera.configure(configuration)\n            geometry = camera_stream_geometry(camera, self._config.stream_name)\n            controls = (\n',
        "camera close actual stream geometry after configure")

    text = replace_once(text,
        '        with self._frame_condition:\n            self._picamera = camera\n            self._model = model\n            self._running = True\n',
        '        with self._frame_condition:\n            self._picamera = camera\n            self._geometry = geometry\n            self._model = model\n            self._running = True\n',
        "camera publish actual stream geometry")

    text = replace_once(text,
        '        with self._frame_condition:\n            self._picamera = None\n            self._frame_condition.notify_all()\n',
        '        with self._frame_condition:\n            self._picamera = None\n            self._geometry = None\n            self._frame_condition.notify_all()\n',
        "camera geometry release")

    text = replace_once(text,
        '    def capture_once_for_test(self, request: Picamera2Request) -> CameraFrameSnapshot:\n        return self._snapshot_from_request(request)\n',
        '''    def capture_once_for_test(
        self,
        request: Picamera2Request,
        geometry: CameraStreamGeometry,
    ) -> CameraFrameSnapshot:
        """Exercise frame closure against an explicit actual stream geometry."""
        if not isinstance(geometry, CameraStreamGeometry):
            raise TypeError("geometry must be CameraStreamGeometry")
        return self._snapshot_from_request(request, geometry)
''', "camera test closure requires actual geometry")

    text = replace_once(text,
        '            with self._lock:\n                camera = self._picamera\n                running = self._running\n            if not running or camera is None:\n                return\n            request: Picamera2Request | None = None\n',
        '''            with self._lock:
                camera = self._picamera
                geometry = self._geometry
                running = self._running
            if not running or camera is None:
                return
            if geometry is None:
                with self._frame_condition:
                    self._last_error = "RuntimeError:camera stream geometry is unavailable"
                    self._running = False
                    self._frame_condition.notify_all()
                return
            request: Picamera2Request | None = None
''', "camera acquisition geometry snapshot")

    text = replace_once(text,
        '                request = camera.capture_request()\n                snapshot = self._snapshot_from_request(request)\n',
        '                request = camera.capture_request()\n                snapshot = self._snapshot_from_request(request, geometry)\n',
        "camera acquisition actual geometry use")

    text = replace_once(text,
        '    def _snapshot_from_request(self, request: Picamera2Request) -> CameraFrameSnapshot:\n',
        '    def _snapshot_from_request(\n        self,\n        request: Picamera2Request,\n        geometry: CameraStreamGeometry,\n    ) -> CameraFrameSnapshot:\n',
        "camera snapshot geometry argument")

    text = replace_once(text,
        '        raw_buffer = request.make_buffer(self._config.stream_name)\n        image_bytes = bytes(raw_buffer)\n        completed_monotonic_ns = self._checked_clock()\n',
        '''        raw_buffer = request.make_buffer(geometry.stream_name)
        image_bytes = bytes(raw_buffer)
        if len(image_bytes) != geometry.frame_size_bytes:
            raise RuntimeError(
                "camera buffer size does not match Picamera2 stream configuration"
            )
        completed_monotonic_ns = self._checked_clock()
''', "camera physical buffer size validation")

    text = replace_once(text,
        '            width=self._config.published_width,\n            height=self._config.published_height,\n            pixel_format=self._config.published_pixel_format,\n            focus_state=focus_state,\n',
        '            width=geometry.width,\n            height=geometry.height,\n            pixel_format=geometry.pixel_format,\n            stride_bytes=geometry.stride_bytes,\n            frame_size_bytes=geometry.frame_size_bytes,\n            focus_state=focus_state,\n',
        "camera snapshot actual geometry publication")

    text = replace_once(text, '\n\ndef build_video_configuration(\n', '''

def camera_stream_geometry(
    camera: Picamera2Device,
    stream_name: str,
) -> CameraStreamGeometry:
    """Read the post-configure Picamera2 buffer contract for one stream."""
    _nonempty_string(stream_name, "stream_name")
    getter = getattr(camera, "stream_configuration", None)
    if not callable(getter):
        raise RuntimeError("Picamera2 stream_configuration is unavailable")
    value = getter(stream_name)
    if not isinstance(value, Mapping):
        raise RuntimeError("Picamera2 stream configuration must be a mapping")
    size = value.get("size")
    if not isinstance(size, (tuple, list)) or len(size) != 2:
        raise RuntimeError("Picamera2 stream size is unavailable")
    return CameraStreamGeometry(
        stream_name=stream_name,
        width=_positive_int(size[0], "stream width"),
        height=_positive_int(size[1], "stream height"),
        pixel_format=_nonempty_string(value.get("format"), "stream format"),
        stride_bytes=_positive_int(value.get("stride"), "stream stride"),
        frame_size_bytes=_positive_int(value.get("framesize"), "stream framesize"),
    )


def build_video_configuration(
''', "camera actual stream geometry reader")

    text = replace_once(text,
        '    "CameraRuntimeStatus",\n    "NativePicamera2Camera",\n',
        '    "CameraRuntimeStatus",\n    "CameraStreamGeometry",\n    "NativePicamera2Camera",\n',
        "camera geometry export")
    text = replace_once(text,
        '    "build_video_configuration",\n    "default_camera_controls",\n',
        '    "build_video_configuration",\n    "camera_stream_geometry",\n    "default_camera_controls",\n',
        "camera geometry helper export")
    return text


def patch_live_camera(text: str) -> str:
    return replace_once(text,
        '                DataField("pixel_format", frame.pixel_format),\n                DataField("exposure_time_ns", frame.exposure_time_ns),\n',
        '                DataField("pixel_format", frame.pixel_format),\n                DataField("stride_bytes", frame.stride_bytes),\n                DataField("frame_size_bytes", frame.frame_size_bytes),\n                DataField("exposure_time_ns", frame.exposure_time_ns),\n',
        "camera health actual buffer geometry")


def patch_camera_tool(text: str) -> str:
    return replace_once(text,
        '            "published_stream": config.stream_name,\n            "published_size": [config.published_width, config.published_height],\n            "published_format": config.published_pixel_format,\n            "main_size": [config.main_width, config.main_height],\n',
        '''            "published_stream": config.stream_name,
            "published_size": (
                [final.frame.width, final.frame.height]
                if final.frame is not None
                else [config.published_width, config.published_height]
            ),
            "published_format": (
                final.frame.pixel_format
                if final.frame is not None
                else config.published_pixel_format
            ),
            "stride_bytes": final.frame.stride_bytes if final.frame is not None else None,
            "frame_size_bytes": (
                final.frame.frame_size_bytes if final.frame is not None else None
            ),
            "main_size": [config.main_width, config.main_height],
''', "camera status actual geometry output")



def patch_hardware_tests(text: str) -> str:
    text = replace_once(text,
        'from pathlib import Path\nfrom types import SimpleNamespace\n',
        'from dataclasses import replace\nfrom pathlib import Path\nfrom types import SimpleNamespace\n',
        "hardware test dataclasses replace import")
    old = '''def _runtime_config():
    return load_bounded_physical_runtime_config(
        PROJECT_ROOT / "conf" / "hardver.json",
        PROJECT_ROOT / "conf" / "fizika.json",
        PROJECT_ROOT / "conf" / "speed_map.json",
        BoundedTeleopProfile(
            "hardware-runtime-test",
            start_tick_id=2,
            active_tick_count=1,
            v_mps=0.05,
            omega_rad_s=0.0,
            max_v_mps=0.10,
            max_omega_rad_s=0.20,
        ),
        sensor_policy=_policy(),
    )
'''
    new = '''def _runtime_config():
    runtime = load_bounded_physical_runtime_config(
        PROJECT_ROOT / "conf" / "hardver.json",
        PROJECT_ROOT / "conf" / "fizika.json",
        PROJECT_ROOT / "conf" / "speed_map.json",
        BoundedTeleopProfile(
            "hardware-runtime-test",
            start_tick_id=2,
            active_tick_count=1,
            v_mps=0.05,
            omega_rad_s=0.0,
            max_v_mps=0.10,
            max_omega_rad_s=0.20,
        ),
        sensor_policy=_policy(),
    )
    # These tests validate fake core hardware. Never open the host camera here.
    assert runtime.sensor_inputs is not None
    sensor_inputs = replace(
        runtime.sensor_inputs,
        camera_device=None,
        inputs=replace(runtime.sensor_inputs.inputs, camera_source=None),
    )
    return replace(runtime, sensor_inputs=sensor_inputs)
'''
    return replace_once(text, old, new, "hardware tests explicitly disable physical camera")


def patch_camera_tests(text: str) -> str:
    text = replace_once(text,
        '    CameraRuntimeStatus,\n    NativePicamera2Camera,\n',
        '    CameraRuntimeStatus,\n    CameraStreamGeometry,\n    NativePicamera2Camera,\n',
        "camera test geometry import")
    text = replace_once(text,
        'class _Request:\n    def __init__(self, timestamp=1_000_000_000, data=b"camera-frame") -> None:\n        self.released = False\n        self.timestamp = timestamp\n        self.data = data\n',
        'class _Request:\n    def __init__(self, timestamp=1_000_000_000, data=None) -> None:\n        self.released = False\n        self.timestamp = timestamp\n        self.data = bytes(640 * 360 * 3) if data is None else data\n',
        "camera test physical-sized default buffer")
    text = replace_once(text,
        '    def configure(self, configuration) -> None:\n        self.events.append("configure")\n\n    def set_controls(self, controls) -> None:\n',
        '''    def configure(self, configuration) -> None:
        self.events.append("configure")

    def stream_configuration(self, stream_name: str):
        spec = self.configuration_kwargs[stream_name]
        width, height = spec["size"]
        pixel_format = spec["format"]
        if pixel_format == "RGB888":
            stride = width * 3
            framesize = stride * height
        elif pixel_format == "YUV420":
            stride = width
            framesize = stride * height * 3 // 2
        else:
            raise AssertionError(pixel_format)
        return {
            "size": (width, height),
            "format": pixel_format,
            "stride": stride,
            "framesize": framesize,
        }

    def set_controls(self, controls) -> None:
''', "camera fake stream configuration")
    text = replace_once(text,
        '    snapshot = owner.capture_once_for_test(_Request())\n    assert snapshot.sensor_timestamp_ns == 1_000_000_000\n',
        '    geometry = CameraStreamGeometry("lores", 640, 360, "RGB888", 1920, 691200)\n    snapshot = owner.capture_once_for_test(_Request(), geometry)\n    assert snapshot.sensor_timestamp_ns == 1_000_000_000\n',
        "camera frame test explicit geometry")
    text = replace_once(text,
        '    assert snapshot.image_bytes == b"camera-frame"\n    assert snapshot.width == 640\n    assert snapshot.height == 360\n',
        '    assert len(snapshot.image_bytes) == 691200\n    assert snapshot.width == 640\n    assert snapshot.height == 360\n    assert snapshot.stride_bytes == 1920\n    assert snapshot.frame_size_bytes == 691200\n',
        "camera frame test actual geometry assertions")
    text = replace_once(text,
        '        width=640,\n        height=360,\n        pixel_format="RGB888",\n        focus_state="2",\n        lens_position=1.0,\n        image_bytes=b"abc",\n',
        '        width=2,\n        height=2,\n        pixel_format="RGB888",\n        stride_bytes=6,\n        frame_size_bytes=12,\n        focus_state="2",\n        lens_position=1.0,\n        image_bytes=b"abcdefghijkl",\n',
        "camera synthetic frame actual geometry")
    text = replace_once(text,
        '    assert values["payload_bytes"] == 3\n    assert values["camera_model"] == "imx708"\n',
        '    assert values["payload_bytes"] == 12\n    assert values["stride_bytes"] == 6\n    assert values["frame_size_bytes"] == 12\n    assert values["camera_model"] == "imx708"\n',
        "camera health geometry assertions")

    marker = '\ndef test_raspberry_pi_timestamp_mapper_is_identity():\n'
    addition = '''

def test_camera_publishes_actual_post_configure_stream_geometry():
    class _AlignedCamera(_ConfiguredCamera):
        def stream_configuration(self, stream_name: str):
            if stream_name == "lores":
                return {
                    "size": (672, 360),
                    "format": "RGB888",
                    "stride": 2048,
                    "framesize": 2048 * 360,
                }
            return super().stream_configuration(stream_name)

    camera = _AlignedCamera(requests=[_Request(data=bytes(2048 * 360))])
    owner = NativePicamera2Camera(
        Picamera2CameraConfig(max_frame_completion_lag_ns=2_000_000_000),
        picamera_factory=lambda index: camera,
        sensor_timestamp_mapper=lambda value: value,
        camera_controls_factory=lambda: {},
        monotonic_ns=lambda: 2_000_000_000,
    )
    assert owner.start()
    edge = owner.wait_for_new_frame(0, timeout_s=1.0)
    assert edge.frame is not None
    assert edge.frame.width == 672
    assert edge.frame.height == 360
    assert edge.frame.stride_bytes == 2048
    assert edge.frame.frame_size_bytes == 2048 * 360
    assert len(edge.frame.image_bytes) == edge.frame.frame_size_bytes
    owner.stop()

'''
    return replace_once(text, marker, addition + marker, "camera actual geometry regression test")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default=".")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    if not (repo / ".git").exists() or not (repo / "v3").is_dir():
        raise SystemExit(f"not an R2B4 checkout: {repo}")

    try:
        head = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        head = "UNKNOWN"
    if head != EXPECTED_BASE:
        print(f"NOTE: HEAD is {head[:12]}, inspected baseline was {EXPECTED_BASE[:12]}; structural anchors will decide compatibility.")

    patchers = {
        "v3/adapters/picamera2_camera.py": patch_picamera_driver,
        "v3/adapters/live_camera.py": patch_live_camera,
        "tools/v3_camera_test.py": patch_camera_tool,
        "tests/test_v3_hardware_runtime.py": patch_hardware_tests,
        "tests/test_v3_camera_foundation.py": patch_camera_tests,
    }
    prepared = {}
    for relative, patcher in patchers.items():
        path = repo / relative
        if not path.is_file():
            raise RuntimeError(f"required source missing: {relative}")
        prepared[path] = patcher(path.read_text(encoding="utf-8"))

    if args.check:
        print("camera upgrade 2.0.2-r1 structural compatibility: PASS")
        print(f"prepared {len(prepared)} files; nothing written")
        return 0

    for path, content in prepared.items():
        path.write_text(content, encoding="utf-8")
    try:
        (repo / "tools/v3_camera_test.py").chmod(0o755)
    except OSError:
        pass
    print("R2B4 camera robustness upgrade 2.0.2-r1 applied")
    print("No camera, motor, LiDAR or IMU was started.")
    print("Run validation_camera_upgrade_2_0_2.sh from this package.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
