import json
import queue
import time
from pathlib import Path

import pytest

from v3.adapters.latest_lidar import MATCHER_CONTRACT_ID
from v3.adapters.native_lidar_port import (
    NativeLidarPort,
    NativeLidarPortConfig,
    load_native_lidar_port_config,
)
from v3.adapters.rplidar_c1 import RplidarC1Config, RplidarPoint, RplidarScan


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class Driver:
    config = RplidarC1Config(stale_timeout_s=0.5)

    def __init__(self, scan):
        self.scan = scan
        self.stop_calls = 0

    def start(self):
        return True

    def stop(self):
        self.stop_calls += 1

    def get_latest_scan(self):
        return self.scan

    def get_runtime_status(self):
        return {"running": True, "connected": True, "last_data_age_s": 0.0}


class FailingStartDriver(Driver):
    def start(self):
        raise RuntimeError("driver start failed")


class FakeEvent:
    def __init__(self):
        self.is_set = False

    def set(self):
        self.is_set = True

    def wait(self, timeout):
        return True


class FakeProcess:
    def __init__(self):
        self.alive = False
        self.join_calls = 0

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive

    def join(self, timeout):
        self.join_calls += 1
        self.alive = False

    def terminate(self):
        self.alive = False


class FakeProcessContext:
    def __init__(self):
        self.process = FakeProcess()

    def Queue(self, maxsize):
        return queue.Queue(maxsize=maxsize)

    def Event(self):
        return FakeEvent()

    def Process(self, **_kwargs):
        return self.process


def _scan():
    return RplidarScan(
        7,
        1_000_000_000,
        (
            RplidarPoint(0.0, 1.0, 20),
            RplidarPoint(90.0, 1.1, 20),
            RplidarPoint(180.0, 1.2, 20),
            RplidarPoint(270.0, 1.3, 20),
        ),
    )


def _config():
    return NativeLidarPortConfig(
        driver=Driver.config,
        danger_zone_m=0.2,
        matcher_config_json="{}",
    )


def test_native_port_publishes_raw_safety_before_optional_matcher_result():
    driver = Driver(_scan())
    port = NativeLidarPort(
        _config(),
        lambda: (0.0, 0.0, 0.0),
        driver=driver,
        monotonic_ns=lambda: 1_010_000_000,
    )
    port._input_queue = queue.Queue(maxsize=1)
    port._result_queue = queue.Queue(maxsize=1)
    port._running = True

    port.poll_once_for_test()

    snapshot = port.get_raw_scan_snapshot()
    assert snapshot is not None
    assert snapshot.raw_scan_id == 7
    assert snapshot.health == "OK"
    assert snapshot.summary["front_clearance_m"] == 1.0
    assert snapshot.summary["rear_clearance_m"] == 1.2
    assert snapshot.summary["left_clearance_m"] == 1.3
    assert snapshot.summary["right_clearance_m"] == 1.1
    packet = port._input_queue.get_nowait()
    assert packet["scan_revision"] == 7
    assert packet["matcher_contract_id"] == MATCHER_CONTRACT_ID
    assert port.get_matcher_result() is None
    with pytest.raises(TypeError):
        snapshot.summary["front_clearance_m"] = 0.0


def test_native_port_accepts_fresh_matcher_lineage_as_independent_result():
    driver = Driver(_scan())
    port = NativeLidarPort(
        _config(),
        lambda: (0.0, 0.0, 0.0),
        driver=driver,
        monotonic_ns=lambda: 1_010_000_000,
    )
    port._input_queue = queue.Queue(maxsize=1)
    port._result_queue = queue.Queue(maxsize=1)
    port._running = True
    port.poll_once_for_test()
    port._input_queue.get_nowait()
    port._result_queue.put_nowait(
        {
            "kind": "result",
            "matcher_contract_id": MATCHER_CONTRACT_ID,
            "scan_revision": 7,
            "captured_monotonic_ns": 1_000_000_000,
            "published_monotonic_ns": 1_005_000_000,
            "matcher_runtime_ms": 3.0,
            "matcher_queue_delay_ms": 1.0,
            "summary": {
                "lidar_pose_x": 0.1,
                "lidar_pose_y": 0.2,
                "lidar_pose_theta": 0.3,
                "lidar_pose_confidence": 0.8,
            },
        }
    )

    port.poll_once_for_test()

    result = port.get_matcher_result()
    assert result is not None
    assert result.source_raw_scan_id == 7
    assert result.summary["matcher_transport"] == "process_latest_only"
    assert result.summary["lidar_pose_confidence"] == 0.8


def test_active_json_closes_to_protected_native_port_config(tmp_path):
    config = load_native_lidar_port_config(
        PROJECT_ROOT / "conf" / "hardver.json",
        PROJECT_ROOT / "conf" / "vezerles.json",
        danger_zone_m=0.1,
    )

    assert config.matcher_start_method == "spawn"
    assert config.input_queue_capacity == config.result_queue_capacity == 1
    assert config.maximum_input_age_ns == config.maximum_result_age_ns == 250_000_000
    assert config.driver.baudrate == 460_800
    assert config.danger_zone_m == 0.1

    hardware = tmp_path / "hardver.json"
    control = tmp_path / "vezerles.json"
    hardware.write_text(json.dumps({"lidar": {}}), encoding="utf-8")
    control.write_text(
        json.dumps(
            {
                "lidar_pose": {
                    "min_valid_distance_m": 0.05,
                    "max_valid_distance_m": 12.0,
                },
                "lidar_runtime": {"latest_scan_queue_size": 2},
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="latest_scan_queue_size"):
        load_native_lidar_port_config(hardware, control, danger_zone_m=0.1)

    hardware.write_text(
        json.dumps({"lidar": {"baudrate": True}}),
        encoding="utf-8",
    )
    control.write_text(
        json.dumps({"lidar_pose": {}, "lidar_runtime": {}}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="lidar.baudrate"):
        load_native_lidar_port_config(hardware, control, danger_zone_m=0.1)


def test_native_owner_cleans_matcher_process_when_driver_start_raises():
    driver = FailingStartDriver(_scan())
    process_context = FakeProcessContext()
    port = NativeLidarPort(
        _config(),
        lambda: (0.0, 0.0, 0.0),
        driver=driver,
        process_context=process_context,
    )

    with pytest.raises(RuntimeError, match="driver start failed"):
        port.start()

    assert process_context.process.is_alive() is False
    assert process_context.process.join_calls == 1
    assert driver.stop_calls == 1


def test_native_owner_starts_and_stops_one_real_latest_only_matcher_process():
    captured_ns = time.monotonic_ns()
    scan = RplidarScan(
        1,
        captured_ns,
        tuple(
            RplidarPoint(float(angle), 1.0 + angle / 1_000.0, 20)
            for angle in range(0, 360, 20)
        ),
    )
    driver = Driver(scan)
    port = NativeLidarPort(
        _config(),
        lambda: (0.0, 0.0, 0.0),
        driver=driver,
    )

    try:
        assert port.start() is True
        deadline = time.monotonic() + 4.0
        while port.get_matcher_result() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        result = port.get_matcher_result()
        assert result is not None
        assert result.source_raw_scan_id == 1
        assert port.get_runtime_status()["matcher_process_alive"] is True
    finally:
        port.stop()

    assert driver.stop_calls == 1
