from dataclasses import FrozenInstanceError
from itertools import chain, repeat
import time
import pytest
from v3.adapters.rplidar_c1 import NativeRplidarC1, RplidarC1Config, RplidarScan, decode_standard_packet

def _packet(*, start: bool, angle_deg: float, distance_m: float, quality: int=15) -> bytes:
    angle_q6 = int(round(angle_deg * 64.0))
    distance_q2 = int(round(distance_m * 4000.0))
    b0 = quality << 2 | (1 if start else 2)
    b1 = (angle_q6 & 127) << 1 | 1
    b2 = angle_q6 >> 7 & 255
    return bytes((b0, b1, b2, distance_q2 & 255, distance_q2 >> 8))

def test_standard_packet_decode_preserves_framing_angle_distance_and_quality():
    decoded = decode_standard_packet(_packet(start=True, angle_deg=12.5, distance_m=1.25, quality=23))
    assert decoded is not None
    assert decoded.new_scan_start is True
    assert decoded.point.angle_deg == 12.5
    assert decoded.point.distance_m == 1.25
    assert decoded.point.quality == 23
    assert decode_standard_packet(b'\x00' * 5) is None

def test_native_driver_recovers_alignment_and_publishes_only_complete_scan():
    timestamps = chain((900000000, 930000000, 970000000), repeat(1000000000))
    driver = NativeRplidarC1(RplidarC1Config(minimum_distance_m=0.05, maximum_distance_m=3.0), serial_factory=lambda *_args, **_kwargs: None, monotonic_ns=lambda: next(timestamps))
    driver.ingest_for_test(b'\xff' + _packet(start=True, angle_deg=0.0, distance_m=1.0) + _packet(start=False, angle_deg=90.0, distance_m=1.5) + _packet(start=False, angle_deg=180.0, distance_m=4.0) + _packet(start=True, angle_deg=1.0, distance_m=1.1))
    scan = driver.get_latest_scan()
    assert scan is not None
    assert scan.revision == 1
    assert scan.captured_monotonic_ns == 1000000000
    assert scan.scan_start_monotonic_ns == 900000000
    assert scan.scan_end_monotonic_ns == 1000000000
    assert scan.measurement_monotonic_ns == 950000000
    assert scan.scan_start_monotonic_ns <= scan.measurement_monotonic_ns <= scan.scan_end_monotonic_ns
    assert tuple(((point.angle_deg, point.distance_m) for point in scan.points)) == ((0.0, 1.0), (90.0, 1.5))
    assert driver.get_runtime_status()['invalid_packet_count'] == 1

def test_mid_revolution_join_waits_for_two_boundaries_before_first_scan():
    timestamps = iter((10, 20, 30, 40, 50, 60, 70))
    driver = NativeRplidarC1(RplidarC1Config(), serial_factory=lambda *_args, **_kwargs: None, monotonic_ns=lambda: next(timestamps))
    driver.ingest_for_test(_packet(start=False, angle_deg=180.0, distance_m=2.0) + _packet(start=False, angle_deg=270.0, distance_m=2.0))
    assert driver.get_latest_scan() is None
    driver.ingest_for_test(_packet(start=True, angle_deg=0.0, distance_m=1.0))
    assert driver.get_latest_scan() is None
    driver.ingest_for_test(b''.join((_packet(start=False, angle_deg=angle, distance_m=1.0) for angle in (90.0, 180.0, 270.0))))
    assert driver.get_latest_scan() is None
    driver.ingest_for_test(_packet(start=True, angle_deg=1.0, distance_m=1.5))
    scan = driver.get_latest_scan()
    assert scan is not None
    assert scan.revision == 1
    assert scan.scan_start_monotonic_ns == 30
    assert scan.scan_end_monotonic_ns == scan.captured_monotonic_ns == 70
    assert scan.measurement_monotonic_ns == 50
    assert tuple(((point.angle_deg, point.distance_m) for point in scan.points)) == ((0.0, 1.0), (90.0, 1.0), (180.0, 1.0), (270.0, 1.0))

def test_same_packets_and_clock_produce_same_scan_measurement_timestamp():
    payload = _packet(start=True, angle_deg=0.0, distance_m=1.0) + _packet(start=False, angle_deg=90.0, distance_m=1.5) + _packet(start=True, angle_deg=1.0, distance_m=1.1)

    def scan_once():
        timestamps = iter((10, 30, 70))
        driver = NativeRplidarC1(RplidarC1Config(), serial_factory=lambda *_args, **_kwargs: None, monotonic_ns=lambda: next(timestamps))
        driver.ingest_for_test(payload)
        return driver.get_latest_scan()
    assert scan_once() == scan_once()

def test_driver_config_is_immutable_and_rejects_unbounded_distance_contract():
    config = RplidarC1Config()
    with pytest.raises(FrozenInstanceError):
        config.baudrate = 115200
    with pytest.raises(ValueError, match='exceed'):
        RplidarC1Config(minimum_distance_m=2.0, maximum_distance_m=1.0)
    with pytest.raises(ValueError, match='midpoint'):
        RplidarScan(revision=1, captured_monotonic_ns=30, scan_start_monotonic_ns=10, scan_end_monotonic_ns=30, measurement_monotonic_ns=21, points=())


class StartupSerial:
    """Finite device responses, exposed only after the corresponding SCAN."""
    def __init__(self, attempts):
        self.attempts = iter(attempts)
        self.pending = []
        self.writes = []
        self.read_sizes = []
        self.is_open = True
        self.on_read = None

    def write(self, value):
        self.writes.append(value)
        if value == b'\xa5\x20':
            self.pending = list(next(self.attempts, ()))
        return len(value)

    def read(self, size):
        self.read_sizes.append(size)
        if self.on_read is not None:
            self.on_read()
        if not self.pending:
            time.sleep(.001)
            return b''
        value = self.pending[0][:size]
        self.pending[0] = self.pending[0][size:]
        if not self.pending[0]:
            self.pending.pop(0)
        return value

    def reset_input_buffer(self):
        self.pending.clear()

    def cancel_read(self):
        pass

    def close(self):
        self.is_open = False


def startup_driver(port):
    driver = NativeRplidarC1(
        RplidarC1Config(command_settle_s=.002, read_timeout_s=.01),
        serial_factory=lambda *_args, **_kwargs: port,
    )
    # These tests own the real start/stop handshake; packet acquisition has
    # separate complete-scan tests above and must not consume our witness bytes.
    driver._run = lambda: None
    return driver


def test_fragmented_startup_ack_preserves_first_measurement_and_closes_on_stop():
    packet = _packet(start=True, angle_deg=0, distance_m=1)
    port = StartupSerial(((b'\xa5', b'\x5a\x05', b'\x00\x00\x40\x81' + packet),))
    driver = startup_driver(port)
    try:
        assert driver.start() is True
        assert port.read_sizes == [7, 6, 4]
        assert b''.join(port.pending) == packet
        status = driver.get_runtime_status()
        assert status['scan_descriptor'] == 'a55a0500004081'
        assert status['startup_reset_count'] == 0
    finally:
        driver.stop()
    assert port.is_open is False


@pytest.mark.parametrize('first', ((), (b'\xa5\x5a',), (b'\xa5\x5a\x05\x00\x00\x40\x82',)))
def test_one_protocol_reset_recovers_silent_partial_or_wrong_scan_ack(first):
    port = StartupSerial((first, (b'\xa5\x5a\x05\x00\x00\x40\x81',)))
    driver = startup_driver(port)
    try:
        assert driver.start() is True
        assert port.writes == [b'\xa5\x25', b'\xa5\x20', b'\xa5\x40', b'\xa5\x25', b'\xa5\x20']
        assert driver.get_runtime_status()['startup_reset_count'] == 1
    finally:
        driver.stop()


@pytest.mark.parametrize('response', ((), (b'\xa5\x5a',), (b'\xa5\x5a\x06\x00\x00\x40\x81',), (b'\xa5\x5a\x05\x00\x00\x00\x81',)))
def test_unacknowledged_start_is_bounded_fails_closed_and_releases_serial(response):
    port = StartupSerial((response, response))
    driver = startup_driver(port)
    started = time.monotonic()
    assert driver.start() is False
    assert time.monotonic() - started < .25
    status = driver.get_runtime_status()
    assert status['running'] is False and status['connected'] is False
    assert status['scan_revision'] == 0 and status['scan_descriptor'] == ''
    assert 'RPLIDAR_SCAN_START_FAILED_AFTER_RESET' in status['last_error']
    assert port.writes.count(b'\xa5\x40') == 1
    assert port.writes[-1] == b'\xa5\x25'
    assert port.is_open is False


def test_cancellation_during_descriptor_wait_never_resets_or_starts_acquisition():
    port = StartupSerial(((),))
    driver = startup_driver(port)
    port.on_read = driver._stop_event.set
    assert driver.start() is False
    assert 'InterruptedError' in driver.get_runtime_status()['last_error']
    assert b'\xa5\x40' not in port.writes
    assert port.is_open is False
