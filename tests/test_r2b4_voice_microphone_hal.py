from __future__ import annotations

from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from v3.adapters.microphone import (
    AudioFrame,
    AudioFramePort,
    MicrophoneIdentity,
    MicrophoneStreamConfig,
    R2B4_USB_PNP_MIC,
    build_arecord_argv,
    resolve_usb_microphone,
)


def _write(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")


def _fake_usb_device(root: Path, name: str = "1-2", card_id: str = "Device") -> None:
    device = root / name
    device.mkdir(parents=True)
    _write(device / "idVendor", "08bb\n")
    _write(device / "idProduct", "2902\n")
    _write(device / "manufacturer", "C-Media Electronics Inc.\n")
    _write(device / "product", "USB PnP Sound Device\n")
    interface = root / f"{name}:1.0" / "sound" / "card1"
    _write(interface / "id", card_id + "\n")


def _frame(sequence: int, payload: bytes = b"\x00\x01") -> AudioFrame:
    return AudioFrame(
        sequence=sequence,
        read_monotonic_ns=sequence * 1_000_000,
        sample_rate_hz=48_000,
        channels=1,
        sample_format="S16_LE",
        sample_count=1,
        pcm=payload,
    )


def test_current_microphone_profile_matches_measured_usb_contract():
    spec = R2B4_USB_PNP_MIC
    assert spec.usb_vid == 0x08BB
    assert spec.usb_pid == 0x2902
    assert spec.manufacturer == "C-Media Electronics Inc."
    assert spec.product == "USB PnP Sound Device"
    assert spec.supported_sample_rates_hz == (44_100, 48_000)
    assert spec.channels == 1
    assert spec.sample_format == "S16_LE"


def test_native_stream_is_48khz_mono_s16le_twenty_ms():
    config = MicrophoneStreamConfig()
    assert config.sample_rate_hz == 48_000
    assert config.channels == 1
    assert config.sample_format == "S16_LE"
    assert config.frame_samples == 960
    assert config.frame_bytes == 1920


def test_hal_rejects_hidden_16khz_resampling():
    with pytest.raises(ValueError, match="native rates"):
        MicrophoneStreamConfig(sample_rate_hz=16_000)


def test_stream_contract_is_immutable():
    config = MicrophoneStreamConfig()
    with pytest.raises(FrozenInstanceError):
        config.sample_rate_hz = 44_100


def test_resolver_maps_unique_usb_device_to_stable_alsa_card_id(tmp_path: Path):
    _fake_usb_device(tmp_path)
    identity = resolve_usb_microphone(usb_sysfs_root=tmp_path)
    assert identity.usb_path == "1-2"
    assert identity.alsa_card_id == "Device"
    assert identity.alsa_pcm_name == "hw:CARD=Device,DEV=0"
    assert identity.serial is None


def test_resolver_fails_closed_when_microphone_is_absent(tmp_path: Path):
    with pytest.raises(LookupError, match="not connected"):
        resolve_usb_microphone(usb_sysfs_root=tmp_path)


def test_resolver_fails_closed_when_two_serialless_microphones_match(tmp_path: Path):
    _fake_usb_device(tmp_path, "1-2", "Device")
    _fake_usb_device(tmp_path, "1-3", "Device_1")
    with pytest.raises(LookupError, match="ambiguous"):
        resolve_usb_microphone(usb_sysfs_root=tmp_path)


def test_arecord_command_uses_direct_hw_without_conversion():
    identity = MicrophoneIdentity(
        usb_vid=0x08BB,
        usb_pid=0x2902,
        manufacturer="C-Media Electronics Inc.",
        product="USB PnP Sound Device",
        serial=None,
        usb_path="1-2",
        alsa_card_id="Device",
        alsa_device=0,
    )
    argv = build_arecord_argv(identity, MicrophoneStreamConfig())
    assert argv == (
        "arecord",
        "-q",
        "-D",
        "hw:CARD=Device,DEV=0",
        "-t",
        "raw",
        "-f",
        "S16_LE",
        "-r",
        "48000",
        "-c",
        "1",
        "-",
    )
    assert "plughw" not in argv


def test_audio_port_is_bounded_and_drops_oldest_without_blocking():
    port = AudioFramePort(capacity_frames=2)
    port.publish(_frame(1))
    port.publish(_frame(2))
    port.publish(_frame(3))
    assert port.overwrite_count == 1
    first_available = port.read_after(0, timeout_s=0)
    assert first_available is not None
    assert first_available.sequence == 2


def test_ring_overwrite_is_not_itself_consumer_loss():
    port = AudioFramePort(capacity_frames=2)
    port.publish(_frame(1))
    port.publish(_frame(2))
    assert port.read_after(1, timeout_s=0).sequence == 2
    port.publish(_frame(3))
    assert port.overwrite_count == 1
    next_frame = port.read_after(2, timeout_s=0)
    assert next_frame is not None
    assert next_frame.sequence == 3


def test_audio_port_rejects_non_monotonic_sequence():
    port = AudioFramePort(capacity_frames=2)
    port.publish(_frame(2))
    with pytest.raises(ValueError, match="strictly increasing"):
        port.publish(_frame(2))
