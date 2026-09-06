import json

import pytest

from v3.capture import V3CaptureError, inspect_capture, load_capture
from v3_validation_helpers import create_general_capture


def test_general_capture_contains_typed_inputs_config_once_and_ordered_l1_l12(tmp_path):
    path = create_general_capture(tmp_path)

    payload = load_capture(path)
    inspected = inspect_capture(path)

    assert payload["schema"] == "R2B4_V3_CAPTURE_V1"
    assert payload["capture_id"] == "general-v3"
    assert "profile" not in payload
    assert set(payload["configuration"]) == {"physics", "speed_map", "hardware"}
    assert payload["ticks"][0]["inputs"]["__type__"] == "TickInputs"
    assert set(payload["ticks"][0]["expected"]["layers"]) == set(
        f"L{index}" for index in range(1, 13)
    )
    assert inspected["status"] == "PASS"
    assert inspected["tick_count"] == 5


def test_general_capture_checksum_rejects_modified_evidence(tmp_path):
    path = create_general_capture(tmp_path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["ticks"][1]["expected"]["layers"]["L10"]["left_mps"] += 0.01
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(V3CaptureError, match="checksum mismatch"):
        load_capture(path)
