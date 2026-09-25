from pathlib import Path


def test_voice_prompt_does_not_overclaim_all_sensor_health():
    root = Path(__file__).resolve().parents[1]
    text = (root / "conf" / "voice_llm_system.md").read_text(encoding="utf-8")
    assert "minden szenzor rendben van" in text
    assert "minden releváns jelentett health forrás" in text
    assert "explicit OK" in text
    assert "SHADOW" in text


def test_user_systemd_unit_routes_to_new_voice_supervisor_without_secret_environmentfile():
    root = Path(__file__).resolve().parents[1]
    text = (root / "deploy" / "systemd" / "r2b4-wake.service").read_text(encoding="utf-8")
    assert "-m r2b4_voice.voice_service" in text
    assert "EnvironmentFile=" not in text
