from pathlib import Path


def test_voice_prompt_does_not_overclaim_all_sensor_health():
    root = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))
    text = (root / "conf" / "voice_llm_system.md").read_text(encoding="utf-8")
    assert "minden szenzor rendben van" in text
    assert "minden releváns jelentett health forrás" in text
    assert "explicit OK" in text
    assert "SHADOW" in text


def test_user_systemd_unit_routes_to_new_voice_supervisor_without_secret_environmentfile():
    root = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))
    text = (root / "deploy" / "systemd" / "r2b4-wake.service").read_text(encoding="utf-8")
    assert "-m r2b4_voice.voice_service" in text
    assert "EnvironmentFile=" not in text
