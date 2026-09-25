import json
from pathlib import Path

from r2b4_voice.self_knowledge import SelfKnowledgeProvider


def test_self_knowledge_is_source_first_bounded_and_never_reads_secret(tmp_path: Path):
    (tmp_path / "conf").mkdir()
    (tmp_path / "v3").mkdir()
    (tmp_path / "runtime" / "captures" / "run_001.mcap.evidence").mkdir(parents=True)
    (tmp_path / "conf" / "hardver.json").write_text(json.dumps({"lidar": {"model": "C1"}}), encoding="utf-8")
    (tmp_path / "conf" / "fizika.json").write_text(json.dumps({"track_m": 0.3557}), encoding="utf-8")
    (tmp_path / "conf" / "vezerles.json").write_text(json.dumps({"max_v_mps": 0.5}), encoding="utf-8")
    (tmp_path / "conf" / "speed_map.json").write_text(json.dumps({"left": [1, 2]}), encoding="utf-8")
    (tmp_path / "conf" / ".wake.env").write_text("GEMINI_API_KEY=TOP_SECRET\n", encoding="utf-8")
    (tmp_path / "STRUKTURALIS_RETEGEK_V3.md").write_text("# L12\nL12 owns final safety and MotorWriter.\n", encoding="utf-8")
    (tmp_path / "v3" / "layers.py").write_text("class L12Safety:\n    pass\n", encoding="utf-8")
    evidence = tmp_path / "runtime" / "captures" / "run_001.mcap.evidence"
    (evidence / "agent_view.json").write_text(json.dumps({"status": "PASS", "replay_status": "MATCH"}), encoding="utf-8")
    (evidence / "diagnosis.json").write_text(json.dumps({"summary": "smooth motion"}), encoding="utf-8")

    (evidence / "runtime_performance.json").write_text(
        json.dumps({
            "runtime_tick": {"average_hz": 49.1, "average_period_ms": 20.36, "period_max_ns": 42000000},
            "slow_tick_correlation": {"period_summary": {"p50_ms": 19.9, "p95_ms": 26.1, "p99_ms": 33.4}},
        }),
        encoding="utf-8",
    )

    provider = SelfKnowledgeProvider(tmp_path)
    config = provider.build("Milyen lidar konfigurációd van?")
    assert "configuration" in config
    assert "C1" in json.dumps(config, ensure_ascii=False)
    assert "TOP_SECRET" not in json.dumps(config, ensure_ascii=False)

    architecture = provider.build("Mit csinál az L12 a source szerint?")
    rendered = json.dumps(architecture, ensure_ascii=False)
    assert "MotorWriter" in rendered
    assert "layers.py" in rendered

    runs = provider.build("Milyen volt az utolsó futás és replay evidence?")
    rendered_runs = json.dumps(runs, ensure_ascii=False)
    assert "PASS" in rendered_runs
    assert "MATCH" in rendered_runs
    assert "smooth motion" in rendered_runs
    assert "runtime_average_hz" in rendered_runs
    assert "49.1" in rendered_runs
    assert "runtime_period_p99_ms" in rendered_runs

    # Natural Hungarian suffixes must still route to the right source-first surfaces.
    inflected_config = provider.build("Mit tudsz a kamerádról és motorodról?")
    assert "configuration" in inflected_config["matched_categories"]

    inflected_architecture = provider.build("Mesélj a rétegrendedről és forrásfájljaidról.")
    categories = set(inflected_architecture["matched_categories"])
    assert "architecture" in categories
    assert "source" in categories

    inflected_runs = provider.build("Milyenek voltak a futásaid?")
    assert "evidence" in inflected_runs["matched_categories"]
