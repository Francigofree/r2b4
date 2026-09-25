from r2b4_voice.safety_intents import is_stop_intent


def test_stop_intent_is_exact_and_accent_tolerant():
    assert is_stop_intent("Állj meg!")
    assert is_stop_intent("Alba, állj meg!")
    assert is_stop_intent("STOP")
    assert is_stop_intent("stop alba")
    assert not is_stop_intent("ne állj meg")
    assert not is_stop_intent("beszélj a stop gombról")
    assert not is_stop_intent("kövess engem")
