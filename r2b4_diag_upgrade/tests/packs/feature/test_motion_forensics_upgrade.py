"""Offline unit checks. Run from repository root after installer."""
from types import SimpleNamespace
from tools.diag.analyzers.motion_forensics import _decision_quality, _event_timeline, _motion

class FakeBundle:
    def __init__(self, rows):
        self.rows = rows
    def has_view(self, name):
        return name in self.rows
    def iter_view(self, name):
        return iter(self.rows[name])

def row(payload, idx, topic='ticks'):
    return SimpleNamespace(payload=payload, log_time_ns=idx * 200000000,
                           message_id=f'msg-{idx}', tick_id=idx,
                           topic=topic, source_pointer='')

def test_empty_events_and_missing_movement_views():
    bundle = FakeBundle({'events': [], 'layers/L3': []})
    assert _event_timeline(bundle)['total_events'] == 0
    assert _decision_quality(bundle)['counts'] == {}
    assert _motion(bundle)['counts'] == {}

def test_navigation_event_source_identity():
    payload = {'source_topic': 'v3.navigation_decision', 'payload': {
        'schema': 'R2B4_NAVIGATION_DECISION_V1', 'monotonic_ns': 123,
        'tick_id': 7, 'event_sequence': 4, 'changes': ['goal'],
        'expected': {'layers': {'L6': {'local_goal': [1, 2]}, 'L12': {'safety_decision': 'ALLOW'}}}}}
    result = _event_timeline(FakeBundle({'events': [row(payload, 1)]}))
    assert result['total_events'] == 1
    assert result['change_counts']['goal'] == 1
    assert result['first_events'][0]['time_ns'] == 123
