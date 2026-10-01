"""Lossless projections only; every projection carries a source JSON pointer."""
import hashlib
import re


def topic_name(topic: str | None) -> str:
    text = topic or 'unknown'
    readable = re.sub(r'[^a-zA-Z0-9_-]+', '_', text).strip('_')[:80] or 'root'
    return readable + '_' + hashlib.sha256(text.encode()).hexdigest()[:12]


def pointer_key(key: str) -> str:
    return key.replace('~', '~0').replace('/', '~1')


def leaves(value, path=''):
    if isinstance(value, dict) and value:
        for key, child in value.items():
            yield from leaves(child, path + '/' + pointer_key(key))
    elif isinstance(value, list) and value:
        for i, child in enumerate(value):
            yield from leaves(child, path + '/' + str(i))
    else:
        yield path, value


def resolve(value, pointer: str):
    for part in pointer.split('/')[1:]:
        key = part.replace('~1', '/').replace('~0', '~')
        value = value[int(key)] if isinstance(value, list) else value[key]
    return value


def views(source):
    if source['decode_status'] != 'JSON':
        return
    payload = source['payload']
    topic = source['topic']
    name = {
        '/r2b4/tick': 'ticks', '/r2b4/event': 'events',
        '/r2b4/runtime': 'runtime', '/r2b4/checkpoint': 'checkpoints',
        '/r2b4/raw_lidar': 'lidar/raw_scans',
    }.get(topic, 'topics/' + topic_name(topic))
    yield name, '', payload
    if isinstance(payload, dict):
        layers = payload.get('expected', {}).get('layers') if isinstance(payload.get('expected'), dict) else None
        if isinstance(layers, dict):
            for layer, value in layers.items():
                if re.fullmatch(r'L(?:[0-9]|1[0-2])', layer):
                    yield 'layers/' + layer, '/expected/layers/' + layer, value
        # These views project actual sensor fields only, with no interpretation.
        for pointer, sample in samples(payload):
            yield 'sensors/' + topic_name(sample['kind']), pointer, sample
        sensors = payload.get('sensors')
        if isinstance(sensors, dict):
            for kind, value in sensors.items():
                yield 'sensors/' + topic_name(kind), '/sensors/' + pointer_key(kind), value


def samples(payload):
    """Locate explicit sample collections without interpreting their values."""
    if not isinstance(payload, dict):
        return
    for root in ('inputs', 'edge_fault'):
        parent = payload.get(root)
        raw = parent.get('raw_devices') if isinstance(parent, dict) else None
        values = raw.get('samples') if isinstance(raw, dict) else None
        if isinstance(values, list):
            for i, sample in enumerate(values):
                if isinstance(sample, dict) and isinstance(sample.get('kind'), str):
                    yield f'/{root}/raw_devices/samples/{i}', sample
