"""Lossless projections only; every projection carries a source JSON pointer."""
import hashlib
import re


def topic_name(topic: str | None) -> str:
    text = topic or 'unknown'
    readable = re.sub(r'[^a-zA-Z0-9_-]+', '_', text).strip('_')[:80] or 'root'
    return readable + '_' + hashlib.sha256(text.encode()).hexdigest()[:12]


def pointer_key(key: str) -> str:
    return key.replace('~', '~0').replace('/', '~1')


# P0: large JSON arrays remain lossless in source payloads, but their scalar
# descendants are not expanded into one generic SQLite field posting each.
# 64 is intentionally structural and deterministic: it catches LiDAR point
# arrays while keeping ordinary short vectors fully indexed.
BULK_ARRAY_MIN_ITEMS = 64

FIELD_INDEX_POLICY = {
    'mode': 'sparse_bulk_arrays_v1',
    'bulk_array_min_items': BULK_ARRAY_MIN_ITEMS,
    'bulk_array_descendants': 'omitted_from_fields',
    'bulk_array_roots': 'stored_in_bulk_arrays',
}


def field_index_entries(value, path=''):
    """Yield sparse field postings as (kind, path, length).

    `field` entries preserve the legacy leaf-path index. `bulk_array` entries
    mark a large list root and stop descent. The payload itself is untouched.
    """
    if isinstance(value, dict) and value:
        for key, child in value.items():
            yield from field_index_entries(child, path + '/' + pointer_key(key))
    elif isinstance(value, list) and value:
        if len(value) >= BULK_ARRAY_MIN_ITEMS:
            yield 'bulk_array', path, len(value)
            return
        for i, child in enumerate(value):
            yield from field_index_entries(child, path + '/' + str(i))
    else:
        yield 'field', path, None


def identifier_entries(value):
    """Preserve identifier indexing independently from sparse field paths."""
    if isinstance(value, dict):
        scalar_kinds = {
            'tick_id': 'tick', 'layer': 'layer', 'layer_id': 'layer',
            'sensor_kind': 'sensor', 'sensor_id': 'sensor',
        }
        for key, child in value.items():
            kind = scalar_kinds.get(key)
            if kind and isinstance(child, (str, int)) and not isinstance(child, bool):
                yield kind, str(child)
            if key in ('layers', 'sensors') and isinstance(child, dict):
                structural_kind = 'layer' if key == 'layers' else 'sensor'
                for name in child:
                    yield structural_kind, str(name)
            if isinstance(child, (dict, list)):
                yield from identifier_entries(child)
    elif isinstance(value, list):
        for child in value:
            if isinstance(child, (dict, list)):
                yield from identifier_entries(child)


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
