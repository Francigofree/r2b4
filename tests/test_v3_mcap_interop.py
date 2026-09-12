"""External format gate; install requirements-interop.txt only in a test venv.

Reference API: https://mcap.dev/docs/python/mcap-apidoc/mcap.reader
"""
import io

import pytest

from v3.mcap_reader import McapReader
from v3.mcap_writer import StdlibMcapWriter


def test_independent_mcap_reader_validates_our_indexes_messages_metadata_and_crcs(tmp_path):
    mcap = pytest.importorskip('mcap.reader', reason='dedicated MCAP interoperability dependency')
    path = tmp_path / 'ours.mcap'
    with path.open('wb') as stream:
        writer = StdlibMcapWriter(stream, chunk_target_bytes=100)
        writer.start()
        tick = writer.register_channel('/tick')
        event = writer.register_channel('/event')
        writer.add_metadata('run', {'id': 'interop'})
        for i in range(30):
            writer.add_message(tick if i % 2 else event, log_time_ns=1000 + i,
                               publish_time_ns=2000 + i, sequence=i, data=str(i).encode())
        writer.finish()
    with path.open('rb') as stream:
        external = mcap.SeekingReader(stream, validate_crcs=True)
        summary = external.get_summary()
        assert summary.statistics.message_count == 30
        assert summary.statistics.chunk_count > 1
        selected = list(external.iter_messages(topics=['/tick'], start_time=1005, end_time=1010))
        assert [msg.sequence for _, _, msg in selected] == [5, 7, 9]
        assert list(external.iter_metadata())[0].metadata == {'id': 'interop'}
    with path.open('rb') as stream:
        rows = list(mcap.NonSeekingReader(stream, validate_crcs=True).iter_messages(log_time_order=False))
        assert [msg.publish_time for _, _, msg in rows] == list(range(2000, 2030))
    own = McapReader(path)
    assert own.inspect(verify_chunks=True).valid
    assert [m.sequence for m in own.iter_messages(topics=['/tick'], start_ns=1005, end_ns=1009)] == [5, 7, 9]


def test_our_reader_accepts_independent_uncompressed_writer(tmp_path):
    external = pytest.importorskip('mcap.writer')
    path = tmp_path / 'external.mcap'
    with path.open('wb') as stream:
        writer = external.Writer(stream, chunk_size=100, compression=external.CompressionType.NONE,
                                 enable_crcs=True, enable_data_crcs=True)
        writer.start(profile='interop')
        channel = writer.register_channel('/tick', 'json', 0)
        writer.add_metadata('run', {'id': 'external'})
        for i in range(12):
            writer.add_message(channel, log_time=100 + i, publish_time=200 + i,
                               sequence=i, data=b'{}')
        writer.finish()
    reader = McapReader(path)
    assert reader.inspect(verify_chunks=True).valid
    assert [m.sequence for m in reader.iter_messages(topics=['/tick'])] == list(range(12))
    assert reader.latest_metadata('run') == {'id': 'external'}
