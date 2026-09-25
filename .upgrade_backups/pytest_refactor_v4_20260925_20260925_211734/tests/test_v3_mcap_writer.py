from __future__ import annotations

import io
import struct
import zlib

from v3.mcap_writer import MAGIC, StdlibMcapWriter


def _u32(data: bytes, offset: int) -> tuple[int, int]:
    return struct.unpack_from("<I", data, offset)[0], offset + 4


def _u64(data: bytes, offset: int) -> tuple[int, int]:
    return struct.unpack_from("<Q", data, offset)[0], offset + 8


def _string(data: bytes, offset: int) -> tuple[str, int]:
    length, offset = _u32(data, offset)
    return data[offset : offset + length].decode(), offset + length


def _records(data: bytes, start: int, end: int):
    offset = start
    while offset < end:
        record_start = offset
        opcode = data[offset]
        length = struct.unpack_from("<Q", data, offset + 1)[0]
        content_start = offset + 9
        content_end = content_start + length
        yield record_start, opcode, data[content_start:content_end]
        offset = content_end
    assert offset == end


def test_stdlib_writer_emits_indexed_crc_protected_mcap() -> None:
    stream = io.BytesIO()
    writer = StdlibMcapWriter(stream, chunk_target_bytes=96)
    writer.start()
    tick = writer.register_channel("/r2b4/tick", message_encoding="json")
    event = writer.register_channel("/r2b4/event", message_encoding="json")
    writer.add_metadata("r2b4.capture", {"capture_id": "smoke"})
    for index in range(6):
        writer.add_message(
            tick,
            log_time_ns=1_000 + index,
            sequence=index,
            data=(b'{"tick":' + str(index).encode() + b"}"),
        )
    writer.add_message(
        event,
        log_time_ns=2_000,
        sequence=1,
        data=b'{"event":"done"}',
    )
    writer.add_metadata("r2b4.capture.final", {"complete": "true"})
    writer.finish()

    data = stream.getvalue()
    assert data.startswith(MAGIC)
    assert data.endswith(MAGIC)

    footer_start = len(data) - len(MAGIC) - (1 + 8 + 20)
    assert data[footer_start] == 0x02
    assert struct.unpack_from("<Q", data, footer_start + 1)[0] == 20
    summary_start = struct.unpack_from("<Q", data, footer_start + 9)[0]
    summary_offset_start = struct.unpack_from("<Q", data, footer_start + 17)[0]
    summary_crc = struct.unpack_from("<I", data, footer_start + 25)[0]
    assert 0 < summary_start < summary_offset_start < footer_start

    calculated_summary_crc = zlib.crc32(data[summary_start:footer_start])
    calculated_summary_crc = zlib.crc32(
        data[footer_start : footer_start + 25], calculated_summary_crc
    ) & 0xFFFFFFFF
    assert summary_crc == calculated_summary_crc

    top = list(_records(data, len(MAGIC), summary_start))
    data_end = top[-1]
    assert data_end[1] == 0x0F
    expected_data_crc = struct.unpack_from("<I", data_end[2], 0)[0]
    assert expected_data_crc == (zlib.crc32(data[: data_end[0]]) & 0xFFFFFFFF)

    chunk_count = 0
    for _, opcode, content in top:
        if opcode != 0x06:
            continue
        chunk_count += 1
        offset = 0
        _, offset = _u64(content, offset)  # start time
        _, offset = _u64(content, offset)  # end time
        uncompressed_size, offset = _u64(content, offset)
        expected_chunk_crc, offset = _u32(content, offset)
        compression, offset = _string(content, offset)
        assert compression == ""
        records_size, offset = _u64(content, offset)
        records = content[offset : offset + records_size]
        assert len(records) == uncompressed_size
        assert expected_chunk_crc == (zlib.crc32(records) & 0xFFFFFFFF)
    assert chunk_count >= 2

    summary_opcodes = [opcode for _, opcode, _ in _records(data, summary_start, summary_offset_start)]
    assert 0x04 in summary_opcodes  # Channel
    assert 0x08 in summary_opcodes  # Chunk Index
    assert 0x0B in summary_opcodes  # Statistics
    assert 0x0D in summary_opcodes  # Metadata Index
