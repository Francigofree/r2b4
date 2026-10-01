"""MCAP framing primitives, independent of the strict production reader."""
from dataclasses import dataclass
import struct

MAGIC = b"\x89MCAP0\r\n"
HEADER, FOOTER, SCHEMA, CHANNEL, MESSAGE, CHUNK = range(1, 7)
METADATA, DATA_END = 12, 15


@dataclass(frozen=True)
class WorkUnit:
    number: int
    path: str
    message_count: int


class Cursor:
    def __init__(self, data: bytes):
        self.data, self.pos = data, 0

    def take(self, n: int) -> bytes:
        if n < 0 or self.pos + n > len(self.data):
            raise ValueError("truncated record content")
        value = self.data[self.pos:self.pos + n]
        self.pos += n
        return value

    def unpack(self, fmt: str):
        return struct.unpack(fmt, self.take(struct.calcsize(fmt)))

    def string(self) -> str:
        return self.take(self.unpack('<I')[0]).decode('utf-8')

    def mapping(self) -> dict:
        end = self.unpack('<I')[0] + self.pos
        result = {}
        while self.pos < end:
            key = self.string()
            result[key] = self.string()
        if self.pos != end:
            raise ValueError('invalid map size')
        return result
