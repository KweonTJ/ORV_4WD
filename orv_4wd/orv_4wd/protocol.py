"""Version 1 wire protocol. All multibyte fields are little endian."""
from dataclasses import dataclass
from enum import IntEnum
import struct
import time

MAGIC = b'OR'
VERSION = 1
MAX_PAYLOAD = 64
HEADER = struct.Struct('<2sBBHB')
STATE = struct.Struct('<IBHH4i4h4h')
STATE_REMOTE = struct.Struct('<IBHH4i4h4h4h')
REMOTE_CONFIG = struct.Struct('<B4BH')
CONFIG = struct.Struct('<4H12H4B4B4b4bHHH')


class Kind(IntEnum):
    CONFIG = 1
    ARM = 2
    DRIVE = 3
    STOP = 4
    RESET = 5
    REMOTE_CONFIG = 6
    HEARTBEAT = 7
    STATE = 128
    ACK = 129


def crc16(data):
    value = 0xffff
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0x1021) & 0xffff if value & 0x8000 else (value << 1) & 0xffff
    return value


def frame(kind, sequence, payload=b''):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload exceeds 64 bytes')
    data = HEADER.pack(MAGIC, VERSION, int(kind), sequence & 0xffff, len(payload)) + payload
    return data + struct.pack('<H', crc16(data[2:]))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.last_rx = 0.0
        self.errors = 0

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        if self.buffer and now - self.last_rx > 0.1:
            self.buffer.clear()
            self.errors += 1
        if data:
            self.last_rx = now
        self.buffer.extend(data)
        packets = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != MAGIC:
                del self.buffer[0]
                continue
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, seq, size = HEADER.unpack_from(self.buffer)
            if version != VERSION or size > MAX_PAYLOAD:
                del self.buffer[0]
                self.errors += 1
                continue
            total = HEADER.size + size + 2
            if len(self.buffer) < total:
                break
            if crc16(self.buffer[2:total-2]) != struct.unpack_from('<H', self.buffer, total-2)[0]:
                del self.buffer[0]
                self.errors += 1
                continue
            packets.append((kind, seq, bytes(self.buffer[HEADER.size:total-2])))
            del self.buffer[:total]
        return packets


@dataclass
class Telemetry:
    uptime_ms: int
    flags: int
    config_sequence: int
    command_sequence: int
    ticks: list
    rpm: list
    pwm: list
    targets: list = None

    @property
    def armed(self):
        return bool(self.flags & 1)

    @property
    def configured(self):
        return bool(self.flags & 2)

    @property
    def remote_active(self):
        return bool(self.flags & 8)

    @property
    def remote_connected(self):
        return bool(self.flags & 16)

    @property
    def remote_configured(self):
        return bool(self.flags & 32)

    @property
    def remote_supported(self):
        return bool(self.flags & 64)

    def encode(self):
        base = STATE.pack(self.uptime_ms, self.flags, self.config_sequence,
                          self.command_sequence, *self.ticks,
                          *(round(v * 100) for v in self.rpm), *self.pwm)
        return base + struct.pack('<4h', *(self.targets or [0]*4)) if self.remote_supported else base

    @classmethod
    def decode(cls, payload):
        data = (STATE_REMOTE if len(payload) == STATE_REMOTE.size else STATE).unpack(payload)
        if bool(data[1] & 64) != (len(payload) == STATE_REMOTE.size):
            raise struct.error('remote capability and STATE size disagree')
        if data[1] & 8 and (not data[1] & 64 or not data[1] & 1):
            raise struct.error('remote ownership requires integrated firmware and armed state')
        return cls(*data[:4], list(data[4:8]), [v / 100 for v in data[8:12]],
                   list(data[12:16]), list(data[16:20]) if len(data) > 16 else None)


def delta32(current, previous):
    return ((current - previous + (1 << 31)) % (1 << 32)) - (1 << 31)
