"""Linux USB serial transport and protocol-compatible simulation."""
import errno
import math
import os
import struct
import time
from .protocol import Kind, Parser, Telemetry, CONFIG, REMOTE_CONFIG, frame


class SerialTransport:
    def __init__(self, path):
        import fcntl
        import termios
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except Exception:
            os.close(self.fd)
            raise

    def write(self, data):
        # Frames are < 74 bytes. Never queue an old motion command indefinitely.
        written = os.write(self.fd, data)
        if written != len(data):
            raise OSError('short serial write')

    def read(self):
        import select
        if not select.select([self.fd], [], [], 0)[0]:
            return b''
        data = os.read(self.fd, 2048)
        if not data:
            raise OSError('serial device disconnected')
        return data

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class MockTransport:
    """First-order motor model, not a traction/electrical simulator."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.start = self.last = self.last_state = clock()
        self.last_command = clock()
        self.parser = Parser()
        self.rx = bytearray()
        self.armed = self.configured = False
        self.config_seq = self.command_seq = 0
        self.rpm = [0.0] * 4
        self.position = [0.0] * 4
        self.targets = [0.0] * 4
        self.pwm = [0] * 4
        self.cpr = [4320] * 4
        self.max_pwm = [100] * 4
        self.max_rpm = 30.0
        self.timeout = 0.3
        self.mode = 1
        self.remote_active = self.remote_connected = self.remote_configured = False
        self.remote_enabled = False
        self.last_heartbeat = clock()

    def write(self, data):
        for kind, seq, payload in self.parser.feed(data, self.clock()):
            result = 0
            if kind == Kind.CONFIG and len(payload) == CONFIG.size and not self.armed:
                cfg = CONFIG.unpack(payload)
                self.cpr = list(cfg[:4])
                self.max_pwm = list(cfg[16:20])
                self.max_rpm = cfg[-3] / 100
                self.timeout = cfg[-1] / 1000
                self.config_seq = seq
                self.configured = True
                self.remote_configured = False
                self.remote_enabled = False
            elif kind == Kind.REMOTE_CONFIG and len(payload) == REMOTE_CONFIG.size and not self.armed:
                enabled, *config = REMOTE_CONFIG.unpack(payload)
                self.remote_enabled = bool(enabled)
                self.remote_map, self.remote_max_rpm = config[:4], config[4] / 100
                self.remote_configured = True
            elif kind == Kind.HEARTBEAT and not payload and self.remote_configured:
                self.last_heartbeat = self.clock()
                continue
            elif kind == Kind.ARM and payload in (b'\x00', b'\x01'):
                if payload == b'\x01' and (not self.configured or self.remote_active):
                    result = 2
                else:
                    self.armed = bool(payload[0])
                    self.targets = [0.0] * 4
                    self.last_command = self.clock()
            elif kind == Kind.DRIVE and len(payload) == 9 and self.armed and not self.remote_active:
                mode, *values = struct.unpack('<B4h', payload)
                if mode not in (1, 2):
                    result = 1
                else:
                    self.mode = mode
                    self.targets = [max(-self.max_rpm, min(self.max_rpm, v/100)) for v in values] if mode == 1 else [max(-m, min(m, v)) for v, m in zip(values, self.max_pwm)]
                    self.last_command = self.clock()
                    self.command_seq = seq
            elif kind == Kind.STOP and not payload:
                self.armed = False
                self.remote_active = False
                self.targets = [0.0] * 4
            elif kind == Kind.RESET and not payload and not self.armed:
                self.position = [0.0] * 4
            else:
                result = 1
            if kind != Kind.DRIVE:
                self.rx.extend(frame(Kind.ACK, seq, bytes([kind, result])))

    def read(self):
        now = self.clock()
        dt = max(0, min(now - self.last, 0.1))
        self.last = now
        if self.armed and (now - self.last_command > self.timeout or
                           (self.remote_active and now-self.last_heartbeat > self.timeout)):
            self.armed = False
            self.remote_active = False
            self.targets = [0.0] * 4
        for i in range(4):
            target = self.targets[i] if self.armed else 0.0
            if self.mode == 2:
                target *= 107 / 255
            target = max(-107*self.max_pwm[i]/255, min(107*self.max_pwm[i]/255, target))
            self.rpm[i] += (target - self.rpm[i]) * (1 - math.exp(-dt / 0.18))
            self.position[i] += self.rpm[i] * self.cpr[i] * dt / 60
            self.pwm[i] = round(target / 107 * 255) if self.armed else 0
        if now - self.last_state >= 0.05:
            self.last_state = now
            state = Telemetry(int((now-self.start)*1000) & 0xffffffff,
                              int(self.armed) | (int(self.configured) << 1) |
                              (int(self.remote_active) << 3) | (int(self.remote_connected) << 4) |
                              (int(self.remote_configured) << 5) | 64,
                              self.config_seq, self.command_seq,
                              [((round(v)+(1<<31)) % (1<<32))-(1<<31) for v in self.position],
                              self.rpm[:], self.pwm[:],
                              [round(v * (100 if self.mode == 1 else 1)) for v in self.targets])
            self.rx.extend(frame(Kind.STATE, 0, state.encode()))
        result = bytes(self.rx)
        self.rx.clear()
        return result

    def close(self):
        self.armed = False
