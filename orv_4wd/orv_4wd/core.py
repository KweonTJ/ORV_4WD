"""Single-owner control state machine, independent of ROS and HTTP threads."""
import math
import struct
import time
from .config import Config, twist_to_rpm
from .protocol import Kind, Parser, Telemetry, delta32, frame


class Controller:
    def __init__(self, factory, config, mock=False, clock=time.monotonic):
        self.factory, self.cfg, self.mock, self.clock = factory, config.validate(), mock, clock
        self.transport = None
        self.parser = Parser()
        self.telemetry = None
        self.last_rx = None
        self.connected_at = self.next_connect = 0.0
        self.config_seq = None
        self.remote_config_seq = None
        self.last_heartbeat = 0.0
        self.sequence = 0
        self.pending = {}
        self.armed = False
        self.arm_at = 0.0
        self.source = 'none'
        self.mode = 1
        self.command = [0.0] * 4
        self.command_at = None
        self.last_tx = 0.0
        self.reason = 'starting'
        self.prev_ticks = None
        self.prev_uptime = None
        self.positions = [0.0] * 4
        self.tick_origin = [0] * 4
        self.x = self.y = self.yaw = 0.0
        self.linear = self.angular = 0.0

    @property
    def ready(self):
        return (self.telemetry is not None and self.last_rx is not None and
                self.clock() - self.last_rx < 0.25 and self.telemetry.configured and
                self.telemetry.config_sequence == self.config_seq and
                (not self.telemetry.remote_supported or self.telemetry.remote_configured) and
                not (self.telemetry.flags & 4))

    def send(self, kind, payload=b'', track=False):
        if self.transport is None:
            raise ValueError('device is disconnected')
        self.sequence = (self.sequence + 1) & 0xffff
        seq = self.sequence
        self.transport.write(frame(kind, seq, payload))
        if track:
            self.pending[seq] = [kind, payload, self.clock(), 0]
        return seq

    def stop(self, reason='stopped'):
        self.armed = False
        self.source = 'none'
        self.command = [0.0] * 4
        self.command_at = None
        self.pending = {s: p for s, p in self.pending.items()
                        if p[0] in (Kind.CONFIG, Kind.REMOTE_CONFIG)}
        self.reason = reason
        if self.transport:
            try:
                self.send(Kind.STOP)
            except OSError:
                self.disconnect('failed to send stop; firmware watchdog will stop output')

    def disconnect(self, reason):
        if self.transport:
            try:
                self.transport.close()
            except OSError:
                pass
        self.transport = None
        self.armed = False
        self.source = 'none'
        self.command = [0.0] * 4
        self.command_at = None
        self.config_seq = None
        self.remote_config_seq = None
        self.pending.clear()
        self.telemetry = None
        self.last_rx = None
        self.prev_ticks = self.prev_uptime = None
        self.linear = self.angular = 0.0
        self.next_connect = self.clock() + 1.0
        self.reason = reason

    def arm(self, source):
        if source not in ('ros', 'gui'):
            raise ValueError('source must be ros or gui')
        if self.telemetry and self.telemetry.remote_active:
            raise ValueError('release controller L1 and wait for wheels to stop before ROS/GUI control')
        if self.armed:
            if self.source != source:
                raise ValueError('stop before changing control source')
            return
        if not self.ready or self.pending:
            raise ValueError('wait for fresh telemetry and configuration acknowledgement')
        if not self.mock and not self.cfg.hardware_confirmed:
            raise ValueError('verify shield, wiring and power, then set hardware_confirmed=true')
        if source == 'ros' and (self.cfg.wheel_radius <= 0 or self.cfg.track_width <= 0):
            raise ValueError('ROS driving requires measured wheel_radius and track_width')
        if any(abs(v) > 2 for v in self.telemetry.rpm):
            raise ValueError('wait for wheels to stop')
        self.command = [0.0] * 4
        self.command_at = None
        self.mode = 1
        self.send(Kind.ARM, b'\x01', track=True)
        self.armed = True
        self.arm_at = self.clock()
        self.source = source
        self.reason = 'arming; awaiting device confirmation'

    def wheels(self, values, mode, source):
        if not self.armed or source != self.source:
            raise ValueError('arm the selected control source before sending motion')
        if not self.ready:
            self.stop('telemetry is stale')
            raise ValueError(self.reason)
        if mode not in ('rpm', 'pwm') or not isinstance(values, list) or len(values) != 4:
            self.stop('invalid wheel command')
            raise ValueError('provide four M1..M4 values and mode rpm or pwm')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            self.stop('non-finite wheel command')
            raise ValueError(self.reason)
        limits = self.cfg.max_pwm if mode == 'pwm' else [self.cfg.max_rpm] * 4
        if any(abs(v) > lim for v, lim in zip(values, limits)):
            self.stop('wheel command exceeds configured limit')
            raise ValueError(self.reason)
        if mode == 'pwm' and source != 'gui':
            raise ValueError('PWM test is available only in GUI control mode')
        self.mode = 1 if mode == 'rpm' else 2
        self.command = [float(v) for v in values]
        self.command_at = self.clock()

    def twist(self, vx, wz, source):
        self.wheels(twist_to_rpm(vx, wz, self.cfg), 'rpm', source)

    def configure(self, patch):
        if self.armed or (self.telemetry and self.telemetry.armed):
            raise ValueError('stop and disarm before changing configuration')
        if self.telemetry and any(abs(v) > 2 for v in self.telemetry.rpm):
            raise ValueError('wait for wheels to stop before changing configuration')
        cfg = self.cfg.updated(patch)
        self.cfg = cfg
        self.config_seq = None
        self.remote_config_seq = None
        self.pending.clear()
        self.prev_ticks = self.prev_uptime = None
        self.reason = 'configuration pending; requires device acknowledgement'

    def reset_odometry(self):
        if self.armed or not self.telemetry or self.telemetry.armed or any(abs(v) > 2 for v in self.telemetry.rpm):
            raise ValueError('reset requires a connected, disarmed, stationary device')
        self.x = self.y = self.yaw = self.linear = self.angular = 0.0
        self.positions = [0.0] * 4
        self.tick_origin = self.telemetry.ticks[:]
        self.prev_ticks = self.telemetry.ticks[:]
        self.prev_uptime = self.telemetry.uptime_ms

    def integrate(self, state):
        if self.prev_ticks is None:
            self.prev_ticks, self.prev_uptime = state.ticks[:], state.uptime_ms
            return
        elapsed_ms = (state.uptime_ms - self.prev_uptime) & 0xffffffff
        if elapsed_ms == 0:
            return
        if elapsed_ms > 2000:
            self.prev_ticks, self.prev_uptime = state.ticks[:], state.uptime_ms
            self.stop('device clock reset or telemetry gap; re-arm required')
            self.config_seq = None
            self.remote_config_seq = None
            return
        angles = [delta32(c, p) * 2 * math.pi / cpr for c, p, cpr in
                  zip(state.ticks, self.prev_ticks, self.cfg.encoder_cpr)]
        dt = elapsed_ms / 1000
        # Reject impossible count jumps rather than moving odometry by kilometres.
        if any(abs(a) > 150 * 2*math.pi/60 * dt * 3 + 0.2 for a in angles):
            self.prev_ticks, self.prev_uptime = state.ticks[:], state.uptime_ms
            self.stop('implausible encoder jump; check CPR, sign and wiring')
            return
        logical = [angles[m-1] for m in self.cfg.motor_map]
        self.positions = [p+a for p, a in zip(self.positions, logical)]
        if self.cfg.wheel_radius > 0 and self.cfg.track_width > 0:
            dl = (logical[0] + logical[1]) * self.cfg.wheel_radius / 2
            dr = (logical[2] + logical[3]) * self.cfg.wheel_radius / 2
            turn = (dr-dl) / self.cfg.track_width
            distance = (dl+dr) / 2
            sinc = math.sin(turn/2)/(turn/2) if abs(turn) > 1e-8 else 1.0
            self.x += distance * sinc * math.cos(self.yaw+turn/2)
            self.y += distance * sinc * math.sin(self.yaw+turn/2)
            self.yaw = math.atan2(math.sin(self.yaw+turn), math.cos(self.yaw+turn))
            self.linear, self.angular = distance/dt, turn/dt
        self.prev_ticks, self.prev_uptime = state.ticks[:], state.uptime_ms

    def step(self):
        now = self.clock()
        if self.transport is None:
            if now < self.next_connect:
                return
            try:
                self.transport = self.factory()
                self.parser = Parser()
                self.connected_at = now
                self.send(Kind.STOP)
                self.reason = 'waiting for device telemetry'
            except (OSError, ValueError) as exc:
                self.disconnect(str(exc))
                return
        try:
            for kind, seq, payload in self.parser.feed(self.transport.read(), now):
                if kind == Kind.ACK and len(payload) == 2:
                    pending = self.pending.get(seq)
                    if pending and payload[0] == pending[0]:
                        del self.pending[seq]
                        if payload[1] != 0:
                            self.stop(f'device rejected command {payload[0]}: {payload[1]}')
                            if payload[0] in (Kind.CONFIG, Kind.REMOTE_CONFIG):
                                raise OSError('device rejected configuration')
                elif kind == Kind.STATE:
                    try:
                        state = Telemetry.decode(payload)
                    except struct.error:
                        self.parser.errors += 1
                        continue
                    self.telemetry, self.last_rx = state, now
                    if (self.config_seq is not None and not any(p[0] == Kind.CONFIG for p in self.pending.values())
                            and (not state.configured or state.config_sequence != self.config_seq)):
                        self.stop('device configuration lost; reconfiguring without re-arming')
                        self.config_seq = None
                        self.remote_config_seq = None
                        self.prev_ticks = self.prev_uptime = None
                    if self.ready:
                        self.integrate(state)
                    if self.armed and state.remote_active:
                        # A rejected/racing host ARM must never keep sending DRIVE.
                        self.stop('controller owns motion; host control cancelled')
                    if self.armed and now-self.arm_at > 0.2 and not state.armed:
                        self.stop('device disarmed; re-arm required')
                    if state.flags & 4:
                        self.stop('device fault (I2C or control loop)')
            if now - (self.last_rx if self.last_rx is not None else self.connected_at) > (0.35 if self.last_rx is not None else 4):
                raise OSError('telemetry timeout')
            if self.telemetry and self.config_seq is None:
                self.config_seq = self.send(Kind.CONFIG, self.cfg.wire(), track=True)
            state = self.telemetry
            if (state and state.remote_supported and state.configured and
                    state.config_sequence == self.config_seq and
                    not any(p[0] == Kind.CONFIG for p in self.pending.values()) and
                    self.remote_config_seq is None):
                self.remote_config_seq = self.send(Kind.REMOTE_CONFIG, self.cfg.remote_wire(self.mock), track=True)
            if (self.ready and self.telemetry.remote_supported and
                    not self.pending and now-self.last_heartbeat >= 0.05):
                self.send(Kind.HEARTBEAT)
                self.last_heartbeat = now
            for seq, item in list(self.pending.items()):
                kind, payload, sent, retries = item
                if now-sent > 0.25:
                    if retries >= 3:
                        raise OSError('command acknowledgement timeout')
                    self.transport.write(frame(kind, seq, payload))
                    item[2], item[3] = now, retries+1
            if self.armed:
                if self.command_at is not None and now-self.command_at > self.cfg.command_timeout:
                    self.stop('command timeout; re-arm required')
                elif not self.ready:
                    self.stop('telemetry/configuration not ready')
                elif now-self.last_tx >= 0.02:
                    factor = 100 if self.mode == 1 else 1
                    self.send(Kind.DRIVE, struct.pack('<B4h', self.mode, *(round(v*factor) for v in self.command)))
                    self.last_tx = now
                    self.reason = 'armed' if self.telemetry.armed else 'arming'
            elif self.ready and self.reason in ('waiting for device telemetry', 'configuration pending; requires device acknowledgement'):
                self.reason = 'ready; disarmed'
        except (OSError, ValueError) as exc:
            self.disconnect(str(exc))

    def status(self):
        state = self.telemetry
        remote = bool(self.ready and state and state.remote_active)
        return {
            'mode': 'mock' if self.mock else 'hardware', 'ready': self.ready,
            'geometry_valid': self.cfg.wheel_radius > 0 and self.cfg.track_width > 0,
            'armed': bool(self.ready and state and state.armed), 'arm_requested': self.armed,
            'source': 'remote' if remote else self.source,
            'reason': 'wireless controller active' if remote else self.reason,
            'remote_supported': bool(state and state.remote_supported),
            'remote_connected': bool(self.ready and state and state.remote_connected),
            'remote_enabled': bool(self.ready and state and state.remote_supported and
                                   self.cfg.remote_enabled and (self.mock or self.cfg.hardware_confirmed)),
            'telemetry_age': None if self.last_rx is None else round(self.clock()-self.last_rx, 3),
            'config_applied': self.ready, 'pending_commands': len(self.pending),
            'crc_errors': self.parser.errors, 'config': self.cfg.as_dict(),
            'ticks': state.ticks if state else [0]*4,
            'relative_ticks': [delta32(c, p) for c, p in zip(state.ticks, self.tick_origin)] if state else [0]*4,
            'rpm': state.rpm if state else [0.0]*4, 'pwm': state.pwm if state else [0]*4,
            'target': [v / 100 for v in state.targets] if remote else self.command[:],
            'command_mode': 'rpm' if remote or self.mode == 1 else 'pwm',
            'odometry': {'x': self.x, 'y': self.y, 'yaw': self.yaw,
                         'linear': self.linear, 'angular': self.angular},
        }

    def close(self):
        self.stop('shutdown')
        if self.transport:
            self.transport.close()
            self.transport = None
