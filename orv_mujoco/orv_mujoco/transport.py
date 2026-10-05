"""UNO-compatible transport backed by wheel torque/contact dynamics, never USB."""
import math
import struct
import time

import numpy as np

from orv_4wd.protocol import CONFIG, Kind, Parser, Telemetry, frame


class MujocoTransport:
    def __init__(self, world, clock=time.monotonic):
        self.world, self.clock = world, clock
        self.start = self.last = self.last_state = self.last_command = clock()
        self.parser, self.rx = Parser(), bytearray()
        self.armed = self.configured = False
        self.config_seq = self.command_seq = self.arm_seq = 0
        self.mode = 1
        self.timeout = .3
        self.cpr = np.full(4, 4320)
        self.kp, self.ki, self.kd = np.ones(4), np.full(4, .5), np.zeros(4)
        self.max_pwm, self.min_pwm = np.full(4, 100), np.zeros(4)
        self.motor_sign = self.encoder_sign = np.ones(4)
        self.max_rpm, self.acceleration = 30., 30.
        self.targets = np.zeros(4)
        self.ramp, self.integral, self.rpm, self.previous_rpm = (np.zeros(4) for _ in range(4))
        self.pwm = np.zeros(4)
        self.encoder_origin = np.zeros(4, dtype=np.int64)
        self.previous_counts = self.counts()
        self.accumulator = self.control_elapsed = 0.
        self.dropped_wall_time = 0.
        self.wall_start = clock()

    def channel_angles(self):
        values = np.zeros(4)
        for logical, motor in enumerate(self.world.cfg['physical_motor_map']):
            values[motor-1] = self.world.data.qpos[self.world.qindices[logical]]
        return values

    def counts(self):
        physical = np.rint(self.channel_angles()*self.world.cfg['physical_encoder_cpr']/(2*math.pi)).astype(np.int64)
        return (physical-self.encoder_origin)*self.encoder_sign.astype(np.int64)

    def stop(self):
        self.armed = False
        for values in (self.targets, self.ramp, self.integral, self.pwm): values[:] = 0
        self.world.data.ctrl[:] = 0

    def write(self, data):
        now = self.clock()
        self.advance(now)
        for kind, seq, payload in self.parser.feed(data, now):
            result = 0
            if kind == Kind.STOP and not payload:
                self.stop()
            elif kind == Kind.CONFIG and len(payload) == CONFIG.size and not self.armed:
                cfg = CONFIG.unpack(payload)
                self.cpr = np.array(cfg[:4])
                gains = np.array(cfg[4:16]).reshape(4, 3)/100
                self.kp, self.ki, self.kd = gains.T
                self.max_pwm, self.min_pwm = np.array(cfg[16:20]), np.array(cfg[20:24])
                self.motor_sign, self.encoder_sign = np.array(cfg[24:28]), np.array(cfg[28:32])
                self.max_rpm, self.acceleration, self.timeout = cfg[-3]/100, cfg[-2]/100, cfg[-1]/1000
                self.config_seq, self.configured = seq, True
                self.previous_counts = self.counts()
                self.stop()
            elif kind == Kind.ARM and payload in (b'\x00', b'\x01'):
                if not payload[0]: self.stop()
                elif not self.configured or (self.armed and seq != self.arm_seq): result = 2
                elif not self.armed:
                    self.stop()
                    self.armed, self.arm_seq, self.command_seq = True, seq, seq
                    self.last_command, self.mode = now, 1
            elif kind == Kind.DRIVE and len(payload) == 9 and self.armed:
                mode, *values = struct.unpack('<B4h', payload)
                difference = (seq-self.command_seq) & 0xffff
                if difference == 0 or difference >= 32768: continue
                values = np.array(values)/(100 if mode == 1 else 1)
                limits = self.max_rpm if mode == 1 else self.max_pwm
                if mode not in (1, 2) or np.any(np.abs(values) > limits):
                    self.stop(); result = 1
                else:
                    if self.mode != mode: self.integral[:], self.ramp[:] = 0, 0
                    self.targets, self.mode = values, mode
                    self.command_seq, self.last_command = seq, now
                    continue
            elif kind == Kind.RESET and not payload and not self.armed:
                self.encoder_origin = np.rint(self.channel_angles()*self.world.cfg['physical_encoder_cpr']/(2*math.pi)).astype(np.int64)
                self.previous_counts = self.counts()
            else:
                result = 1
            self.rx.extend(frame(Kind.ACK, seq, bytes([kind, result])))

    def control(self, elapsed):
        counts = self.counts()
        self.rpm = (counts-self.previous_counts)*60/(self.cpr*elapsed)
        self.previous_counts = counts
        if not self.armed:
            self.pwm[:] = 0
        elif self.mode == 2:
            self.pwm = np.clip(self.targets, -self.max_pwm, self.max_pwm)
        else:
            self.ramp += np.clip(self.targets-self.ramp, -self.acceleration*elapsed, self.acceleration*elapsed)
            error = self.ramp-self.rpm
            proposed = np.clip(self.integral+self.ki*error*elapsed, -self.max_pwm, self.max_pwm)
            output = self.ramp*(255/107)+self.kp*error+proposed-self.kd*(self.rpm-self.previous_rpm)/elapsed
            self.integral = np.where((np.abs(output) <= self.max_pwm) | (output*error < 0), proposed, self.integral)
            zero = np.abs(self.ramp) < .01
            self.integral[zero], output[zero] = 0, 0
            self.pwm = np.clip(output, -self.max_pwm, self.max_pwm)
        self.pwm = np.where((np.abs(self.pwm) > .01) & (np.abs(self.pwm) < self.min_pwm),
                            np.sign(self.pwm)*self.min_pwm, self.pwm)
        self.pwm = np.rint(self.pwm)
        self.previous_rpm = self.rpm.copy()

    def advance(self, now):
        elapsed = max(0., now-self.last)
        self.last = now
        if self.armed and (now-self.last_command > self.timeout or elapsed > .25): self.stop()
        # Bound catch-up: a paused desktop must not replay seconds of old motion.
        self.dropped_wall_time += max(0., elapsed-.1)
        self.accumulator += min(elapsed, .1)
        dt = self.world.cfg['timestep']
        while self.accumulator + 1e-12 >= dt:
            self.control_elapsed += dt
            if self.control_elapsed + 1e-12 >= self.world.cfg['control_period']:
                self.control(self.control_elapsed)
                self.control_elapsed = 0.
            torques = np.zeros(4)
            for logical, motor in enumerate(self.world.cfg['physical_motor_map']):
                channel = motor-1
                if not self.armed or self.pwm[channel] == 0: continue  # coast
                omega = self.world.data.qvel[self.world.vindices[logical]]
                duty = self.pwm[channel]*self.motor_sign[channel]/255
                torque = self.world.cfg['stall_torque']*(duty-omega/(self.world.cfg['no_load_rpm']*2*math.pi/60))
                torques[logical] = np.clip(torque, -self.world.cfg['stall_torque'], self.world.cfg['stall_torque'])
            self.world.step(torques)
            self.accumulator -= dt

    def read(self):
        now = self.clock()
        self.advance(now)
        if now-self.last_state >= .05:
            self.last_state = now
            ticks = [((int(v)+(1 << 31)) % (1 << 32))-(1 << 31) for v in self.counts()]
            state = Telemetry(round(self.world.data.time*1000) & 0xffffffff,
                              int(self.armed) | (int(self.configured) << 1),
                              self.config_seq, self.command_seq, ticks,
                              np.clip(self.rpm, -327.67, 327.67).tolist(), self.pwm.astype(int).tolist())
            self.rx.extend(frame(Kind.STATE, 0, state.encode()))
        result = bytes(self.rx)
        self.rx.clear()
        return result

    def reset_world(self):
        self.stop()
        # Retain simulation uptime so the driver does not mistake this for an MCU reset.
        uptime = self.world.data.time
        self.world.reset()
        self.world.data.time = uptime
        self.encoder_origin[:] = 0
        self.previous_counts = self.counts()
        self.rpm[:], self.previous_rpm[:] = 0, 0

    def close(self):
        self.stop()
