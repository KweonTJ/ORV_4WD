"""Validated configuration. Channel arrays are always in M1..M4 order."""
from dataclasses import dataclass, field, asdict, fields, replace
import math
from .protocol import CONFIG, REMOTE_CONFIG


@dataclass
class Config:
    wheel_radius: float = 0.0
    track_width: float = 0.0
    hardware_confirmed: bool = False
    remote_enabled: bool = True
    remote_max_rpm: float = 15.0
    # Logical joint order: front left, rear left, front right, rear right.
    motor_map: list = field(default_factory=lambda: [1, 2, 3, 4])
    encoder_cpr: list = field(default_factory=lambda: [4320, 4320, 4320, 4320])
    motor_sign: list = field(default_factory=lambda: [1, 1, 1, 1])
    encoder_sign: list = field(default_factory=lambda: [1, 1, 1, 1])
    kp: list = field(default_factory=lambda: [1.0] * 4)
    ki: list = field(default_factory=lambda: [0.5] * 4)
    kd: list = field(default_factory=lambda: [0.0] * 4)
    max_pwm: list = field(default_factory=lambda: [100] * 4)
    min_pwm: list = field(default_factory=lambda: [0] * 4)
    max_rpm: float = 30.0
    accel_rpm_s: float = 30.0
    command_timeout: float = 0.5
    firmware_timeout_ms: int = 300

    def validate(self):
        for key, lo, hi in [('wheel_radius', 0, 1), ('track_width', 0, 3),
                            ('max_rpm', 1, 150), ('accel_rpm_s', 1, 300),
                            ('command_timeout', 0.1, 2), ('remote_max_rpm', 1, 150)]:
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f'{key}: expected finite number in [{lo}, {hi}]')
        if type(self.hardware_confirmed) is not bool:
            raise ValueError('hardware_confirmed must be boolean')
        if type(self.remote_enabled) is not bool:
            raise ValueError('remote_enabled must be boolean')
        if not isinstance(self.motor_map, list) or sorted(self.motor_map) != [1, 2, 3, 4] or any(type(v) is not int for v in self.motor_map):
            raise ValueError('motor_map must be a permutation of [1,2,3,4]')
        for key, lo, hi, integral in [('encoder_cpr', 1, 60000, True),
                                     ('kp', 0, 100, False), ('ki', 0, 100, False),
                                     ('kd', 0, 100, False), ('max_pwm', 1, 255, True),
                                     ('min_pwm', 0, 254, True),
                                     ('motor_sign', -1, 1, True), ('encoder_sign', -1, 1, True)]:
            values = getattr(self, key)
            if not isinstance(values, list) or len(values) != 4:
                raise ValueError(f'{key} requires four values in M1..M4 order')
            for v in values:
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi or (integral and type(v) is not int):
                    raise ValueError(f'invalid {key}: {values}')
                if key.endswith('_sign') and v not in (-1, 1):
                    raise ValueError('signs must be -1 or +1')
        if any(a > b for a, b in zip(self.min_pwm, self.max_pwm)):
            raise ValueError('min_pwm cannot exceed max_pwm')
        if type(self.firmware_timeout_ms) is not int or not 100 <= self.firmware_timeout_ms <= 1000:
            raise ValueError('firmware_timeout_ms must be an integer in [100,1000]')
        return self

    def updated(self, patch):
        if not isinstance(patch, dict) or set(patch) - {f.name for f in fields(self)}:
            raise ValueError('unknown configuration field')
        return replace(self, **patch).validate()

    def as_dict(self):
        return asdict(self)

    def wire(self):
        self.validate()
        gains = [round(getattr(self, k)[i] * 100) for i in range(4) for k in ('kp', 'ki', 'kd')]
        return CONFIG.pack(*self.encoder_cpr, *gains, *self.max_pwm, *self.min_pwm,
                           *self.motor_sign, *self.encoder_sign, round(self.max_rpm * 100),
                           round(self.accel_rpm_s * 100), self.firmware_timeout_ms)

    def remote_wire(self, mock=False):
        self.validate()
        return REMOTE_CONFIG.pack(int(self.remote_enabled and (mock or self.hardware_confirmed)),
                                  *self.motor_map, round(min(self.remote_max_rpm, self.max_rpm)*100))


def twist_to_rpm(vx, wz, cfg):
    if not all(math.isfinite(v) for v in (vx, wz)):
        raise ValueError('velocity must be finite')
    if cfg.wheel_radius <= 0 or cfg.track_width <= 0:
        raise ValueError('measure and set wheel_radius and track_width first')
    scale = 60 / (2 * math.pi * cfg.wheel_radius)
    left = (vx - wz * cfg.track_width / 2) * scale
    right = (vx + wz * cfg.track_width / 2) * scale
    ratio = max(1.0, abs(left) / cfg.max_rpm, abs(right) / cfg.max_rpm)
    channels = [0.0] * 4
    for motor, value in zip(cfg.motor_map, [left, left, right, right]):
        channels[motor-1] = value / ratio
    return channels
