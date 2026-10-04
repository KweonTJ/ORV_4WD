import math
import struct
import pytest
from orv_4wd.config import Config, twist_to_rpm
from orv_4wd.core import Controller
from orv_4wd.protocol import Kind, Parser, Telemetry, crc16, frame, delta32, CONFIG
from orv_4wd.transport import MockTransport


class Clock:
    def __init__(self): self.t = 10.0
    def __call__(self): return self.t
    def advance(self, dt): self.t += dt


def make_core(mock=True, cfg=None):
    clock = Clock()
    instances = []
    def factory():
        device = MockTransport(clock)
        instances.append(device)
        return device
    core = Controller(factory, cfg or Config(wheel_radius=0.05, track_width=0.3), mock, clock)
    tick(core, clock, 0.3)
    assert core.ready
    return core, clock, instances


def tick(core, clock, duration):
    for _ in range(round(duration/0.01)):
        clock.advance(0.01)
        core.step()


def drive(core, clock, values, duration=1):
    for _ in range(round(duration/0.1)):
        core.wheels(values, 'rpm', core.source)
        tick(core, clock, 0.1)


def test_crc_golden_fragmented_and_corrupted_stream():
    assert crc16(b'123456789') == 0x29b1
    parser = Parser()
    good = frame(Kind.DRIVE, 0x1234, struct.pack('<B4h', 1, -25500, 0, 1234, 25500))
    bad = bytearray(good); bad[9] ^= 0xff
    assert parser.feed(b'noise'+bad+good[:5], 1.0) == []
    assert parser.feed(good[5:], 1.01) == [(Kind.DRIVE, 0x1234, good[7:-2])]
    assert parser.errors > 0


def test_partial_packet_expiry_and_size_limit():
    parser = Parser()
    parser.feed(b'OR\x01\x03\x00\x00\x40', 1)
    assert parser.feed(frame(Kind.STOP, 7), 1.2) == [(Kind.STOP, 7, b'')]
    with pytest.raises(ValueError): frame(1, 1, b'x'*65)


@pytest.mark.parametrize('patch', [
    {'motor_map': [1, 1, 3, 4]}, {'motor_sign': [0, 1, 1, 1]},
    {'encoder_cpr': [0, 4320, 4320, 4320]}, {'kp': [float('nan')]*4},
    {'max_rpm': float('inf')}, {'min_pwm': [101]*4},
    {'max_pwm': [100.0]*4}, {'unexpected': 1}, {'hardware_confirmed': 'true'},
])
def test_configuration_rejects_invalid_values(patch):
    with pytest.raises(ValueError): Config().updated(patch)


def test_configuration_wire_layout():
    cfg = Config(encoder_cpr=[4000, 4100, 4200, 4320], motor_sign=[-1, 1, -1, 1],
                 kp=[1, 2, 3, 4], ki=[0.5]*4, kd=[0]*4)
    data = CONFIG.unpack(cfg.wire())
    assert len(cfg.wire()) == 54
    assert data[:4] == (4000, 4100, 4200, 4320)
    assert data[4:10] == (100, 50, 0, 200, 50, 0)
    assert data[24:28] == (-1, 1, -1, 1)
    assert data[-3:] == (3000, 3000, 300)


def test_inverse_kinematics_mapping_and_ratio_preserving_limit():
    cfg = Config(wheel_radius=0.05, track_width=0.3, motor_map=[4, 2, 1, 3])
    values = twist_to_rpm(0, 0.5, cfg)
    assert values[0] > 0 and values[1] < 0 and values[2] > 0 and values[3] < 0
    assert twist_to_rpm(100, 0, cfg) == [30]*4
    with pytest.raises(ValueError): twist_to_rpm(1, 0, Config())


def test_counts_signed_wrap():
    assert delta32(-2147483648, 2147483647) == 1
    assert delta32(2147483647, -2147483648) == -1


def test_forward_reverse_and_rotation_odometry():
    core, clock, _ = make_core()
    core.arm('ros')
    drive(core, clock, [20]*4)
    assert core.x > 0.05 and abs(core.yaw) < 1e-9
    start = core.x
    drive(core, clock, [-20]*4, 2)
    assert core.x < start
    drive(core, clock, [-20, -20, 20, 20], 1)
    assert core.yaw > 0.2


def test_no_motion_without_arm_and_no_source_stealing():
    core, clock, _ = make_core()
    with pytest.raises(ValueError): core.wheels([10]*4, 'rpm', 'gui')
    core.arm('ros')
    with pytest.raises(ValueError): core.arm('gui')
    with pytest.raises(ValueError): core.wheels([10]*4, 'rpm', 'gui')
    assert core.command == [0]*4


def test_command_timeout_latches_disarm_and_stale_command_not_replayed():
    core, clock, _ = make_core()
    core.arm('gui'); drive(core, clock, [20]*4, 0.5)
    tick(core, clock, 0.7)
    assert not core.armed and not core.telemetry.armed
    assert core.command == [0]*4
    tick(core, clock, 1)
    core.arm('gui'); tick(core, clock, 0.5)
    assert all(abs(v) < 0.5 for v in core.telemetry.rpm)


def test_physical_hardware_gate_and_geometry_gate():
    core, clock, _ = make_core(mock=False)
    with pytest.raises(ValueError, match='hardware_confirmed'): core.arm('gui')
    core.configure({'hardware_confirmed': True, 'wheel_radius': 0.0})
    tick(core, clock, 0.3)
    with pytest.raises(ValueError, match='measured'): core.arm('ros')
    core.arm('gui')  # allows lifted-wheel calibration without known geometry


def test_runtime_configuration_requires_stationary_and_acknowledged_device():
    core, clock, _ = make_core()
    core.arm('gui')
    with pytest.raises(ValueError): core.configure({'kp': [2.0]*4})
    core.stop()
    tick(core, clock, 0.1)
    core.configure({'encoder_cpr': [3960]*4})
    assert not core.ready
    tick(core, clock, 0.3)
    assert core.ready and core.cfg.encoder_cpr == [3960]*4


def test_device_reboot_reconfigures_but_never_rearms():
    core, clock, instances = make_core()
    core.arm('gui'); drive(core, clock, [10]*4, 0.3)
    device = instances[-1]
    device.configured = device.armed = False
    device.start = clock()
    device.position = [0]*4
    tick(core, clock, 0.5)
    assert core.ready and not core.armed and not core.telemetry.armed


def test_link_failure_reconnects_disarmed():
    core, clock, instances = make_core()
    core.arm('gui'); drive(core, clock, [10]*4, 0.3)
    def broken_read(): raise OSError('USB unplugged')
    instances[-1].read = broken_read
    tick(core, clock, 0.1)
    assert not core.armed and not core.ready
    tick(core, clock, 1.5)
    assert len(instances) == 2 and core.ready and not core.armed


def test_firmware_watchdog_independent_of_host():
    clock = Clock(); device = MockTransport(clock)
    device.write(frame(Kind.CONFIG, 1, Config().wire()))
    device.write(frame(Kind.ARM, 2, b'\x01'))
    device.write(frame(Kind.DRIVE, 3, struct.pack('<B4h', 1, 1000, 1000, 1000, 1000)))
    clock.advance(0.31); device.read()
    assert not device.armed and device.targets == [0]*4


def test_bad_motion_and_infinite_values_stop():
    core, clock, _ = make_core()
    core.arm('gui')
    with pytest.raises(ValueError): core.wheels([float('nan')]*4, 'rpm', 'gui')
    assert not core.armed


def test_no_fabricated_odometry_when_dimensions_unknown():
    core, clock, _ = make_core(cfg=Config())
    core.arm('gui'); drive(core, clock, [10]*4)
    assert core.positions[0] > 0
    assert core.x == core.y == core.yaw == 0
