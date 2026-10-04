import struct

import pytest

from orv_4wd.config import Config
from orv_4wd.protocol import REMOTE_CONFIG, Telemetry
from test_control import make_core, tick, drive


def test_remote_config_requires_hardware_confirmation_and_preserves_map():
    cfg = Config(motor_map=[4, 2, 1, 3], remote_max_rpm=50, max_rpm=20)
    assert REMOTE_CONFIG.unpack(cfg.remote_wire()) == (0, 4, 2, 1, 3, 2000)
    assert REMOTE_CONFIG.unpack(cfg.remote_wire(mock=True))[0] == 1
    assert REMOTE_CONFIG.unpack(cfg.updated({'hardware_confirmed': True}).remote_wire())[0] == 1
    with pytest.raises(ValueError): cfg.updated({'remote_enabled': 1})
    with pytest.raises(ValueError): cfg.updated({'remote_max_rpm': float('nan')})


def test_state_legacy_and_extended_roundtrip():
    for flags, targets in [(3, None), (3 | 8 | 16 | 32 | 64, [1000, -1000, 1500, -1500])]:
        state = Telemetry(200, flags, 1, 2, [1, -2, 3, -4], [1, -2, 3, -4], [5]*4, targets)
        decoded = Telemetry.decode(state.encode())
        assert decoded == state
        assert len(state.encode()) == (49 if targets else 41)
    invalid = bytearray(state.encode()[:41])
    with pytest.raises(struct.error): Telemetry.decode(invalid)


def remote_sample(core, clock, device):
    # Inject receiver input into the serial mock; firmware arbitration is tested
    # separately by compiling and running the actual sketch against host stubs.
    assert device.remote_enabled
    device.remote_active = device.remote_connected = device.armed = True
    device.mode = 1
    device.targets = [10, 10, 10, 10]
    device.last_command = clock()
    tick(core, clock, 0.06)


def test_remote_reports_source_targets_and_odometry_without_host_drive():
    core, clock, devices = make_core()
    device = devices[-1]
    for _ in range(15): remote_sample(core, clock, device)
    status = core.status()
    assert not core.armed  # host never acquires motion ownership
    assert status['source'] == 'remote' and status['armed']
    assert status['target'] == [10]*4 and status['remote_connected']
    assert status['odometry']['x'] > 0.02
    with pytest.raises(ValueError, match='controller'): core.arm('ros')
    with pytest.raises(ValueError): core.configure({'kp': [2]*4})
    with pytest.raises(ValueError): core.reset_odometry()
    core.stop()
    tick(core, clock, 0.1)
    assert not core.status()['armed'] and not device.remote_active


def test_host_heartbeat_loss_stops_remote_even_with_fresh_receiver_commands():
    core, clock, devices = make_core()
    device = devices[-1]
    remote_sample(core, clock, device)
    for _ in range(10):
        clock.advance(0.05)
        device.last_command = clock()  # radio still sends input, but Pi is gone
        device.read()
    assert not device.armed and not device.remote_active


def test_remote_configuration_tracks_host_calibration_updates():
    core, clock, devices = make_core()
    core.configure({'remote_enabled': False, 'motor_map': [4, 2, 1, 3], 'remote_max_rpm': 12.0})
    tick(core, clock, 0.3)
    device = devices[-1]
    assert core.ready and not device.remote_enabled
    assert device.remote_map == [4, 2, 1, 3] and device.remote_max_rpm == 12
