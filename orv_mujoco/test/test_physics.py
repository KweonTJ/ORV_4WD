import math
from pathlib import Path

import numpy as np
import pytest

from orv_4wd.config import Config
from orv_4wd.core import Controller
from orv_mujoco.model import World, settings
from orv_mujoco.transport import MujocoTransport

PACKAGE = Path(__file__).resolve().parents[1]


class Clock:
    def __init__(self): self.t = 10.
    def __call__(self): return self.t


def create(max_pwm=100):
    world = World(settings(PACKAGE/'config/physics.yaml'), PACKAGE/'meshes')
    clock = Clock()
    transport = MujocoTransport(world, clock)
    core = Controller(lambda: transport, Config(wheel_radius=.044, track_width=.210,
                      max_pwm=[max_pwm]*4), mock=True, clock=clock)
    advance(core, clock, .6)
    assert core.ready
    return world, clock, transport, core


def advance(core, clock, duration, command=None):
    for _ in range(round(duration/.01)):
        if command: core.twist(*command, 'ros')
        clock.t += .01
        core.step()


def test_measured_geometry_mass_and_stable_ground_contact():
    world, clock, transport, core = create()
    assert world.model.nu == 4 and world.model.nq == 11
    assert np.sum(world.model.body_mass) == pytest.approx(2.5)
    assert np.max(np.abs(world.data.qvel)) < .001
    assert abs(world.pose()[0][2]) < .0001
    assert world.data.ncon >= 4
    for name, sign in [('front_left', 1), ('front_right', -1)]:
        center = world.data.body(name+'_wheel_link').xpos
        assert center[:2] == pytest.approx([.1, sign*.105])
        assert center[2] == pytest.approx(.044, abs=.0001)
    assert not core.armed and not transport.armed


def test_forward_reverse_encoder_odometry_and_watchdog():
    world, clock, transport, core = create()
    core.arm('ros')
    advance(core, clock, 3, (.06, 0))
    assert world.pose()[0][0] > .12
    assert abs(core.x-world.pose()[0][0]) < .015
    assert abs(world.pose()[0][1]) < .005
    # RELEASE coasts; wait for physical drag to slow the wheels before rearming.
    core.stop(); advance(core, clock, 6)
    start = world.pose()[0][0]
    core.arm('ros'); advance(core, clock, 3, (-.06, 0))
    assert world.pose()[0][0] < start-.12
    advance(core, clock, 1)
    assert not core.armed and not transport.armed
    assert np.all(world.data.ctrl == 0)


def test_turning_has_physical_slip_separate_from_encoder_estimate():
    world, clock, transport, core = create()
    core.arm('ros'); advance(core, clock, 6, (0, .45))
    q = world.pose()[1]
    actual_yaw = 2*math.atan2(q[3], q[0])
    assert actual_yaw > .05
    assert abs(core.yaw-actual_yaw) > .1  # ground truth is not copied from odometry
    assert np.all(np.isfinite(world.data.qpos))


def test_pwm_limit_changes_physical_response():
    distances = []
    for limit in (10, 100):
        world, clock, transport, core = create(limit)
        core.arm('ros'); advance(core, clock, 3, (.12, 0))
        assert np.max(np.abs(transport.pwm)) <= limit
        distances.append(world.pose()[0][0])
    assert distances[1] > 2*distances[0] > 0


def test_pushing_chassis_does_not_fabricate_encoder_motion():
    world, clock, transport, core = create()
    world.data.qpos[0] += 1.
    advance(core, clock, .2)
    assert world.pose()[0][0] > .99 and abs(core.x) < .001
    transport.reset_world()
    assert np.allclose(world.data.qpos[:2], 0)
    assert np.all(transport.counts() == 0)


def test_mirror_sets_pose_without_running_physics():
    world, _, _, _ = create()
    time = world.data.time
    world.mirror([1., 2., 0.], [1, 0, 0, 0], [.1, .2, .3, .4])
    assert world.data.time == time
    assert world.data.qpos[:3] == pytest.approx([1, 2, 0])
    assert world.data.qpos[world.qindices] == pytest.approx([.1, .2, .3, .4])


def test_invalid_mass_rejected(tmp_path):
    path = tmp_path/'physics.yaml'
    text = (PACKAGE/'config/physics.yaml').read_text().replace('total_mass: 2.5', 'total_mass: 0.1')
    path.write_text(text)
    with pytest.raises(ValueError, match='total_mass'): settings(path)
