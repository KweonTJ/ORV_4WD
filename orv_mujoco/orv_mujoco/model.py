"""Measured ORV geometry, explicit provisional inertias, and MuJoCo scene."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

WHEELS = ('front_left', 'rear_left', 'front_right', 'rear_right')


def share():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('orv_mujoco'))


def settings(path=None):
    cfg = yaml.safe_load(Path(path or share() / 'config/physics.yaml').read_text())
    for key in ('wheel_radius', 'wheel_width', 'track_width', 'wheelbase', 'chassis_length',
                'chassis_width', 'chassis_height', 'ground_clearance', 'total_mass', 'wheel_mass',
                'wheel_armature', 'wheel_damping', 'stall_torque', 'no_load_rpm',
                'physical_encoder_cpr', 'timestep', 'control_period'):
        if isinstance(cfg[key], bool) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    if cfg['total_mass'] <= 4*cfg['wheel_mass']:
        raise ValueError('total_mass must exceed the mass of all four wheels')
    if not 0.0002 <= cfg['timestep'] <= 0.005 or not 0.005 <= cfg['control_period'] <= 0.05:
        raise ValueError('unsupported physics/control timestep')
    if cfg['terrain'] not in ('flat', 'course'):
        raise ValueError('terrain must be flat or course')
    if len(cfg['friction']) != 3 or any(not math.isfinite(v) or v < 0 for v in cfg['friction']):
        raise ValueError('friction requires three finite nonnegative values')
    if sorted(cfg['physical_motor_map']) != [1, 2, 3, 4]:
        raise ValueError('physical_motor_map must be a permutation of 1..4')
    return cfg


def scene_xml(cfg, assets):
    def add(parent, tag, **attrs):
        return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    def vector(values): return ' '.join(f'{v:.9g}' for v in values)
    root = ET.Element('mujoco', model='ORV 4WD digital twin')
    add(root, 'compiler', angle='radian', meshdir=str(assets), inertiafromgeom='false')
    add(root, 'option', timestep=cfg['timestep'], gravity='0 0 -9.81', integrator='implicitfast',
        cone='elliptic', iterations=80, tolerance='1e-9')
    add(root, 'statistic', center='0 0 .08', extent=1)
    visual = add(root, 'visual')
    add(visual, 'global', offwidth=1600, offheight=1000)
    add(visual, 'headlight', ambient='.35 .35 .35', diffuse='.45 .45 .45', specular='.2 .2 .2')
    add(visual, 'rgba', haze='.82 .87 .93 1')
    asset = add(root, 'asset')
    add(asset, 'texture', name='sky', type='skybox', builtin='gradient', rgb1='.72 .81 .9',
        rgb2='.95 .97 1', width=512, height=3072)
    add(asset, 'texture', name='tiles', type='2d', builtin='checker', rgb1='.24 .30 .37',
        rgb2='.30 .37 .44', width=512, height=512)
    add(asset, 'material', name='floor', texture='tiles', texrepeat='10 10', texuniform='true', reflectance='.1')
    manifest = json.loads((Path(assets) / 'manifest.json').read_text())
    for body in manifest.values():
        for group in body['groups']:
            # Visuals are open surfaces in places; inertia comes from explicit inertials.
            add(asset, 'mesh', name=group['name'], file=group['file'], inertia='shell')
    default = add(root, 'default')
    add(default, 'geom', friction=vector(cfg['friction']), condim=6,
        solref='.006 1', solimp='.95 .99 .001')
    world = add(root, 'worldbody')
    add(world, 'light', pos='1 -1 3', dir='-.2 .2 -1', directional='true', diffuse='.65 .65 .65')
    add(world, 'geom', name='ground', type='plane', size='5 5 .01', material='floor')
    if cfg['terrain'] == 'course':
        add(world, 'geom', name='ramp', type='box', pos='1.3 0 .08', size='.4 .25 .01',
            euler='0 -.174533 0', rgba='.9 .58 .16 1')
        add(world, 'geom', name='step', type='box', pos='-.8 .5 .006', size='.2 .25 .006', rgba='.2 .65 .68 1')
        for x, y in ((.6, .7), (1.8, .7), (-.7, -.7)):
            add(world, 'geom', type='cylinder', pos=f'{x} {y} .075', size='.04 .075', rgba='.96 .38 .18 1')
    base = add(world, 'body', name='base_link', pos='0 0 .002')
    add(base, 'freejoint', name='base_free')
    length, width, height = (cfg['chassis_'+key] for key in ('length', 'width', 'height'))
    body_z = cfg['ground_clearance'] + height/2
    mass = cfg['total_mass'] - 4*cfg['wheel_mass']
    inertia = [mass*(width**2+height**2)/12, mass*(length**2+height**2)/12,
               mass*(length**2+width**2)/12]
    add(base, 'inertial', pos=f'0 0 {body_z}', mass=mass, diaginertia=vector(inertia))
    add(base, 'geom', name='chassis_collision', type='box', pos=f'0 0 {body_z}',
        size=vector([length/2, width/2, height/2]), group=3, rgba='.1 .7 .9 .15')
    # Mesh scaling follows the same baseline dimensions as the RViz URDF.
    for group in manifest['v3_chassis']['groups']:
        mesh = asset.find(f"mesh[@name='{group['name']}']")
        mesh.set('scale', vector([length/.290, width/.155, height/.067]))
        add(base, 'geom', type='mesh', mesh=group['name'], pos=f'0 0 {body_z}',
            rgba=group['rgba'], contype=0, conaffinity=0, group=1)
    r, w, m = cfg['wheel_radius'], cfg['wheel_width'], cfg['wheel_mass']
    wheel_inertia = [m*(3*r*r+w*w)/12, m*r*r/2, m*(3*r*r+w*w)/12]
    for group in manifest['rubber_wheel']['groups']:
        asset.find(f"mesh[@name='{group['name']}']").set('scale', vector([r/.044, w/.035, r/.044]))
    actuator = add(root, 'actuator')
    for name, xsign, side in zip(WHEELS, (1, -1, 1, -1), (1, 1, -1, -1)):
        x, y = xsign*cfg['wheelbase']/2, side*cfg['track_width']/2
        add(base, 'geom', type='cylinder', fromto=f'{x} {side*width/2} {r} {x} {y} {r}',
            size='.006', rgba='.65 .7 .75 1', contype=0, conaffinity=0, group=1)
        wheel = add(base, 'body', name=name+'_wheel_link', pos=vector([x, y, r]))
        add(wheel, 'inertial', pos='0 0 0', mass=m, diaginertia=vector(wheel_inertia))
        add(wheel, 'joint', name=name+'_wheel_joint', type='hinge', axis='0 1 0',
            damping=cfg['wheel_damping'], armature=cfg['wheel_armature'])
        add(wheel, 'geom', name=name+'_contact', type='cylinder', size=vector([r, w/2]),
            euler=f'{math.pi/2} 0 0', group=3, rgba='.2 .7 .9 .2')
        for group in manifest['rubber_wheel']['groups']:
            add(wheel, 'geom', type='mesh', mesh=group['name'], rgba=group['rgba'],
                euler=f'{math.pi if side < 0 else 0} 0 0', contype=0, conaffinity=0, group=1)
        add(actuator, 'motor', name=name+'_motor', joint=name+'_wheel_joint', gear=1,
            ctrllimited='true', ctrlrange=f"{-cfg['stall_torque']} {cfg['stall_torque']}")
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')


class World:
    def __init__(self, config=None, assets=None):
        self.cfg = config or settings()
        self.xml = scene_xml(self.cfg, assets or share() / 'meshes')
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.qindices = [self.model.joint(n+'_wheel_joint').qposadr[0] for n in WHEELS]
        self.vindices = [self.model.joint(n+'_wheel_joint').dofadr[0] for n in WHEELS]
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def step(self, torques):
        self.data.ctrl[:] = torques
        mujoco.mj_step(self.model, self.data)

    def pose(self):
        return self.data.qpos[:3].copy(), self.data.qpos[3:7].copy()

    def mirror(self, position, quaternion, joints):
        self.data.qpos[:3] = position
        self.data.qpos[3:7] = quaternion  # MuJoCo w,x,y,z
        self.data.qpos[self.qindices] = joints
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)


def print_domain():
    from orv_4wd.domain_config import domain_id
    print(domain_id(settings()['simulation_domain_id']))
