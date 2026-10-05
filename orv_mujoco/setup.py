from glob import glob
from setuptools import setup

setup(
    name='orv_mujoco', version='0.1.0', packages=['orv_mujoco'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/orv_mujoco']),
                ('share/orv_mujoco', ['package.xml', 'requirements.txt']),
                *[(f'share/orv_mujoco/{folder}', glob(f'{folder}/*'))
                  for folder in ('config', 'launch', 'meshes')]],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='ktj', maintainer_email='ktj@example.invalid', license='Apache-2.0',
    description='ORV four-wheel MuJoCo physics simulation and read-only hardware twin',
    entry_points={'console_scripts': ['simulator = orv_mujoco.node:main',
                                     'dashboard = orv_mujoco.dashboard:main',
                                     'mirror = orv_mujoco.mirror:main',
                                     'domain_id = orv_mujoco.model:print_domain']},
)
