from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent,
                            IncludeLaunchDescription, GroupAction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from orv_4wd.domain_config import domain_id, default_domain_config, load_domains


def launch_nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('orv_mujoco'))
    physics = Path(arg('physics_config')).expanduser()
    cfg = yaml.safe_load(physics.read_text())
    domain = domain_id(arg('domain_id') or cfg['simulation_domain_id'])
    vehicle, operator = load_domains(default_domain_config())
    if domain in (vehicle, operator):
        raise ValueError('Simulation domain must differ from both configured real vehicle/operator domains')
    env = {'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1'}
    arguments = ['--physics-config', str(physics), '--terrain', arg('terrain')]
    integrated = arg('viewer') == 'true' and arg('gui') == 'true'
    if arg('viewer') == 'true' and not integrated: arguments.append('--viewer')
    simulator = Node(package='orv_mujoco', executable='dashboard' if integrated else 'simulator',
        name='orv_driver', output='screen',
        arguments=arguments, additional_env=env,
        parameters=[arg('driver_config'), {'mode': 'mujoco', 'api_port': int(arg('api_port')),
                    'api_bind': '127.0.0.1', 'api_enabled': True,
                    'profile_path': str(Path.home()/'.config/orv_4wd/mujoco_profile.yaml')}])
    urdf = Path(get_package_share_directory('orv_description'))/'urdf/orv.urdf.xacro'
    description = xacro.process_file(str(urdf), mappings={key: str(cfg[key]) for key in
                    ('wheel_radius', 'wheel_width', 'track_width', 'chassis_length',
                     'chassis_height', 'ground_clearance', 'wheelbase', 'chassis_width')}).toxml()
    actions = [simulator, Node(package='robot_state_publisher', executable='robot_state_publisher',
                              additional_env=env, parameters=[{'robot_description': description}])]
    if arg('gui') == 'true' and not integrated:
        actions.append(Node(package='orv_gui', executable='tuner', additional_env={
            **env, 'ORV_API_URL': 'http://127.0.0.1:'+arg('api_port'), 'ORV_AUTO_CONNECT': '1'}))
    if arg('rviz') == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', additional_env=env,
                            arguments=['-d', str(urdf.parent.parent/'rviz/orv.rviz')]))
    if arg('bridge') == 'true':
        bridge_launch = Path(get_package_share_directory('orv_bringup'))/'launch/domain_bridge.launch.py'
        actions.append(GroupAction([SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bridge_launch)), launch_arguments={
                'config': str(share/'config/domain_bridge.yaml'), 'vehicle_domain_id': str(domain),
                'operator_domain_id': str(operator), 'rviz': 'false'}.items())]))
    actions.append(RegisterEventHandler(OnProcessExit(target_action=simulator,
                    on_exit=[EmitEvent(event=Shutdown(reason='ORV simulator exited'))])))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('orv_mujoco'))
    return LaunchDescription([
        DeclareLaunchArgument('physics_config', default_value=str(share/'config/physics.yaml')),
        DeclareLaunchArgument('driver_config', default_value=str(share/'config/driver.yaml')),
        DeclareLaunchArgument('domain_id', default_value=''),
        DeclareLaunchArgument('api_port', default_value='8766'),
        DeclareLaunchArgument('terrain', default_value='flat', choices=['flat', 'course']),
        DeclareLaunchArgument('viewer', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('bridge', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_nodes),
    ])
