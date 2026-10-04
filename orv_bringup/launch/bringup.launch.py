from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from orv_4wd.domain_config import default_domain_config, load_domains


def launch_nodes(context):
    arg = lambda key: LaunchConfiguration(key).perform(context)
    mode = arg('mode')
    if mode not in ('mock', 'hardware'):
        raise ValueError('mode must be mock or hardware')
    vehicle, _ = load_domains(arg('domain_config'), vehicle=arg('vehicle_domain_id'))
    environment = {'ROS_DOMAIN_ID': str(vehicle)}
    default = Path(get_package_share_directory('orv_bringup'))/'config'/f'{mode}.yaml'
    config = Path(arg('config')).expanduser() if arg('config') else default
    values = yaml.safe_load(config.read_text())['orv_driver']['ros__parameters']
    radius, track = values.get('wheel_radius', 0), values.get('track_width', 0)
    urdf = Path(get_package_share_directory('orv_description'))/'urdf/orv.urdf.xacro'
    description = xacro.process_file(str(urdf), mappings={
        'wheel_radius': str(radius if radius > 0 else 0.044),
        # If a custom config omits center spacing, use URDF dimensions for visualization.
        'track_width': str(track if track > 0 else 0.0),
    }).toxml()
    return [
        Node(package='orv_4wd', executable='driver', name='orv_driver', output='screen',
             additional_env=environment,
             parameters=[str(config), {'mode': mode, 'port': arg('port'),
                         'api_bind': arg('api_bind'), 'api_port': int(arg('api_port')),
                         'profile_path': arg('profile_path'),
                         'api_enabled': arg('api_enabled').lower() == 'true'}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             additional_env=environment,
             parameters=[{'robot_description': description}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mock', description='mock or hardware; always starts disarmed'),
        DeclareLaunchArgument('domain_config', default_value=str(default_domain_config())),
        DeclareLaunchArgument('vehicle_domain_id', default_value='', description='Empty: use domain_config'),
        DeclareLaunchArgument('config', default_value='', description='ROS YAML; empty selects config/<mode>.yaml'),
        DeclareLaunchArgument('port', default_value='/dev/serial/by-id/SET_YOUR_UNO_DEVICE'),
        DeclareLaunchArgument('api_bind', default_value='127.0.0.1'),
        DeclareLaunchArgument('api_port', default_value='8765'),
        DeclareLaunchArgument('api_enabled', default_value='true'),
        DeclareLaunchArgument('profile_path', default_value=str(Path.home()/'.config/orv_4wd/profile.yaml')),
        OpaqueFunction(function=launch_nodes),
    ])
