import os
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError, get_package_prefix, get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from orv_4wd.domain_config import default_domain_config, load_domains


def launch_bridge(context):
    arg = lambda key: LaunchConfiguration(key).perform(context)
    config = Path(arg('config')).expanduser().resolve()
    vehicle, operator = load_domains(config, arg('vehicle_domain_id'), arg('operator_domain_id'))
    try:
        prefix = Path(get_package_prefix('domain_bridge'))
    except PackageNotFoundError:
        # A user-local extraction of the official Humble Debian package.
        prefix = Path.home() / '.local/share/orv_4wd/domain_bridge/opt/ros/humble'
    executable = prefix / 'lib/domain_bridge/domain_bridge'
    if not executable.is_file():
        raise RuntimeError('Install domain_bridge: sudo apt install ros-humble-domain-bridge')
    environment = {'ROS_DOMAIN_ID': str(operator)}
    bridge_environment = dict(environment, LD_LIBRARY_PATH=os.pathsep.join(filter(None, (
        str(prefix / 'lib'), os.environ.get('LD_LIBRARY_PATH', ''),
    ))))
    actions = [ExecuteProcess(
        cmd=[str(executable), '--from', str(vehicle), '--to', str(operator), str(config)],
        name='orv_domain_bridge', output='screen', additional_env=bridge_environment,
    )]
    if arg('rviz').lower() == 'true':
        rviz_config = Path(get_package_share_directory('orv_description')) / 'rviz/orv.rviz'
        actions.append(Node(
            package='rviz2', executable='rviz2', name='orv_remote_rviz',
            arguments=['-d', str(rviz_config)], additional_env=environment,
            remappings=[('/tf', '/orv/tf'), ('/tf_static', '/orv/tf_static'),
                        ('/robot_description', '/orv/robot_description')],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(default_domain_config())),
        DeclareLaunchArgument('vehicle_domain_id', default_value='', description='Empty: use config'),
        DeclareLaunchArgument('operator_domain_id', default_value='', description='Empty: use config'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_bridge),
    ])
