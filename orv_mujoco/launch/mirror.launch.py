from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from orv_4wd.domain_config import default_domain_config, load_domains


def nodes(context):
    config = LaunchConfiguration('physics_config').perform(context)
    _, operator = load_domains(default_domain_config())
    return [Node(package='orv_mujoco', executable='mirror', output='screen',
                 additional_env={'ROS_DOMAIN_ID': str(operator)},
                 arguments=['--physics-config', config])]


def generate_launch_description():
    share = Path(get_package_share_directory('orv_mujoco'))
    return LaunchDescription([
        DeclareLaunchArgument('physics_config', default_value=str(share/'config/physics.yaml')),
        OpaqueFunction(function=nodes),
    ])
