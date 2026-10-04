"""Show the static model without starting the motor driver."""

import math
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def display_nodes(context):
    share = Path(get_package_share_directory('orv_description'))
    dimensions = {
        key: LaunchConfiguration(key).perform(context)
        for key in ('wheel_radius', 'outer_width', 'wheel_width', 'track_width',
                    'chassis_length', 'chassis_height', 'ground_clearance', 'axle_radius')
    }
    for key, value in dimensions.items():
        if key == 'track_width' and float(value) == 0:
            continue
        if not math.isfinite(float(value)) or float(value) <= 0:
            raise ValueError(f'{key} must be a finite positive distance in meters')
    track = float(dimensions['track_width']) or (
        float(dimensions['outer_width']) - float(dimensions['wheel_width'])
    )
    if track <= float(dimensions['wheel_width']):
        raise ValueError('Wheel center spacing must exceed wheel width')
    description = xacro.process_file(
        str(share / 'urdf/orv.urdf.xacro'), mappings=dimensions
    ).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'rate': 20}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz/orv.rviz')], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wheel_radius', default_value='0.044',
                              description='Wheel radius in meters; 88 mm diameter'),
        DeclareLaunchArgument('outer_width', default_value='0.245',
                              description='Measured distance between outer wheel edges in meters'),
        DeclareLaunchArgument('wheel_width', default_value='0.035',
                              description='Measured width of each tire in meters'),
        DeclareLaunchArgument('chassis_length', default_value='0.290',
                              description='Measured chassis front-to-rear length in meters'),
        DeclareLaunchArgument('chassis_height', default_value='0.067',
                              description='Measured chassis body height in meters'),
        DeclareLaunchArgument('ground_clearance', default_value='0.021',
                              description='Distance from ground to chassis bottom in meters'),
        DeclareLaunchArgument('axle_radius', default_value='0.006',
                              description='Visualization axle radius in meters; not measured'),
        DeclareLaunchArgument('track_width', default_value='0.0',
                              description='Center spacing override; 0 derives outer_width - wheel_width'),
        OpaqueFunction(function=display_nodes),
    ])
