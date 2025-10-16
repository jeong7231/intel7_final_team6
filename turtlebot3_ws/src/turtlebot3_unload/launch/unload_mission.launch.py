#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config/destinations.yaml',
        description='Destination configuration YAML (package-relative or absolute path).'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.'
    )

    mission_node = Node(
        package='turtlebot3_unload',
        executable='mission_manager',
        name='turtlebot3_unload_mission',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'destination_config': LaunchConfiguration('config_file')},
        ]
    )

    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        mission_node,
    ])
