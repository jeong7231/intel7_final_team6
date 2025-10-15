#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    dumpbox_start_heartbeat = LaunchConfiguration('dumpbox_start_heartbeat')

    mission_node = Node(
        package='turtlebot3_unload',
        executable='mission_manager',
        name='turtlebot3_unload_mission',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'destination_config': config_file},
        ]
    )

    dumpbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_dumpbox'),
                'launch',
                'robot_bringup.launch.py'
            )
        ),
        launch_arguments={
            'start_heartbeat': dumpbox_start_heartbeat,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value='config/destinations.yaml'),
        DeclareLaunchArgument('dumpbox_start_heartbeat', default_value='true'),
        mission_node,
        dumpbox_launch,
    ])
