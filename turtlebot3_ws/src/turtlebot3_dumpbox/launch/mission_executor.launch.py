#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for mission executor'),

        Node(
            package='turtlebot3_dumpbox',
            executable='dumpbox_mission_executor',
            name='dumpbox_mission_executor',
            namespace=namespace,
            output='screen')
    ])
