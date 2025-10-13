#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_dumpbox',
            executable='dumpbox_heartbeat',
            name='dumpbox_heartbeat',
            output='screen'
        )
    ])
