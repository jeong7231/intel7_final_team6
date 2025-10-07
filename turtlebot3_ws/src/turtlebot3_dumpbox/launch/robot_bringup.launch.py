#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_heartbeat = LaunchConfiguration('start_heartbeat')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_heartbeat',
            default_value='true',
            description='Launch heartbeat publisher'),

        Node(
            package='turtlebot3_dumpbox',
            executable='dumpbox_servo_controller',
            name='dumpbox_servo_controller',
            output='screen'),

        Node(
            package='turtlebot3_dumpbox',
            executable='dumpbox_heartbeat',
            name='dumpbox_heartbeat',
            output='screen',
            condition=IfCondition(start_heartbeat))
    ])
