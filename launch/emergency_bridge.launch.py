#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_cmd = DeclareLaunchArgument(
        'input_cmd_vel',
        default_value='/cmd_vel_nav',
        description='Source velocity topic (e.g., NAV2 output).'
    )

    output_cmd = DeclareLaunchArgument(
        'output_cmd_vel',
        default_value='/cmd_vel',
        description='Actual robot velocity topic.'
    )

    stop_topic = DeclareLaunchArgument(
        'stop_topic',
        default_value='/emergency_stop',
        description='Bool topic that engages emergency stop when true.'
    )

    resume_topic = DeclareLaunchArgument(
        'resume_topic',
        default_value='/emergency_resume',
        description='Bool topic that releases emergency stop when true.'
    )

    status_topic = DeclareLaunchArgument(
        'status_topic',
        default_value='/emergency/state',
        description='Bool topic publishing current emergency state.'
    )

    hold_period = DeclareLaunchArgument(
        'hold_period',
        default_value='0.1',
        description='Seconds between zero-velocity publications while stopped.'
    )

    bridge_node = Node(
        package='turtlebot3_emergency',
        executable='emergency_bridge',
        name='turtlebot3_emergency_bridge',
        output='screen',
        parameters=[{
            'input_cmd_vel': LaunchConfiguration('input_cmd_vel'),
            'output_cmd_vel': LaunchConfiguration('output_cmd_vel'),
            'stop_topic': LaunchConfiguration('stop_topic'),
            'resume_topic': LaunchConfiguration('resume_topic'),
            'status_topic': LaunchConfiguration('status_topic'),
            'hold_period': LaunchConfiguration('hold_period'),
        }]
    )

    return LaunchDescription([
        input_cmd,
        output_cmd,
        stop_topic,
        resume_topic,
        status_topic,
        hold_period,
        bridge_node,
    ])
