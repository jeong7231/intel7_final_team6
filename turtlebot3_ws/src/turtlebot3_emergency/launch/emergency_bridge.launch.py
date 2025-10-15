#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stop_topic = DeclareLaunchArgument(
        'stop_topic',
        default_value='/emergency_stop',
        description='Bool topic engaging emergency stop.'
    )

    resume_topic = DeclareLaunchArgument(
        'resume_topic',
        default_value='/emergency_resume',
        description='Bool topic releasing emergency stop.'
    )

    status_topic = DeclareLaunchArgument(
        'status_topic',
        default_value='/emergency/state',
        description='Bool topic publishing emergency state.'
    )

    motor_service = DeclareLaunchArgument(
        'motor_service',
        default_value='/motor_power',
        description='Motor power SetBool service name.'
    )

    service_wait_timeout = DeclareLaunchArgument(
        'service_wait_timeout',
        default_value='1.0',
        description='Seconds to wait for motor power service availability.'
    )

    bridge_node = Node(
        package='turtlebot3_emergency',
        executable='emergency_bridge',
        name='turtlebot3_emergency_bridge',
        output='screen',
        parameters=[{
            'stop_topic': LaunchConfiguration('stop_topic'),
            'resume_topic': LaunchConfiguration('resume_topic'),
            'status_topic': LaunchConfiguration('status_topic'),
            'motor_service': LaunchConfiguration('motor_service'),
            'service_wait_timeout': LaunchConfiguration('service_wait_timeout'),
        }]
    )

    return LaunchDescription([
        stop_topic,
        resume_topic,
        status_topic,
        motor_service,
        service_wait_timeout,
        bridge_node,
    ])
