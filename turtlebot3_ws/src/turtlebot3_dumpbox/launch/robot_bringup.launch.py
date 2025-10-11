#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_heartbeat = LaunchConfiguration('start_heartbeat')
    opencr_port = LaunchConfiguration('opencr_port')
    opencr_id = LaunchConfiguration('opencr_id')
    opencr_baud_rate = LaunchConfiguration('opencr_baud_rate')
    opencr_protocol_version = LaunchConfiguration('opencr_protocol_version')
    servo_pwm_neutral = LaunchConfiguration('servo_pwm_neutral')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_heartbeat',
            default_value='true',
            description='Launch heartbeat publisher'),
        DeclareLaunchArgument(
            'opencr_port',
            default_value='/dev/ttyACM0',
            description='OpenCR USB port (e.g. /dev/ttyACM0)'),
        DeclareLaunchArgument(
            'opencr_id',
            default_value='200',
            description='OpenCR Dynamixel ID'),
        DeclareLaunchArgument(
            'opencr_baud_rate',
            default_value='1000000',
            description='Baud rate for OpenCR'),
        DeclareLaunchArgument(
            'opencr_protocol_version',
            default_value='2.0',
            description='Dynamixel protocol version'),
        DeclareLaunchArgument(
            'servo_pwm_neutral',
            default_value='1500',
            description='Threshold PWM for open/close decision'),

        Node(
            package='turtlebot3_dumpbox',
            executable='dumpbox_servo_controller',
            name='dumpbox_servo_controller',
            output='screen',
            parameters=[{
                'opencr_port': opencr_port,
                'opencr_id': opencr_id,
                'opencr_baud_rate': opencr_baud_rate,
                'opencr_protocol_version': opencr_protocol_version,
                'servo_pwm_neutral': servo_pwm_neutral,
            }]),

        Node(
            package='turtlebot3_dumpbox',
            executable='dumpbox_heartbeat',
            name='dumpbox_heartbeat',
            output='screen',
            condition=IfCondition(start_heartbeat))
    ])
