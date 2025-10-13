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
    image_topic = LaunchConfiguration('aruco_image_topic')
    camera_info_topic = LaunchConfiguration('aruco_camera_info_topic')
    marker_size = LaunchConfiguration('aruco_marker_size')
    aruco_dictionary = LaunchConfiguration('aruco_dictionary')
    camera_frame = LaunchConfiguration('aruco_camera_frame')

    parking_status_topic = LaunchConfiguration('parking_status_topic')
    parking_reset_service = LaunchConfiguration('parking_reset_service')

    dumpbox_start_heartbeat = LaunchConfiguration('dumpbox_start_heartbeat')

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_aruco'),
                'launch',
                'aruco_pose.launch.py'
            )
        ),
        launch_arguments={
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'marker_size': marker_size,
            'aruco_dictionary': aruco_dictionary,
            'camera_frame': camera_frame,
        }.items()
    )

    parking_node = Node(
        package='turtlebot3_automatic_parking_vision',
        executable='turtlebot3_automatic_parking_vision',
        name='turtlebot3_automatic_parking_vision',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'status_topic': parking_status_topic},
            {'reset_service': parking_reset_service},
        ]
    )

    mission_node = Node(
        package='turtlebot3_auto_unload',
        executable='mission_manager',
        name='auto_unload_mission',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'destination_config': config_file},
            {'parking_status_topic': parking_status_topic},
            {'parking_reset_service': parking_reset_service},
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
        DeclareLaunchArgument('aruco_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('aruco_camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('aruco_marker_size', default_value='0.04'),
        DeclareLaunchArgument('aruco_dictionary', default_value='DICT_5X5_250'),
        DeclareLaunchArgument('aruco_camera_frame', default_value='camera_rgb_frame'),
        DeclareLaunchArgument('parking_status_topic', default_value='/automatic_parking/status'),
        DeclareLaunchArgument('parking_reset_service', default_value='/automatic_parking/reset'),
        DeclareLaunchArgument('dumpbox_start_heartbeat', default_value='true'),
        aruco_launch,
        parking_node,
        mission_node,
        dumpbox_launch,
    ])
