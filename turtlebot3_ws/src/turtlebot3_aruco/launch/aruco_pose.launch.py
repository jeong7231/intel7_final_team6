#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    marker_size = LaunchConfiguration('marker_size')
    dictionary = LaunchConfiguration('aruco_dictionary')
    camera_frame = LaunchConfiguration('camera_frame')
    publish_debug_image = LaunchConfiguration('publish_debug_image')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='이미지 토픽'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera_info',
            description='카메라 정보 토픽'),
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.04',
            description='마커 한 변의 실제 길이 [m]'),
        DeclareLaunchArgument(
            'aruco_dictionary',
            default_value='DICT_5X5_250',
            description='OpenCV ArUco 딕셔너리 이름'),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_rgb_frame',
            description='카메라 기준 프레임명'),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='false',
            description='검출 결과 디버그 이미지 퍼블리시 여부'),
        Node(
            package='turtlebot3_aruco',
            executable='aruco_pose_node',
            name='turtlebot3_aruco_tracker',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'camera_info_topic': camera_info_topic,
                'marker_size': marker_size,
                'aruco_dictionary': dictionary,
                'camera_frame': camera_frame,
                'publish_debug_image': publish_debug_image,
            }]
        )
    ])
