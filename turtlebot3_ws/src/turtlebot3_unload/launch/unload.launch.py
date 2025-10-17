import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('turtlebot3_unload'),
        'config', 'destinations.yaml'
    )
    cfg = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=TextSubstitution(text=default_cfg),
            description='path to YAML'
        ),
        Node(
            package='turtlebot3_unload',
            executable='unload_node',
            name='turtlebot3_unload',
            output='screen',
            parameters=[{'config_file': cfg}],
        ),
    ])
