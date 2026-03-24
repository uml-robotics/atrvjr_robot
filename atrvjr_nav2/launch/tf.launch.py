"""
Launch the ATRV Jr TF publisher.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('atrvjr_nav2')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'transforms.yaml'),
            description='Path to the TF publisher parameters YAML file',
        ),

        Node(
            package='atrvjr_nav2',
            executable='tf_publisher',
            name='atrvjr_tf_publisher',
            output='screen',
            parameters=[params_file],
        ),
    ])
