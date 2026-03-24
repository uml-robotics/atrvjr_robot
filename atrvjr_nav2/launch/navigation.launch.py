"""
Launch the full Nav2 navigation stack for the ATRV Jr.

Includes:
  - TF publisher (static + dynamic transforms)
  - AMCL localisation
  - Map server
  - Nav2 planner, controller, behavior, BT navigator
  - Lifecycle managers

Usage:
  ros2 launch atrvjr_nav2 navigation.launch.py
  ros2 launch atrvjr_nav2 navigation.launch.py map:=/path/to/map.yaml use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg = get_package_share_directory('atrvjr_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ── Launch arguments ─────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg, 'maps', 'hallway.yaml'),
        description='Full path to the map YAML file',
    )

    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        description='Full path to the nav2 parameters YAML file',
    )

    tf_params_arg = DeclareLaunchArgument(
        'tf_params_file',
        default_value=os.path.join(pkg, 'config', 'transforms.yaml'),
        description='Full path to the TF publisher parameters YAML file',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes',
    )

    # ── TF publisher ─────────────────────────────────────────────────────
    tf_node = Node(
        package='atrvjr_nav2',
        executable='tf_publisher',
        name='atrvjr_tf_publisher',
        output='screen',
        parameters=[
            LaunchConfiguration('tf_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ── Nav2 bringup (localization + navigation) ─────────────────────────
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map':            LaunchConfiguration('map'),
            'params_file':    LaunchConfiguration('nav2_params_file'),
            'use_sim_time':   LaunchConfiguration('use_sim_time'),
            'autostart':      LaunchConfiguration('autostart'),
        }.items(),
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file':  LaunchConfiguration('nav2_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart':    LaunchConfiguration('autostart'),
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        nav2_params_arg,
        tf_params_arg,
        use_sim_time_arg,
        autostart_arg,

        tf_node,
        nav2_localization,
#        nav2_navigation,
    ])
