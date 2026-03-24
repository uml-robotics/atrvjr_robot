from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
 
 
def generate_launch_description():
    bringup_dir = get_package_share_directory('atrvjr_bringup')
    nav2_dir = get_package_share_directory('atrvjr_nav2')
    map_dir = os.path.join(nav2_dir, 'maps')
    tcp_dir = get_package_share_directory('ros_tcp_endpoint')
 
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/atrv-jr/ros2_ws/src/atrvjr_robot/atrvjr_nav2/maps/hallway.yaml',
        description='Full path to the map yaml file'
    )
 
    return LaunchDescription([
        map_arg,
 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'atrv_drivers.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tcp_dir, 'launch', 'endpoint.py')
            ),
        ),
    
    ])
 
