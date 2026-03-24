from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
 
 
def generate_launch_description():
    bringup_dir = get_package_share_directory('atrvjr_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    nav_dir = get_package_share_directory('atrvjr_nav2')
    nav_launch_dir = os.path.join(nav_dir, 'launch')
 
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'front_camera.launch.py')
            ),
        ),
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(
#                os.path.join(launch_dir, 'rear_camera.launch.py')
#            ),
#        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rear_lidar.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'tf.launch.py')
            ),
        ),
    ])
 
