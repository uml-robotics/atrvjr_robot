'''

<!-- Core robot nodes -->

<launch>

  <include file="$(find atrvjr_bringup)/launch/rflex.launch"/>
  <!--<include file="$(find atrvjr_bringup)/launch/ptu.launch"/>-->
  <include file="$(find atrvjr_bringup)/launch/sick.launch"/>
  <include file="$(find atrvjr_bringup)/launch/urg.launch"/>

</launch>


'''

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    rflex_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('atrvjr_bringup'),'launch','rflex.launch.py'])))
    sick_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('atrvjr_bringup'),'launch','sick.launch.py'])))
    urg_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('atrvjr_bringup'),'launch','urg.launch.py'])))
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        arguments=['joy_dev:=/dev/input/js0 joy_config:=xbox'],
        output='screen'
    )
    # Controller UUID's: 
    #   XBox Controller ~ 68:6C:E6:79:EE:21
    return LaunchDescription([
        rflex_launch,
        #sick_launch,
        #urg_launch,
        teleop_node
    ])