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


'''def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='atrvjr_bringup',
            executable='rflex',
            name='rflex',
            output='screen',
        ),
        Node(
            package='atrvjr_bringup',
            executable='sick',
            name='sick',
            output='screen',
        ),
        Node(
            package='atrvjr_bringup',
            executable='urg',
            name='urg',
            output='screen',
        ),
    ])'''