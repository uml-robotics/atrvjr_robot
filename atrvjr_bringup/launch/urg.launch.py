'''

<launch>

<!-- HOKUYO URG -->
  <node pkg="hokuyo_node" type="hokuyo_node" name="hokuyo_node" required="true">
    <param name="port" value="/dev/ttyACM0"/>
    <param name="frame_id" value="base_laser_link1"/>
    <remap from="scan" to="scan_urg"/>
  </node>
  <!-- Broadcast the transform regarding urg's position on the robot, relative to center of rotation -->
  <node pkg="tf" type="static_transform_publisher" name="tf_broadcaster_urg" args="-0.3 0 0.7 0 0 1.0 0 base_link /base_laser_link1 100"/>

</launch>


'''

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    
    config_file_path = PathJoinSubstitution([FindPackageShare('atrvjr_bringup'), 'config', 'urg_config.yaml'])
    
    return LaunchDescription([
        Node(
            package='hokuyo_node',
            executable='hokuyo_node',
            name='hokuyo_node',
            parameters=[config_file_path],
            remappings=[('scan', 'scan_urg')],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_broadcaster_urg',
            arguments=['-0.3', '0', '0.7', '0', '0', '1.0', '0', 'base_link', '/base_laser_link1', '100'],
        ),
    ])
