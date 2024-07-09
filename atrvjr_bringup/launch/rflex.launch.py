'''

<launch>

<!-- Rflex -->
  <node pkg="rflex" type="atrvjr" name="atrvjr" output="screen">
    <param name="port" value="/dev/ttyR1"/>
    <remap from="/atrvjr/odom" to="/odom"/>
    <remap from="/atrvjr/cmd_vel" to="/cmd_vel"/>

    <!-- Low-level configuration, use caution -->
    <param name="odo_distance_conversion" value="90000"/>
    <param name="odo_angle_conversion" value="36000"/>
    <param name="trans_acceleration" value="1.5"/>
    <param name="trans_torque" value="0.075"/>
    <param name="rot_acceleration" value="2.5"/>
    <param name="rot_torque" value="0.7"/>
  </node>

</launch>


'''

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='rflex',
            executable='atrvjr',
            name='atrvjr',
            output='screen',
            parameters=[{
                'port': '/dev/ttyR1',
                'odo_distance_conversion': 90000,
                'odo_angle_conversion': 36000,
                'trans_acceleration' : 1.5,
                'trans_torque' : 0.075,
                'rot_acceleration' : 2.5,
                'rot_torque' : 0.7
            }],

           # remappings=[('/atrvjr/odom', '/odom'), ('/atrvjr/cmd_vel', '/cmd_vel'),],

        ),
    ])