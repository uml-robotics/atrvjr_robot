'''

<launch>

 <!--Pan-tilt unit -->
  <node pkg="ptu46" type="ptu46" name="ptu46" output="screen">
    <param name ="port" value="/dev/ttyR2"/>
  </node>

</launch>


'''

# https://github.com/roncapat/ROS2-HAL-Flir-PTU-D46

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='ptu46',
            executable='ptu46',
            name='ptu46',
            output='screen',
            parameters=[{
                'port': '/dev/ttyR2',
            }],
        ),
    ])