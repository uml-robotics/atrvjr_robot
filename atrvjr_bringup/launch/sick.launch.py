'''

<launch>

  <node pkg="sicktoolbox_wrapper" type="sicklms" name="sicklms" required="true">
    <param name="port" value="/dev/ttyR3"/>
    <param name="baud" value="38400"/>
    <param name="frame_id" value="base_laser_link0"/>
    <remap from="scan" to="scan_sick"/>
  </node>
  <!-- Broadcast the transform regarding Sick's position on the robot, relative to center of rotation -->
  <node pkg="tf" type="static_transform_publisher" name="tf_broadcaster_sick" args="0.4 0 0.1 0 0 0 1.0 base_link /base_laser_link0 100"/>

</launch>


'''

from launch import LaunchDescription

from launch_ros.actions import Node

# https://github.com/SICKAG/sick_scan_xd

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='sicktoolbox_wrapper',
            executable='sicklms',
            name='sicklms',
            parameters=[{
                'port': '/dev/ttyR3',
                'baud': 38400,
                'frame_id': 'base_laser_link0',
            }],
            remappings=[('scan', 'scan_sick')],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_broadcaster_sick',
            arguments=['0.4', '0', '0.1', '0', '0', '0', '1.0', 'base_link', '/base_laser_link0', '100'],
        ),
    ])