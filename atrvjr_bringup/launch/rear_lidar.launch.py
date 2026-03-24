from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node',
            parameters=[
                '/home/atrv-jr/ros2_ws/src/atrvjr_robot/atrvjr_bringup/config/urg_node_serial.yaml'
            ],
        ),
    ])
