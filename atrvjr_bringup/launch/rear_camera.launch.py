from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='rear_usb_cam_node',
            namespace='rear_camera',
            parameters=[
                {'param_name': 'value'},
                '/home/atrv-jr/ros2_ws/src/atrvjr_robot/atrvjr_bringup/config/rear_camera.yaml'
            ],
        ),
    ])
