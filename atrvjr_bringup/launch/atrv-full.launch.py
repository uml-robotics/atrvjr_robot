#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- URG Node ---
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[{
            #'ip_address': '192.168.0.10',  # or comment out if using USB
             'serial_port': '/dev/ttyACM0',
            # 'serial_baud': 115200,
        }]
    )

    # --- Front Camera ---
    front_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='front_camera',
        namespace='front_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',
            'frame_id': 'front_camera_link',
            'image_width': 1280,
            'image_height': 720,
            'framerate': 10.0,
            'pixel_format': 'yuyv',
        }]
    )

    # --- Rear Camera ---
    rear_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='rear_camera',
        namespace='rear_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'frame_id': 'rear_camera_link',
            'image_width': 1280,
            'image_height': 720,
            'framerate': 10.0,
            'pixel_format': 'yuyv',
        }]
    )

    # --- Include your RFLEX launch file ---
    atrvjr_bringup_dir = get_package_share_directory('atrvjr_bringup')
    rflex_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(atrvjr_bringup_dir, 'launch', 'rflex.launch.py')
        )
    )

    return LaunchDescription([
        urg_node,
        front_cam,
        rear_cam,
        rflex_launch
    ])
