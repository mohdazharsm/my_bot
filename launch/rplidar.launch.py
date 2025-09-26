import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform publisher for laser frame
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='laser_static_transform_publisher',
        #     arguments=['0.122', '0', '0.212', '0', '0', '0', 'chassis', 'laser_frame']
        # ),
        
        # RPLIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.0-port0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        )
    ])