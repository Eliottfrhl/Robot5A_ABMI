import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='comm_ue',
            executable='joints_command',
            name='ue_joints_command',
            output='screen'
        ),
        Node(
            package='comm_ue',
            executable='image',
            name='ue_image_publisher',
            output='screen'
        ),
        Node(
            package='comm_ue',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen'
        ),
    ])
