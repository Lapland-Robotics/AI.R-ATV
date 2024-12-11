import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='atv',
        #     executable='atv',
        #     name='atv',
        #     output='screen'
        # ),
        # Node(
        #     package='atv',
        #     executable='data_collector',
        #     name='data_collector',
        #     output='screen'
        # ), 
        Node(
            package='atv',
            executable='thermal_camera_publisher',
            name='thermal_camera_publisher',
            output='screen'
        )
    ])
