import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='esp32_microros_controller',
            executable='esp32_controller_node',
            name='esp32_controller_node',
            output='screen'
        ),
        Node(
            package='esp32_microros_controller',
            executable='data_collector',
            name='data_collector',
            output='screen'
        )
    ])
