import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = 'ros2_ws/src/atv/urdf/snower.urdf'
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
        
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
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='atv',
            executable='odom_to_baselink_tf',
            name='odom_to_baselink_tf',
            output='screen'
        ),
        Node(
            package='atv',
            executable='map_to_odom_tf',
            name='odom_to_baselink_tf',
            output='screen'
        ),
    ])
