import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import xacro

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='atv',
            executable='data_collector',
            name='data_collector',
            output='screen'
        ), 
        Node(
            package='atv',
            executable='thermal_camera_publisher',
            name='thermal_camera_publisher',
            output='screen'
        ),
    ])
