import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav_param_file_name = 'snower.yaml'
    slam_param_file_name = 'slamtoolbox_online_async_params.yaml.yaml'
    nav_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    slam_toolbox_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')
    map_dir = LaunchConfiguration(
        'map', default=os.path.join('ros2_ws/src/navigator', 'maps', 'construction_lab.yaml'))
    nav_param_dir = LaunchConfiguration(
        'params_file', default=os.path.join('ros2_ws/src/navigator','param', nav_param_file_name))
    slam_param_dir = LaunchConfiguration(
        'params_file', default=os.path.join('ros2_ws/src/navigator','param', slam_param_file_name))


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=nav_param_dir,
            description='Full path to param file to load'),
    
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time true if using a simulator (Gazebo)'),
        
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pc_to_laserscan",
            parameters=[{
                'min_height': -0.5,
                'max_height': 0.2,
                'range_min': 0.3,
                'range_max': 10.00,
                'scan_time': 0.1,
                'angle_increment': 0.0123,     # ≈ 2π/512
                'concurrency_level': 3
            }],
            remappings=[
                ("cloud_in", "/ouster/points"), 
                ("scan", "/scan")
                ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_toolbox_file_dir, '/online_async_launch.py']),
            launch_arguments={
                'params_file': slam_param_dir,
            }.items(),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav_param_dir,
            }.items(),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav_launch_file_dir, '/bringup_launch.py']),
        #     launch_arguments={
        #         'use_sim_time': use_sim_time,
        #         'map': map_dir,
        #         'params_file': nav_param_dir,
        #   }.items(),
        # ),
    ])
