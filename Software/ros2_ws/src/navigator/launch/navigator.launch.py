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
    scan_topic = LaunchConfiguration('scan_topic', default='/scan_filtered')
    nav_param_file_name = 'snower.yaml'
    slam_param_file_name = 'slamtoolbox_online_async_params.yaml'
    scan_filter_file_name = 'scan_filter.yaml'
    nav_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    slam_toolbox_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')
    scan_filter_file_dir = os.path.join(get_package_share_directory('laser_filters'), 'launch')
    map_dir = LaunchConfiguration(
        'map', default=os.path.join('ros2_ws/src/navigator', 'maps', 'construction_lab.yaml'))
    nav_param_dir = LaunchConfiguration(
        'nav_params_file', default=os.path.join('ros2_ws/src/navigator','param', nav_param_file_name))
    slam_param_dir = LaunchConfiguration(
        'slam_pslam_params_file', default=os.path.join('ros2_ws/src/navigator','param', slam_param_file_name))
    scan_filter_param_dir = LaunchConfiguration(
        'scan_filter_params_file', default=os.path.join('ros2_ws/src/navigator','param', scan_filter_file_name))


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'nav_params_file',
            default_value=nav_param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_param_dir,
            description='Full path to param file to load'),
        
        DeclareLaunchArgument(
            'scan_filter_params_file',
            default_value=scan_filter_param_dir,
            description='Full path to param file to load'),
    
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time true if using a simulator (Gazebo)'),

        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_filtered',
            description='Use filtered scan topic'),

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

        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[scan_filter_param_dir],
            remappings=[
                ("scan", "/scan"),
                ("scan_filtered", "/scan_filtered")],
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
